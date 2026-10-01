/*
 *  This file is a part of KNOSSOS.
 *
 *  (C) Copyright 2007-2018
 *  Max-Planck-Gesellschaft zur Foerderung der Wissenschaften e.V.
 *
 *  KNOSSOS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 of
 *  the License as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  For further information, visit https://knossos.app
 *  or contact knossosteam@gmail.com
 */

#include "segmentation/objectinventory.h"

#include "annotation/annotation.h"
#include "loader.h"
#include "segmentation/cubeloader.h"
#include "segmentation/labelonlyloading.h"
#include "segmentation/segmentation.h"
#include "segmentation/undostack.h"
#include "skeleton/skeletonizer.h"
#include "stateInfo.h"
#include "viewer.h"

#include <QProgressDialog>

#include <quazip.h>
#include <quazipfile.h>

#include <snappy.h>

#include <QBuffer>
#include <QCryptographicHash>
#include <QDataStream>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QProgressDialog>
#include <QMetaMethod>
#include <QSaveFile>
#include <QSettings>
#include <QStandardPaths>
#include <QTimer>

#include <algorithm>
#include <cmath>

namespace objinv {

QString stateName(const State s) {
    switch (s) {
    case State::Idle: return QObject::tr("idle");
    case State::Probing: return QObject::tr("checking magnifications");
    case State::Scanning: return QObject::tr("scanning");
    case State::Paused: return QObject::tr("paused");
    case State::Stalled: return QObject::tr("stalled");
    case State::Complete: return QObject::tr("complete");
    case State::Unsupported: return QObject::tr("unsupported");
    case State::Failed: return QObject::tr("failed");
    }
    return {};
}

namespace {

constexpr auto CACHE_MAGIC = "KNOSINV";
constexpr quint32 CACHE_VERSION = 1;
constexpr int CACHE_HEADER_BYTES = 128;
constexpr quint64 BYTES_PER_CUBE_GUESS = 150 * 1024;// what a compressed overlay block runs to
constexpr int GIVE_UP_AFTER = 3;                    // retries per block
constexpr int STALL_AFTER = 3;                      // consecutive blocks given up on
constexpr qint64 FLUSH_INTERVAL_MS = 250;
constexpr qint64 CHECKPOINT_MIN_MS = 30000;
constexpr quint64 CHECKPOINT_MIN_CUBES = 256;
constexpr quint64 ALL_ABSENT_WARN = 256;            // a wrong magnification, suspected late

/* KNOSSOS can only sweep a layer whose blocks it can name and decode one at a time.
 *
 * WebKnossos and Brainmaps answer whole subvolumes from a POST body, which is a different
 * request shape entirely. A SNAPPY layer has no backing storage at all — the loader
 * zero-fills it and the only content is the unsaved annotation. And `knossosCubeUrl`
 * throws for types that have no file extension in the type map, so even asking is unsafe. */
bool sweepable(const Dataset & layer, QString & why) {
    if (!layer.isOverlay()) {
        why = QObject::tr("that layer is image data, not a segmentation");
        return false;
    }
    if (layer.type == Dataset::CubeType::SNAPPY) {
        why = QObject::tr("this segmentation has no stored data — it exists only as unsaved annotation");
        return false;
    }
    if (layer.type != Dataset::CubeType::SEGMENTATION_SZ_ZIP
            && layer.type != Dataset::CubeType::SEGMENTATION_UNCOMPRESSED_64) {
        why = QObject::tr("segmentation blocks in this format cannot be read on their own yet");
        return false;
    }
    if (layer.api != Dataset::API::Heidelbrain && layer.api != Dataset::API::PyKnossos) {
        why = QObject::tr("this dataset serves whole subvolumes rather than blocks, which the sweep cannot request yet");
        return false;
    }
    return true;
}

// A copy of the layer that reads a different magnification, leaving the real one alone.
Dataset atMag(Dataset layer, const std::size_t magIndex) {
    layer.magIndex = magIndex;
    layer.magnification = 1 << magIndex;
    if (layer.scales.size() > magIndex) {
        layer.scale = layer.scales[magIndex];
        layer.scaleFactor = layer.scale / layer.scales[0];
    }
    return layer;
}

std::int32_t stepAlong(const Dataset & layer, const int axis) {
    const float f = axis == 0 ? layer.scaleFactor.x : axis == 1 ? layer.scaleFactor.y : layer.scaleFactor.z;
    return std::max(1, static_cast<std::int32_t>(std::lround(f)));
}

std::int64_t extentAlong(const Dataset & layer, const int axis) {
    return axis == 0 ? layer.boundary.x : axis == 1 ? layer.boundary.y : layer.boundary.z;
}

quint64 cubesFor(const Dataset & layer, const std::size_t magIndex) {
    const auto probe = atMag(layer, magIndex);
    quint64 n = 1;
    for (int a = 0; a < 3; ++a) {
        n *= blocksAlong(extentAlong(probe, a), a == 0 ? probe.cubeShape.x : a == 1 ? probe.cubeShape.y : probe.cubeShape.z,
                         stepAlong(probe, a));
    }
    return n;
}

/* The cache is keyed on what the layer *is*, not on where it currently sits in the layer
 * list — `Segmentation::layerId` is an index the user can reassign from the Layers panel.
 * The query string is stripped because a .k.toml may carry a rotating access token in it,
 * and hashing that would mean a fresh, empty cache every session. */
QString cacheKeyFor(const Dataset & layer, const int mag) {
    auto bare = layer.url;
    bare.setQuery(QString{});
    QString key = bare.toString(QUrl::FullyEncoded) + QChar{'\0'}
            + layer.experimentname + QChar{'\0'}
            + QString::number(static_cast<int>(layer.type)) + QChar{'\0'}
            + QString("%1,%2,%3").arg(layer.cubeShape.x).arg(layer.cubeShape.y).arg(layer.cubeShape.z) + QChar{'\0'}
            + QString("%1,%2,%3").arg(layer.boundary.x).arg(layer.boundary.y).arg(layer.boundary.z) + QChar{'\0'}
            + QString::number(mag);
    return key;
}

QString cacheDir() {
    const auto base = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    return base + "/objectinventory";
}

struct CacheContents {
    std::vector<Record> records;
    quint64 cursor{0};
    quint64 cubesDone{0}, cubesAbsent{0}, cubesEmpty{0}, cubesCorrupt{0};
    qint64 scannedAt{0};
    bool complete{false}, truncated{false};
    bool valid{false};
};

CacheContents readCache(const QString & path, const Dataset & layer, const int mag) {
    CacheContents out;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() < CACHE_HEADER_BYTES) {
        return out;
    }
    const auto header = file.read(CACHE_HEADER_BYTES);
    QDataStream in(header);
    in.setByteOrder(QDataStream::LittleEndian);
    char magic[8]{};
    in.readRawData(magic, 8);
    if (QByteArray(magic, 7) != QByteArray(CACHE_MAGIC)) {
        return out;
    }
    quint32 version{0}, recordSize{0};
    in >> version >> recordSize;
    if (version != CACHE_VERSION || recordSize != sizeof(Record)) {
        return out;// a newer or older layout; rescanning is cheaper than translating
    }
    char keyHash[20]{};
    in.readRawData(keyHash, 20);
    const auto wanted = QCryptographicHash::hash(cacheKeyFor(layer, mag).toUtf8(), QCryptographicHash::Sha1);
    if (QByteArray(keyHash, 20) != wanted) {
        return out;
    }
    quint32 storedMag{0};
    qint32 boundary[3]{}, cubeShape[3]{};
    in >> storedMag;
    for (int a = 0; a < 3; ++a) { in >> boundary[a]; }
    for (int a = 0; a < 3; ++a) { in >> cubeShape[a]; }
    // belt and braces behind the key hash, in case the dataset was re-exported in place
    if (boundary[0] != layer.boundary.x || boundary[1] != layer.boundary.y || boundary[2] != layer.boundary.z
            || cubeShape[0] != layer.cubeShape.x || cubeShape[1] != layer.cubeShape.y || cubeShape[2] != layer.cubeShape.z) {
        return out;
    }
    quint64 recordCount{0};
    quint8 complete{0}, truncated{0};
    in >> out.cursor >> out.cubesDone >> out.cubesAbsent >> out.cubesEmpty >> out.cubesCorrupt
       >> recordCount >> out.scannedAt >> complete >> truncated;

    const auto expected = static_cast<qint64>(recordCount) * static_cast<qint64>(sizeof(Record));
    if (file.size() != CACHE_HEADER_BYTES + expected + static_cast<qint64>(sizeof(quint32))) {
        return out;
    }
    const auto blob = file.read(expected);
    if (blob.size() != expected) {
        return out;
    }
    quint32 storedCrc{0};
    QDataStream crcIn(file.read(sizeof(quint32)));
    crcIn.setByteOrder(QDataStream::LittleEndian);
    crcIn >> storedCrc;
    if (storedCrc != qChecksum(blob.constData(), static_cast<uint>(blob.size()))) {
        return out;
    }
    out.records.resize(recordCount);
    if (recordCount != 0) {
        std::memcpy(out.records.data(), blob.constData(), static_cast<std::size_t>(expected));
    }
    out.complete = complete != 0;
    out.truncated = truncated != 0;
    out.valid = true;
    return out;
}

bool writeCacheFile(const QString & path, const Dataset & layer, const int mag, const CacheContents & c) {
    QDir{}.mkpath(QFileInfo(path).absolutePath());
    QSaveFile file(path);// atomic replace: a half-written cache is never read back
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    QByteArray header(CACHE_HEADER_BYTES, '\0');
    {
        QDataStream out(&header, QIODevice::WriteOnly);
        out.setByteOrder(QDataStream::LittleEndian);
        out.writeRawData(CACHE_MAGIC, 8);
        out << CACHE_VERSION << static_cast<quint32>(sizeof(Record));
        const auto hash = QCryptographicHash::hash(cacheKeyFor(layer, mag).toUtf8(), QCryptographicHash::Sha1);
        out.writeRawData(hash.constData(), 20);
        out << static_cast<quint32>(mag);
        out << static_cast<qint32>(layer.boundary.x) << static_cast<qint32>(layer.boundary.y) << static_cast<qint32>(layer.boundary.z);
        out << static_cast<qint32>(layer.cubeShape.x) << static_cast<qint32>(layer.cubeShape.y) << static_cast<qint32>(layer.cubeShape.z);
        out << c.cursor << c.cubesDone << c.cubesAbsent << c.cubesEmpty << c.cubesCorrupt
            << static_cast<quint64>(c.records.size()) << c.scannedAt
            << static_cast<quint8>(c.complete ? 1 : 0) << static_cast<quint8>(c.truncated ? 1 : 0);
    }
    file.write(header);
    const auto bytes = static_cast<qint64>(c.records.size() * sizeof(Record));
    if (bytes != 0) {
        file.write(reinterpret_cast<const char *>(c.records.data()), bytes);
    }
    QByteArray crc;
    {
        QDataStream out(&crc, QIODevice::WriteOnly);
        out.setByteOrder(QDataStream::LittleEndian);
        out << static_cast<quint32>(qChecksum(reinterpret_cast<const char *>(c.records.data()),
                                              static_cast<uint>(bytes)));
    }
    file.write(crc);
    return file.commit();
}

/* Keeps the cache directory from growing without bound. Nothing here is precious — it is
 * all re-derivable — so age and then total size are enough. */
void pruneCache() {
    QDir dir(cacheDir());
    if (!dir.exists()) {
        return;
    }
    auto files = dir.entryInfoList({"*.knoinv"}, QDir::Files, QDir::Time | QDir::Reversed);
    const auto cutoff = QDateTime::currentDateTime().addDays(-90);
    qint64 total = 0;
    for (auto it = files.begin(); it != files.end(); ) {
        if (it->lastModified() < cutoff) {
            QFile::remove(it->absoluteFilePath());
            it = files.erase(it);
        } else {
            total += it->size();
            ++it;
        }
    }
    constexpr qint64 CAP = 512ll * 1024 * 1024;
    for (auto it = files.begin(); it != files.end() && total > CAP; ++it) {
        total -= it->size();
        QFile::remove(it->absoluteFilePath());
    }
}

}

// ---------------------------------------------------------------------------- Scanner

Scanner::Scanner() {
    qnam.setRedirectPolicy(QNetworkRequest::NoLessSafeRedirectPolicy);// a CDN that redirects is not "absent"
}

Scanner::~Scanner() = default;

void Scanner::moveToThread(QThread * target) {
    qnam.moveToThread(target);
    QObject::moveToThread(target);
}

CoordOfCube Scanner::cubeAt(const std::uint64_t code) const {
    std::uint32_t x{0}, y{0}, z{0};
    mortonDecode3(code, x, y, z);
    return CoordOfCube(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
}

bool Scanner::inGrid(const std::uint64_t code) const {
    std::uint32_t x{0}, y{0}, z{0};
    mortonDecode3(code, x, y, z);
    return x < grid[0] && y < grid[1] && z < grid[2];
}

/* Which magnifications are really there.
 *
 * `highestAvailableMag` cannot be trusted: for a .k.toml it is derived from the length of
 * the declared VoxelSize_nm list, and it is then copied onto every layer from the image
 * layer. A real dataset checked during development declares six levels and serves four.
 * Asking for a level that is not there yields 404s, which — because the loader turns those
 * into zero-filled blocks — would read as "this volume contains no objects".
 *
 * Three probes per level rather than one, because a sparse volume may legitimately not have
 * a block at its centre. Runs inside a local event loop: this thread has nothing else to do
 * while probing, and cancel() still arrives, being a queued slot. */
void Scanner::probeMags(Dataset layer, const int lowestMag, const int highestMag, const Coordinate nearby) {
    QVector<MagOption> options;
    QString why;
    if (!sweepable(layer, why)) {
        emit magsProbed(options);
        emit finished(State::Unsupported, why);
        return;
    }
    struct Pending { MagOption option; std::vector<QNetworkReply *> replies; };
    std::vector<Pending> pending;
    QEventLoop loop;
    int outstanding = 0;

    for (int mag = std::max(1, lowestMag); mag <= std::max(1, highestMag); mag *= 2) {
        const auto magIndex = static_cast<std::size_t>(std::lround(std::log2(mag)));
        if (layer.scales.size() <= magIndex) {
            continue;// no scale for this level, so its coordinates would be meaningless
        }
        const auto probe = atMag(layer, magIndex);
        MagOption option;
        option.mag = mag;
        option.magIndex = magIndex;
        option.cubes = cubesFor(layer, magIndex);
        option.estBytes = option.cubes * BYTES_PER_CUBE_GUESS;

        /* Where to look.
         *
         * The crosshair first, because the user is looking at their data and the block
         * under it is the likeliest in the volume to hold something. Then a spread through
         * the volume. Three points on one axis — what this used to do — is hopeless against
         * a sparse segmentation: on a real dataset all three landed in empty regions, the
         * level was written off as absent, and the scan refused to run on a volume that
         * was full of objects. */
        std::vector<Coordinate> spots;
        const Coordinate b = probe.boundary;
        if (nearby.x >= 0 && nearby.y >= 0 && nearby.z >= 0
                && nearby.x < b.x && nearby.y < b.y && nearby.z < b.z) {
            spots.push_back(nearby);
        }
        // seven more, spread through the volume. Enough to usually find sparse data, few
        // enough that the whole sampling pass is one round trip's worth of latency.
        spots.push_back({b.x / 2, b.y / 2, b.z / 2});
        for (const auto f : {1, 3}) {
            spots.push_back({b.x * f / 4, b.y / 2, b.z / 2});
            spots.push_back({b.x / 2, b.y * f / 4, b.z / 2});
            spots.push_back({b.x / 2, b.y / 2, b.z * f / 4});
        }

        Pending p;
        p.option = option;
        for (const auto & spot : spots) {
            QNetworkRequest request;
            try {
                request = QNetworkRequest{probe.knossosCubeUrl(probe.global2cube(spot))};
            } catch (const std::exception &) {
                break;// no file extension for this type; sweepable() should have caught it
            }
            if (probe.url.scheme() == "file") {
                p.option.sampled = p.option.sampled || QFileInfo::exists(request.url().toLocalFile());
                continue;
            }
            request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
            request.setPriority(QNetworkRequest::LowPriority);
            auto * reply = qnam.head(request);
            ++outstanding;
            p.replies.push_back(reply);
            QObject::connect(reply, &QNetworkReply::finished, &loop, [&outstanding, &loop]() {
                if (--outstanding == 0) { loop.quit(); }
            });
        }
        pending.push_back(std::move(p));
    }
    if (outstanding != 0) {
        QTimer::singleShot(20000, &loop, &QEventLoop::quit);// never hang on a dead host
        loop.exec();
    }
    for (auto & p : pending) {
        for (auto * reply : p.replies) {
            const auto code = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
            p.option.sampled = p.option.sampled || (reply->error() == QNetworkReply::NoError && code == 200);
            reply->deleteLater();
        }
        options.push_back(p.option);
    }
    emit magsProbed(options);
}

void Scanner::startScan(ScanSpec s) {
    QThread::currentThread()->setPriority(QThread::IdlePriority);
    cancel();
    spec = std::move(s);
    QString why;
    if (!sweepable(spec.layer, why)) {
        emit finished(State::Unsupported, why);
        return;
    }
    for (int a = 0; a < 3; ++a) {
        grid[a] = blocksAlong(extentAlong(spec.layer, a),
                              a == 0 ? spec.layer.cubeShape.x : a == 1 ? spec.layer.cubeShape.y : spec.layer.cubeShape.z,
                              stepAlong(spec.layer, a));
    }
    const auto span = mortonSpan(grid[0], grid[1], grid[2]);
    cubesTotal = static_cast<quint64>(grid[0]) * grid[1] * grid[2];
    decodeBuffer.assign(static_cast<std::size_t>(spec.layer.cubeShape.prod()), 0);

    acc.clear();
    acc.setIdCap(spec.idCap);
    cubesDone = cubesAbsent = cubesEmpty = cubesCorrupt = 0;
    emitted = 0;
    dirty.clear();
    attempts.clear();
    consecutiveGiveUps = 0;
    scheduledRetries = 0;
    warnedAllAbsent = false;

    // pick up where a previous run left off, if the cache is still valid
    const auto cached = readCache(spec.cachePath, spec.layer, spec.magnification);
    if (cached.valid) {
        auto records = cached.records;
        cubesDone = cached.cubesDone;
        cubesAbsent = cached.cubesAbsent;
        cubesEmpty = cached.cubesEmpty;
        cubesCorrupt = cached.cubesCorrupt;
        acc.adopt(std::move(records));
        sweep.init(cached.cursor, span, covers());
    } else {
        sweep.init(spec.startCursor, span, covers());
    }
    running = true;
    paused = false;
    lastFlushMs = lastCheckpointMs = QDateTime::currentMSecsSinceEpoch();
    cubesAtCheckpoint = cubesDone;

    if (!acc.records().empty()) {
        // hand the GUI what came out of the cache before doing any new work
        emit appended(0, acc.records());
        emitted = acc.records().size();
    }
    emit progress(cubesDone, cubesTotal, acc.records().size());
    step();
}

void Scanner::setPaused(const bool on) {
    if (!running || paused == on) {
        return;
    }
    paused = on;
    if (paused) {
        flushDeltas(true);
        maybeCheckpoint(true);
    } else {
        step();
    }
}

void Scanner::setLoaderBusy(const bool busy) {
    const auto was = loaderBusy;
    loaderBusy = busy;
    if (was && !busy) {
        step();
    }
}

/* Drops every outstanding request.
 *
 * The disconnect has to come first. abort() can emit finished() synchronously, and that
 * handler erases the reply from `inFlight` — so aborting while iterating that map pulls the
 * iterator out from under the loop. Severing the connections means the handlers never run,
 * which also stops a cancelled request from calling step() back into a scan that is over.
 * The map is moved out before any of it, so even a handler that did run finds nothing. */
void Scanner::abortInFlight() {
    auto outstanding = std::move(inFlight);
    inFlight.clear();// moved-from is valid but unspecified; say what it is
    for (auto & pair : outstanding) {
        QObject::disconnect(pair.second, nullptr, this, nullptr);
        pair.second->abort();
        pair.second->deleteLater();
    }
}

void Scanner::cancel() {
    if (!running) {
        return;
    }
    running = false;
    abortInFlight();
    flushDeltas(true);
    maybeCheckpoint(true);
}

void Scanner::stop(const State outcome, const QString & detail) {
    running = false;
    abortInFlight();
    flushDeltas(true);
    writeCache(outcome == State::Complete);
    emit finished(outcome, detail);
}

/* One block per turn of the event loop.
 *
 * Self-posting rather than looping means pause, cancel and the loader brake arrive as
 * ordinary queued slots with at most one block of latency, so none of this needs atomics
 * or a mutex, and the thread is never wedged inside a loop that has to be interrupted. */
void Scanner::step() {
    if (!running || paused || stepQueued) {
        return;
    }
    issue();
    flushDeltas(false);
    maybeCheckpoint(false);
    emit progress(cubesDone, cubesTotal, acc.records().size());

    if (sweep.finished() && inFlight.empty() && scheduledRetries == 0) {
        stop(State::Complete, {});
        return;
    }
    /* Nothing is reposted while the only outstanding work is a block waiting out its
     * backoff — its retry will call back in. Otherwise this would spin the event loop for
     * the length of the wait. */
    if (!inFlight.empty() || (!loaderBusy && !sweep.exhausted())) {
        stepQueued = true;
        QMetaObject::invokeMethod(this, [this]() {
            stepQueued = false;
            step();
        }, Qt::QueuedConnection);
    }
}

void Scanner::issue() {
    const auto local = spec.layer.url.scheme() == "file";
    std::uint64_t code = 0;
    while (running && !paused && !loaderBusy
           && static_cast<int>(inFlight.size()) < std::max(1, spec.maxInFlight)
           && sweep.next(code, covers())) {
        const auto cube = cubeAt(code);
        if (local) {
            switch (readLocal(cube, decodeBuffer)) {
            case Fetch::Decoded: ingest(cube, decodeBuffer); break;
            case Fetch::Absent: ++cubesAbsent; break;
            case Fetch::Corrupt:
                ++cubesCorrupt;
                if (cubesCorrupt % 100 == 1) {
                    qDebug() << "object inventory: block" << cube << "did not decode";
                }
                break;
            }
            completeCube(code, true);
            return;// one block per turn, so a local sweep stays as polite as a remote one
        }
        requestRemote(cube, code);
    }
}

Scanner::Fetch Scanner::readLocal(const CoordOfCube & cube, std::vector<std::uint64_t> & out) {
    QString path;
    try {
        path = spec.layer.knossosCubeUrl(cube).toLocalFile();
    } catch (const std::exception & e) {
        qDebug() << "object inventory: cannot address block" << cube << e.what();
        return Fetch::Corrupt;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return Fetch::Absent;
    }
    return decode(file.readAll(), out) ? Fetch::Decoded : Fetch::Corrupt;
}

void Scanner::requestRemote(const CoordOfCube & cube, const std::uint64_t code) {
    QNetworkRequest request;
    try {
        request = spec.layer.apiSwitch(cube);
    } catch (const std::exception & e) {
        ++cubesCorrupt;
        completeCube(code, false);
        qDebug() << "object inventory: cannot address block" << cube << e.what();
        return;
    }
    request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferNetwork);
    request.setPriority(QNetworkRequest::LowPriority);// yield to the loader inside Qt too
    auto * reply = qnam.get(request);
    inFlight[code] = reply;
    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, code, cube]() {
        const auto it = inFlight.find(code);
        if (it == std::end(inFlight) || it->second != reply) {
            reply->deleteLater();
            return;// cancelled, or superseded by a rescan
        }
        inFlight.erase(it);
        const auto error = reply->error();
        const auto status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const auto payload = reply->readAll();
        reply->deleteLater();

        if (error == QNetworkReply::ContentNotFoundError || status == 404) {
            // absent is normal: a sparse volume simply has no file there
            ++cubesAbsent;
            consecutiveGiveUps = 0;
            completeCube(code, false);
        } else if (error == QNetworkReply::NoError) {
            if (decode(payload, decodeBuffer)) {
                ingest(cube, decodeBuffer);
                consecutiveGiveUps = 0;
                completeCube(code, true);
            } else {
                ++cubesCorrupt;
                if (cubesCorrupt % 100 == 1) {
                    qDebug() << "object inventory: block" << cube << "did not decode";
                }
                consecutiveGiveUps = 0;
                completeCube(code, false);
            }
        } else {
            /* Transient. Retried with backoff, and if it keeps failing the sweep stalls
             * rather than recording those blocks as empty — a cached "no objects here" from
             * a dropped connection would be indistinguishable from the truth. */
            const auto tries = ++attempts[code];
            if (tries <= GIVE_UP_AFTER) {
                const int delay = 1000 * (1 << (2 * (tries - 1)));// 1s, 4s, 16s
                ++scheduledRetries;
                QTimer::singleShot(delay, this, [this, cube, code]() {
                    --scheduledRetries;
                    if (running && inFlight.count(code) == 0) {
                        requestRemote(cube, code);
                    }
                    step();
                });
                return;
            }
            attempts.erase(code);
            if (++consecutiveGiveUps >= STALL_AFTER) {
                stop(State::Stalled, QObject::tr("the network stopped answering — resuming continues from block %1").arg(sweep.position()));
                return;
            }
            ++cubesAbsent;
            completeCube(code, false);
        }
        step();
    });
}

bool Scanner::decode(const QByteArray & payload, std::vector<std::uint64_t> & out) const {
    const auto voxels = static_cast<std::size_t>(spec.layer.cubeShape.prod());
    const auto expected = voxels * sizeof(std::uint64_t);
    if (out.size() != voxels) {
        out.assign(voxels, 0);
    }
    if (spec.layer.type == Dataset::CubeType::SEGMENTATION_UNCOMPRESSED_64) {
        if (static_cast<std::size_t>(payload.size()) != expected) {
            return false;
        }
        std::memcpy(out.data(), payload.constData(), expected);
        return true;
    }
    // SEGMENTATION_SZ_ZIP: a zip holding one snappy stream. Same as the loader does, minus
    // the residency bookkeeping and the reslice notification, which would be wrong here.
    auto data = payload;
    QBuffer buffer(&data);
    QuaZip archive(&buffer);// QuaZip needs random access, hence the buffer
    if (!archive.open(QuaZip::mdUnzip)) {
        return false;
    }
    bool ok = false;
    archive.goToFirstFile();
    QuaZipFile file(&archive);
    if (file.open(QIODevice::ReadOnly)) {
        const auto inner = file.readAll();
        std::size_t uncompressed{0};
        if (snappy::GetUncompressedLength(inner.constData(), inner.size(), &uncompressed) && uncompressed == expected) {
            ok = snappy::RawUncompress(inner.constData(), inner.size(), reinterpret_cast<char *>(out.data()));
        }
    }
    archive.close();
    return ok;
}

void Scanner::ingest(const CoordOfCube & cube, const std::vector<std::uint64_t> & voxels) {
    CubeGeometry geom;
    const auto origin = spec.layer.cube2global(cube);
    geom.origin[0] = origin.x;
    geom.origin[1] = origin.y;
    geom.origin[2] = origin.z;
    geom.shape[0] = spec.layer.cubeShape.x;
    geom.shape[1] = spec.layer.cubeShape.y;
    geom.shape[2] = spec.layer.cubeShape.z;
    for (int a = 0; a < 3; ++a) {
        geom.step[a] = stepAlong(spec.layer, a);
    }
    // physical spacing at the scan magnification, so "nearest the middle" means in tissue
    geom.nmPerVoxel[0] = spec.layer.scale.x;
    geom.nmPerVoxel[1] = spec.layer.scale.y;
    geom.nmPerVoxel[2] = spec.layer.scale.z;

    const auto before = acc.records().size();
    acc.ingestCube(voxels.data(), geom, spec.backgroundId);
    if (acc.records().size() == before && acc.touched().empty()) {
        ++cubesEmpty;
    }
    for (const auto idx : acc.touched()) {
        if (idx < emitted) {
            dirty.insert(static_cast<std::uint32_t>(idx));
        }
    }
}

void Scanner::completeCube(const std::uint64_t code, bool) {
    ++cubesDone;
    attempts.erase(code);
    sweep.complete(code, covers());
    /* A long opening run of missing blocks usually means the magnification is not really
     * there. Usually, not certainly: the walk starts at the volume corner, and a
     * segmentation exported without empty blocks can legitimately have nothing there. So
     * this says so once and carries on, rather than failing the scan — deciding whether the
     * level exists is the probe's job, and it looks at three places spread through the
     * volume instead of one corner. */
    if (!warnedAllAbsent && cubesDone >= ALL_ABSENT_WARN && cubesAbsent == cubesDone) {
        warnedAllAbsent = true;
        emit warning(QObject::tr("The first %1 blocks at %2× are all missing — if the whole scan comes back "
                                 "empty, that magnification probably has no segmentation.")
                     .arg(cubesDone).arg(spec.magnification));
    }
}

void Scanner::flushDeltas(const bool force) {
    const auto now = QDateTime::currentMSecsSinceEpoch();
    if (!force && now - lastFlushMs < FLUSH_INTERVAL_MS) {
        return;
    }
    lastFlushMs = now;
    const auto & records = acc.records();
    if (!dirty.empty()) {
        std::vector<std::uint32_t> indices(std::begin(dirty), std::end(dirty));
        std::sort(std::begin(indices), std::end(indices));
        std::vector<Record> revisedRecords;
        revisedRecords.reserve(indices.size());
        for (const auto i : indices) {
            revisedRecords.push_back(records[i]);
        }
        dirty.clear();
        emit revised(std::move(indices), std::move(revisedRecords));
    }
    if (records.size() > emitted) {
        std::vector<Record> tail(std::begin(records) + static_cast<std::ptrdiff_t>(emitted), std::end(records));
        const auto first = emitted;
        emitted = records.size();
        emit appended(first, std::move(tail));
    }
}

/* Checkpointing is time-budgeted rather than counted, because the whole file is rewritten
 * every time: a record's position never changes but its contents keep changing, so there is
 * nothing to append. Small files checkpoint often, large ones back off, and the write never
 * takes more than a couple of per cent of the sweep. */
void Scanner::maybeCheckpoint(const bool force) {
    const auto now = QDateTime::currentMSecsSinceEpoch();
    if (!force) {
        if (cubesDone - cubesAtCheckpoint < CHECKPOINT_MIN_CUBES) {
            return;
        }
        if (now - lastCheckpointMs < std::max(CHECKPOINT_MIN_MS, 50 * lastCheckpointCost)) {
            return;
        }
    }
    if (cubesDone == cubesAtCheckpoint && !force) {
        return;
    }
    QElapsedTimer timer;
    timer.start();
    writeCache(false);
    lastCheckpointCost = timer.elapsed();
    lastCheckpointMs = QDateTime::currentMSecsSinceEpoch();
    cubesAtCheckpoint = cubesDone;
}

void Scanner::writeCache(const bool complete) {
    if (spec.cachePath.isEmpty()) {
        return;
    }
    CacheContents c;
    c.records = acc.records();
    c.cursor = sweep.position();
    c.cubesDone = cubesDone;
    c.cubesAbsent = cubesAbsent;
    c.cubesEmpty = cubesEmpty;
    c.cubesCorrupt = cubesCorrupt;
    c.scannedAt = QDateTime::currentSecsSinceEpoch();
    c.complete = complete;
    c.truncated = acc.truncated();
    writeCacheFile(spec.cachePath, spec.layer, spec.magnification, c);
}

// -------------------------------------------------------------------------- Inventory

Inventory::Inventory() {
    worker = std::make_unique<Scanner>();
    workerThread.setObjectName("ObjectInventory");
    worker->moveToThread(&workerThread);

    QObject::connect(worker.get(), &Scanner::magsProbed, this, [this](QVector<MagOption> options) {
        // only the sampled flags are taken; the counts were worked out locally
        for (const auto & probed : options) {
            for (auto & known : mags) {
                if (known.mag == probed.mag) {
                    known.sampled = probed.sampled;
                }
            }
        }
        probedThisDataset = true;
        if (currentState == State::Probing) {
            setState(State::Idle);
        }
        emit magOptionsChanged();
    });
    QObject::connect(worker.get(), &Scanner::progress, this, [this](quint64 d, quint64 t, quint64 objects) {
        if (doneAtStart == 0 && done == 0) {
            doneAtStart = d;// whatever the cache already had; the rate must not include it
        }
        done = d;
        total = t;
        emit progressChanged(d, t, objects);
    });
    QObject::connect(worker.get(), &Scanner::appended, this, [this](const quint64 first, std::vector<Record> batch) {
        if (first == 0) {
            recs.clear();
            index.clear();
        }
        if (first != recs.size()) {
            return;// out of step with the worker; the next full start will resynchronise
        }
        const auto count = batch.size();
        recs.reserve(recs.size() + count);
        for (auto & r : batch) {
            index.emplace(r.id, recs.size());
            recs.push_back(r);
        }
        if (count != 0) {
            emit recordsAppended(first, count);
        }
    });
    QObject::connect(worker.get(), &Scanner::revised, this, [this](std::vector<std::uint32_t> indices, std::vector<Record> updated) {
        if (indices.size() != updated.size() || indices.empty()) {
            return;
        }
        for (std::size_t k = 0; k < indices.size(); ++k) {
            if (indices[k] < recs.size()) {
                recs[indices[k]] = updated[k];
            }
        }
        emit recordsRevised(indices.front(), indices.back());
    });
    QObject::connect(worker.get(), &Scanner::warning, this, &Inventory::warning);
    /* Recorded whoever did the writing and whatever was selected at the time, which is the
     * point: a false positive removed with the bucket on background never selects anything. */
    QObject::connect(&Segmentation::singleton(), &Segmentation::subobjectPainted,
                     this, &Inventory::noteSubobjectPainted);
    QObject::connect(&Segmentation::singleton(), &Segmentation::subobjectOverwritten,
                     this, &Inventory::noteSubobjectOverwritten);
    QObject::connect(worker.get(), &Scanner::finished, this, [this](State outcome, QString detail) {
        if (outcome == State::Complete) {
            timestamp = QDateTime::currentDateTime();
        }
        setState(outcome, detail);
        if (!detail.isEmpty()) {
            emit warning(detail);
        }
    });

    /* Every argument of a signal that crosses to the worker thread has to be a type Qt can
     * queue, and it goes by the *name* moc recorded, not the type behind it — "std::size_t"
     * is not a name Qt knows even though it is an unsigned long. Getting that wrong drops
     * the call at emit time with a warning that is easy to miss, which is how the scan came
     * to run, report its progress, and hand over none of what it found. Checked here so the
     * next one says so at startup. */
    for (const auto * mop : {&Scanner::staticMetaObject, &Inventory::staticMetaObject}) {
    const auto & mo = *mop;
    for (int i = mo.methodOffset(); i < mo.methodCount(); ++i) {
        const auto method = mo.method(i);
        if (method.methodType() != QMetaMethod::Signal && method.methodType() != QMetaMethod::Slot) {
            continue;
        }
        for (int a = 0; a < method.parameterCount(); ++a) {
            if (method.parameterType(a) == QMetaType::UnknownType) {
                const auto types = method.parameterTypes();
                qWarning() << "object inventory:" << method.methodSignature() << "argument"
                           << (a < types.size() ? types.at(a) : QByteArray{"?"})
                           << "has no metatype and cannot cross threads";
            }
        }
    }
    }

    workerThread.start();
}

void Inventory::suspend() {
    if (!workerThread.isRunning()) {
        return;// a blocking call into a thread with no event loop would never come back
    }
    QMetaObject::invokeMethod(worker.get(), "cancel", Qt::BlockingQueuedConnection);
    workerThread.quit();
    workerThread.wait();
}

void Inventory::setState(const State s, const QString & detail) {
    currentDetail = detail;
    if (currentState == s) {
        return;
    }
    currentState = s;
    emit stateChanged(s);
}

std::optional<std::size_t> Inventory::indexOfId(const std::uint64_t id) const {
    const auto it = index.find(id);
    return it == std::end(index) ? std::nullopt : std::optional<std::size_t>{it->second};
}

bool Inventory::layerUsable(QString & why) const {
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        why = tr("no segmentation layer is loaded");
        return false;
    }
    return sweepable(Dataset::datasets[layerId], why);
}

std::size_t Inventory::cubeBudget() const {
    const auto layerId = Segmentation::singleton().layerId;
    const auto local = layerId < Dataset::datasets.size() && Dataset::datasets[layerId].url.scheme() == "file";
    QSettings settings;
    /* Remote blocks cost bandwidth, local ones cost only disk, so the local budget buys a
     * whole magnification level of recall for nothing anyone is waiting on. */
    return settings.value(local ? "objectInventoryLocalCubeBudget" : "objectInventoryCubeBudget",
                          local ? 96000 : 12000).toULongLong();
}

int Inventory::autoMag() const {
    const auto budget = cubeBudget();
    auto sorted = mags;
    // finest first, so the answer is the most detail the budget allows
    std::sort(std::begin(sorted), std::end(sorted), [](const MagOption & a, const MagOption & b) { return a.mag < b.mag; });
    /* A level where a sample was found is preferred, but one where none was is still
     * offered: a sparse segmentation can easily have nothing at any of the sampled blocks
     * while being full of objects elsewhere, and refusing on that evidence is how a scan
     * came to do nothing at all on a volume that had plenty to find. */
    for (const auto requireSample : {true, false}) {
        for (const auto & option : sorted) {
            if ((option.sampled || !requireSample) && option.cubes <= budget) {
                return option.mag;
            }
        }
    }
    int coarsest = 0;// nothing fits the budget; the coarsest is the least bad, and it is reported
    for (const auto & option : mags) {
        coarsest = std::max(coarsest, option.mag);
    }
    return coarsest;
}

QString Inventory::cachePathFor(const int magnification) const {
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return {};
    }
    const auto key = cacheKeyFor(Dataset::datasets[layerId], magnification);
    const auto hash = QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Sha1).toHex();
    return cacheDir() + "/" + QString::fromLatin1(hash) + ".knoinv";
}

void Inventory::buildMagOptions() {
    mags.clear();
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return;
    }
    const auto & layer = Dataset::datasets[layerId];
    for (int mag = std::max(1, layer.lowestAvailableMag); mag <= std::max(1, layer.highestAvailableMag); mag *= 2) {
        const auto magIndex = static_cast<std::size_t>(std::lround(std::log2(mag)));
        if (layer.scales.size() <= magIndex) {
            continue;// no voxel size declared for this level, so its coordinates are meaningless
        }
        MagOption option;
        option.mag = mag;
        option.magIndex = magIndex;
        option.cubes = cubesFor(layer, magIndex);
        option.estBytes = option.cubes * BYTES_PER_CUBE_GUESS;
        mags.push_back(option);
    }
}

void Inventory::probe() {
    QString why;
    if (!layerUsable(why)) {
        setState(State::Unsupported, why);
        return;
    }
    if (currentState == State::Probing) {
        return;// already asked; whatever wanted a scan is waiting in pendingScanMag
    }
    const auto & layer = Dataset::datasets[Segmentation::singleton().layerId];
    setState(State::Probing);
    // the block under the crosshair is the likeliest in the volume to hold something
    const auto nearby = ::state->viewerState != nullptr ? ::state->viewerState->currentPosition : Coordinate{-1, -1, -1};
    QMetaObject::invokeMethod(worker.get(), "probeMags", Qt::QueuedConnection,
                              Q_ARG(Dataset, layer), Q_ARG(int, layer.lowestAvailableMag),
                              Q_ARG(int, layer.highestAvailableMag), Q_ARG(Coordinate, nearby));
}

void Inventory::startScan(const int requested) {
    QString why;
    if (!layerUsable(why)) {
        setState(State::Unsupported, why);
        emit warning(why);
        return;
    }
    if (mags.empty()) {
        buildMagOptions();// needs no network, so the scan never waits on one
        emit magOptionsChanged();
    }
    pendingScanMag = -1;
    const auto chosen = requested != 0 ? requested : autoMag();
    if (chosen == 0) {
        /* Not necessarily an error: a dataset can have no stored segmentation at all, with
         * every label living in the annotation instead. Say which, because the two call for
         * completely different things from the reader. */
        const auto message = tr("this dataset declares no magnifications to scan.");
        setState(State::Failed, message);
        emit warning(message);
        return;
    }
    const auto found = std::find_if(std::begin(mags), std::end(mags),
                                    [chosen](const MagOption & o) { return o.mag == chosen; });
    if (found == std::end(mags)) {
        const auto message = tr("This dataset does not declare magnification %1.").arg(chosen);
        setState(State::Failed, message);
        emit warning(message);
        return;
    }
    if (!found->sampled) {
        // worth saying, but not worth refusing over — see autoMag()
        emit warning(tr("Nothing was found in the blocks sampled at %1×, which is normal for a sparse "
                        "segmentation. Scanning anyway.").arg(chosen));
    }
    // by value: what follows emits, and an iterator into a QVector is not worth trusting across that
    const MagOption option = *found;
    if (option.cubes > cubeBudget()) {
        emit warning(tr("Scanning at %1× means %2 blocks, past the %3 the budget allows — this will take a while.")
                     .arg(chosen).arg(option.cubes).arg(cubeBudget()));
    }

    /* Announced first. Everything below — pruning the cache directory, clearing the list,
     * reading settings — happens on this thread, and until the state changed the button
     * still said "Scan", so a click looked like it had been ignored. */
    setState(State::Scanning);
    pruneCache();
    mag = chosen;
    fromAnnotation = false;
    recs.clear();
    index.clear();
    warnedTruncated = false;
    done = total = 0;
    /* Said now rather than when the worker's first batch arrives: until then the table would
     * still be indexing rows of a list that has just been emptied. */
    emit recordsAppended(0, 0);

    QSettings settings;
    ScanSpec spec;
    spec.layer = atMag(Dataset::datasets[Segmentation::singleton().layerId], option.magIndex);
    spec.magIndex = option.magIndex;
    spec.magnification = chosen;
    spec.backgroundId = Segmentation::singleton().getBackgroundId();
    spec.idCap = settings.value("objectInventoryIdCap", 500000).toULongLong();
    spec.maxInFlight = settings.value("objectInventoryMaxInFlight", 4).toInt();
    spec.cachePath = cachePathFor(chosen);

    scanClock.start();
    doneAtStart = 0;// set from the first progress report, which carries the resumed count
    QMetaObject::invokeMethod(worker.get(), "startScan", Qt::QueuedConnection, Q_ARG(objinv::ScanSpec, spec));
}

void Inventory::pause() {
    if (currentState != State::Scanning) {
        return;
    }
    setState(State::Paused);
    QMetaObject::invokeMethod(worker.get(), "setPaused", Qt::QueuedConnection, Q_ARG(bool, true));
}

void Inventory::resume() {
    if (currentState == State::Paused) {
        setState(State::Scanning);
        QMetaObject::invokeMethod(worker.get(), "setPaused", Qt::QueuedConnection, Q_ARG(bool, false));
    } else if (currentState == State::Stalled) {
        startScan(mag);// the cache still holds the cursor, so this continues rather than restarts
    }
}

void Inventory::cancel() {
    if (currentState != State::Scanning && currentState != State::Paused) {
        return;
    }
    QMetaObject::invokeMethod(worker.get(), "cancel", Qt::QueuedConnection);
    setState(State::Idle);
}

void Inventory::rescan() {
    const auto path = cachePathFor(mag != 0 ? mag : autoMag());
    QMetaObject::invokeMethod(worker.get(), "cancel", Qt::BlockingQueuedConnection);
    if (!path.isEmpty()) {
        QFile::remove(path);
    }
    recs.clear();
    index.clear();
    done = total = 0;
    setState(State::Idle);
    startScan(mag);
}

void Inventory::onDatasetChanged() {
    QMetaObject::invokeMethod(worker.get(), "cancel", Qt::QueuedConnection);
    recs.clear();
    index.clear();
    mags.clear();
    probedThisDataset = false;
    pendingScanMag = -1;
    fromAnnotation = false;
    mag = 0;
    done = total = 0;
    timestamp = {};
    setState(State::Idle);
    emit magOptionsChanged();
    emit recordsAppended(0, 0);
    QString why;
    if (layerUsable(why)) {
        buildMagOptions();
        emit magOptionsChanged();
        loadCache();// local, cheap, and tells the user a list already exists
    } else {
        setState(State::Unsupported, why);
    }
}

bool Inventory::datasetHasSegmentation() const {
    return std::any_of(std::begin(mags), std::end(mags), [](const MagOption & o){ return o.sampled; });
}

void Inventory::scanAnnotation(QWidget * const parent) {
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size() || !Dataset::datasets[layerId].isOverlay()) {
        emit warning(tr("No segmentation layer is loaded."));
        return;
    }
    const auto & layer = Dataset::datasets[layerId];
    const auto voxels = static_cast<std::size_t>(layer.cubeShape.prod());
    const auto expected = voxels * sizeof(std::uint64_t);

    /* Holds the loader's cache for the duration, which stalls block loading — so it runs
     * behind a modal dialog, where nothing is browsing anyway, the way mesh generation
     * does the same walk. */
    const auto guard = Loader::Controller::singleton().getAllModifiedCubes(layerId);
    const auto & byMag = guard.cubes;
    std::size_t total = 0;
    for (const auto & set : byMag) {
        total += set.size();
    }
    if (total == 0) {
        setState(State::Idle, {});
        emit warning(tr("The annotation has no painted blocks yet."));
        return;
    }

    QSettings settings;
    Accumulator acc;
    acc.setIdCap(settings.value("objectInventoryIdCap", 500000).toULongLong());
    const auto background = Segmentation::singleton().getBackgroundId();
    std::vector<std::uint64_t> buffer(voxels, 0);

    QProgressDialog progress(tr("Reading the annotation…"), tr("Cancel"), 0, static_cast<int>(total), parent);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(400);
    int donePieces = 0;
    bool cancelled = false;

    for (std::size_t magIndex = 0; magIndex < byMag.size() && !cancelled; ++magIndex) {
        const auto atThisMag = atMag(layer, magIndex);
        CubeGeometry geom;
        geom.shape[0] = layer.cubeShape.x;
        geom.shape[1] = layer.cubeShape.y;
        geom.shape[2] = layer.cubeShape.z;
        for (int a = 0; a < 3; ++a) {
            geom.step[a] = stepAlong(atThisMag, a);
        }
        geom.nmPerVoxel[0] = atThisMag.scale.x;
        geom.nmPerVoxel[1] = atThisMag.scale.y;
        geom.nmPerVoxel[2] = atThisMag.scale.z;

        for (const auto & entry : byMag[magIndex]) {
            progress.setValue(donePieces++);
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
            if (progress.wasCanceled()) {
                cancelled = true;
                break;
            }
            std::size_t uncompressed{0};
            if (!snappy::GetUncompressedLength(entry.second.data(), entry.second.size(), &uncompressed)
                    || uncompressed != expected
                    || !snappy::RawUncompress(entry.second.data(), entry.second.size(),
                                              reinterpret_cast<char *>(buffer.data()))) {
                continue;// a block that will not decode tells us nothing; skip it
            }
            const auto origin = atThisMag.cube2global(entry.first);
            geom.origin[0] = origin.x;
            geom.origin[1] = origin.y;
            geom.origin[2] = origin.z;
            acc.ingestCube(buffer.data(), geom, background);
        }
    }
    progress.setValue(static_cast<int>(total));

    recs = acc.records();
    index.clear();
    for (std::size_t i = 0; i < recs.size(); ++i) {
        index.emplace(recs[i].id, i);
    }
    mag = 1;// annotation cubes carry their own magnification; the list is in mag-1 coordinates
    done = total;
    this->total = total;
    timestamp = QDateTime::currentDateTime();
    fromAnnotation = true;
    setState(cancelled ? State::Idle : State::Complete, {});
    emit recordsAppended(0, recs.size());
    emit progressChanged(done, this->total, recs.size());
    if (acc.truncated()) {
        emit warning(tr("Stopped at %1 objects — raise the ceiling to list more.").arg(recs.size()));
    }
}

bool Inventory::eraseObject(const quint64 soid, QWidget * const parent) {
    const auto at = indexOfId(soid);
    if (!at) {
        emit warning(tr("That object is not in the list."));
        return false;
    }
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return false;
    }
    const auto & layer = Dataset::datasets[layerId];
    const auto background = Segmentation::singleton().getBackgroundId();
    if (soid == background) {
        emit warning(tr("That is the background id; there is nothing to erase."));
        return false;
    }
    const auto record = recs[*at];// by value: the walk below moves the crosshair and loads blocks
    const Coordinate first{record.bboxMin[0], record.bboxMin[1], record.bboxMin[2]};
    const Coordinate last{record.bboxMax[0], record.bboxMax[1], record.bboxMax[2]};

    const auto cubeExtent = layer.scaleFactor.componentMul(layer.cubeShape);
    const auto cubeBegin = layer.global2cube(first);
    const auto cubeEnd = layer.global2cube(last) + 1;
    const auto blocks = static_cast<int>(std::max(1, (cubeEnd.x - cubeBegin.x))
                                       * std::max(1, (cubeEnd.y - cubeBegin.y))
                                       * std::max(1, (cubeEnd.z - cubeBegin.z)));

    const UndoScope undoScope(tr("Erase object %1").arg(soid));
    QProgressDialog progress(tr("Erasing object %1…").arg(soid), tr("Cancel"), 0, blocks, parent);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(400);
    const LabelOnlyLoading labelOnly;// the image data is irrelevant here

    const auto startPosition = ::state->viewerState->currentPosition;
    std::size_t erased = 0, missing = 0;
    int visited = 0;
    bool cancelled = false;
    for (int cz = cubeBegin.z; cz < cubeEnd.z && !cancelled; ++cz)
    for (int cy = cubeBegin.y; cy < cubeEnd.y && !cancelled; ++cy)
    for (int cx = cubeBegin.x; cx < cubeEnd.x && !cancelled; ++cx) {
        progress.setValue(visited++);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
        if (progress.wasCanceled()) {
            cancelled = true;
            break;
        }
        const CoordOfCube cube{cx, cy, cz};
        const auto cubeFirst = layer.cube2global(cube);
        const auto cubeLast = cubeFirst + cubeExtent - 1;
        const Coordinate regionFirst{std::max(first.x, cubeFirst.x), std::max(first.y, cubeFirst.y), std::max(first.z, cubeFirst.z)};
        const Coordinate regionLast{std::min(last.x, cubeLast.x), std::min(last.y, cubeLast.y), std::min(last.z, cubeLast.z)};

        if (!regionCubeResidency(regionFirst, regionLast).second.empty()) {
            ::state->viewer->setPosition(cubeFirst + cubeExtent / 2, USERMOVE_NEUTRAL);
            // the loader needs a moment; asked again afterwards rather than assumed
            QElapsedTimer waited;
            waited.start();
            while (!Loader::Controller::singleton().isFinished() && waited.elapsed() < 30000) {
                QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 20);
                if (progress.wasCanceled()) {
                    cancelled = true;
                    break;
                }
            }
            if (!regionCubeResidency(regionFirst, regionLast).second.empty()) {
                ++missing;
                continue;
            }
        }
        erased += processRegionReplacing(regionFirst, regionLast, soid, background);
    }
    progress.setValue(blocks);
    ::state->viewer->setPosition(startPosition, USERMOVE_NEUTRAL);

    if (erased != 0) {
        setFlag(soid, Flag::Erased, true);
    }
    if (missing != 0) {
        // never silently: a skipped block means part of the object is still there
        emit warning(tr("Erased %1 voxels of object %2, but %3 block(s) would not load and were skipped — "
                        "some of it may remain.").arg(erased).arg(soid).arg(missing));
    } else if (cancelled) {
        emit warning(tr("Cancelled after erasing %1 voxels of object %2. What was erased is kept.").arg(erased).arg(soid));
    } else {
        emit warning(tr("Erased %1 voxels of object %2.").arg(erased).arg(soid));
    }
    return erased != 0;
}

void Inventory::ensureProbed() {
    if (probedThisDataset || currentState == State::Probing || currentState == State::Scanning) {
        return;
    }
    probe();
}

/* Whatever a previous session left behind, without doing any network work. Tries the
 * levels that could plausibly have been used, finest first, and takes the first hit. */
void Inventory::loadCache() {
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return;
    }
    const auto & layer = Dataset::datasets[layerId];
    for (int candidate = 1; candidate <= std::max(1, layer.highestAvailableMag); candidate *= 2) {
        const auto cached = readCache(cachePathFor(candidate), layer, candidate);
        if (!cached.valid || cached.records.empty()) {
            continue;
        }
        recs = cached.records;
        index.clear();
        for (std::size_t i = 0; i < recs.size(); ++i) {
            index.emplace(recs[i].id, i);
        }
        mag = candidate;
        timestamp = QDateTime::fromSecsSinceEpoch(cached.scannedAt);
        done = cached.cubesDone;
        total = cubesFor(layer, static_cast<std::size_t>(std::lround(std::log2(candidate))));
        setState(cached.complete ? State::Complete : State::Idle,
                 cached.complete ? QString{} : tr("a previous scan stopped partway — resume to finish it"));
        emit recordsAppended(0, recs.size());
        return;
    }
}

void Inventory::onLoaderProgress(const int count) {
    /* Hysteresis: without it a single scroll would produce a stutter of stop and start. The
     * sweep resumes only once the loader has been quiet for a moment. */
    const auto now = QDateTime::currentMSecsSinceEpoch();
    if (count > 0) {
        loaderIdleSince = 0;
        QMetaObject::invokeMethod(worker.get(), "setLoaderBusy", Qt::QueuedConnection, Q_ARG(bool, true));
        return;
    }
    if (loaderIdleSince == 0) {
        loaderIdleSince = now;
        QTimer::singleShot(800, this, [this]() {
            if (loaderIdleSince != 0 && QDateTime::currentMSecsSinceEpoch() - loaderIdleSince >= 750) {
                QMetaObject::invokeMethod(worker.get(), "setLoaderBusy", Qt::QueuedConnection, Q_ARG(bool, false));
            }
        });
    }
}

quint8 Inventory::flagsFor(const std::uint64_t soid) const {
    const auto it = flags.find(soid);
    return it == std::end(flags) ? 0 : it->second;
}

void Inventory::setFlag(const std::uint64_t soid, const Flag flag, const bool on) {
    auto & bits = flags[soid];
    const auto before = bits;
    bits = on ? static_cast<quint8>(bits | flag) : static_cast<quint8>(bits & ~flag);
    if (bits == 0) {
        flags.erase(soid);
    }
    if (bits != before) {
        emit flagsChanged();
    }
}

std::size_t Inventory::countWith(const Flag flag) const {
    return static_cast<std::size_t>(std::count_if(std::begin(flags), std::end(flags),
                                                  [flag](const auto & pair){ return (pair.second & flag) != 0; }));
}

void Inventory::noteSubobjectPainted(const quint64 soid) {
    // only ids the list knows about; a brand new object is not part of this inventory
    if (index.count(soid) != 0) {
        setFlag(soid, Flag::Painted, true);
    }
}

void Inventory::noteSubobjectOverwritten(const quint64 soid) {
    if (index.count(soid) != 0) {
        setFlag(soid, Flag::Erased, true);
    }
}

void Inventory::refreshSkeletonFlags() {
    bool changed = false;
    std::unordered_set<std::uint64_t> withNodes;
    for (const auto & tree : Skeletonizer::singleton().skeletonState.trees) {
        for (auto it = std::begin(tree.subobjectCount); it != std::end(tree.subobjectCount); ++it) {
            if (it.value() > 0) {
                withNodes.insert(it.key());
            }
        }
    }
    for (const auto soid : withNodes) {
        if (index.count(soid) != 0 && (flagsFor(soid) & Flag::Skeleton) == 0) {
            flags[soid] |= Flag::Skeleton;
            changed = true;
        }
    }
    // a tree being deleted takes the mark with it, so this is a refresh rather than a union
    for (auto it = std::begin(flags); it != std::end(flags); ) {
        if ((it->second & Flag::Skeleton) != 0 && withNodes.count(it->first) == 0) {
            it->second = static_cast<quint8>(it->second & ~Flag::Skeleton);
            changed = true;
        }
        it = it->second == 0 ? flags.erase(it) : std::next(it);
    }
    if (changed) {
        emit flagsChanged();
    }
}

/* Travels with the annotation, keyed by id.
 *
 * It is a record of what has been done, not a property of the dataset, so it belongs to the
 * annotation rather than to the per-dataset cache — and keying on the id means a rescan that
 * finds things in a different order keeps all of it. */
QByteArray Inventory::flagsJson() const {
    if (flags.empty()) {
        return {};
    }
    QJsonObject bits;
    for (const auto & [soid, value] : flags) {
        if (value != 0) {
            bits[QString::number(soid)] = value;
        }
    }
    QJsonObject root;
    root["version"] = 2;
    root["mag"] = mag;
    root["flags"] = bits;
    return QJsonDocument{root}.toJson(QJsonDocument::Compact);
}

void Inventory::importStateJson(const QByteArray & json) {
    const auto document = QJsonDocument::fromJson(json);
    if (!document.isObject()) {
        return;
    }
    const auto root = document.object();
    flags.clear();
    const auto bits = root["flags"].toObject();
    for (auto it = bits.begin(); it != bits.end(); ++it) {
        bool ok = false;
        const auto soid = it.key().toULongLong(&ok);
        const auto value = static_cast<quint8>(it.value().toInt());
        if (ok && value != 0) {
            flags[soid] = value;
        }
    }
    // version 1 stored only the visited ids, as a list
    for (const auto value : root["visited"].toArray()) {
        bool ok = false;
        const auto soid = value.toString().toULongLong(&ok);
        if (ok) {
            flags[soid] |= Flag::Visited;
        }
    }
    emit flagsChanged();
}

bool Inventory::mayBeStale() const {
    return !recs.empty() && Annotation::singleton().unsavedChanges;
}

std::optional<int> Inventory::etaSeconds() const {
    if (currentState != State::Scanning || total == 0 || done <= doneAtStart || !scanClock.isValid()) {
        return std::nullopt;
    }
    const auto elapsed = scanClock.elapsed();
    if (elapsed < 2000) {
        return std::nullopt;// too early for the rate to mean anything
    }
    const auto didThisRun = done - doneAtStart;
    const auto left = total > done ? total - done : 0;
    if (left == 0) {
        return 0;
    }
    const auto perBlockMs = static_cast<double>(elapsed) / static_cast<double>(didThisRun);
    return static_cast<int>(perBlockMs * static_cast<double>(left) / 1000.0);
}

QString Inventory::progressLine() const {
    if (total == 0) {
        return {};
    }
    auto line = tr("%1% — %2 of %3 blocks").arg(percentDone()).arg(done).arg(total);
    if (const auto eta = etaSeconds()) {
        const auto secs = *eta;
        const auto pretty = secs < 60 ? tr("under a minute")
                          : secs < 3600 ? tr("about %n minute(s)", "", (secs + 30) / 60)
                                        : tr("about %1 h %2 m").arg(secs / 3600).arg((secs % 3600) / 60);
        line += tr(" · %1 left").arg(pretty);
    }
    return line;
}

QString Inventory::statusLine() const {
    if (currentState == State::Unsupported) {
        return tr("Object inventory unavailable: %1").arg(currentDetail);
    }
    if (currentState == State::Failed && recs.empty()) {
        /* A refusal has to stay on screen. This used to be an eight-second message in the
         * status bar while the tab went on saying "press Scan to build one", so pressing
         * Scan on a dataset with no stored segmentation looked like a dead button. */
        return tr("Nothing to scan: %1").arg(currentDetail);
    }
    if (recs.empty()) {
        return currentState == State::Scanning
                ? tr("Scanning at %1× — %2, nothing found yet").arg(mag).arg(progressLine())
                : tr("No object inventory yet — press Scan to build one");
    }
    QString line = tr("%n object(s)", "", static_cast<int>(recs.size()));
    line += fromAnnotation ? tr(" · from the annotation") : tr(" · %1×").arg(mag);
    if (currentState == State::Scanning) {
        line += tr(" · scanning, %1").arg(progressLine());
    } else if (timestamp.isValid()) {
        line += tr(" · scanned %1").arg(timestamp.toString(Qt::ISODate));
    }
    if (currentState == State::Stalled || currentState == State::Paused) {
        line += tr(" · %1").arg(stateName(currentState));
    }
    /* Said out loud rather than left to be discovered. The two sources have opposite
     * blind spots: a dataset sweep sees what the server stores and misses everything
     * painted since, while an annotation scan sees exactly what has been painted and
     * nothing that was already baked into the volume. */
    if (fromAnnotation) {
        line += tr(" · everything painted in this annotation, and nothing already in the dataset");
    } else {
        line += tr(" · excludes unsaved edits");
        if (mag > 1) {
            line += tr(" and objects under about %1 voxels").arg(mag * mag * mag);
        }
    }
    if (mayBeStale() && !fromAnnotation) {
        line += tr(" · may be out of date");
    }
    return line;
}

}
