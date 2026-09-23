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
#include "segmentation/segmentation.h"
#include "stateInfo.h"

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
#include <QLockFile>
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
constexpr int ALL_ABSENT_ABORT = 32;                // a wrong magnification, caught late

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
void Scanner::probeMags(Dataset layer, const int lowestMag, const int highestMag) {
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

        std::vector<Coordinate> spots;
        const Coordinate b = probe.boundary;
        spots.push_back({b.x / 2, b.y / 2, b.z / 2});
        const auto longest = b.x >= b.y && b.x >= b.z ? 0 : (b.y >= b.z ? 1 : 2);
        for (const auto frac : {1, 3}) {
            Coordinate at{b.x / 2, b.y / 2, b.z / 2};
            (longest == 0 ? at.x : longest == 1 ? at.y : at.z) = (longest == 0 ? b.x : longest == 1 ? b.y : b.z) * frac / 4;
            spots.push_back(at);
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
                p.option.present = p.option.present || QFileInfo::exists(request.url().toLocalFile());
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
            p.option.present = p.option.present || (reply->error() == QNetworkReply::NoError && code == 200);
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

void Scanner::cancel() {
    if (!running) {
        return;
    }
    running = false;
    for (auto & pair : inFlight) {
        pair.second->abort();
        pair.second->deleteLater();
    }
    inFlight.clear();
    flushDeltas(true);
    maybeCheckpoint(true);
}

void Scanner::stop(const State outcome, const QString & detail) {
    running = false;
    for (auto & pair : inFlight) {
        pair.second->abort();
        pair.second->deleteLater();
    }
    inFlight.clear();
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
    if (cubesDone == ALL_ABSENT_ABORT && cubesAbsent == cubesDone) {
        stop(State::Failed, QObject::tr("no segmentation blocks found at magnification %1 — try a finer one")
             .arg(spec.magnification));
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
        mags = std::move(options);
        probedThisDataset = true;
        if (currentState == State::Probing) {
            setState(State::Idle);
        }
        emit magOptionsChanged();
        if (pendingScanMag >= 0) {
            const auto requested = pendingScanMag;
            pendingScanMag = -1;
            startScan(requested);
        }
    });
    QObject::connect(worker.get(), &Scanner::progress, this, [this](quint64 d, quint64 t, quint64 objects) {
        done = d;
        total = t;
        emit progressChanged(d, t, objects);
    });
    QObject::connect(worker.get(), &Scanner::appended, this, [this](std::size_t first, std::vector<Record> batch) {
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
    QObject::connect(worker.get(), &Scanner::finished, this, [this](State outcome, QString detail) {
        if (outcome == State::Complete) {
            timestamp = QDateTime::currentDateTime();
        }
        setState(outcome, detail);
        if (!detail.isEmpty()) {
            emit warning(detail);
        }
    });

    workerThread.start();
}

void Inventory::suspend() {
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
    int coarsestPresent = 0;
    for (const auto & option : mags) {
        if (option.present) {
            coarsestPresent = std::max(coarsestPresent, option.mag);
        }
    }
    // finest first, so the answer is the most detail the budget allows
    auto sorted = mags;
    std::sort(std::begin(sorted), std::end(sorted), [](const MagOption & a, const MagOption & b) { return a.mag < b.mag; });
    for (const auto & option : sorted) {
        if (option.present && option.cubes <= budget) {
            return option.mag;
        }
    }
    return coarsestPresent;// nothing fits; the coarsest is the least bad, and it is reported
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

void Inventory::probe() {
    QString why;
    if (!layerUsable(why)) {
        setState(State::Unsupported, why);
        return;
    }
    const auto & layer = Dataset::datasets[Segmentation::singleton().layerId];
    setState(State::Probing);
    QMetaObject::invokeMethod(worker.get(), "probeMags", Qt::QueuedConnection,
                              Q_ARG(Dataset, layer), Q_ARG(int, layer.lowestAvailableMag),
                              Q_ARG(int, layer.highestAvailableMag));
}

void Inventory::startScan(const int requested) {
    QString why;
    if (!layerUsable(why)) {
        setState(State::Unsupported, why);
        emit warning(why);
        return;
    }
    if (!probedThisDataset) {
        // probing is a prerequisite, and it is fast; the request waits for it to report back
        pendingScanMag = requested;
        probe();
        return;
    }
    pendingScanMag = -1;
    const auto chosen = requested != 0 ? requested : autoMag();
    if (chosen == 0) {
        const auto message = tr("No segmentation blocks were found at any magnification.");
        setState(State::Failed, message);
        emit warning(message);
        return;
    }
    const auto found = std::find_if(std::begin(mags), std::end(mags),
                                    [chosen](const MagOption & o) { return o.mag == chosen; });
    if (found == std::end(mags) || !found->present) {
        const auto message = tr("Magnification %1 has no segmentation blocks.").arg(chosen);
        setState(State::Failed, message);
        emit warning(message);
        return;
    }
    if (found->cubes > cubeBudget()) {
        emit warning(tr("Scanning at %1× means %2 blocks, past the %3 the budget allows — this will take a while.")
                     .arg(chosen).arg(found->cubes).arg(cubeBudget()));
    }

    pruneCache();
    mag = chosen;
    recs.clear();
    index.clear();
    warnedTruncated = false;
    done = total = 0;

    QSettings settings;
    ScanSpec spec;
    spec.layer = atMag(Dataset::datasets[Segmentation::singleton().layerId], found->magIndex);
    spec.magIndex = found->magIndex;
    spec.magnification = chosen;
    spec.backgroundId = Segmentation::singleton().getBackgroundId();
    spec.idCap = settings.value("objectInventoryIdCap", 500000).toULongLong();
    spec.maxInFlight = settings.value("objectInventoryMaxInFlight", 4).toInt();
    spec.cachePath = cachePathFor(chosen);

    setState(State::Scanning);
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
    mag = 0;
    done = total = 0;
    timestamp = {};
    setState(State::Idle);
    emit magOptionsChanged();
    emit recordsAppended(0, 0);
    QString why;
    if (layerUsable(why)) {
        loadCache();// local, cheap, and tells the user a list already exists
    } else {
        setState(State::Unsupported, why);
    }
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

bool Inventory::mayBeStale() const {
    return !recs.empty() && Annotation::singleton().unsavedChanges;
}

QString Inventory::statusLine() const {
    if (currentState == State::Unsupported) {
        return tr("Object inventory unavailable: %1").arg(currentDetail);
    }
    if (recs.empty()) {
        return currentState == State::Scanning
                ? tr("Scanning at %1× — %2 of %3 blocks, nothing found yet").arg(mag).arg(done).arg(total)
                : tr("No object inventory yet — press Scan to build one");
    }
    QString line = tr("%n object(s)", "", static_cast<int>(recs.size()));
    line += tr(" · %1×").arg(mag);
    if (currentState == State::Scanning) {
        line += tr(" · scanning, %1 of %2 blocks").arg(done).arg(total);
    } else if (timestamp.isValid()) {
        line += tr(" · scanned %1").arg(timestamp.toString(Qt::ISODate));
    }
    if (currentState == State::Stalled || currentState == State::Paused) {
        line += tr(" · %1").arg(stateName(currentState));
    }
    /* Said out loud rather than left to be discovered: a coarse sweep cannot see small
     * objects, and no sweep can see edits that have not been written back. */
    line += tr(" · excludes unsaved edits");
    if (mag > 1) {
        line += tr(" and objects under about %1 voxels").arg(mag * mag * mag);
    }
    if (mayBeStale()) {
        line += tr(" · may be out of date");
    }
    return line;
}

}
