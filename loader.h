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

#pragma once

#include "coordinate.h"
#include "dataset.h"
#include "segmentation/segmentation.h"
#include "usermove.h"

#include <QCoreApplication>
#include <QFutureSynchronizer>
#include <QFutureWatcher>
#include <QMutex>
#include <QNetworkReply>
#include <QNetworkAccessManager>
#include <QObject>
#include <QSemaphore>
#include <QThread>
#include <QThreadPool>
#include <QtGlobal>
#include <QTimer>
#include <QWaitCondition>

#include <boost/multi_array.hpp>
#include <boost/optional/optional.hpp>

#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/* Calculate movement trajectory for loading based on how many last single movements */
#define LL_CURRENT_DIRECTIONS_SIZE (20)
/* Max number of metrics allowed for sorting loading order */
#define LL_METRIC_NUM (20)

#define LM_LOCAL    0
#define LM_FTP      1

namespace Loader{
class Controller;
class Worker;
}

namespace Loader {
using DecompressionResult = std::tuple<bool, void*, QIODevice*>;
/* The blocks a load is limited to, or null for the whole supercube.
 *
 * A move always makes the M³ supercube around its centre the resident set, and normally
 * fetches all of it. A caller that knows which blocks it needs — the shape-interpolation
 * commit walks a planned set — passes them here, and the rest of the supercube is left
 * unfetched instead of downloaded for nothing. Eviction is unchanged. */
using CubeFilter = std::shared_ptr<const std::unordered_set<CoordOfCube>>;
class Worker : public QObject {
    Q_OBJECT
    friend class Loader::Controller;
    friend boost::multi_array_ref<uint64_t, 3> getCube(const Coordinate & pos);
    friend void Segmentation::clear();
private:
    QThreadPool decompressionPool;//let pool be alive just after ~Worker
    QFutureSynchronizer<void> sync;
    QThreadPool localPool;
    QNetworkAccessManager qnam;

    template<typename T>
    using ptr = std::unique_ptr<T>;
    using DecompressionOperationPtr = ptr<QFutureWatcher<DecompressionResult>>;
    using OpenWatcher = QFutureWatcher<boost::optional<bool>>;
    std::vector<std::unordered_map<CoordOfCube, ptr<OpenWatcher>>> slotOpen;
    std::vector<ptr<OpenWatcher>> solitaryConfinement;// disconnected worker threads that wait for IO
    std::vector<std::unordered_map<CoordOfCube, QNetworkReply*>> slotDownload;
    std::vector<std::unordered_map<CoordOfCube, DecompressionOperationPtr>> slotDecompression;
    std::vector<std::list<std::vector<std::uint8_t>>> slotChunk;// slot ownership
    std::vector<std::list<void *>> freeSlots;
    int currentMaxMetric;

    std::atomic_bool isFinished{false};
    std::size_t loaderMagnification{0};
    std::size_t loaderCacheSize{0};
    void CalcLoadOrderMetric(float halfSc, floatCoordinate currentMetricPos, const UserMoveType userMoveType, const floatCoordinate & direction, float *metrics);
    floatCoordinate find_close_xyz(floatCoordinate direction);
    std::vector<CoordOfCube> DcoiFromPos(const Coordinate &currentOrigin, const UserMoveType userMoveType, const floatCoordinate & direction);
    uint loadCubes();
    void snappyCacheBackupRaw(const std::size_t layerId, const CoordOfCube &, const void * cube);
    void snappyCacheClear();

    decltype(slotDecompression)::value_type::iterator finalizeDecompression(QFutureWatcher<DecompressionResult> & watcher, decltype(freeSlots)::value_type & freeSlots, decltype(slotDecompression)::value_type & decompressions, const CoordOfCube & cubeCoord);
    void abortDownloadsFinishDecompression();
    template<typename Func>
    void abortDownloadsFinishDecompression(std::size_t, Func);

    decltype(Dataset::datasets) datasets;

    /* Server payloads, as downloaded, of recently loaded blocks — see payloadFind().
     *
     * A block that was never edited is simply dropped when it leaves the supercube, and the
     * next time it is needed it is downloaded again. Panning back and forth, adopting a wide
     * slice and then accepting over it, or any walk that has to move the supercube more than
     * once all fetched the same blocks repeatedly. The payload is the compressed bytes the
     * server sent, ~50–200 KB a block, so a modest budget covers thousands of blocks and
     * a hit costs a decompression and no network. Edited blocks never come from here:
     * snappyCache is consulted first and always wins. */
    struct PayloadKey {
        std::size_t layerId;
        std::size_t mag;
        CoordOfCube cube;
        bool operator==(const PayloadKey & other) const { return layerId == other.layerId && mag == other.mag && cube == other.cube; }
    };
    struct PayloadKeyHash {
        std::size_t operator()(const PayloadKey & key) const {
            return std::hash<CoordOfCube>{}(key.cube) ^ (key.layerId * 0x9E3779B97F4A7C15ull) ^ (key.mag << 48);
        }
    };
    using PayloadList = std::list<std::pair<PayloadKey, QByteArray>>;
    PayloadList payloadLru;// most recently used first
    std::unordered_map<PayloadKey, PayloadList::iterator, PayloadKeyHash> payloadIndex;
    std::size_t payloadBytes{0};
    std::uint64_t payloadFreshness{0};// Dataset::freshness the cached payloads were fetched under
    const QByteArray * payloadFind(std::size_t layerId, const CoordOfCube & cube);
    void payloadStore(std::size_t layerId, std::size_t mag, const CoordOfCube & cube, QByteArray bytes);
    void payloadClear(std::size_t layerId);

    /* Downloads made ahead of need — see prefetch(). Their bytes only ever go into the
     * payload cache; nothing is decompressed or made resident until a move asks for it. */
    std::unordered_map<PayloadKey, QNetworkReply *, PayloadKeyHash> prefetches;
    /* Where the last prefetch() asked to fetch for. Kept, not consumed: a move cancels what
     * is in flight (see downloadAndLoadCubes), and the rest is asked for again once that
     * move's own blocks are in. */
    std::vector<Coordinate> prefetchCentres;
    void startPrefetches();
    // cancels one, keeping its bytes if it had already arrived
    decltype(prefetches)::iterator dropPrefetch(decltype(prefetches)::iterator it);
    void abortPrefetches(std::size_t layerId);
    void abortPrefetches();
    // a prefetch already under way for a block a move now wants, handed over to that move
    QNetworkReply * takePrefetch(std::size_t layerId, const CoordOfCube & cube);
public://matsch
    /* Bytes of payload kept; set from the dataset dialog (DATASET_PAYLOAD_MIB). Read on the
     * loader thread, written from the GUI one. */
    static std::atomic<std::size_t> payloadBudget;
    // mean payload size of what is held, for estimating how many blocks the budget covers
    std::atomic<std::size_t> payloadMeanBytes{0};
    /* Starts downloading the blocks the supercubes around `centres` would need, into the
     * payload cache only, so that moving there later costs a decompression rather than a
     * round trip. Supersedes any earlier prefetch that is no longer wanted. */
    void prefetch(const std::vector<Coordinate> centres);
    /* Where blocks came from, for the shape-interpolation commit's log line: one count per
     * block made resident, by source. Only ever read as a difference across an operation. */
    std::atomic<std::uint64_t> fromNetwork{0}, fromPayloadCache{0}, fromEdits{0}, fromPrefetch{0};
    using CacheQueue = std::unordered_set<CoordOfCube>;
    std::vector<std::vector<CacheQueue>> modifiedCacheQueue;
    using SnappySet = std::unordered_map<CoordOfCube, std::string>;
    std::vector<std::vector<SnappySet>> snappyCache;
    QMutex snappyCacheMutex;
    QMutex snappyFlushConditionMutex;
    QWaitCondition snappyFlushCondition;

    void moveToThread(QThread * targetThread);//reimplement to move qnam

    void unloadCurrentMagnification(const std::size_t);
    void unloadCurrentMagnification();
    void markCubeAsModified(const std::size_t layerId, const CoordOfCube &cubeCoord, const int magnification);
    void snappyCacheSupplySnappy(const std::size_t layerId, const CoordOfCube, const quint64 cubeMagnification, const std::string cube);
    /* Overwriting counterpart to snappyCacheSupplySnappy, for undo.
     *
     * That one emplace()s, which is a no-op when the coord already has an entry — and an
     * edited cube always does, since being edited is what put it there. It is also one
     * blocking round trip per cube. This assigns, and takes the whole set at once. */
    void snappyCacheReplace(const std::size_t layerId, const quint64 cubeMagnification, const SnappySet cubes);
    void flushIntoSnappyCache();
    void broadcastProgress(bool startup = false);
    Worker();
    virtual ~Worker() override;
signals:
    void progress(bool incremented, int count);
public slots:
    void cleanup(const Coordinate center);
    void downloadAndLoadCubes(const unsigned int loadingNr, const Coordinate center, const UserMoveType userMoveType, const floatCoordinate & direction, Dataset::list_t changedDatasets, const size_t cacheSize, const Loader::CubeFilter only);
};

class Controller : public QObject {
    Q_OBJECT
    friend class Loader::Worker;
    QThread workerThread;
public:
    std::unique_ptr<Loader::Worker> worker;
    std::atomic_uint loadingNr{0};
    static Controller & singleton(){
        static Loader::Controller & loader = *new Loader::Controller;
        return loader;
    }
    void suspendLoader();
    Controller();
    virtual ~Controller() override;
    void unloadCurrentMagnification();

    void startLoading(const Coordinate & center, const UserMoveType userMoveType, const floatCoordinate &direction, CubeFilter only = {});
    // see Worker::prefetch; queued onto the loader thread
    void prefetch(const std::vector<Coordinate> & centres);
    // how many blocks the payload cache holds at the payloads' current mean size
    std::size_t payloadCapacityBlocks() const;
    template<typename... Args>
    void snappyCacheSupplySnappy(Args&&... args) {
        emit snappyCacheSupplySnappySignal(std::forward<Args>(args)...);
    }
    template<typename... Args>
    void snappyCacheReplace(Args&&... args) {
        emit snappyCacheReplaceSignal(std::forward<Args>(args)...);
    }
    void markCubeAsModified(const std::size_t layerId, const CoordOfCube &cubeCoord, const int magnification);

    struct LockedSnappy {
        std::unique_lock<QMutex> locker;
        decltype(Loader::Worker::snappyCache[0]) & cubes;
        explicit LockedSnappy(QMutex & mutex, decltype(cubes) & cache) : locker(mutex), cubes(cache) {}
    };
    auto getAllModifiedCubes(const std::size_t layerId) {
        QMutexLocker lock(&worker->snappyFlushConditionMutex);
        //signal to run in loader thread
        QTimer::singleShot(0, worker.get(), &Worker::flushIntoSnappyCache);
        worker->snappyFlushCondition.wait(&worker->snappyFlushConditionMutex);
        return LockedSnappy{worker->snappyCacheMutex, worker->snappyCache[layerId]};
    }
    /* What the loader is holding, so "where did the memory go" is answerable rather than
     * guessable. The snappy cache is the interesting number: it is the unsaved annotation,
     * it grows with every distinct cube ever modified, and nothing shrinks it short of
     * clearing the annotation — saving writes the cubes out but keeps them, because this is
     * also what an evicted cube is re-hydrated from. */
    struct MemoryReport {
        std::size_t slotsTotal{0}, slotsFree{0}, slotBytes{0};
        std::size_t snappyCubes{0}, snappyBytes{0};
    };
    MemoryReport memoryReport();

public slots:
    bool isFinished();
    bool hasSnappyCache();
signals:
    void progress(int count);
    void refCountChange(bool isIncrement, int refCount);
    void unloadCurrentMagnificationSignal();
    void loadSignal(const unsigned int loadingNr, const Coordinate center, const UserMoveType userMoveType, const floatCoordinate & direction, const Dataset::list_t & changedDatasets, const quint64 cacheSize, const Loader::CubeFilter & only);
    void markCubeAsModifiedSignal(const std::size_t layerId, const CoordOfCube &cubeCoord, const int magnification);
    void snappyCacheSupplySnappySignal(const std::size_t layerId, const CoordOfCube, const quint64 cubeMagnification, const std::string cube);
    void snappyCacheReplaceSignal(const std::size_t layerId, const quint64 cubeMagnification, const Loader::Worker::SnappySet cubes);
};
}//namespace Loader

Q_DECLARE_METATYPE(Loader::Worker::SnappySet)
Q_DECLARE_METATYPE(Loader::CubeFilter)
