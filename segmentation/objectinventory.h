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

/* A background sweep of the segmentation layer, so existing objects can be found.
 *
 * The problem: refining an annotation means locating it first, and KNOSSOS has no index.
 * The object list holds what was clicked or came out of a mergelist; the loader keeps a few
 * hundred blocks resident and tells Segmentation nothing about them. On a volume of 13948 ×
 * 7211 × 12186 that is 596 448 blocks at magnification 1 — reading them to find out what is
 * in there is not an option.
 *
 * What makes it tractable is that the magnification pyramid of a segmentation layer is
 * downsampled by mode, not by averaging, so the coarse levels carry *real* object ids.
 * Reading one level up shrinks the work eightfold and still names the same objects. The
 * cost is recall: measured on a real dataset, 2× found 2203 objects, 4× found 1352 and 8×
 * found 609, and what drops out is small — a median of 5 to 7 voxels. So the level is a
 * choice, made by cube-count budget and overridable, not a constant.
 *
 * Three rules this file exists to keep:
 *
 *   1. It never touches the loader. `startLoading` bumps the loading number, aborting the
 *      user's in-flight blocks, and evicts everything outside the new supercube. A sweep
 *      driven that way would fight the person using the program for hours. So the sweep
 *      fetches its own blocks, on its own thread, through its own network manager, and
 *      stands aside whenever the loader has work (see `setLoaderBusy`).
 *
 *   2. It never touches Segmentation or Dataset::datasets. The magnification fields are
 *      global and `getRawCube` reads them, so a sweep that switched them would silently
 *      redirect every paint stroke. The worker gets a Dataset *by value* with the fields
 *      overridden. And `subobjectFromId` creates on lookup, so a sweep that "checked" its
 *      ids against Segmentation would materialise a hundred thousand objects.
 *
 *   3. A missing block and an empty one are different things. The loader deliberately
 *      conflates them — a 404 becomes a zero-filled cube, which is right for drawing and
 *      disastrous here, because it would turn "this magnification does not exist" or "the
 *      network dropped" into a confident, empty, wrong answer that then gets cached. */

#include "dataset.h"
#include "segmentation/inventoryaccumulator.h"

#include <QDateTime>
#include <QElapsedTimer>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace objinv {

enum class State {
    Idle,       // nothing running; there may or may not be a list from a previous sweep
    Probing,    // finding out which magnifications actually exist
    Scanning,
    Paused,
    Stalled,    /* the network gave up. Distinct from Idle because the cursor has NOT moved
                 * past the blocks that failed — resuming continues rather than pretending
                 * those blocks were empty. */
    Complete,
    Unsupported,// this layer cannot be swept; see `detail`
    Failed,
};
QString stateName(State);

/* What is known to have been done to an object.
 *
 * Kept by subobject id rather than by position in the list, so it survives a rescan that
 * discovers things in a different order, and stored with the annotation — it is a record of
 * work, not a property of the dataset.
 *
 * Painted and Erased are both "you have dealt with this": a false positive removed with the
 * bucket on background never gets selected, so going only by what Segmentation knows about
 * would leave it looking untouched for ever. */
enum Flag : quint8 {
    Visited  = 1 << 0,// stepped to with the navigation keys
    Painted  = 1 << 1,// voxels written with this id
    Erased   = 1 << 2,// voxels of this id overwritten by something else, background included
    Skeleton = 1 << 3,// a skeleton node sits on it
};

// What one magnification would cost, and whether anything was found in the blocks sampled.
struct MagOption {
    int mag{1};
    std::size_t magIndex{0};
    quint64 cubes{0};
    quint64 estBytes{0};
    /* A sample block was found at this level.
     *
     * Only ever evidence *for*. A segmentation is usually sparse — the exporter writes a
     * block only where something is labelled — so a sample that comes back missing says
     * nothing about whether the level exists, only that nothing was labelled there. It is
     * therefore not allowed to veto a scan; it orders the choice of default. */
    bool sampled{false};
};

/* Everything the worker needs, handed over by value. There is deliberately no pointer back
 * to anything on the GUI thread. */
struct ScanSpec {
    Dataset layer;            // a private copy, magnification fields already overridden
    std::size_t magIndex{0};
    int magnification{1};
    std::uint64_t backgroundId{0};// a user setting, so it is read once and carried
    std::uint64_t startCursor{0}; // Z-order index to resume from
    std::size_t idCap{500000};
    int maxInFlight{4};
    QString cachePath;
};

class Scanner : public QObject {
    Q_OBJECT
public:
    Scanner();
    ~Scanner() override;
    void moveToThread(QThread * target);// reimplemented to carry qnam across, as the loader does

public slots:
    void probeMags(Dataset layer, int lowestMag, int highestMag, Coordinate nearby);
    void startScan(objinv::ScanSpec spec);
    void setPaused(bool);
    void cancel();
    /* The brake. Any movement of the crosshair makes the loader emit progress, so this one
     * existing signal covers browsing, magnification changes and annotation loads without
     * new plumbing. While it is true nothing new is requested; what is already in flight
     * finishes, so there is no torn block to clean up. */
    void setLoaderBusy(bool);

signals:
    void magsProbed(QVector<objinv::MagOption> options);
    void progress(quint64 cubesDone, quint64 cubesTotal, quint64 objects);
    /* quint64, not std::size_t.
     *
     * moc records a parameter by the spelling in the declaration, and Qt has no metatype
     * under the name "std::size_t" — so a queued connection carrying one is refused at emit
     * time and the call is dropped. The scan then ran, reported its progress, and delivered
     * none of what it found. Use names Qt knows for anything crossing the thread. */
    void appended(quint64 firstIndex, std::vector<objinv::Record> records);
    void revised(std::vector<std::uint32_t> indices, std::vector<objinv::Record> records);
    void finished(objinv::State outcome, QString detail);
    void warning(QString);

private:
    void step();
    void issue();
    /* Why a block produced nothing. Kept apart on purpose: "there is no file here" is
     * ordinary for a sparse volume, while "the file is unreadable" is a fault, and the
     * wrong-magnification guard reads the difference. */
    enum class Fetch { Decoded, Absent, Corrupt };
    Fetch readLocal(const CoordOfCube &, std::vector<std::uint64_t> & out);
    void requestRemote(const CoordOfCube &, std::uint64_t code);
    void completeCube(std::uint64_t code, bool decoded);
    void abortInFlight();
    Fetch decode(const CoordOfCube & cube, const QByteArray & payload, std::vector<std::uint64_t> & out) const;
    void ingest(const CoordOfCube &, const std::vector<std::uint64_t> &);
    void flushDeltas(bool force);
    void maybeCheckpoint(bool force);
    void writeCache(bool complete);
    void stop(State outcome, const QString & detail);
    CoordOfCube cubeAt(std::uint64_t code) const;
    bool inGrid(std::uint64_t code) const;
    /* The walk runs over a grid padded to a power of two; the dataset is not one, so most
     * indices are padding. This is what tells the cursor which are real. */
    auto covers() const { return [this](const std::uint64_t code) { return inGrid(code); }; }

    QNetworkAccessManager qnam;
    Accumulator acc;
    ScanSpec spec;
    /* One buffer for all of them. Several replies can be outstanding, but each is decoded
     * and folded in inside its own finished handler, and handlers on one thread run to
     * completion in turn, so they never overlap. 16 MiB once instead of per request. */
    std::vector<std::uint64_t> decodeBuffer;

    SweepCursor sweep;            // how far the walk has got, in a form a resume can trust
    std::uint32_t grid[3]{1, 1, 1};
    quint64 cubesTotal{0}, cubesDone{0}, cubesAbsent{0}, cubesEmpty{0}, cubesCorrupt{0};
    std::unordered_map<std::uint64_t, QNetworkReply *> inFlight;
    std::unordered_map<std::uint64_t, int> attempts;
    int consecutiveGiveUps{0};
    bool warnedAllAbsent{false};
    int scheduledRetries{0};      // blocks waiting out a backoff, so the walk is not yet done
    bool running{false}, paused{false}, loaderBusy{false}, stepQueued{false};

    std::size_t emitted{0};
    std::unordered_set<std::uint32_t> dirty;// already-emitted records that changed
    qint64 lastFlushMs{0}, lastCheckpointMs{0}, lastCheckpointCost{0};
    quint64 cubesAtCheckpoint{0};
};

class Inventory : public QObject {
    Q_OBJECT
    QThread workerThread;
    std::unique_ptr<Scanner> worker;
    Inventory();
public:
    /* Leaked on purpose, for the same reason Loader::Controller is: a static destructor
     * must not race a running thread. Shut down explicitly from MainWindow::closeEvent. */
    static Inventory & singleton() {
        static Inventory & inv = *new Inventory;
        return inv;
    }
    void suspend();

    State state() const { return currentState; }
    QString detail() const { return currentDetail; }
    int scanMag() const { return mag; }
    const std::vector<Record> & records() const { return recs; }
    std::optional<std::size_t> indexOfId(std::uint64_t) const;
    const QVector<MagOption> & magOptions() const { return mags; }
    QDateTime scannedAt() const { return timestamp; }
    bool complete() const { return currentState == State::Complete; }
    quint64 cubesDone() const { return done; }
    quint64 cubesTotal() const { return total; }
    int percentDone() const { return total == 0 ? 0 : static_cast<int>(100 * done / total); }
    /* Seconds still to go, or nothing when there is not enough to go on yet.
     *
     * From the rate achieved so far over the blocks still to do. It is a rough figure by
     * nature — the sweep stands aside whenever the loader is busy, so it speeds up and
     * slows down with whatever else is happening — but "about four minutes" is the thing
     * worth knowing, and a progress bar alone does not say it. */
    std::optional<int> etaSeconds() const;
    QString progressLine() const;
    // Whether the annotation has been edited since the sweep, i.e. whether the list may lag.
    bool mayBeStale() const;
    quint8 flagsFor(std::uint64_t soid) const;
    void setFlag(std::uint64_t soid, Flag, bool on);
    std::size_t countWith(Flag) const;
    /* Re-reads which objects carry a skeleton node.
     *
     * A tree already counts the subobject ids its nodes sit on, so this is a walk over the
     * trees rather than over the volume. It only knows about nodes whose subobject was
     * recorded when they were placed, which is what happens in the segmentation-aware
     * tracing modes — a node dropped with no segmentation loaded leaves no trace to find. */
    void refreshSkeletonFlags();
    QByteArray flagsJson() const;
    void importStateJson(const QByteArray &);
    QString statusLine() const;
    /* Finds out which magnifications exist, if that has not been done for this dataset yet.
     * Called when the Inventory tab is first shown rather than on every dataset load: it is
     * network work, and a dataset is opened far more often than the list is wanted. */
    void ensureProbed();
    /* Whether a dataset scan is possible at all: a volume can have no stored segmentation,
     * with every label living in the annotation instead. */
    bool datasetHasSegmentation() const;

    /* The finest magnification whose block count fits the budget, among those that were
     * found to exist. Falls back to the coarsest present one, and says so. */
    int autoMag() const;
    std::size_t cubeBudget() const;

public slots:
    void startScan(int mag = 0);// 0 = auto
    /* Tallies the annotation's own cubes rather than the dataset's.
     *
     * The dataset sweep reads what the server stores, which on a volume whose segmentation
     * was never baked out is nothing at all — there the annotation *is* the segmentation,
     * and it is also the only part that is ever out of date in the other direction. Runs
     * on this thread against the loader's cache of modified cubes: bounded by what has
     * been painted, no network, and no magnification to choose, since those cubes are held
     * at whatever magnification they were painted at. */
    void scanAnnotation(QWidget * parent = nullptr);
    /* Sets every voxel of one object to background.
     *
     * For removing a false positive outright rather than painting over it. Walks the
     * object's own bounding box, which the scan already recorded, making each block
     * resident first — the same discipline the interpolation commit uses, because a write
     * to a block that is not loaded is dropped without a word. Undoable as one step. */
    bool eraseObject(quint64 soid, QWidget * parent = nullptr);
    void pause();
    void resume();
    void cancel();
    void rescan();
    void onDatasetChanged();
    void onLoaderProgress(int count);

public slots:
    void noteSubobjectPainted(quint64 soid);
    void noteSubobjectOverwritten(quint64 soid);

signals:
    void stateChanged(objinv::State);
    void flagsChanged();
    void progressChanged(quint64 done, quint64 total, quint64 objects);
    /* quint64 rather than std::size_t here too. These are delivered directly, both ends
     * being on the GUI thread, so the name would not matter today — but it would the moment
     * anything moved, and silently. Names Qt knows, on every signal. */
    void recordsAppended(quint64 first, quint64 count);
    void recordsRevised(quint64 first, quint64 last);
    void magOptionsChanged();
    void warning(QString);

private:
    void setState(State, const QString & detail = {});
    bool layerUsable(QString & why) const;
    void loadCache();
    QString cachePathFor(int mag) const;
    void probe();
    /* The magnification list, without asking the network anything.
     *
     * Block counts come from the dataset's own declared extent and voxel sizes, so the
     * choice of level needs no round trip. Sampling only adds the note about whether
     * anything was found at a few blocks, which is why a scan no longer waits for it. */
    void buildMagOptions();

    std::vector<Record> recs;
    std::unordered_map<std::uint64_t, std::size_t> index;
    std::unordered_map<std::uint64_t, quint8> flags;
    QVector<MagOption> mags;
    State currentState{State::Idle};
    QString currentDetail;
    int mag{0};
    quint64 done{0}, total{0};
    QDateTime timestamp;
    bool probedThisDataset{false};
    bool fromAnnotation{false};// the list came from the annotation, not from the dataset
    bool warnedTruncated{false};
    /* A scan asked for before the magnifications were known. Probing has to happen first —
     * it is what decides which levels exist — so the request waits here for it. -1 is no
     * request pending, 0 means "whatever the budget picks". */
    int pendingScanMag{-1};
    qint64 loaderIdleSince{0};
    QElapsedTimer scanClock;
    quint64 doneAtStart{0};// a resumed scan must not count the blocks it inherited
};

}

Q_DECLARE_METATYPE(objinv::State)
Q_DECLARE_METATYPE(objinv::ScanSpec)
Q_DECLARE_METATYPE(objinv::MagOption)
Q_DECLARE_METATYPE(QVector<objinv::MagOption>)
Q_DECLARE_METATYPE(objinv::Record)
Q_DECLARE_METATYPE(std::vector<objinv::Record>)
