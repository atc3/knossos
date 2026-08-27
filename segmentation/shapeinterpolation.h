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
#include "segmentation/segmentationsplit.h"
#include "segmentation/sislice.h"

#include <QObject>
#include <QString>

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

class QWidget;

/* Shape interpolation: paint an object on a few slices, let KNOSSOS fill in the rest.
 *
 * Each painted slice is kept here as its own 2D mask, independent of the cube cache. That
 * is deliberate: KNOSSOS only keeps an M³ supercube around the current position resident
 * (loader.cpp), so a slice the user painted 500 sections ago is long evicted by the time
 * they paint the next one. Interpolation therefore never reads a cube — it is pure
 * computation on two 2D masks — and memory stays O(slice area) rather than O(span × area).
 *
 * The masks are indexed in *current-mag voxel* units: index = (global - origin) / step,
 * where step is Dataset::scaleFactor captured when the session started. The mag is pinned
 * for the lifetime of a session so this mapping cannot shift underneath us.
 */

class ShapeInterpolation : public QObject {
    Q_OBJECT
public:
    enum class Axis { X = 0, Y = 1, Z = 2 };

    static ShapeInterpolation & singleton() {
        static ShapeInterpolation instance;
        return instance;
    }

    bool active() const { return started; }
    std::size_t sliceCount() const { return slices.size(); }
    const std::map<int, SISlice> & sliceMap() const { return slices; }
    int normalAxis() const { return axis; }
    // ViewportType the session is pinned to; VIEWPORT_XY/XZ/ZY map onto brush_t::view_t
    int normalAxisViewport() const { return static_cast<int>(view); }
    int uAxis() const { return uAxisIdx; }
    int vAxis() const { return vAxisIdx; }
    Coordinate voxelStep() const { return step; }
    std::uint64_t subobjectId() const { return soid; }

    // Absorb a brush stamp that has just been applied to the overlay. Reads the affected
    // voxels back out of the cubes rather than re-rasterising the brush, so the mask can
    // never drift from what was actually painted (erase included). Returns false, with a
    // reason, if the stamp does not belong to the running session.
    bool absorbStamp(const Coordinate & centerPos, const brush_t &, const std::uint64_t soid, QString & reason);

    // Start a chain without painting first, so an existing slice can simply be clicked.
    void beginAt(brush_t::view_t view, std::uint64_t soid);

    // Same idea for a flood fill, which covers a region rather than a brush footprint.
    // Only the plane at `depth` is absorbed: a key slice is by definition planar, so a 3D
    // fill contributes just its intersection with the current slice.
    bool absorbRegion(const Coordinate & first, const Coordinate & last, int depth, std::uint64_t soid, QString & reason);

    // Turn the interpolated preview at `depth` into a real painted key slice: write its
    // voxels into the overlay and add it to the chain. Called the moment the user starts
    // painting on a previewed slice, so correcting a few pixels doesn't mean redrawing the
    // whole outline. Returns false if there was no preview there to bake.
    bool materializeAt(int depth, QString & note);

    /* Adopt whatever is already painted in the plane through `seed` as a key slice.
     *
     * This is also what a fresh slice does when you simply paint on it: capturing only the
     * brush footprint would make a stamp dropped onto an existing outline *become* the key
     * slice, which is the "square in the middle of the blob" bug. Returns false when the
     * plane holds nothing of the chain's object.
     *
     * `relabelFrom`, when set, first rewrites that object's voxels in this plane to the
     * chain's id — a voxel-level steal of someone else's outline, leaving their object
     * untouched everywhere else. */
    bool adoptPlaneAt(const Coordinate & seed, QString & note, std::optional<std::uint64_t> relabelFrom = std::nullopt, QWidget * parent = nullptr);

    // depth navigation over the painted slices
    int depthOf(const Coordinate & pos) const;
    std::optional<int> firstDepth() const;
    std::optional<int> lastDepth() const;
    std::optional<int> prevDepth(int depth) const;
    std::optional<int> nextDepth(int depth) const;
    bool hasSliceAt(int depth) const { return slices.find(depth) != std::end(slices); }

    /* Does the chain cover this position — as a key slice or as interpolation?
     *
     * Interpolated voxels do not exist in the overlay until the chain is accepted, so a
     * click there reads as background. This is what lets one be recognised anyway. */
    bool covers(const Coordinate & pos);

    // Interpolated mask at `depth`, or nullptr when `depth` is not strictly between two
    // painted slices, when preview is off, or when the region exceeds the budget (see
    // lastError()). The result is owned by the controller and invalidated by the next call.
    const SISlice * previewAt(int depth);

    /* An axis-aligned rectangle of mask, ready to be drawn as one textured quad.
     *
     * For the viewport the chain lives in this is just the key slice or its interpolation.
     * For the other two ortho viewports it is a cross-section cut through the whole
     * interpolated volume, which is what makes the chain visible from the side while you
     * are building it. */
    struct PlanarMask {
        int fixedAxis{2}, fixedCoord{0};// plane normal, and where along it the plane sits
        int uAxis{0}, vAxis{1};         // dataset axes mapped onto mask u and v (uAxis < vAxis)
        int uMin{0}, vMin{0}, uStep{1}, vStep{1}, uSize{0}, vSize{0};
        const std::vector<std::uint8_t> * mask{nullptr};
        // whole plane is a painted key slice. In a cross-section the two are mixed, and
        // there a mask value of 2 marks key-slice voxels and 1 interpolated ones.
        bool keySlice{false};
    };
    // `viewportType` is a ViewportType; VIEWPORT_XY/XZ/ZY only.
    bool planarMaskFor(int viewportType, const Coordinate & pos, PlanarMask & out);
    bool previewEnabled() const { return preview; }
    void setPreviewEnabled(bool enabled);
    // bumped whenever the slice set changes, so callers can cache derived data
    std::uint64_t generation() const { return gen; }
    QString lastError() const { return error; }
    // one-line summary for the status bar: plane, key slices, span
    QString summary() const;
    // human name of the plane the chain is pinned to ("xy" / "xz" / "zy")
    QString planeName() const;

    // When two key slices don't overlap, a plain distance-transform blend collapses to
    // nothing — and it visibly pinches the shape well before that. Aligning the two
    // signed distance fields on their centroids before blending removes both artefacts,
    // which matters because a structure drifting laterally between slices is the normal
    // case in EM. Off means bit-for-bit Paintera behaviour.
    bool centroidAlignment() const { return alignCentroids; }
    void setCentroidAlignment(bool enabled);
    bool removeSliceAt(int depth);

    struct WriteResult {
        bool ok{false};
        bool cancelled{false};
        std::size_t depthsWritten{0};
        std::size_t cubesWritten{0};
        std::size_t cubesMissing{0};// cubes we could not make resident, i.e. dropped voxels
        QString message;
    };
    // Writes the whole chain — painted slices and every interpolated slice between them.
    WriteResult commit(QWidget * parent);
    // Destructive rollback: erases the painted key slices back to background.
    WriteResult eraseSlices(QWidget * parent);

    /* The whole chain, for undo to carry alongside the voxels.
     *
     * Key slices are in-memory masks, not overlay data, so rolling back the voxels without
     * them leaves the preview drawing an object that is no longer there. */
    struct State {
        bool started{false};
        brush_t::view_t view{brush_t::view_t::xy};
        int axis{2}, uAxisIdx{0}, vAxisIdx{1};
        Coordinate step{1, 1, 1};
        std::size_t magIndex{0}, layerId{0};
        std::uint64_t soid{0};
        std::map<int, SISlice> slices;
    };
    State saveState() const;
    void restoreState(const State &);

    void reset();

signals:
    void changed();

private:
    ShapeInterpolation() = default;
    void begin(const brush_t &, std::uint64_t soid);

    bool started{false};
    brush_t::view_t view{brush_t::view_t::xy};
    int axis{2};                    // index of the slice normal: 0=x, 1=y, 2=z
    int uAxisIdx{0}, vAxisIdx{1};   // the two in-plane axes, in increasing index order
    Coordinate step{1, 1, 1};       // Dataset::scaleFactor pinned at session start
    std::size_t magIndex{0};
    std::size_t layerId{0};
    std::uint64_t soid{0};
    std::map<int, SISlice> slices;

    // Cached signed distance transforms for one adjacent slice pair, on a common padded
    // grid. Recomputing these is the expensive part; stepping through depths is not.
    struct Interpolant {
        int z1{0}, z2{0};
        int uMin{0}, vMin{0}, uStep{1}, vStep{1}, uSize{0}, vSize{0};
        std::vector<float> d1, d2;
        float c1u{0}, c1v{0}, c2u{0}, c2v{0};// mask-index centroids of the two slices
    };
    /* Cached per adjacent slice pair, keyed by the lower depth. Computing a signed distance
     * transform is the expensive part of all this; sampling one is not. A cross-section
     * walks every pair at once, so keeping only the most recent one would recompute the
     * whole chain on every crosshair move. */
    std::map<int, Interpolant> interpolants;
    std::uint64_t interpolantsGen{0};
    const Interpolant * interpolantFor(int z1, int z2);
    // true if the interpolated shape covers grid index (uIdx, vIdx) at `depth`
    bool sampleInterpolant(const Interpolant &, int depth, int uIdx, int vIdx) const;
    // fills `out` (uSize*vSize) with the interpolated mask at `depth`
    void blendInto(const Interpolant &, int depth, std::vector<std::uint8_t> & out) const;

    // cross-section through the whole chain, cached per (plane, position, generation)
    SISlice crossSection;
    int crossSectionAxis{-1}, crossSectionCoord{0};
    std::uint64_t crossSectionGen{0};
    bool crossSectionValid{false};
    bool buildCrossSection(int fixedAxis, int fixedCoord);
    // mask at `depth`, painted or interpolated, ignoring the preview toggle
    const SISlice * maskAtDepth(int depth);
    /* Fills `slice` from the overlay across its plane.
     *
     * With `mayLoad`, walks outward block by block wherever the object runs off the edge of
     * one, pulling those blocks in — so adopting a slice picks up the whole outline, not
     * just the part on screen. The loader takes a centre of its own, so this never moves
     * the crosshair. Without it, only what is already in memory is read, which is what a
     * brush stroke wants: painting must not start loading things. */
    bool seedSliceFromPlane(SISlice & slice, const Coordinate & seed, bool & truncated, bool mayLoad, QWidget * parent = nullptr);
    WriteResult writeAll(bool wholeChain, std::uint64_t value, const QString & title, QWidget * parent);

    bool alignCentroids{true};

    SISlice previewSlice;
    std::uint64_t previewGen{0};
    bool previewValid{false};
    bool preview{true};
    std::uint64_t gen{1};
    QString error;
};

// Component access by axis index, since Coordinate has no operator[]. Templated so that
// floatCoordinate (the voxel scales) keeps its precision instead of being rounded to int.
template<typename C>
inline auto axisGet(const C & c, const int i) { return i == 0 ? c.x : i == 1 ? c.y : c.z; }
template<typename C, typename T>
inline void axisSet(C & c, const int i, const T v) { (i == 0 ? c.x : i == 1 ? c.y : c.z) = v; }
