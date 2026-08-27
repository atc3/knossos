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

#include "shapeinterpolation.h"

#include "dataset.h"
#include "segmentation/distancetransform.h"
#include "segmentation/cubeloader.h"
#include "segmentation/labelonlyloading.h"
#include "segmentation/segmentation.h"
#include "annotation/annotation.h"
#include "loader.h"
#include "stateInfo.h"
#include "viewer.h"

#include <QApplication>
#include <QElapsedTimer>
#include <QObject>
#include <QProgressDialog>

#include <algorithm>

namespace {
// margin around the union of two slice bounding boxes, so a shape shrinking towards
// nothing has room to do so instead of being clipped at the border
constexpr int PAD = 16;
// refuse to interpolate a region larger than this many mask pixels; the transient cost is
// two float distance transforms, i.e. 8 bytes per pixel
constexpr std::size_t MAX_PIXELS = 2048 * 2048;
// ceiling on the cached distance transforms across all slice pairs (floats, so ×4 bytes)
constexpr std::size_t MAX_CACHED_FLOATS = 16 * 1024 * 1024;
}

#include <cmath>
#include <limits>

void ShapeInterpolation::begin(const brush_t & brush, const std::uint64_t newSoid) {
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    view = brush.view;
    // VIEWPORT_XY/XZ/ZY cast directly onto brush_t::view_t (viewportortho.cpp), and the
    // slice normal is the axis the viewport does not span.
    axis = (view == brush_t::view_t::xy) ? 2 : (view == brush_t::view_t::xz) ? 1 : 0;
    uAxisIdx = (axis == 0) ? 1 : 0;
    vAxisIdx = (axis == 2) ? 1 : 2;
    step = Dataset::current().scaleFactor;
    magIndex = dataset.magIndex;
    layerId = Segmentation::singleton().layerId;
    soid = newSoid;
    slices.clear();
    started = true;
}

void ShapeInterpolation::beginAt(const brush_t::view_t newView, const std::uint64_t newSoid) {
    brush_t brush;
    brush.view = newView;
    begin(brush, newSoid);
}

int ShapeInterpolation::depthOf(const Coordinate & pos) const {
    return axisGet(pos, axis);
}

/* Reads the chain's object out of one whole plane, bounded by what is actually in memory.
 * One plane of the default 3×3×3 supercube is 384×384 voxels, so this is cheap. */
bool ShapeInterpolation::seedSliceFromPlane(SISlice & slice, const Coordinate & seed, bool & truncated) {
    const auto depth = depthOf(seed);
    auto box = residentBoxAround(seed);
    axisSet(box.first, axis, depth);
    axisSet(box.second, axis, depth);

    slice.depth = depth;
    slice.uStep = axisGet(step, uAxisIdx);
    slice.vStep = axisGet(step, vAxisIdx);
    // snap onto the mag voxel lattice, so every slice in the session shares one grid and
    // two slices can be compared index-for-index without a sub-voxel offset
    slice.uMin = siFloorDiv(axisGet(box.first, uAxisIdx), slice.uStep) * slice.uStep;
    slice.vMin = siFloorDiv(axisGet(box.first, vAxisIdx), slice.vStep) * slice.vStep;

    truncated = !regionCubeResidency(box.first, box.second).second.empty();
    readRegion(box.first, box.second, [this, &slice](const std::uint64_t voxel, const Coordinate & pos){
        if (voxel == soid) {
            slice.set(slice.uIndexOf(axisGet(pos, uAxisIdx)), slice.vIndexOf(axisGet(pos, vAxisIdx)), 1);
        }
    });
    slice.shrinkToFit();
    return !slice.empty();
}

bool ShapeInterpolation::adoptPlaneAt(const Coordinate & seed, QString & note, const std::optional<std::uint64_t> relabelFrom) {
    if (!started) {
        return false;
    }
    const auto depth = depthOf(seed);

    if (relabelFrom && *relabelFrom != soid && *relabelFrom != Segmentation::singleton().getBackgroundId()) {
        // voxel-level steal: rewrite that object's voxels in this plane only
        auto box = residentBoxAround(seed);
        axisSet(box.first, axis, depth);
        axisSet(box.second, axis, depth);
        const auto other = *relabelFrom;
        const auto touched = processRegionReplacing(box.first, box.second, other, soid);
        if (touched == 0) {
            note = QObject::tr("Nothing of that object is in this plane.");
            return false;
        }
    }

    SISlice slice;
    bool truncated{false};
    if (!seedSliceFromPlane(slice, seed, truncated)) {
        note = QObject::tr("Nothing painted with id %1 in this plane.").arg(soid);
        return false;
    }
    slices[depth] = std::move(slice);
    previewValid = false;
    ++gen;
    emit changed();
    note = truncated
        ? QObject::tr("Adopted slice %1, but it runs past the loaded blocks — scroll there and click again to pick up the rest.").arg(depth)
        : QObject::tr("Adopted slice %1 as a key slice.").arg(depth);
    return true;
}

bool ShapeInterpolation::absorbStamp(const Coordinate & centerPos, const brush_t & brush, const std::uint64_t stampSoid, QString & reason) {
    if (!started) {
        begin(brush, stampSoid);
    } else if (brush.view != view) {
        if (slices.size() <= 1) {
            begin(brush, stampSoid);// nothing worth keeping yet, just move the chain here
        } else {
            reason = QObject::tr("Shape interpolation is running in the %1 plane with %2 key slices. Press Esc to end it, then paint here to start a chain in this plane.")
                         .arg(planeName()).arg(slices.size());
            return false;
        }
    } else if (Dataset::datasets[Segmentation::singleton().layerId].magIndex != magIndex) {
        reason = QObject::tr("Shape interpolation is pinned to the magnification it was started at. Press Esc to start over.");
        return false;
    } else if (Segmentation::singleton().layerId != layerId) {
        reason = QObject::tr("Shape interpolation is pinned to the segmentation layer it was started at. Press Esc to start over.");
        return false;
    } else if (stampSoid != soid) {
        reason = QObject::tr("Shape interpolation is running on another object. Press Esc to start over.");
        return false;
    }

    const auto depth = depthOf(centerPos);
    auto & slice = slices[depth];
    if (slice.uSize == 0) {
        // A fresh slice takes whatever is already painted across the plane, not just what
        // this stamp covered. Otherwise dropping one stamp onto an existing outline makes
        // the stamp the key slice — a brush-sized blob in the middle of the real object.
        bool truncated{false};
        seedSliceFromPlane(slice, centerPos, truncated);
        slice.depth = depth;
    }

    // Read back what is actually in the overlay now that the stamp has been applied. This
    // keeps paint and erase in sync with the mask by construction, and costs one small
    // brush-sized region read per stamp.
    readBrushRegion(centerPos, brush, [&](const std::uint64_t voxel, const Coordinate & pos){
        if (axisGet(pos, axis) != depth) {
            return; // a 3D brush would span depths; only the stamp’s own plane is a key slice
        }
        slice.set(slice.uIndexOf(axisGet(pos, uAxisIdx)), slice.vIndexOf(axisGet(pos, vAxisIdx)), voxel == soid ? 1 : 0);
    });

    if (slice.empty()) {
        slices.erase(depth);
    }
    ++gen;
    emit changed();
    return true;
}

bool ShapeInterpolation::absorbRegion(const Coordinate & first, const Coordinate & last, const int depth, const std::uint64_t regionSoid, QString & reason) {
    if (!started) {
        reason = QObject::tr("Shape interpolation: paint a slice first, then fill.");
        return false;
    }
    if (regionSoid != soid) {
        reason = QObject::tr("Shape interpolation is running on another object. Press Esc to start over.");
        return false;
    }

    auto & slice = slices[depth];
    if (slice.uSize == 0) {
        slice.depth = depth;
        slice.uStep = axisGet(step, uAxisIdx);
        slice.vStep = axisGet(step, vAxisIdx);
        slice.uMin = siFloorDiv(axisGet(first, uAxisIdx), slice.uStep) * slice.uStep;
        slice.vMin = siFloorDiv(axisGet(first, vAxisIdx), slice.vStep) * slice.vStep;
    }

    Coordinate planeFirst = first, planeLast = last;
    axisSet(planeFirst, axis, depth);
    axisSet(planeLast, axis, depth);
    readRegion(planeFirst, planeLast, [&](const std::uint64_t voxel, const Coordinate & pos){
        slice.set(slice.uIndexOf(axisGet(pos, uAxisIdx)), slice.vIndexOf(axisGet(pos, vAxisIdx)), voxel == soid ? 1 : 0);
    });

    if (slice.empty()) {
        slices.erase(depth);
    }
    ++gen;
    emit changed();
    return true;
}

bool ShapeInterpolation::materializeAt(const int depth, QString & note) {
    if (!started || hasSliceAt(depth)) {
        return false;// already a real key slice
    }
    const auto * interpolated = maskAtDepth(depth);
    if (interpolated == nullptr || interpolated->count() == 0) {
        return false;
    }
    // copy first: inserting into `slices` may invalidate the cached preview it points at
    auto baked = *interpolated;
    baked.depth = depth;

    Coordinate first, last;
    axisSet(first, axis, depth);
    axisSet(last, axis, depth);
    axisSet(first, uAxisIdx, baked.uMin);
    axisSet(last, uAxisIdx, baked.uCoordOf(baked.uSize) - baked.uStep);
    axisSet(first, vAxisIdx, baked.vMin);
    axisSet(last, vAxisIdx, baked.vCoordOf(baked.vSize) - baked.vStep);
    first = first.capped(Annotation::singleton().movementAreaMin, Annotation::singleton().movementAreaMax + 1);
    last = last.capped(Annotation::singleton().movementAreaMin, Annotation::singleton().movementAreaMax + 1);

    const auto intended = regionCubeResidency(first, last);
    writeVoxelsWhere(first, last, [this, &baked](const Coordinate & pos){
        return baked.at(baked.uIndexOf(axisGet(pos, uAxisIdx)), baked.vIndexOf(axisGet(pos, vAxisIdx))) != 0;
    }, soid, true);

    slices[depth] = std::move(baked);
    previewValid = false;
    ++gen;
    emit changed();

    note = intended.second.empty()
        ? QObject::tr("Interpolated slice %1 is now a painted slice — edit it directly.").arg(depth)
        : QObject::tr("Interpolated slice %1 is now a painted slice, but %2 block(s) weren’t loaded and are missing from it.").arg(depth).arg(intended.second.size());
    return true;
}

std::optional<int> ShapeInterpolation::firstDepth() const {
    return slices.empty() ? std::nullopt : std::optional<int>{std::begin(slices)->first};
}

std::optional<int> ShapeInterpolation::lastDepth() const {
    return slices.empty() ? std::nullopt : std::optional<int>{std::rbegin(slices)->first};
}

std::optional<int> ShapeInterpolation::prevDepth(const int depth) const {
    const auto it = slices.lower_bound(depth);
    return it == std::begin(slices) ? std::nullopt : std::optional<int>{std::prev(it)->first};
}

std::optional<int> ShapeInterpolation::nextDepth(const int depth) const {
    const auto it = slices.upper_bound(depth);
    return it == std::end(slices) ? std::nullopt : std::optional<int>{it->first};
}

bool ShapeInterpolation::removeSliceAt(const int depth) {
    if (slices.erase(depth) == 0) {
        return false;
    }
    ++gen;
    emit changed();
    return true;
}

void ShapeInterpolation::reset() {
    if (!started && slices.empty()) {
        return;
    }
    slices.clear();
    started = false;
    soid = 0;
    interpolants.clear();
    crossSectionValid = false;
    previewValid = false;
    error.clear();
    ++gen;
    emit changed();
}

void ShapeInterpolation::setPreviewEnabled(const bool enabled) {
    if (preview != enabled) {
        preview = enabled;
        emit changed();
    }
}

const ShapeInterpolation::Interpolant * ShapeInterpolation::interpolantFor(const int z1, const int z2) {
    if (interpolantsGen != gen) {
        interpolants.clear();// any slice edit invalidates every distance transform
        interpolantsGen = gen;
    }
    const auto cached = interpolants.find(z1);
    if (cached != std::end(interpolants) && cached->second.z2 == z2) {
        return &cached->second;
    }

    const auto & s1 = slices.at(z1);
    const auto & s2 = slices.at(z2);

    // Common grid: the union of both bounding boxes plus a margin. Both slices are snapped
    // to the same lattice (see absorbStamp), so index arithmetic is exact.
    const auto uStep = s1.uStep;
    const auto vStep = s1.vStep;
    const auto uMin = std::min(s1.uMin, s2.uMin) - PAD * uStep;
    const auto vMin = std::min(s1.vMin, s2.vMin) - PAD * vStep;
    const auto uMax = std::max(s1.uCoordOf(s1.uSize), s2.uCoordOf(s2.uSize)) + PAD * uStep;
    const auto vMax = std::max(s1.vCoordOf(s1.vSize), s2.vCoordOf(s2.vSize)) + PAD * vStep;
    const auto uSize = (uMax - uMin) / uStep;
    const auto vSize = (vMax - vMin) / vStep;
    if (uSize <= 0 || vSize <= 0) {
        error = QObject::tr("Shape interpolation: empty region.");
        return nullptr;
    }
    const auto pixels = static_cast<std::size_t>(uSize) * vSize;
    if (pixels > MAX_PIXELS) {
        error = QObject::tr("Shape interpolation: the region spanned by slices %1 and %2 is %3×%4 voxels, over the %5×%5 limit. Zoom in, or split the object into shorter runs.")
                    .arg(z1).arg(z2).arg(uSize).arg(vSize).arg(2048);
        return nullptr;
    }

    Interpolant in;
    const auto onCommonGrid = [&](const SISlice & slice, float & cu, float & cv){
        std::vector<std::uint8_t> bin(pixels, 0);
        const auto uOff = (slice.uMin - uMin) / uStep;
        const auto vOff = (slice.vMin - vMin) / vStep;
        for (int v = 0; v < slice.vSize; ++v) {
            for (int u = 0; u < slice.uSize; ++u) {
                if (slice.at(u, v) != 0) {
                    bin[static_cast<std::size_t>(v + vOff) * uSize + (u + uOff)] = 1;
                }
            }
        }
        distance_transform::maskCentroid(bin, uSize, cu, cv);
        return bin;
    };

    // physical spacing of one mask pixel, so anisotropic datasets interpolate correctly
    const auto & scale = Dataset::current().scales[0];// nm per mag1 voxel, per axis
    const auto su = axisGet(scale, uAxisIdx) * uStep;
    const auto sv = axisGet(scale, vAxisIdx) * vStep;

    distance_transform::signedEdt(onCommonGrid(s1, in.c1u, in.c1v), in.d1, uSize, vSize, su, sv);
    distance_transform::signedEdt(onCommonGrid(s2, in.c2u, in.c2v), in.d2, uSize, vSize, su, sv);

    in.z1 = z1;
    in.z2 = z2;
    in.uMin = uMin;
    in.vMin = vMin;
    in.uStep = uStep;
    in.vStep = vStep;
    in.uSize = uSize;
    in.vSize = vSize;
    error.clear();

    // keep the cache from growing without bound on a long chain
    std::size_t cachedFloats = 2 * pixels;
    for (const auto & [key, entry] : interpolants) {
        (void)key;
        cachedFloats += entry.d1.size() + entry.d2.size();
    }
    if (cachedFloats > MAX_CACHED_FLOATS) {
        interpolants.clear();
    }
    return &(interpolants[z1] = std::move(in));
}

const SISlice * ShapeInterpolation::maskAtDepth(const int depth) {
    const auto painted = slices.find(depth);
    if (painted != std::end(slices)) {
        return &painted->second;
    }
    if (slices.size() < 2) {
        return nullptr;
    }
    const auto lo = prevDepth(depth);
    const auto hi = nextDepth(depth);
    if (!lo || !hi) {
        return nullptr;// outside the painted range: nothing to interpolate between
    }
    if (previewValid && previewSlice.depth == depth && previewGen == gen) {
        return &previewSlice;
    }
    const auto * in = interpolantFor(*lo, *hi);
    if (in == nullptr) {
        return nullptr;
    }

    std::vector<std::uint8_t> mask;
    blendInto(*in, depth, mask);

    previewSlice = SISlice{};
    previewSlice.depth = depth;
    previewSlice.uMin = in->uMin;
    previewSlice.vMin = in->vMin;
    previewSlice.uStep = in->uStep;
    previewSlice.vStep = in->vStep;
    previewSlice.uSize = in->uSize;
    previewSlice.vSize = in->vSize;
    previewSlice.adoptMask(std::move(mask));
    previewGen = gen;
    previewValid = true;
    return &previewSlice;
}

const SISlice * ShapeInterpolation::previewAt(const int depth) {
    if (!started || !preview || hasSliceAt(depth)) {
        return nullptr;// a painted slice is already shown as real overlay voxels
    }
    return maskAtDepth(depth);
}

void ShapeInterpolation::setCentroidAlignment(const bool enabled) {
    if (alignCentroids != enabled) {
        alignCentroids = enabled;
        previewValid = false;
        emit changed();
    }
}

void ShapeInterpolation::blendInto(const Interpolant & in, const int depth, std::vector<std::uint8_t> & out) const {
    const auto span = static_cast<float>(in.z2 - in.z1);
    const auto t = span != 0.f ? (depth - in.z1) / span : 0.f;
    const auto du = alignCentroids ? (in.c2u - in.c1u) : 0.f;
    const auto dv = alignCentroids ? (in.c2v - in.c1v) : 0.f;
    distance_transform::blendSigned(in.d1, in.d2, in.uSize, in.vSize, t, du, dv, out);
}

namespace {
// How long to wait for the loader to make a cube resident before giving up on it. A miss
// is reported as a shortfall rather than silently dropping voxels, which is what
// writeVoxel()/processRegion() would otherwise do.
constexpr int LOAD_TIMEOUT_MS = 30000;

bool awaitLoader(QProgressDialog & progress) {
    QElapsedTimer timer;
    timer.start();
    while (!Loader::Controller::singleton().isFinished()) {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 20);
        if (progress.wasCanceled() || timer.elapsed() > LOAD_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

Coordinate componentMax(const Coordinate & a, const Coordinate & b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
Coordinate componentMin(const Coordinate & a, const Coordinate & b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
}

ShapeInterpolation::WriteResult ShapeInterpolation::commit(QWidget * const parent) {
    if (slices.size() < 2) {
        WriteResult r;
        r.message = QObject::tr("Shape interpolation needs at least two painted slices before it can interpolate.");
        return r;
    }
    return writeAll(true, soid, QObject::tr("Writing interpolated shape…"), parent);
}

ShapeInterpolation::WriteResult ShapeInterpolation::eraseSlices(QWidget * const parent) {
    return writeAll(false, Segmentation::singleton().getBackgroundId(), QObject::tr("Erasing painted slices…"), parent);
}

/* Walks the region of interest cube by cube, making each cube resident before writing it.
 *
 * KNOSSOS keeps only an M³ supercube around the current position in memory and drops
 * writes to anything outside it without a word (cubeloader.cpp). There is no on-demand
 * cube loading anywhere in the codebase, so the only lever is to move the position and let
 * the loader follow. Doing that per cube keeps peak memory at one supercube no matter how
 * far apart the key slices are — which is the whole point of doing this in KNOSSOS rather
 * than in Paintera — and the residency check means the common case, where the user just
 * painted everything and it is all still resident, costs no movement at all.
 *
 * Each cube is marked modified immediately after it is written, before the position moves
 * again: eviction only preserves a cube's edits if it is already in modifiedCacheQueue
 * (loader.cpp), so deferring the marking to the end would throw away everything the walk
 * had scrolled past. */
ShapeInterpolation::WriteResult ShapeInterpolation::writeAll(const bool wholeChain, const std::uint64_t value, const QString & title, QWidget * const parent) {
    WriteResult result;
    if (!started || slices.empty()) {
        result.message = QObject::tr("Nothing to write.");
        return result;
    }

    std::vector<int> depths;
    if (wholeChain) {
        const auto dStep = std::max(1, axisGet(step, axis));
        for (auto d = std::begin(slices)->first; d <= std::rbegin(slices)->first; d += dStep) {
            depths.push_back(d);
        }
    } else {
        for (const auto & [depth, slice] : slices) {
            (void)slice;
            depths.push_back(depth);
        }
    }

    const auto startPosition = state->viewerState->currentPosition;
    const auto & areaMin = Annotation::singleton().movementAreaMin;
    const auto & areaMax = Annotation::singleton().movementAreaMax;

    QProgressDialog progress(title, QObject::tr("Cancel"), 0, static_cast<int>(depths.size()), parent);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(400);// don’t flash a dialog for the usual, instant case

    // only the overlay matters here; don’t drag the EM data along behind every move
    const LabelOnlyLoading labelOnly;

    CubeCoordSet written;
    /* Dirty-marking is batched rather than done per write.
     *
     * coordCubesMarkChanged() is a BlockingQueuedConnection round trip to the loader thread
     * plus a reslice notification to every viewport, so marking once per (depth, cube)
     * costs hundreds of round trips on a long chain and dominates the commit. Batching is
     * only safe as long as the set is flushed before the position moves: eviction preserves
     * a cube's edits solely when it is already queued as modified (loader.cpp). */
    CubeCoordSet unmarked;
    const auto flushMarks = [&unmarked, &written](){
        if (!unmarked.empty()) {
            coordCubesMarkChanged(unmarked);
            written.insert(std::begin(unmarked), std::end(unmarked));
            unmarked.clear();
        }
    };
    int done = 0;
    for (const auto depth : depths) {
        progress.setValue(done++);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
        if (progress.wasCanceled()) {
            result.cancelled = true;
            break;
        }

        const auto * slice = maskAtDepth(depth);
        if (slice == nullptr || slice->count() == 0) {
            continue;
        }
        // the interpolated grid is padded well beyond the shape; clip to the painted extent
        Coordinate first, last;
        axisSet(first, axis, depth);
        axisSet(last, axis, depth);
        axisSet(first, uAxisIdx, slice->uMin);
        axisSet(last, uAxisIdx, slice->uCoordOf(slice->uSize) - slice->uStep);
        axisSet(first, vAxisIdx, slice->vMin);
        axisSet(last, vAxisIdx, slice->vCoordOf(slice->vSize) - slice->vStep);
        first = first.capped(areaMin, areaMax + 1);
        last = last.capped(areaMin, areaMax + 1);

        const auto inside = [this, slice](const Coordinate & pos){
            return slice->at(slice->uIndexOf(axisGet(pos, uAxisIdx)), slice->vIndexOf(axisGet(pos, vAxisIdx))) != 0;
        };

        const auto & dataset = Dataset::current();
        const auto cubeExtent = dataset.scaleFactor.componentMul(dataset.cubeShape);
        const auto cubeBegin = dataset.global2cube(first);
        const auto cubeEnd = dataset.global2cube(last) + 1;
        for (int cz = cubeBegin.z; cz < cubeEnd.z; ++cz)
        for (int cy = cubeBegin.y; cy < cubeEnd.y; ++cy)
        for (int cx = cubeBegin.x; cx < cubeEnd.x; ++cx) {
            const CoordOfCube cube{cx, cy, cz};
            const auto cubeFirst = dataset.cube2global(cube);
            const auto cubeLast = cubeFirst + cubeExtent - 1;
            const auto regionFirst = componentMax(first, cubeFirst);
            const auto regionLast = componentMin(last, cubeLast);

            if (!regionCubeResidency(regionFirst, regionLast).second.empty()) {
                // not resident: pull the loader over to it, then wait. Everything written
                // so far must be marked first, or moving away discards it.
                flushMarks();
                state->viewer->setPosition(cubeFirst + cubeExtent / 2, USERMOVE_NEUTRAL);
                if (!awaitLoader(progress)) {
                    result.cancelled = progress.wasCanceled();
                }
            }
            const auto touched = writeVoxelsWhere(regionFirst, regionLast, inside, value, false);
            if (touched.empty()) {
                ++result.cubesMissing;
            } else {
                unmarked.insert(std::begin(touched), std::end(touched));
            }
            if (result.cancelled) {
                break;
            }
        }
        ++result.depthsWritten;
        if (result.cancelled) {
            break;
        }
    }
    flushMarks();
    progress.setValue(static_cast<int>(depths.size()));

    state->viewer->setPosition(startPosition, USERMOVE_NEUTRAL);
    result.cubesWritten = written.size();
    result.ok = !result.cancelled && result.cubesMissing == 0;
    if (result.cancelled) {
        result.message = QObject::tr("Cancelled after writing %n slice(s). What was written is kept.", "", static_cast<int>(result.depthsWritten));
    } else if (result.cubesMissing != 0) {
        // never silently: a dropped cube means a hole in the object
        result.message = QObject::tr("Wrote %n slice(s), but %1 block(s) could not be loaded in time and were skipped — the object may have holes there.", "", static_cast<int>(result.depthsWritten))
                             .arg(result.cubesMissing);
    } else {
        result.message = QObject::tr("Wrote %n slice(s) across %1 block(s).", "", static_cast<int>(result.depthsWritten)).arg(result.cubesWritten);
    }
    return result;
}

bool ShapeInterpolation::sampleInterpolant(const Interpolant & in, const int depth, const int uIdx, const int vIdx) const {
    if (uIdx < 0 || vIdx < 0 || uIdx >= in.uSize || vIdx >= in.vSize) {
        return false;
    }
    const auto span = static_cast<float>(in.z2 - in.z1);
    const auto t = span != 0.f ? (depth - in.z1) / span : 0.f;
    const auto du = alignCentroids ? (in.c2u - in.c1u) : 0.f;
    const auto dv = alignCentroids ? (in.c2v - in.c1v) : 0.f;
    constexpr float FAR = 1e20f;
    const auto sample = [&](const std::vector<float> & d, const int u, const int v){
        return (u < 0 || v < 0 || u >= in.uSize || v >= in.vSize) ? FAR : d[static_cast<std::size_t>(v) * in.uSize + u];
    };
    const auto a = sample(in.d1, uIdx + static_cast<int>(std::lround(-t * du)), vIdx + static_cast<int>(std::lround(-t * dv)));
    const auto b = sample(in.d2, uIdx + static_cast<int>(std::lround((1.f - t) * du)), vIdx + static_cast<int>(std::lround((1.f - t) * dv)));
    return ((1.f - t) * a + t * b) <= 0.f;
}

/* Cuts the whole chain — key slices and interpolations alike — with a plane that contains
 * the interpolation axis, which is what the two viewports the chain does not live in are
 * showing. Costs one evaluation per (depth, position) pair rather than a full mask per
 * depth, and is cached until the crosshair leaves the plane or a slice is edited. */
bool ShapeInterpolation::buildCrossSection(const int fixedAxis, const int fixedCoord) {
    if (crossSectionValid && crossSectionAxis == fixedAxis && crossSectionCoord == fixedCoord && crossSectionGen == gen) {
        return crossSection.count() != 0;
    }
    crossSection = SISlice{};
    crossSectionValid = true;
    crossSectionAxis = fixedAxis;
    crossSectionCoord = fixedCoord;
    crossSectionGen = gen;
    if (slices.size() < 2) {
        return false;
    }
    const auto wAxis = (fixedAxis == uAxisIdx) ? vAxisIdx : uAxisIdx;
    const bool fixedIsU = fixedAxis == uAxisIdx;

    // build every pair up front so the extent below covers the padded interpolation grids
    std::vector<const Interpolant *> pairs;
    for (auto it = std::begin(slices); std::next(it) != std::end(slices); ++it) {
        pairs.push_back(interpolantFor(it->first, std::next(it)->first));
    }

    int wMin = std::numeric_limits<int>::max(), wMax = std::numeric_limits<int>::min(), wStep = 1;
    const auto extend = [&](const int lo, const int size, const int st){
        wStep = st;
        wMin = std::min(wMin, lo);
        wMax = std::max(wMax, lo + size * st);
    };
    for (const auto * in : pairs) {
        if (in != nullptr) {
            extend(fixedIsU ? in->vMin : in->uMin, fixedIsU ? in->vSize : in->uSize, fixedIsU ? in->vStep : in->uStep);
        }
    }
    for (const auto & [depth, slice] : slices) {
        (void)depth;
        extend(fixedIsU ? slice.vMin : slice.uMin, fixedIsU ? slice.vSize : slice.uSize, fixedIsU ? slice.vStep : slice.uStep);
    }
    if (wMax <= wMin) {
        return false;
    }

    const auto aStep = std::max(1, axisGet(step, axis));
    const auto aMin = std::begin(slices)->first;
    const auto aSize = (std::rbegin(slices)->first - aMin) / aStep + 1;
    const auto wSize = (wMax - wMin) / wStep;
    if (static_cast<std::size_t>(aSize) * wSize > MAX_PIXELS) {
        error = QObject::tr("Shape interpolation: the cross-section of this chain is too large to preview.");
        return false;
    }

    // uAxis is whichever of the two spanned dataset axes has the lower index, so the quad
    // the renderer emits and this mask agree on orientation without any special casing
    const bool axisIsU = axis < wAxis;
    const auto uSize = axisIsU ? aSize : wSize;
    const auto vSize = axisIsU ? wSize : aSize;
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(uSize) * vSize, 0);

    auto pair = std::begin(slices);
    for (int ai = 0; ai < aSize; ++ai) {
        const auto depth = aMin + ai * aStep;
        const auto painted = slices.find(depth);
        while (std::next(pair) != std::end(slices) && std::next(pair)->first < depth) {
            ++pair;
        }
        const Interpolant * in = nullptr;
        if (painted == std::end(slices)) {
            const auto lo = prevDepth(depth), hi = nextDepth(depth);
            if (lo && hi) {
                in = interpolantFor(*lo, *hi);
            }
        }
        for (int wi = 0; wi < wSize; ++wi) {
            const auto wCoord = wMin + wi * wStep;
            bool set = false;
            if (painted != std::end(slices)) {
                const auto & sl = painted->second;
                set = fixedIsU ? sl.at(sl.uIndexOf(fixedCoord), sl.vIndexOf(wCoord))
                               : sl.at(sl.uIndexOf(wCoord), sl.vIndexOf(fixedCoord));
            } else if (in != nullptr) {
                const auto uIdx = fixedIsU ? (fixedCoord - in->uMin) / in->uStep : (wCoord - in->uMin) / in->uStep;
                const auto vIdx = fixedIsU ? (wCoord - in->vMin) / in->vStep : (fixedCoord - in->vMin) / in->vStep;
                set = sampleInterpolant(*in, depth, uIdx, vIdx);
            }
            if (set) {
                const auto u = axisIsU ? ai : wi;
                const auto v = axisIsU ? wi : ai;
                mask[static_cast<std::size_t>(v) * uSize + u] = (painted != std::end(slices)) ? 2 : 1;
            }
        }
    }

    crossSection.depth = fixedCoord;
    crossSection.uMin = axisIsU ? aMin : wMin;
    crossSection.vMin = axisIsU ? wMin : aMin;
    crossSection.uStep = axisIsU ? aStep : wStep;
    crossSection.vStep = axisIsU ? wStep : aStep;
    crossSection.uSize = uSize;
    crossSection.vSize = vSize;
    crossSection.adoptMask(std::move(mask));
    return crossSection.count() != 0;
}

bool ShapeInterpolation::planarMaskFor(const int viewportType, const Coordinate & pos, PlanarMask & out) {
    if (!started || !preview) {
        return false;
    }
    // VIEWPORT_XY/XZ/ZY are 0/1/2 and their plane normals are z/y/x, i.e. 2 - type
    const auto planeNormal = 2 - viewportType;
    if (planeNormal < 0 || planeNormal > 2) {
        return false;
    }

    if (planeNormal == axis) {// the viewport the chain lives in
        const auto depth = axisGet(pos, axis);
        const auto painted = slices.find(depth);
        const auto isKeySlice = painted != std::end(slices);
        // a key slice is drawn too, in its own colour, so which slices are in the chain is
        // obvious at a glance rather than something you have to remember
        const auto * slice = isKeySlice ? &painted->second : previewAt(depth);
        if (slice == nullptr || slice->count() == 0) {
            return false;
        }
        out = {axis, depth, uAxisIdx, vAxisIdx, slice->uMin, slice->vMin, slice->uStep, slice->vStep, slice->uSize, slice->vSize, &slice->mask, isKeySlice};
        return true;
    }

    const auto fixedCoord = axisGet(pos, planeNormal);
    if (!buildCrossSection(planeNormal, fixedCoord)) {
        return false;
    }
    const auto wAxis = (planeNormal == uAxisIdx) ? vAxisIdx : uAxisIdx;
    out = {planeNormal, fixedCoord, std::min(axis, wAxis), std::max(axis, wAxis),
           crossSection.uMin, crossSection.vMin, crossSection.uStep, crossSection.vStep,
           crossSection.uSize, crossSection.vSize, &crossSection.mask, false};
    return true;
}

QString ShapeInterpolation::planeName() const {
    return view == brush_t::view_t::xy ? QStringLiteral("xy") : view == brush_t::view_t::xz ? QStringLiteral("xz") : QStringLiteral("zy");
}

QString ShapeInterpolation::summary() const {
    if (!started || slices.empty()) {
        return QObject::tr("Shape interpolation: paint a slice to start a chain");
    }
    const auto axisName = axis == 0 ? QStringLiteral("x") : axis == 1 ? QStringLiteral("y") : QStringLiteral("z");
    if (slices.size() == 1) {
        return QObject::tr("Shape interpolation: %1 plane · 1 key slice at %2 %3 · paint another slice to interpolate")
                   .arg(planeName()).arg(axisName).arg(std::begin(slices)->first);
    }
    return QObject::tr("Shape interpolation: %1 plane · %2 key slices · %3 %4–%5%6")
               .arg(planeName())
               .arg(slices.size())
               .arg(axisName)
               .arg(std::begin(slices)->first)
               .arg(std::rbegin(slices)->first)
               .arg(preview ? QString{} : QObject::tr(" · preview off"));
}
