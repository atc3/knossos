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

#include "floodfill.h"

#include "annotation/annotation.h"
#include "dataset.h"
#include "loader.h"
#include "segmentation/labelonlyloading.h"
#include "segmentation/holefill.h"
#include "segmentation/shapeinterpolation.h"
#include "segmentation/segmentation.h"
#include "segmentation/undostack.h"
#include "stateInfo.h"
#include "viewer.h"

#include <QApplication>
#include <QElapsedTimer>
#include <QObject>
#include <QProgressDialog>

namespace {
// Cap on how many times a loading fill will move the view and resume. Each round makes one
// more block resident, so this bounds both the wall time and how far the view wanders.
constexpr int MAX_LOAD_ROUNDS = 64;
constexpr int LOAD_TIMEOUT_MS = 20000;

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
}

FloodFillReport runFloodFill(const FloodFillRequest & request, QWidget * const parent) {
    FloodFillReport report;
    auto areaMin = Annotation::singleton().movementAreaMin;
    /* movementAreaMax is exclusive — outsideMag1MovementArea() and updateMovementArea()
     * both treat it that way — but floodFillFrom() tests its bound with `>`, so passing it
     * through unchanged let every fill run one voxel past the movement area, and past the
     * dataset boundary in the default case where the two coincide. That plane sits inside
     * the last cube, so it reads as whatever the cube holds beyond the data: background,
     * everywhere, wrapping the volume. Hence fills escaping along the edge. */
    auto areaMax = Annotation::singleton().movementAreaMax - 1;

    if (Annotation::singleton().outsideMovementArea(request.seed)) {
        report.message = QObject::tr("Fill: the seed is outside the movement area.");
        return report;
    }

    /* And hold the fill off the outermost voxel shell of the dataset proper — see
     * Segmentation::floodFillAvoidsDatasetEdge for why that shell leaks too.
     *
     * Inset by one voxel *of the current magnification*, which is the granularity the walk
     * steps at; at mag 1 that is the single outermost plane. A seed placed in the shell on
     * purpose is exempt, so a deliberate click there still does what it looks like it will
     * instead of silently filling nothing. */
    if (Segmentation::singleton().floodFillAvoidsDatasetEdge) {
        const auto & scaleFactor = Dataset::current().scaleFactor;
        const Coordinate inset{std::max(1, static_cast<int>(scaleFactor.x)),
                               std::max(1, static_cast<int>(scaleFactor.y)),
                               std::max(1, static_cast<int>(scaleFactor.z))};
        // boundary is the extent, so the last valid voxel is boundary - 1 and the shell to
        // keep out of is that plane and the one at 0
        const auto edgeMin = inset;
        const auto edgeMax = Dataset::current().boundary - Coordinate{1, 1, 1} - inset;
        const auto seedInShell = request.seed.x < edgeMin.x || request.seed.y < edgeMin.y || request.seed.z < edgeMin.z
                              || request.seed.x > edgeMax.x || request.seed.y > edgeMax.y || request.seed.z > edgeMax.z;
        if (!seedInShell) {
            areaMin = {std::max(areaMin.x, edgeMin.x), std::max(areaMin.y, edgeMin.y), std::max(areaMin.z, edgeMin.z)};
            areaMax = {std::min(areaMax.x, edgeMax.x), std::min(areaMax.y, edgeMax.y), std::min(areaMax.z, edgeMax.z)};
        }
    }

    // writing the background id is an erase, not a fill, and the History window should say so
    const bool erasing = request.fillsoid == Segmentation::singleton().getBackgroundId();

    // the whole fill, including any load-and-continue rounds, is one undo step — which is
    // the entire point: a fill that escapes through a gap is exactly what you need to undo
    const UndoScope undoScope(request.threeDimensional ? (erasing ? QObject::tr("3D erase") : QObject::tr("3D fill"))
                                                       : (erasing ? QObject::tr("2D erase") : QObject::tr("2D fill")));

    FloodFillOptions options;
    options.threeDimensional = request.threeDimensional;
    options.view = request.view;

    const auto targetSoid = readVoxel(request.seed);
    auto result = floodFill(request.seed, request.fillsoid, options, areaMin, areaMax);
    if (result.seedNotLoaded) {
        report.message = QObject::tr("Fill: the block under the cursor is not loaded yet.");
        return report;
    }
    if (result.seedAlreadyFilled) {
        report.ok = true;
        report.message = erasing ? QObject::tr("Erase: that voxel is already background.")
                                 : QObject::tr("Fill: that voxel already belongs to this object.");
        return report;
    }

    const auto accumulate = [&report](const FloodFillResult & r){
        if (r.voxelsFilled != 0) {
            if (!report.didSomething) {
                report.filledMin = r.filledMin;
                report.filledMax = r.filledMax;
            } else {
                report.filledMin = {std::min(report.filledMin.x, r.filledMin.x), std::min(report.filledMin.y, r.filledMin.y), std::min(report.filledMin.z, r.filledMin.z)};
                report.filledMax = {std::max(report.filledMax.x, r.filledMax.x), std::max(report.filledMax.y, r.filledMax.y), std::max(report.filledMax.z, r.filledMax.z)};
            }
            report.didSomething = true;
        }
        report.voxelsFilled += r.voxelsFilled;
        report.cubesWritten += r.cubes.size();
        report.hitCap = report.hitCap || r.hitCap;
    };
    accumulate(result);

    // Optionally follow the fill into blocks that were not resident. Each round marks the
    // cubes it wrote before the view moves — eviction only preserves a cube's edits if it
    // is already queued as modified (loader.cpp), so the order matters.
    const bool mayLoad = request.mayLoadCubes && !request.threeDimensional;
    if (mayLoad && !result.deferred.empty()) {
        const auto startPosition = state->viewerState->currentPosition;
        const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
        const auto cubeExtent = dataset.scaleFactor.componentMul(dataset.cubeShape);

        const LabelOnlyLoading labelOnly;// only the overlay matters while chasing the fill
        QProgressDialog progress(QObject::tr("Filling across blocks…"), QObject::tr("Stop"), 0, MAX_LOAD_ROUNDS, parent);
        progress.setWindowModality(Qt::WindowModal);
        progress.setMinimumDuration(400);

        while (!result.deferred.empty() && !result.pendingCubes.empty() && report.loadRounds < MAX_LOAD_ROUNDS) {
            progress.setValue(static_cast<int>(report.loadRounds));
            if (progress.wasCanceled()) {
                break;
            }
            ++report.loadRounds;
            const auto target = dataset.cube2global(*std::begin(result.pendingCubes)) + cubeExtent / 2;
            state->viewer->setPosition(target, USERMOVE_NEUTRAL);
            if (!awaitLoader(progress)) {
                break;
            }
            const auto resume = result.deferred;
            result = floodFillFrom(resume, targetSoid, request.fillsoid, options, areaMin, areaMax);
            accumulate(result);
            if (result.voxelsFilled == 0 && result.deferred.size() >= resume.size()) {
                break;// no progress this round, don’t spin
            }
        }
        progress.setValue(MAX_LOAD_ROUNDS);
        state->viewer->setPosition(startPosition, USERMOVE_NEUTRAL);
    }
    report.boundaryStops = result.deferred.size();

    report.ok = true;
    // a fill that writes the background id is an erase, and saying "filled" for it reads as
    // if the wrong thing just happened to a few thousand voxels
    const auto dim = request.threeDimensional ? QObject::tr("3D") : QObject::tr("2D");
    const auto what = (erasing ? QObject::tr("%1 erase") : QObject::tr("%1 fill")).arg(dim);
    if (!report.didSomething) {
        report.message = erasing ? QObject::tr("%1: nothing to erase here.").arg(what)
                                 : QObject::tr("%1: nothing to fill here.").arg(what);
    } else if (report.hitCap) {
        report.message = erasing
            ? QObject::tr("%1: stopped at the %2 voxel safety limit after removing %3 voxels — the label reaches further than expected.")
                  .arg(what).arg(FloodFillOptions{}.maxVoxels).arg(report.voxelsFilled)
            : QObject::tr("%1: stopped at the %2 voxel safety limit after filling %3 voxels — the region is probably leaking into the background.")
                  .arg(what).arg(FloodFillOptions{}.maxVoxels).arg(report.voxelsFilled);
    } else if (report.boundaryStops != 0) {
        report.message = request.threeDimensional
            ? QObject::tr("%1: %2 voxels in %3 block(s). Stopped at the edge of the loaded blocks — a 3D fill never loads more.").arg(what).arg(report.voxelsFilled).arg(report.cubesWritten)
            : QObject::tr("%1: %2 voxels in %3 block(s). Stopped at the edge of the loaded blocks — enable “Fill may load more blocks” to continue past it.").arg(what).arg(report.voxelsFilled).arg(report.cubesWritten);
    } else if (report.loadRounds != 0) {
        report.message = QObject::tr("%1: %2 voxels in %3 block(s), loading %4 extra block(s) on the way.").arg(what).arg(report.voxelsFilled).arg(report.cubesWritten).arg(report.loadRounds);
    } else {
        report.message = QObject::tr("%1: %2 voxels in %3 block(s).").arg(what).arg(report.voxelsFilled).arg(report.cubesWritten);
    }
    return report;
}

namespace {
/* Growing the examined region.
 *
 * A stroke's own bounding box is rarely enough: the outline it closed may be mostly older
 * paint lying outside it. So the region is grown on whichever sides the object still
 * crosses, until it stops crossing any of them — at which point the border is known to be
 * outside the shape and "cannot reach the border" means "enclosed".
 *
 * Grown in blocks rather than a voxel at a time, because each round costs a full region
 * read. The caps are what stops a stroke on one end of a vessel running the length of it:
 * past them the boundary is simply treated as an opening, so an object too big to bound
 * gets nothing filled rather than something wrong filled. */
/* Each round costs a full region read, so the margin doubles rather than stepping.
 *
 * A fixed step meant a shape needing 1000 voxels of headroom took eleven reads of an
 * ever-larger region before it settled — the lag between letting go and the inside filling
 * in. Doubling gets to the same place in four, and the common case (a small outline clear
 * of its own bounding box) still settles on the first. */
constexpr int HOLE_GROW_START = 48;   // mag1 voxels of margin on the first attempt
constexpr int HOLE_MAX_ROUNDS = 7;    // 48 → 3072 voxels of margin
constexpr std::size_t HOLE_MAX_PIXELS = 4096 * 4096;
// A 3D brush spans depths, and each is a separate plane to examine. Bounded so that a
// large 3D brush cannot turn one stroke into hundreds of region reads.
constexpr int HOLE_MAX_PLANES = 64;
}

HoleFillReport fillEnclosedHoles(const HoleFillRequest & request) {
    HoleFillReport report;
    const auto axis = (request.view == brush_t::view_t::xy) ? 2 : (request.view == brush_t::view_t::xz) ? 1 : 0;
    if (request.view == brush_t::view_t::arb) {
        return report;// no axis-aligned plane to enclose anything in
    }
    const auto uAxis = (axis == 0) ? 1 : 0;
    const auto vAxis = (axis == 2) ? 1 : 2;
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    const Coordinate step{std::max(1, static_cast<int>(dataset.scaleFactor.x)),
                          std::max(1, static_cast<int>(dataset.scaleFactor.y)),
                          std::max(1, static_cast<int>(dataset.scaleFactor.z))};
    const auto uStep = axisGet(step, uAxis);
    const auto vStep = axisGet(step, vAxis);
    const auto depthStep = std::max(1, axisGet(step, axis));

    const auto areaMin = Annotation::singleton().movementAreaMin;
    const auto areaMax = Annotation::singleton().movementAreaMax - 1;

    const auto firstDepth = axisGet(request.strokeMin, axis);
    const auto lastDepth = axisGet(request.strokeMax, axis);
    const auto planes = (lastDepth - firstDepth) / depthStep + 1;
    if (planes > HOLE_MAX_PLANES) {
        return report;// a 3D brush this deep is not what this gesture is for
    }

    std::vector<std::uint8_t> solid, filled;
    std::size_t totalFilled{0};
    for (auto depth = firstDepth; depth <= lastDepth; depth += depthStep) {
        // the plane's resident extent, which is as far as anything can be read or written
        Coordinate centre = request.strokeMin;
        axisSet(centre, uAxis, (axisGet(request.strokeMin, uAxis) + axisGet(request.strokeMax, uAxis)) / 2);
        axisSet(centre, vAxis, (axisGet(request.strokeMin, vAxis) + axisGet(request.strokeMax, vAxis)) / 2);
        axisSet(centre, axis, depth);
        const auto resident = residentBoxAround(centre);

        const auto lowLimit = [&](const int axisIdx){ return std::max(axisGet(areaMin, axisIdx), axisGet(resident.first, axisIdx)); };
        const auto highLimit = [&](const int axisIdx){ return std::min(axisGet(areaMax, axisIdx), axisGet(resident.second, axisIdx)); };

        // start one growth block out from the stroke, so a shape drawn tight to its own
        // bounding box still has an outside ring to escape through
        auto margin = HOLE_GROW_START;
        auto uLow = axisGet(request.strokeMin, uAxis) - margin;
        auto uHigh = axisGet(request.strokeMax, uAxis) + margin;
        auto vLow = axisGet(request.strokeMin, vAxis) - margin;
        auto vHigh = axisGet(request.strokeMax, vAxis) + margin;

        /* The bounds the last successful read actually used.
         *
         * Kept apart from the ones the growth loop is still moving, so the write can never
         * be handed a region that does not match the grid `enclosed` was computed on —
         * which is the one way this could put voxels somewhere nobody drew. */
        int w{0}, h{0};
        int readULow{0}, readUHigh{0}, readVLow{0}, readVHigh{0};
        bool bounded{false};
        for (int round = 0; round <= HOLE_MAX_ROUNDS; ++round) {
            const auto clampedULow = std::max(uLow, lowLimit(uAxis));
            const auto clampedUHigh = std::min(uHigh, highLimit(uAxis));
            const auto clampedVLow = std::max(vLow, lowLimit(vAxis));
            const auto clampedVHigh = std::min(vHigh, highLimit(vAxis));
            bounded = clampedULow != uLow || clampedUHigh != uHigh || clampedVLow != vLow || clampedVHigh != vHigh;
            uLow = clampedULow; uHigh = clampedUHigh; vLow = clampedVLow; vHigh = clampedVHigh;
            if (uHigh < uLow || vHigh < vLow) {
                break;
            }

            w = (uHigh - uLow) / uStep + 1;
            h = (vHigh - vLow) / vStep + 1;
            if (static_cast<std::size_t>(w) * h > HOLE_MAX_PIXELS) {
                bounded = true;
                break;
            }

            Coordinate first, last;
            axisSet(first, axis, depth);      axisSet(last, axis, depth);
            axisSet(first, uAxis, uLow);      axisSet(last, uAxis, uHigh);
            axisSet(first, vAxis, vLow);      axisSet(last, vAxis, vHigh);

            readULow = uLow; readUHigh = uHigh; readVLow = vLow; readVHigh = vHigh;
            solid.assign(static_cast<std::size_t>(w) * h, 0);
            readRegion(first, last, [&](const std::uint64_t voxel, const Coordinate & pos){
                if (voxel != request.soid) {
                    return;
                }
                const auto u = (axisGet(pos, uAxis) - readULow) / uStep;
                const auto v = (axisGet(pos, vAxis) - readVLow) / vStep;
                if (u >= 0 && v >= 0 && u < w && v < h) {
                    solid[static_cast<std::size_t>(v) * w + u] = 1;
                }
            });

            const auto contact = holefill::touchesBorder(solid, w, h);
            if (!contact.any()) {
                bounded = false;
                break;// the border is outside the shape; the region is big enough
            }
            if (round == HOLE_MAX_ROUNDS) {
                bounded = true;
                break;
            }
            // grow only where it is needed, and only where there is room left to grow
            bool grew{false};
            const auto widen = [&](int & edge, const int delta, const int limit, const bool low){
                const auto wanted = edge + delta;
                const auto allowed = low ? std::max(wanted, limit) : std::min(wanted, limit);
                if (allowed != edge) {
                    edge = allowed;
                    grew = true;
                }
            };
            margin *= 2;
            if (contact.left)   { widen(uLow, -margin, lowLimit(uAxis), true); }
            if (contact.right)  { widen(uHigh, margin, highLimit(uAxis), false); }
            if (contact.top)    { widen(vLow, -margin, lowLimit(vAxis), true); }
            if (contact.bottom) { widen(vHigh, margin, highLimit(vAxis), false); }
            if (!grew) {
                bounded = true;
                break;// nowhere left to go; the boundary counts as an opening
            }
        }
        report.boundedEarly = report.boundedEarly || bounded;
        if (w <= 0 || h <= 0 || solid.empty()) {
            continue;
        }

        if (static_cast<std::size_t>(w) * h != solid.size() || holefill::enclosed(solid, w, h, filled) == 0) {
            continue;// this plane closed nothing, which is the usual answer
        }

        Coordinate first, last;
        axisSet(first, axis, depth);      axisSet(last, axis, depth);
        axisSet(first, uAxis, readULow);  axisSet(last, uAxis, readUHigh);
        axisSet(first, vAxis, readVLow);  axisSet(last, vAxis, readVHigh);
        std::size_t filledHere{0};
        Coordinate hereMin, hereMax;
        writeVoxelsWhere(first, last, [&](const Coordinate & pos){
            const auto u = (axisGet(pos, uAxis) - readULow) / uStep;
            const auto v = (axisGet(pos, vAxis) - readVLow) / vStep;
            if (u < 0 || v < 0 || u >= w || v >= h || filled[static_cast<std::size_t>(v) * w + u] == 0) {
                return false;
            }
            if (filledHere == 0) {
                hereMin = hereMax = pos;
            } else {
                hereMin = {std::min(hereMin.x, pos.x), std::min(hereMin.y, pos.y), std::min(hereMin.z, pos.z)};
                hereMax = {std::max(hereMax.x, pos.x), std::max(hereMax.y, pos.y), std::max(hereMax.z, pos.z)};
            }
            ++filledHere;
            return true;
        }, request.soid, true, false);// false: an enclosed voxel is inside the object, not something painted over

        if (filledHere == 0) {
            continue;
        }
        if (totalFilled == 0) {
            report.filledMin = hereMin;
            report.filledMax = hereMax;
        } else {
            report.filledMin = {std::min(report.filledMin.x, hereMin.x), std::min(report.filledMin.y, hereMin.y), std::min(report.filledMin.z, hereMin.z)};
            report.filledMax = {std::max(report.filledMax.x, hereMax.x), std::max(report.filledMax.y, hereMax.y), std::max(report.filledMax.z, hereMax.z)};
        }
        totalFilled += filledHere;
        ++report.planesFilled;

        // the key slice has to learn about the interior too, or accepting the chain would
        // write the outline back over it
        auto & si = ShapeInterpolation::singleton();
        if (si.active() && si.subobjectId() == request.soid && si.normalAxis() == axis) {
            QString reason;
            si.absorbRegion(centre, hereMin, hereMax, depth, request.soid, reason);
        }
    }

    report.voxelsFilled = totalFilled;
    if (totalFilled != 0) {
        report.message = report.boundedEarly
            ? QObject::tr("Filled %n enclosed voxel(s). Part of the outline ran past the loaded blocks, so anything enclosed out there was left alone.", "", static_cast<int>(totalFilled))
            : QObject::tr("Filled %n enclosed voxel(s).", "", static_cast<int>(totalFilled));
    }
    return report;
}
