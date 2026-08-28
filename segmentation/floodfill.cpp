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
    const auto & areaMin = Annotation::singleton().movementAreaMin;
    const auto & areaMax = Annotation::singleton().movementAreaMax;

    if (Annotation::singleton().outsideMovementArea(request.seed)) {
        report.message = QObject::tr("Fill: the seed is outside the movement area.");
        return report;
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
