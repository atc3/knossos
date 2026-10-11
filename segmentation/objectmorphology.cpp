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

#include "objectmorphology.h"

#include "segmentation/morphregion.h"

#include "annotation/annotation.h"
#include "dataset.h"
#include "segmentation/cubeloader.h"
#include "segmentation/segmentation.h"
#include "segmentation/undostack.h"
#include "stateInfo.h"
#include "viewer.h"

#include <QApplication>
#include <QObject>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace {

/* How much of the volume one run may touch.
 *
 * A smooth holds the label indices, the result, the blurred mask, the distance field and
 * the argmin field over the region, at four bytes a voxel each — so the cap is really a
 * memory budget, about 160 MB at this value. It is not a residency limit: the supercube
 * around the crosshair is an 896-voxel box at magnification 1, far more than this. */
constexpr std::size_t MAX_REGION_VOXELS = 8u * 1000u * 1000u;

Coordinate perAxisStep() {
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    return {std::max(1, static_cast<int>(dataset.scaleFactor.x)),
            std::max(1, static_cast<int>(dataset.scaleFactor.y)),
            std::max(1, static_cast<int>(dataset.scaleFactor.z))};
}

/* Nanometres per voxel at the magnification being painted at.
 *
 * scales[magIndex] is the declared size where the dataset lists one per level; falling back
 * to the magnification-1 size times the scale factor covers a layer whose list is shorter
 * than its pyramid, which parseToml permits. Physical rather than voxel units all the way
 * through is what makes one radius mean the same thing in z as in x. */
floatCoordinate voxelSpacing() {
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    if (dataset.magIndex < dataset.scales.size()) {
        return dataset.scales[dataset.magIndex];
    }
    const auto & base = dataset.scales.at(0);
    return {base.x * dataset.scaleFactor.x, base.y * dataset.scaleFactor.y, base.z * dataset.scaleFactor.z};
}

Coordinate componentMin(const Coordinate & a, const Coordinate & b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
Coordinate componentMax(const Coordinate & a, const Coordinate & b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

QString operationName(const morphology::Operation op) {
    switch (op) {
    case morphology::Operation::Dilate: return QObject::tr("Dilate");
    case morphology::Operation::Erode: return QObject::tr("Erode");
    case morphology::Operation::Smooth: break;
    }
    return QObject::tr("Smooth");
}

MorphReport run(const MorphRequest & request) {
    MorphReport report;
    auto & seg = Segmentation::singleton();
    const auto what = operationName(request.op);

    if (!seg.enabled || !seg.hasSegData()) {
        report.message = QObject::tr("%1: there is no segmentation layer to work on.").arg(what);
        return report;
    }
    if (Annotation::singleton().magLock && Dataset::datasets[seg.layerId].magIndex != Annotation::singleton().magLock.value()) {
        report.message = QObject::tr("%1: painting is locked to another magnification.").arg(what);
        return report;
    }
    const auto ids = seg.selectedSubobjectIds(request.fragmentsOnly);
    if (ids.empty()) {
        report.message = QObject::tr("%1: select an object first.").arg(what);
        return report;
    }
    if (!(request.radius > 0)) {
        report.message = QObject::tr("%1: the radius is zero, so there is nothing to do.").arg(what);
        return report;
    }

    const auto step = perAxisStep();
    const auto spacing = voxelSpacing();
    const double spacingArr[3]{spacing.x, spacing.y, spacing.z};

    /* The apron: how far past the written region the arithmetic has to see.
     *
     * For a dilate or an erode that is the radius, because nothing beyond it can affect a
     * voxel. For a smooth it is the Gaussian's own reach, which is three sigma — the
     * distance cap still applies on top, but the blurred value at a written voxel depends on
     * mask out to the kernel's edge, and truncating that is what pulls a face down to 0.63
     * and shaves it. One extra voxel on top of either, so the apron is never exactly flush. */
    int pad[3]{0, 0, 0};
    for (int a = 0; a < 3; ++a) {
        const auto radiusVoxels = static_cast<int>(std::ceil(request.radius / std::max(1e-6, spacingArr[a])));
        const auto blurVoxels = request.op == morphology::Operation::Smooth
                ? morphology::gaussianRadius(request.radius / std::max(1e-6, spacingArr[a]))
                : 0;
        pad[a] = std::max(radiusVoxels, blurVoxels) + 1;
    }

    const auto areaMin = Annotation::singleton().movementAreaMin;
    const auto areaMax = Annotation::singleton().movementAreaMax - 1;// exclusive, as everywhere
    const auto resident = residentBoxAround(state->viewerState->currentPosition);
    const auto limitMin = componentMax(areaMin, resident.first);
    const auto limitMax = componentMin(areaMax, resident.second);

    /* The region, shrunk until it fits the budget.
     *
     * Shrunk and reported rather than refused: someone who asks for twice what the memory
     * allows wants the operation over as much as it can manage, and to be told, not an
     * error dialog. */
    auto extent = std::max(1, request.extent);
    while (true) {
        const auto side = static_cast<std::size_t>(extent);
        if (side * side * side <= MAX_REGION_VOXELS || extent <= 1) {
            break;
        }
        extent = extent * 3 / 4;
        report.extentReduced = true;
    }

    const auto centre = state->viewerState->currentPosition;
    const int centreArr[3]{centre.x, centre.y, centre.z};
    const int stepArr[3]{step.x, step.y, step.z};
    const int limitMinArr[3]{limitMin.x, limitMin.y, limitMin.z};
    const int limitMaxArr[3]{limitMax.x, limitMax.y, limitMax.z};
    const auto region = morphregion::plan(centreArr, extent, stepArr, pad, limitMinArr, limitMaxArr);
    if (region.empty) {
        report.message = QObject::tr("%1: there is nothing to work on at the crosshair.").arg(what);
        return report;
    }
    const auto & read = region.read;
    const auto & write = region.write;
    report.apronIncomplete = region.apronIncomplete;
    report.regionVoxels = write.voxels();
    /* Which of the two limits did the cutting, separately, because they mean different
     * things to the user: the movement area is a choice they made and the loaded blocks are
     * a transient they can fix by waiting or moving. */
    const Coordinate wantMin{centre.x - (extent / 2) * step.x, centre.y - (extent / 2) * step.y, centre.z - (extent / 2) * step.z};
    const Coordinate wantMax{centre.x + (extent / 2) * step.x, centre.y + (extent / 2) * step.y, centre.z + (extent / 2) * step.z};
    report.clippedByArea = componentMax(wantMin, areaMin) != wantMin || componentMin(wantMax, areaMax) != wantMax;
    report.clippedByResidency = componentMax(wantMin, resident.first) != wantMin || componentMin(wantMax, resident.second) != wantMax;

    const Coordinate readFirst{read.first[0], read.first[1], read.first[2]};
    const Coordinate readLast{read.last[0], read.last[1], read.last[2]};
    const Coordinate writeFirst{write.first[0], write.first[1], write.first[2]};
    const Coordinate writeLast{write.last[0], write.last[1], write.last[2]};

    /* A block that is not in memory reads as background, which is indistinguishable from
     * real background — the same trap floodFillFrom() exists to avoid. Here it would erode
     * the object away along the seam, so a missing block is a reason to stop and say so
     * rather than to guess. In practice it only happens just after a jump, while the loader
     * is still catching up. */
    const auto residency = regionCubeResidency(readFirst, readLast);
    if (!residency.second.empty()) {
        report.message = QObject::tr("%1: %n block(s) in the region are still loading — try again in a moment.", "", static_cast<int>(residency.second.size())).arg(what);
        return report;
    }

    // palette 0 is the background id, so a voxel never visited reads as background
    morphology::Block block;
    block.dim[0] = read.dim[0]; block.dim[1] = read.dim[1]; block.dim[2] = read.dim[2];
    block.labels.assign(read.voxels(), 0);
    std::vector<std::uint64_t> paletteIds{seg.getBackgroundId()};
    block.selected.assign(1, 0);
    std::unordered_map<std::uint64_t, std::uint32_t> paletteOf{{seg.getBackgroundId(), 0}};
    const std::unordered_set<std::uint64_t> selectedIds{std::begin(ids), std::end(ids)};

    readRegion(readFirst, readLast, [&](const std::uint64_t voxel, const Coordinate & pos){
        const int at[3]{pos.x, pos.y, pos.z};
        if (!read.inRange(at)) {
            return;// the traversal steps from each block's origin, so it can step past us
        }
        auto it = paletteOf.find(voxel);
        if (it == std::end(paletteOf)) {
            const auto index = static_cast<std::uint32_t>(paletteIds.size());
            paletteIds.push_back(voxel);
            block.selected.push_back(selectedIds.count(voxel) != 0 ? 1 : 0);
            it = paletteOf.emplace(voxel, index).first;
        }
        block.labels[read.index(at)] = it->second;
    });

    std::size_t selectedVoxels = 0;
    for (std::size_t i = 0; i < block.voxels(); ++i) {
        selectedVoxels += block.selectedAt(i) ? 1 : 0;
    }
    if (selectedVoxels == 0) {
        report.ok = true;
        report.message = QObject::tr("%1: none of the selected object is in this region — move the crosshair onto it.").arg(what);
        return report;
    }

    morphology::Params params;
    params.op = request.op;
    params.radius = request.radius;
    params.spacing[0] = spacingArr[0]; params.spacing[1] = spacingArr[1]; params.spacing[2] = spacingArr[2];
    params.direction = request.direction;
    params.infill = request.infill;
    params.threshold = request.threshold;
    params.background = 0;// palette index of the background id, by construction
    if (request.infill == morphology::Infill::Replace && selectedIds.count(request.replacement) != 0) {
        /* Replacing the object's voxels with one of its own ids is not an erode, it is a
         * relabel — and it would be reported as voxels removed while leaving every one of
         * them in the object. Treated as the background, which is what "give these up"
         * means. */
        params.infill = morphology::Infill::Background;
    } else if (request.infill == morphology::Infill::Replace) {
        auto it = paletteOf.find(request.replacement);
        if (it == std::end(paletteOf)) {
            const auto index = static_cast<std::uint32_t>(paletteIds.size());
            paletteIds.push_back(request.replacement);
            block.selected.push_back(selectedIds.count(request.replacement) != 0 ? 1 : 0);
            it = paletteOf.emplace(request.replacement, index).first;
        }
        params.replacement = it->second;
    }

    const auto result = morphology::apply(block, params);

    const UndoScope undoScope(QObject::tr("%1 object").arg(what));
    const auto cubes = writeVoxelsFrom(writeFirst, writeLast, [&](const Coordinate & pos) -> std::optional<std::uint64_t> {
        const int at[3]{pos.x, pos.y, pos.z};
        if (!read.inRange(at)) {
            return std::nullopt;
        }
        const auto i = read.index(at);
        if (result.labels[i] == block.labels[i]) {
            return std::nullopt;// unchanged, which is almost every voxel
        }
        return paletteIds[result.labels[i]];
    });

    /* Counted from the written region rather than taken from the result, because the result
     * covers the apron too and the apron is deliberately not written. */
    for (std::size_t i = 0; i < block.voxels(); ++i) {
        if (result.labels[i] == block.labels[i]) {
            continue;
        }
        const auto x = static_cast<int>(i % block.dim[0]);
        const auto y = static_cast<int>((i / block.dim[0]) % block.dim[1]);
        const auto z = static_cast<int>(i / (static_cast<std::size_t>(block.dim[0]) * block.dim[1]));
        const int at[3]{read.first[0] + x * step.x, read.first[1] + y * step.y, read.first[2] + z * step.z};
        if (!write.contains(at)) {
            continue;
        }
        if (block.selectedAt(i)) {
            ++report.removed;
        } else {
            ++report.added;
        }
    }
    report.cubesWritten = cubes.size();
    report.ok = true;

    if (report.added == 0 && report.removed == 0) {
        report.message = QObject::tr("%1: nothing changed at this radius.").arg(what);
    } else if (request.op == morphology::Operation::Dilate) {
        report.message = QObject::tr("%1: added %n voxel(s)", "", static_cast<int>(report.added)).arg(what);
    } else if (request.op == morphology::Operation::Erode) {
        report.message = QObject::tr("%1: removed %n voxel(s)", "", static_cast<int>(report.removed)).arg(what);
    } else {
        report.message = QObject::tr("%1: added %2 and removed %3 voxels")
                .arg(what).arg(report.added).arg(report.removed);
    }
    if (report.added != 0 || report.removed != 0) {
        report.message += QObject::tr(" in %n block(s).", "", static_cast<int>(report.cubesWritten));
        const auto side = QObject::tr(" Over a %1-voxel region about the crosshair.").arg(extent);
        report.message += side;
        if (report.extentReduced) {
            report.message += QObject::tr(" The region was reduced to keep within the memory budget.");
        }
        if (report.clippedByArea || report.clippedByResidency) {
            report.message += QObject::tr(" It was cut short by the movement area or the loaded blocks.");
        }
        if (report.apronIncomplete) {
            report.message += QObject::tr(" One edge had no margin to read, so the result may be a voxel out along it.");
        }
    }
    return report;
}

}// namespace

double defaultMorphRadius() {
    if (Dataset::datasets.empty() || Segmentation::singleton().layerId >= Dataset::datasets.size()
            || Dataset::datasets[Segmentation::singleton().layerId].scales.empty()) {
        return 2.0;// no dataset to ask; the dialog will be re-defaulted when one opens
    }
    const auto spacing = voxelSpacing();
    // Paintera's own default: two voxels along the finest axis, expressed as a length
    return 2.0 * std::min(std::min(spacing.x, spacing.y), spacing.z);
}

MorphReport runObjectMorphology(const MorphRequest & request, QWidget * const) {
    /* Suspended for the same reason the flood fill is: the whole thing is one synchronous
     * pass over the overlay, and a repaint partway through would draw a half-morphed object.
     * Nothing here returns to the event loop or moves the view, so no load can start and
     * nothing can be evicted from under the read. */
    QApplication::setOverrideCursor(Qt::WaitCursor);
    auto report = state->viewer->suspend([&request]{ return run(request); });
    QApplication::restoreOverrideCursor();
    if (report.added != 0 || report.removed != 0) {
        state->viewer->run();
    }
    return report;
}
