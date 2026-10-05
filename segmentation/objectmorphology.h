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

/* Dilate, erode or smooth the selected object, over a bounded region around the crosshair.
 *
 * Policy layer over morphology.h the way floodfill.h is over cubeloader's flood fill: that
 * header holds every decision about what the result is, this one holds the decisions about
 * how much of the volume to do it to.
 *
 * ## Why a region rather than the whole object
 *
 * Because that is the only answer that is both bounded and honest, and because it is what
 * the reference implementation does. Paintera restricts its own morphology to the cells
 * intersecting `intervalsToSmooth`, which Common.kt fills from the viewer's visible
 * intervals — so "smooth label" there means "smooth the part of it you are looking at", not
 * "smooth all of it". Doing otherwise would mean discovering an object's full extent first,
 * which for a vessel crossing the volume is every block in the dataset.
 *
 * The region is therefore a box about the crosshair, clipped to the movement area and to
 * the blocks actually in memory. Worth knowing: that clip is almost never what limits it.
 * The loader keeps an M³ supercube resident, M = 7, so ±3 blocks of 128 are already in
 * memory around the crosshair — an 896-voxel box at magnification 1. The ceiling that bites
 * is arithmetic, not residency: a smooth holds five arrays over the region at four bytes a
 * voxel, so MAX_REGION_VOXELS caps it well below what is loaded. **Nothing here moves the
 * view or asks the loader for anything**, which is the main reason it is safe to run from a
 * menu entry in the middle of painting.
 *
 * ## The apron
 *
 * A blur or a distance transform needs to see past the edge of what it writes, or the first
 * and last few planes of the region come out wrong — a zero-extended Gaussian falls to
 * about 0.63 at the face, which is enough to push a voxel the wrong side of a 0.5 threshold
 * and shave a layer off a flat face that nothing was wrong with. So the region that is
 * *read* is grown by the kernel's reach and only the inside of it is *written*, which is
 * Paintera's PaddedCell idea without the cell machinery. When the apron could not be read
 * in full — the crosshair up against the movement area, or a block not loaded yet — the
 * report says so instead of quietly keeping the dubious edge.
 */

#include "coordinate.h"
#include "segmentation/morphology.h"

#include <QString>

#include <cstdint>

class QWidget;

struct MorphRequest {
    morphology::Operation op{morphology::Operation::Dilate};
    double radius{0};                                           // nanometres
    morphology::Direction direction{morphology::Direction::Both};// Smooth only
    morphology::Infill infill{morphology::Infill::Background};
    std::uint64_t replacement{0};
    double threshold{0.5};
    /* Operate on the one id a stroke would paint rather than on every id of the selected
     * object. See Segmentation::selectedSubobjectIds(). */
    bool fragmentsOnly{false};
    // side of the region, in voxels of the current magnification
    int extent{128};
};

struct MorphReport {
    bool ok{false};
    std::size_t added{0};
    std::size_t removed{0};
    std::size_t cubesWritten{0};
    std::size_t regionVoxels{0};
    bool clippedByResidency{false};
    bool clippedByArea{false};
    bool extentReduced{false};
    bool apronIncomplete{false};
    QString message;
};

MorphReport runObjectMorphology(const MorphRequest & request, QWidget * parent);

// The radius a fresh dialog should offer: Paintera's 2 voxels along the finest axis.
double defaultMorphRadius();
