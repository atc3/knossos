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

/* Policy layer over cubeloader's residency-aware flood fill.
 *
 * cubeloader::floodFillFrom() does the traversal and stops dead at the edge of what is in
 * memory. This decides what to do about that: by default nothing — the fill simply ends
 * at the block boundary and says so — but a 2D fill can optionally be allowed to pull the
 * loader along and carry on. That is deliberately not the default: it moves the view, it
 * evicts whatever the user was looking at, and on a large connected region it can walk a
 * very long way. */

#include "coordinate.h"
#include "segmentation/cubeloader.h"
#include "segmentation/segmentationsplit.h"

#include <QString>

#include <cstdint>

class QWidget;

struct FloodFillRequest {
    Coordinate seed;
    std::uint64_t fillsoid{0};
    bool threeDimensional{false};
    brush_t::view_t view{brush_t::view_t::xy};
    // 2D only. A 3D fill is always confined to the blocks already in memory: it is far
    // more prone to escaping through a gap, and chasing it across a dataset with the
    // loader in tow is not something to offer behind a checkbox.
    bool mayLoadCubes{false};
};

struct FloodFillReport {
    bool ok{false};
    bool didSomething{false};
    std::size_t voxelsFilled{0};
    std::size_t cubesWritten{0};
    std::size_t boundaryStops{0};
    std::size_t loadRounds{0};
    bool hitCap{false};
    Coordinate filledMin, filledMax;
    QString message;
};

FloodFillReport runFloodFill(const FloodFillRequest & request, QWidget * parent);
