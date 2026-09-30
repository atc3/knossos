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

/* Working out which blocks a planar mask has to be written into, and where in each.
 *
 * This is the part of committing an interpolation that decides *what* gets written, kept
 * apart from the part that makes blocks resident and does the writing. Two reasons.
 *
 * The first is that it is all integer arithmetic over three axes in an order that varies
 * with the viewing plane, a magnification lattice that the block grid need not line up
 * with, and a movement area whose upper bound is exclusive — which is to say it is exactly
 * the kind of code that is wrong in ways nobody notices, because a mistake here does not
 * crash or warn, it quietly leaves a piece of the object unwritten.
 *
 * The second is that skipping blocks the mask does not reach is a large speed-up — a block
 * nothing is written to never enters the loader's cache, so every later slice fetched it
 * again — and a speed-up in this position has to be provably result-preserving. With the
 * plan separated out, "skipping changes nothing" is a property of a pure function over a
 * mask and a grid, which a test can assert directly against the unskipped plan. See
 * tests/planarwrite_test.cpp. */

#include "segmentation/sislice.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace planarwrite {

// Which global axis each role maps to: the slice normal, and the two in-plane axes.
struct Axes {
    int depth{2}, u{0}, v{1};
    bool sane() const {
        return depth >= 0 && depth < 3 && u >= 0 && u < 3 && v >= 0 && v < 3
                && depth != u && depth != v && u != v;
    }
};

struct Block {
    int cube[3]{};          // block coordinate
    int first[3]{}, last[3]{};// the region to write inside it, global and inclusive
};

struct Plan {
    std::vector<Block> blocks;
    int first[3]{}, last[3]{};// the mask's extent at this depth, clamped to the movement area
    std::size_t skipped{0};   // blocks inside the extent that the mask does not reach
    bool clamped{false};      // the movement area cut the extent
};

/* `cubeShape` is in voxels of the scan magnification and `step` is how many
 * magnification-1 voxels one of those spans, so a block covers cubeShape*step along each
 * axis. `areaMax` is exclusive, matching Annotation::movementAreaMax.
 *
 * With `skipUnreached` the plan leaves out blocks holding none of the mask. That is the
 * optimisation, and it is the only difference between the two plans: every block that is
 * kept is identical, and every set voxel of the mask is still covered exactly once. */
inline Plan plan(const SISlice & mask, const int depth, const Axes & axes,
                 const int cubeShape[3], const int step[3],
                 const int areaMin[3], const int areaMax[3], const bool skipUnreached) {
    Plan out;
    if (!axes.sane() || mask.uSize <= 0 || mask.vSize <= 0 || mask.uStep <= 0 || mask.vStep <= 0
            || mask.mask.size() < static_cast<std::size_t>(mask.uSize) * mask.vSize) {
        return out;
    }
    for (int a = 0; a < 3; ++a) {
        if (cubeShape[a] <= 0 || step[a] <= 0) {
            return out;
        }
    }
    int first[3]{}, last[3]{};
    first[axes.depth] = last[axes.depth] = depth;
    first[axes.u] = mask.uMin;
    last[axes.u] = mask.uCoordOf(mask.uSize) - mask.uStep;
    first[axes.v] = mask.vMin;
    last[axes.v] = mask.vCoordOf(mask.vSize) - mask.vStep;
    for (int a = 0; a < 3; ++a) {
        // areaMax is exclusive, so the last coordinate it admits is areaMax - 1
        const auto lo = areaMin[a];
        const auto hi = areaMax[a] - 1;
        const auto cf = std::max(lo, std::min(first[a], hi));
        const auto cl = std::max(lo, std::min(last[a], hi));
        out.clamped = out.clamped || cf != first[a] || cl != last[a];
        first[a] = cf;
        last[a] = cl;
        out.first[a] = cf;
        out.last[a] = cl;
        if (cf > cl) {
            return out;// the movement area excludes this plane entirely
        }
    }

    int extent[3]{}, cubeLo[3]{}, cubeHi[3]{};
    for (int a = 0; a < 3; ++a) {
        extent[a] = cubeShape[a] * step[a];
        cubeLo[a] = siFloorDiv(first[a], extent[a]);
        cubeHi[a] = siFloorDiv(last[a], extent[a]);
    }

    const auto uCubes = cubeHi[axes.u] - cubeLo[axes.u] + 1;
    const auto vCubes = cubeHi[axes.v] - cubeLo[axes.v] + 1;
    std::vector<std::uint8_t> reached;
    siReachedBlocks(mask, cubeLo[axes.u], cubeLo[axes.v], uCubes, vCubes,
                    extent[axes.u], extent[axes.v], reached);

    for (int dz = cubeLo[2]; dz <= cubeHi[2]; ++dz)
    for (int dy = cubeLo[1]; dy <= cubeHi[1]; ++dy)
    for (int dx = cubeLo[0]; dx <= cubeHi[0]; ++dx) {
        const int cube[3]{dx, dy, dz};
        const auto uIdx = cube[axes.u] - cubeLo[axes.u];
        const auto vIdx = cube[axes.v] - cubeLo[axes.v];
        const auto holdsMask = uIdx >= 0 && uIdx < uCubes && vIdx >= 0 && vIdx < vCubes
                && reached[static_cast<std::size_t>(vIdx) * uCubes + uIdx] != 0;
        if (!holdsMask) {
            ++out.skipped;
            if (skipUnreached) {
                continue;
            }
        }
        Block block;
        for (int a = 0; a < 3; ++a) {
            block.cube[a] = cube[a];
            const auto cubeFirst = cube[a] * extent[a];
            const auto cubeLast = cubeFirst + extent[a] - 1;
            block.first[a] = std::max(first[a], cubeFirst);
            block.last[a] = std::min(last[a], cubeLast);
        }
        out.blocks.push_back(block);
    }
    return out;
}

}
