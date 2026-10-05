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

/* Which box a morphology run reads, which box it writes, and how they sit on the
 * magnification lattice.
 *
 * Split out of objectmorphology.cpp and kept Qt-free for the same reason planarwrite.h was
 * split out of shape interpolation: it is integer arithmetic over three axes where a
 * mistake does not crash or warn, it quietly puts voxels one place over. The specific trap
 * is that processRegion() steps the lattice from each *block's* origin and then caps the
 * coordinate it reports to the region it was handed — so a region whose own bounds are off
 * the lattice gets its first plane reported at the capped, off-lattice position, and
 * (pos - origin) / step truncates onto the neighbouring cell. At magnification 1 the step
 * is 1, nothing can be off the lattice, and the whole class of mistake is invisible.
 *
 * So both boxes are built from one origin and one step: the read origin is snapped down
 * onto the lattice, and the write box is placed at whole multiples of the step from it.
 * Every coordinate either traversal can report is then exactly addressable in the read
 * grid, which morphregion_test checks by simulating the traversal rather than by argument.
 */

#include <algorithm>
#include <cstddef>

namespace morphregion {

inline int floorDiv(const int a, const int b) {
    const auto q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}
inline int ceilDiv(const int a, const int b) {
    return -floorDiv(-a, b);
}

struct Box {
    int first[3]{0, 0, 0};
    int last[3]{0, 0, 0};
    int step[3]{1, 1, 1};
    int dim[3]{0, 0, 0};

    std::size_t voxels() const {
        return static_cast<std::size_t>(dim[0]) * dim[1] * dim[2];
    }
    bool contains(const int p[3]) const {
        for (int a = 0; a < 3; ++a) {
            if (p[a] < first[a] || p[a] > last[a]) {
                return false;
            }
        }
        return true;
    }
    // valid only for a coordinate on this box's own lattice; see inRange() first
    std::size_t index(const int p[3]) const {
        const auto x = (p[0] - first[0]) / step[0];
        const auto y = (p[1] - first[1]) / step[1];
        const auto z = (p[2] - first[2]) / step[2];
        return (static_cast<std::size_t>(z) * dim[1] + y) * dim[0] + x;
    }
    bool inRange(const int p[3]) const {
        for (int a = 0; a < 3; ++a) {
            const auto i = (p[a] - first[a]) / step[a];
            if (p[a] < first[a] || i >= dim[a]) {
                return false;
            }
        }
        return true;
    }
    bool onLattice(const int p[3]) const {
        for (int a = 0; a < 3; ++a) {
            if ((p[a] - first[a]) % step[a] != 0) {
                return false;
            }
        }
        return true;
    }
};

struct Plan {
    Box read, write;
    // the write box was cut short by the limits rather than by the requested extent
    bool clipped{false};
    /* Some face of the read box has less than the full `pad` beyond the write box, so the
     * blur or the distance transform saw a truncated neighbourhood along it. Reported rather
     * than hidden: a zero-extended Gaussian falls to about 0.63 at a face, which is enough
     * to push a voxel the wrong side of a 0.5 threshold. */
    bool apronIncomplete{false};
    bool empty{true};
};

/* `extent` is the side of the written box in voxels of the current magnification, `pad` the
 * apron in voxels per axis, and the limits are inclusive global bounds (the caller has
 * already turned an exclusive movement-area maximum into an inclusive one and intersected
 * it with what is resident). */
inline Plan plan(const int centre[3], const int extent, const int step[3], const int pad[3],
                 const int limitMin[3], const int limitMax[3]) {
    Plan result;
    for (int a = 0; a < 3; ++a) {
        if (step[a] <= 0 || pad[a] < 0) {
            return result;// a nonsense lattice yields no plan rather than a wrong one
        }
    }
    if (extent <= 0) {
        return result;
    }
    const auto half = extent / 2;

    int wantWriteFirst[3], wantWriteLast[3];
    for (int a = 0; a < 3; ++a) {
        wantWriteFirst[a] = std::max(centre[a] - half * step[a], limitMin[a]);
        wantWriteLast[a] = std::min(centre[a] + half * step[a], limitMax[a]);
        result.clipped = result.clipped
                || centre[a] - half * step[a] < limitMin[a]
                || centre[a] + half * step[a] > limitMax[a];
    }

    for (int a = 0; a < 3; ++a) {
        const auto wantReadFirst = wantWriteFirst[a] - pad[a] * step[a];
        const auto wantReadLast = wantWriteLast[a] + pad[a] * step[a];
        const auto clampedFirst = std::max(wantReadFirst, limitMin[a]);
        // snapped down onto the global lattice, which is the multiples of the step: block
        // origins are multiples of cubeShape x step, itself a multiple of the step
        result.read.first[a] = floorDiv(clampedFirst, step[a]) * step[a];
        result.read.last[a] = std::min(wantReadLast, limitMax[a]);
        result.read.step[a] = step[a];
        if (result.read.last[a] < result.read.first[a]) {
            return result;
        }
        result.read.dim[a] = (result.read.last[a] - result.read.first[a]) / step[a] + 1;
        result.apronIncomplete = result.apronIncomplete
                || result.read.first[a] > floorDiv(wantReadFirst, step[a]) * step[a]
                || result.read.last[a] < wantReadLast;
    }

    for (int a = 0; a < 3; ++a) {
        result.write.step[a] = step[a];
        // whole steps from the read origin, so the write bounds are on the read lattice
        const auto firstIdx = std::max(0, ceilDiv(wantWriteFirst[a] - result.read.first[a], step[a]));
        const auto lastIdx = std::min(result.read.dim[a] - 1,
                                      floorDiv(wantWriteLast[a] - result.read.first[a], step[a]));
        if (lastIdx < firstIdx) {
            return result;
        }
        result.write.first[a] = result.read.first[a] + firstIdx * step[a];
        result.write.last[a] = result.read.first[a] + lastIdx * step[a];
        result.write.dim[a] = lastIdx - firstIdx + 1;
    }
    result.empty = false;
    return result;
}

}// namespace morphregion
