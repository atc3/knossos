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

/* Where to move the loader so that a known set of blocks is each made resident once.
 *
 * The loader only ever holds the M³ supercube around one position, so anything that has
 * to touch more blocks than that walks the position around. Walking it to each block in
 * turn, as the shape-interpolation commit used to, centres every window on a block at
 * the edge of what is needed — half of it wasted — and a walk that comes back to a block
 * finds it evicted. This plans the walk once: tile the needed blocks' bounding box with
 * whole supercubes, drop the tiles that hold nothing, and hand every needed block to
 * exactly one tile. A caller then visits each tile once and does all its work on those
 * blocks while they are resident.
 *
 * Deliberately free of Qt and of any KNOSSOS type, so it can be tested on its own — see
 * tests/blocktour_test.cpp. */

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <vector>

namespace blocktour {

using Block = std::array<int, 3>;

inline int floorDiv(const long long a, const long long b) {
    const auto q = a / b;
    return static_cast<int>((a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q);
}

/* The blocks the loader makes resident when moved to the centre of `centre`.
 *
 * The same arithmetic as Loader::Worker::DcoiFromPos and insideCurrentSupercube: half a
 * field of view of extent·(M−1)/2 either side of the position, in whole blocks. `extent`
 * is a block's size in magnification-1 voxels along each axis. */
inline void windowAround(const Block & centre, const int M, const int extent[3], Block & lo, Block & hi) {
    for (int a = 0; a < 3; ++a) {
        const long long e = extent[a];
        const long long pos = centre[a] * e + e / 2;
        const long long half = e * (M - 1) / 2;
        lo[a] = floorDiv(pos - half, e);
        hi[a] = floorDiv(pos + half, e);
    }
}

struct Window {
    Block centre{};          // move the loader to the middle of this block
    Block lo{}, hi{};        // what that makes resident, inclusive
    std::vector<Block> blocks;// the needed blocks this window is responsible for
    bool contains(const Block & b) const {
        return b[0] >= lo[0] && b[0] <= hi[0] && b[1] >= lo[1] && b[1] <= hi[1] && b[2] >= lo[2] && b[2] <= hi[2];
    }
};

/* The centre block whose window starts at `lo` along one axis, found by asking
 * windowAround() rather than by inverting it, so the two cannot disagree. */
inline int centreFor(const int lo, const int axis, const int M, const int extent[3]) {
    Block c{0, 0, 0}, wlo, whi;
    c[axis] = lo + (M - 1) / 2;
    windowAround(c, M, extent, wlo, whi);
    c[axis] += lo - wlo[axis];
    return c[axis];
}

/* Plans the tour over `needed`. Windows come back in serpentine order — consecutive ones
 * share a face where they can — and each needed block appears in exactly one window's
 * `blocks`. Windows that would hold no needed block are left out. */
inline std::vector<Window> plan(const std::vector<Block> & needed, const int M, const int extent[3]) {
    std::vector<Window> tour;
    if (needed.empty() || M <= 0 || extent[0] <= 0 || extent[1] <= 0 || extent[2] <= 0) {
        return tour;
    }
    Block lo = needed.front(), hi = needed.front();
    for (const auto & b : needed) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], b[a]);
            hi[a] = std::max(hi[a], b[a]);
        }
    }
    // window starts along each axis: every M blocks from the low edge, the last pulled back
    // so it ends on the high edge rather than hanging past it
    std::array<std::vector<int>, 3> starts;
    for (int a = 0; a < 3; ++a) {
        const auto lastStart = std::max(lo[a], hi[a] - M + 1);
        for (int s = lo[a]; ; s += M) {
            const auto start = std::min(s, lastStart);
            starts[a].push_back(start);
            if (start >= lastStart) {
                break;
            }
        }
    }
    // serpentine: x fastest, reversing on alternate rows, y reversing on alternate planes
    std::vector<Window> candidates;
    for (std::size_t iz = 0; iz < starts[2].size(); ++iz) {
        for (std::size_t jy = 0; jy < starts[1].size(); ++jy) {
            const auto iy = (iz % 2 == 0) ? jy : starts[1].size() - 1 - jy;
            for (std::size_t jx = 0; jx < starts[0].size(); ++jx) {
                const auto ix = ((iz * starts[1].size() + jy) % 2 == 0) ? jx : starts[0].size() - 1 - jx;
                Window w;
                w.centre = {centreFor(starts[0][ix], 0, M, extent),
                            centreFor(starts[1][iy], 1, M, extent),
                            centreFor(starts[2][iz], 2, M, extent)};
                windowAround(w.centre, M, extent, w.lo, w.hi);
                candidates.push_back(w);
            }
        }
    }
    // each needed block goes to the first window, in tour order, that holds it
    for (const auto & b : needed) {
        for (auto & w : candidates) {
            if (w.contains(b)) {
                w.blocks.push_back(b);
                break;
            }
        }
    }
    for (auto & w : candidates) {
        if (!w.blocks.empty()) {
            tour.push_back(std::move(w));
        }
    }
    return tour;
}


/* Of the centres whose window holds every block in `blocks`, the one nearest `prefer`,
 * axis by axis: the move that keeps the most of the window around `prefer` resident.
 * For work near where the user is looking, so that making a few blocks resident does not
 * evict the view the work is then sent back to. Returns false if no window holds them all. */
inline bool centreNearest(const std::vector<Block> & blocks, const Block & prefer, const int M, const int extent[3], Block & centre) {
    if (blocks.empty() || M <= 0) {
        return false;
    }
    for (int a = 0; a < 3; ++a) {
        int lo = blocks.front()[a], hi = lo;
        for (const auto & b : blocks) {
            lo = std::min(lo, b[a]);
            hi = std::max(hi, b[a]);
        }
        bool found = false;
        int bestDistance = 0;
        for (int c = hi - M; c <= lo + M; ++c) {
            Block probe{0, 0, 0}, wlo, whi;
            probe[a] = c;
            windowAround(probe, M, extent, wlo, whi);
            if (wlo[a] > lo || whi[a] < hi) {
                continue;
            }
            const int distance = std::abs(c - prefer[a]);
            if (!found || distance < bestDistance) {
                found = true;
                bestDistance = distance;
                centre[a] = c;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

}
