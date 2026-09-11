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

/* Filling the interior of an outline, once the outline closes.
 *
 * Draw a ring and the middle stays empty; the obvious thing to want is for it to fill in
 * when the ring closes. "Closes" is the whole problem, and it is a question about the
 * *plane*, not about the stroke: the stroke may only have drawn the last third of a circle
 * whose other two thirds were painted an hour ago, and that still encloses something.
 *
 * So the test is not "did this stroke form a loop" but "which parts of this plane can no
 * longer reach the outside". Anything that cannot is interior, whatever drew the wall
 * around it and whenever.
 *
 * Kept free of Qt and of KNOSSOS's cube machinery so the part that is easy to get subtly
 * wrong — the connectivity — can be tested on its own. See tests/holefill_test.cpp. */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace holefill {

/* Marks every cell that is not solid and cannot reach the grid border.
 *
 * `solid` is row-major, w*h, non-zero where the id being drawn already is. `out` is sized
 * to match and set non-zero exactly on the enclosed cells. Returns how many there were, so
 * a caller can skip the write when a stroke closed nothing — which is most strokes.
 *
 * The escape walk is 4-connected, which is what makes a hand-drawn outline behave. A round
 * brush dragged diagonally leaves voxels touching only at their corners; the eye reads
 * that as a closed wall, and an 8-connected escape walk would slip through the gap and
 * report the interior as open. Taking the walls as 8-connected and the space as
 * 4-connected is the standard way round, and it is the one that matches what was drawn.
 *
 * The border of the grid is the outside. Callers therefore have to hand in a region whose
 * edge really is outside the shape — grown until the object stops touching it — or a shape
 * running off the edge reads as open. That is the safe direction to fail in: nothing is
 * filled that was not demonstrably enclosed. */
inline std::size_t enclosed(const std::vector<std::uint8_t> & solid, const int w, const int h,
                            std::vector<std::uint8_t> & out) {
    out.assign(solid.size(), 0);
    if (w <= 0 || h <= 0 || solid.size() != static_cast<std::size_t>(w) * h) {
        return 0;
    }
    const auto index = [w](const int x, const int y){ return static_cast<std::size_t>(y) * w + x; };

    // reachable-from-the-border, flooded from every open cell on the edge
    std::vector<std::uint8_t> outside(solid.size(), 0);
    std::vector<int> stack;// cell indices; explicit, since a region can be millions of cells
    const auto push = [&](const int x, const int y){
        const auto i = index(x, y);
        if (solid[i] == 0 && outside[i] == 0) {
            outside[i] = 1;
            stack.push_back(static_cast<int>(i));
        }
    };
    for (int x = 0; x < w; ++x) {
        push(x, 0);
        push(x, h - 1);
    }
    for (int y = 0; y < h; ++y) {
        push(0, y);
        push(w - 1, y);
    }
    while (!stack.empty()) {
        const auto i = stack.back();
        stack.pop_back();
        const auto x = i % w;
        const auto y = i / w;
        if (x > 0)     { push(x - 1, y); }
        if (x < w - 1) { push(x + 1, y); }
        if (y > 0)     { push(x, y - 1); }
        if (y < h - 1) { push(x, y + 1); }
    }

    std::size_t count{0};
    for (std::size_t i = 0; i < solid.size(); ++i) {
        if (solid[i] == 0 && outside[i] == 0) {
            out[i] = 1;
            ++count;
        }
    }
    return count;
}

/* Whether any solid cell sits on the grid border.
 *
 * The caller grows its region until this is false, because only then is the border known
 * to be outside the shape. Reported per side so the growth only pays for the directions
 * that need it. */
struct BorderContact {
    bool left{false}, right{false}, top{false}, bottom{false};
    bool any() const { return left || right || top || bottom; }
};

inline BorderContact touchesBorder(const std::vector<std::uint8_t> & solid, const int w, const int h) {
    BorderContact contact;
    if (w <= 0 || h <= 0 || solid.size() != static_cast<std::size_t>(w) * h) {
        return contact;
    }
    for (int y = 0; y < h; ++y) {
        contact.left = contact.left || solid[static_cast<std::size_t>(y) * w] != 0;
        contact.right = contact.right || solid[static_cast<std::size_t>(y) * w + (w - 1)] != 0;
    }
    for (int x = 0; x < w; ++x) {
        contact.top = contact.top || solid[x] != 0;
        contact.bottom = contact.bottom || solid[static_cast<std::size_t>(h - 1) * w + x] != 0;
    }
    return contact;
}

}
