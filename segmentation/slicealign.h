/*
 *  This file is a part of KNOSSOS.
 *
 *  (C) Copyright 2007-2018
 *  Max-Planck-Gesellschaft zur Foerderung der Wissenschaften e.V.
 *
 *  KNOSSOS is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 of
 *  the License as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  For further information, visit https://knossos.app
 *  or contact knossosteam@gmail.com
 */

#pragma once

/* The translation that best lines up two key slices, for shape interpolation.
 *
 * Interpolating between two slices that have drifted sideways needs to know how far they
 * drifted. Their centroids answer that for compact shapes and badly for irregular ones: a
 * branch appearing on one side drags the centroid towards it although the body has not
 * moved, and the blend then smears the body sideways. Here the offset is the one that
 * makes the two masks overlap the most — translation only, no rotation or warping — which
 * follows the bulk of the shape and ignores what is new on one side.
 *
 * Every shift is tried, but on a coarse copy: the masks are averaged down 2× at a time
 * until they fit in COARSEST pixels a side, every shift at which they can touch is scored
 * there, and the winner is refined level by level, ±2 pixels at each. The coarsest search
 * is what makes it global — a shape that moved further than its own width is still found
 * — and the refinement is what makes it cheap. Whatever has not been refined when the
 * time budget runs out is scaled up from the last level reached, so a huge slice gets a
 * slightly coarser answer rather than a stall.
 *
 * Free of Qt and of KNOSSOS types, so it can be tested on its own — see
 * tests/slicealign_test.cpp. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace slicealign {

struct Shift {
    int du{0}, dv{0};// mask 2 sits at mask 1 + (du, dv), in grid pixels
    bool found{false};// false if either mask is empty
    int levelReached{0};// 0 = refined to full resolution; k = accurate to about 2^k pixels
};

namespace detail {

constexpr int COARSEST = 48;
constexpr int REFINE = 2;

struct Level {
    int w{0}, h{0};
    std::vector<float> a, b;// coverage in [0, 1]
    std::vector<std::uint32_t> aNonzero;// indices into a, so a score skips the empty part
};

inline void indexNonzero(Level & level) {
    level.aNonzero.clear();
    for (std::size_t i = 0; i < level.a.size(); ++i) {
        if (level.a[i] != 0.f) {
            level.aNonzero.push_back(static_cast<std::uint32_t>(i));
        }
    }
}

inline Level halve(const Level & fine) {
    Level coarse;
    coarse.w = (fine.w + 1) / 2;
    coarse.h = (fine.h + 1) / 2;
    coarse.a.assign(static_cast<std::size_t>(coarse.w) * coarse.h, 0.f);
    coarse.b.assign(coarse.a.size(), 0.f);
    for (int v = 0; v < fine.h; ++v) {
        for (int u = 0; u < fine.w; ++u) {
            const auto from = static_cast<std::size_t>(v) * fine.w + u;
            const auto to = static_cast<std::size_t>(v / 2) * coarse.w + u / 2;
            coarse.a[to] += 0.25f * fine.a[from];
            coarse.b[to] += 0.25f * fine.b[from];
        }
    }
    indexNonzero(coarse);
    return coarse;
}

// Σ a(x) · b(x + s)
inline double overlap(const Level & level, const int su, const int sv) {
    double sum = 0;
    for (const auto i : level.aNonzero) {
        const int u = static_cast<int>(i % level.w) + su;
        const int v = static_cast<int>(i / level.w) + sv;
        if (u >= 0 && v >= 0 && u < level.w && v < level.h) {
            sum += static_cast<double>(level.a[i]) * level.b[static_cast<std::size_t>(v) * level.w + u];
        }
    }
    return sum;
}

/* Best of the shifts in [uLo, uHi] × [vLo, vHi]. Ties — common on a coarse grid, and on
 * symmetric shapes at any level — go to the shift nearest `prefer`, which is the centroid
 * offset: when overlap cannot tell, the old answer is a reasonable one. */
inline void best(const Level & level, const int uLo, const int uHi, const int vLo, const int vHi,
                 const double preferU, const double preferV, int & bu, int & bv) {
    double bestScore = -1, bestDistance = 0;
    for (int sv = vLo; sv <= vHi; ++sv) {
        for (int su = uLo; su <= uHi; ++su) {
            const auto score = overlap(level, su, sv);
            const auto distance = (su - preferU) * (su - preferU) + (sv - preferV) * (sv - preferV);
            // relative tolerance: sums of floats that should tie rarely tie exactly
            const auto tol = 1e-9 * std::max(1.0, bestScore);
            if (score > bestScore + tol || (std::abs(score - bestScore) <= tol && distance < bestDistance)) {
                bestScore = score;
                bestDistance = distance;
                bu = su;
                bv = sv;
            }
        }
    }
}

inline bool centroid(const std::vector<std::uint8_t> & m, const int w, double & cu, double & cv) {
    double su = 0, sv = 0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < m.size(); ++i) {
        if (m[i] != 0) {
            su += static_cast<double>(i % w);
            sv += static_cast<double>(i / w);
            ++n;
        }
    }
    if (n == 0) {
        return false;
    }
    cu = su / n;
    cv = sv / n;
    return true;
}

}

/* `m1`, `m2`: w × h binary masks on the same grid, row-major. `budgetMs` bounds the
 * refinement; the coarse search always runs. */
inline Shift bestTranslation(const std::vector<std::uint8_t> & m1, const std::vector<std::uint8_t> & m2,
                             const int w, const int h, const int budgetMs = 400) {
    using namespace detail;
    Shift result;
    double c1u, c1v, c2u, c2v;
    if (w <= 0 || h <= 0 || !centroid(m1, w, c1u, c1v) || !centroid(m2, w, c2u, c2v)) {
        return result;
    }
    const auto started = std::chrono::steady_clock::now();

    std::vector<Level> levels(1);
    levels[0].w = w;
    levels[0].h = h;
    levels[0].a.assign(std::begin(m1), std::end(m1));
    levels[0].b.assign(std::begin(m2), std::end(m2));
    for (auto & x : levels[0].a) { x = x != 0.f ? 1.f : 0.f; }
    for (auto & x : levels[0].b) { x = x != 0.f ? 1.f : 0.f; }
    indexNonzero(levels[0]);
    while (std::max(levels.back().w, levels.back().h) > COARSEST) {
        levels.push_back(halve(levels.back()));
    }

    // every shift at which the two can touch, at the coarsest level
    auto k = static_cast<int>(levels.size()) - 1;
    const double scaleK = std::ldexp(1.0, -k);
    const auto & top = levels[k];
    int bu = 0, bv = 0;
    best(top, -(top.w - 1), top.w - 1, -(top.h - 1), top.h - 1, (c2u - c1u) * scaleK, (c2v - c1v) * scaleK, bu, bv);

    while (k > 0) {
        if (std::chrono::steady_clock::now() - started > std::chrono::milliseconds(budgetMs)) {
            break;
        }
        --k;
        const double scale = std::ldexp(1.0, -k);
        const int cu = 2 * bu, cv = 2 * bv;
        best(levels[k], cu - REFINE, cu + REFINE, cv - REFINE, cv + REFINE, (c2u - c1u) * scale, (c2v - c1v) * scale, bu, bv);
    }
    result.du = bu * (1 << k);
    result.dv = bv * (1 << k);
    result.levelReached = k;
    result.found = true;
    return result;
}

}
