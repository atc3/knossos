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

/* An order to visit inventory objects in that keeps block loads down.
 *
 * Jumping to an object makes the loader hold the M³ supercube of blocks around it, so the
 * cost of a jump is the number of blocks in the new window that were not in the old one.
 * Finding the order that minimises the total is a travelling-salesman problem; this is the
 * greedy answer to it, which is most of the benefit:
 *
 *   - Objects whose windows are the same are one stop: all of them are taken together,
 *     since moving between them loads nothing.
 *   - From the current window, go to the remaining window that adds the fewest new blocks
 *     (equivalently, overlaps the current one most), nearer first on a tie, then whichever
 *     holds the earliest object in the input order.
 *   - When nothing remaining overlaps — a cluster is used up — go to the nearest window
 *     instead, by block distance, so the next cluster starts close by.
 *
 * Windows are bucketed by their low corner, so finding the overlapping candidates looks at
 * a fixed (2M−1)³ neighbourhood, nearest first and stopping as soon as nothing further out
 * can be cheaper, rather than every remaining object; sparse stretches are crossed a coarse
 * cell at a time. A list of a few hundred thousand rows orders in about a second.
 *
 * Deliberately free of Qt and of any KNOSSOS type, so it can be tested on its own — see
 * tests/inventoryroute_test.cpp. */

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <vector>

namespace invroute {

using Int3 = std::array<int, 3>;

// the blocks a position makes resident, inclusive on both ends
struct Window {
    Int3 lo;
    Int3 hi;
};

inline std::int64_t volume(const Window & w) {
    std::int64_t v = 1;
    for (int a = 0; a < 3; ++a) {
        v *= std::max(0, w.hi[a] - w.lo[a] + 1);
    }
    return v;
}

inline std::int64_t overlap(const Window & a, const Window & b) {
    std::int64_t v = 1;
    for (int k = 0; k < 3; ++k) {
        v *= std::max(0, std::min(a.hi[k], b.hi[k]) - std::max(a.lo[k], b.lo[k]) + 1);
    }
    return v;
}

// blocks a move from `from` to `to` has to load
inline std::int64_t newBlocks(const Window & from, const Window & to) {
    return volume(to) - overlap(from, to);
}

namespace detail {
struct KeyHash {
    std::size_t operator()(const Int3 & k) const {
        std::uint64_t h = static_cast<std::uint32_t>(k[0]);
        h = h * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint32_t>(k[1]);
        h = h * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint32_t>(k[2]);
        return static_cast<std::size_t>(h ^ (h >> 29));
    }
};
inline std::int64_t squaredDistance(const Int3 & a, const Int3 & b) {
    std::int64_t d = 0;
    for (int k = 0; k < 3; ++k) {
        const std::int64_t e = a[k] - b[k];
        d += e * e;
    }
    return d;
}
inline int chebyshev(const Int3 & a, const Int3 & b) {
    return std::max({std::abs(a[0] - b[0]), std::abs(a[1] - b[1]), std::abs(a[2] - b[2])});
}
}

/* The visiting order, as indices into `windows`, starting from the window the viewer is in
 * now. `M` is the supercube edge in blocks, which bounds how far apart two windows can be
 * and still overlap. Every index appears exactly once. */
inline std::vector<std::uint32_t> plan(const std::vector<Window> & windows, const Window & start, const int M) {
    struct Bucket {
        Window window;// of its first object; windows sharing a low corner differ at most by a block at the far edge
        std::vector<std::uint32_t> members;// ascending, so input order survives within a stop
    };
    std::unordered_map<Int3, Bucket, detail::KeyHash> buckets;
    buckets.reserve(windows.size());
    /* Low corners further apart than the largest window extent cannot overlap. That is M−1
     * for a cube of M blocks; measured rather than assumed, so a window clipped or widened
     * at a cube boundary is still found. */
    int reach = std::max(0, M - 1);
    for (std::uint32_t i = 0; i < windows.size(); ++i) {
        auto & bucket = buckets[windows[i].lo];
        if (bucket.members.empty()) {
            bucket.window = windows[i];
        }
        bucket.members.push_back(i);
        for (int a = 0; a < 3; ++a) {
            reach = std::max(reach, windows[i].hi[a] - windows[i].lo[a]);
        }
    }
    /* For the far search: the same buckets, grouped into coarse cells of COARSE³ low
     * corners, so a gap between clusters is crossed a coarse cell at a time. Removed
     * buckets are left in their cell and skipped when met. */
    constexpr int COARSE = 8;
    const auto coarseOf = [](const Int3 & k) {
        const auto down = [](const int v) { return v >= 0 ? v / COARSE : -((-v + COARSE - 1) / COARSE); };
        return Int3{down(k[0]), down(k[1]), down(k[2])};
    };
    std::unordered_map<Int3, std::vector<Int3>, detail::KeyHash> coarse;
    for (const auto & entry : buckets) {
        coarse[coarseOf(entry.first)].push_back(entry.first);
    }

    std::vector<std::uint32_t> order;
    order.reserve(windows.size());
    auto current = start;
    /* The neighbourhood's offsets, nearest shell first. A move whose low corner is k blocks
     * off along some axis loads at least k whole faces of the window, so once a shell has
     * produced something cheaper than k+1 faces, no further shell can beat it. */
    std::vector<Int3> offsets;
    std::vector<std::size_t> shellEnd;// offsets[shellEnd[k-1]..shellEnd[k]) are shell k
    for (int k = 0; k <= reach; ++k) {
        for (int dx = -k; dx <= k; ++dx) {
            for (int dy = -k; dy <= k; ++dy) {
                for (int dz = -k; dz <= k; ++dz) {
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) == k) {
                        offsets.push_back({dx, dy, dz});
                    }
                }
            }
        }
        shellEnd.push_back(offsets.size());
    }
    const std::int64_t face = static_cast<std::int64_t>(reach + 1) * (reach + 1);

    struct Best {
        Int3 key{};
        bool found{false};
        std::int64_t cost{std::numeric_limits<std::int64_t>::max()};
        std::int64_t distance{std::numeric_limits<std::int64_t>::max()};
        std::uint32_t first{std::numeric_limits<std::uint32_t>::max()};
        void offer(const Int3 & k, const std::int64_t c, const std::int64_t d, const std::uint32_t f) {
            if (found && (c != cost ? c > cost : d != distance ? d > distance : f >= first)) {
                return;
            }
            *this = {k, true, c, d, f};
        }
    };
    // drops removed buckets from a coarse cell's list, and the cell once it is empty
    const auto liveKeys = [&](const decltype(coarse)::iterator it) -> std::vector<Int3> & {
        auto & keys = it->second;
        keys.erase(std::remove_if(std::begin(keys), std::end(keys), [&](const Int3 & k) { return buckets.count(k) == 0; }), std::end(keys));
        return keys;
    };

    while (!buckets.empty()) {
        Best best;
        const auto offerOverlapping = [&](const Int3 & key, const Bucket & bucket) {
            if (overlap(current, bucket.window) > 0) {
                best.offer(key, newBlocks(current, bucket.window), detail::squaredDistance(current.lo, key), bucket.members.front());
            }
        };
        /* Two ways to find the overlapping windows: probe every low corner in reach, or
         * read the lists of the few coarse cells that cover the same box. Probing wins
         * where objects are dense, the lists where they are sparse; pick per step. */
        const Int3 boxLo{current.lo[0] - reach, current.lo[1] - reach, current.lo[2] - reach};
        const Int3 boxHi{current.lo[0] + reach, current.lo[1] + reach, current.lo[2] + reach};
        const auto cLo = coarseOf(boxLo), cHi = coarseOf(boxHi);
        std::size_t listed = 0;
        std::vector<decltype(coarse)::iterator> cells;
        for (int x = cLo[0]; x <= cHi[0]; ++x) {
            for (int y = cLo[1]; y <= cHi[1]; ++y) {
                for (int z = cLo[2]; z <= cHi[2]; ++z) {
                    const auto it = coarse.find({x, y, z});
                    if (it != std::end(coarse)) {
                        cells.push_back(it);
                        listed += it->second.size();
                    }
                }
            }
        }
        if (listed < offsets.size()) {
            for (const auto it : cells) {
                for (const auto & k : liveKeys(it)) {
                    if (std::max({std::abs(k[0] - current.lo[0]), std::abs(k[1] - current.lo[1]), std::abs(k[2] - current.lo[2])}) <= reach) {
                        offerOverlapping(k, buckets.find(k)->second);
                    }
                }
            }
        } else {
            for (std::size_t k = 0, o = 0; k < shellEnd.size(); ++k) {
                for (; o < shellEnd[k]; ++o) {
                    const auto & d = offsets[o];
                    const auto it = buckets.find({current.lo[0] + d[0], current.lo[1] + d[1], current.lo[2] + d[2]});
                    if (it != std::end(buckets)) {
                        offerOverlapping(it->first, it->second);
                    }
                }
                if (best.found && best.cost < static_cast<std::int64_t>(k + 1) * face) {
                    break;
                }
            }
        }
        if (!best.found) {
            /* Nothing left overlaps, so every choice loads a whole window: take the nearest
             * by block distance. Coarse cells are searched in shells growing outward; once
             * a shell r holds anything, shell r+1 can still hold something nearer (cells
             * are COARSE wide) but shell r+2 cannot, so those two shells decide it exactly. */
            const auto offerAny = [&](const Int3 & key) {
                const auto it = buckets.find(key);
                if (it != std::end(buckets)) {
                    best.offer(key, detail::chebyshev(current.lo, key), detail::squaredDistance(current.lo, key), it->second.members.front());
                }
            };
            const auto here = coarseOf(current.lo);
            const auto visitShell = [&](const int r) {
                bool any = false;
                for (int dx = -r; dx <= r; ++dx) {
                    for (int dy = -r; dy <= r; ++dy) {
                        const bool onFace = std::abs(dx) == r || std::abs(dy) == r;
                        for (int dz = -r; dz <= r; dz += (onFace || r == 0) ? 1 : 2 * r) {// the shell's surface only
                            const auto it = coarse.find({here[0] + dx, here[1] + dy, here[2] + dz});
                            if (it == std::end(coarse)) {
                                continue;
                            }
                            const auto & keys = liveKeys(it);
                            for (const auto & k : keys) {
                                offerAny(k);
                                any = true;
                            }
                            if (keys.empty()) {
                                coarse.erase(it);
                            }
                        }
                    }
                }
                return any;
            };
            for (int r = 0; ; ++r) {
                const std::int64_t shellSide = 2 * static_cast<std::int64_t>(r) + 3;
                if (shellSide * shellSide * 6 > static_cast<std::int64_t>(coarse.size())) {
                    for (const auto & entry : buckets) {// the shells would cost more than looking at everything
                        offerAny(entry.first);
                    }
                    break;
                }
                if (visitShell(r)) {
                    visitShell(r + 1);
                    break;
                }
            }
        }
        const auto chosen = buckets.find(best.key);
        order.insert(std::end(order), std::begin(chosen->second.members), std::end(chosen->second.members));
        current = chosen->second.window;
        buckets.erase(chosen);
    }
    return order;
}

}
