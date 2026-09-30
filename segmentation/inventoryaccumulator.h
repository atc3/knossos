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

/* Tallying which object ids exist in the dataset, and where each one is.
 *
 * Refining an existing annotation means finding it first, and nothing in KNOSSOS knows
 * what is out there: the object list only ever holds what was clicked, loaded from a
 * mergelist, or collected from the handful of blocks currently resident. So a background
 * sweep reads the segmentation layer at a coarse magnification and tallies it. This header
 * is the tallying — the sweep itself, and everything that talks to the network or to the
 * rest of KNOSSOS, lives in objectinventory.cpp.
 *
 * Two things here are easy to get subtly wrong and are therefore kept free of Qt and of
 * the cube machinery, so they can be tested on their own (see
 * tests/inventoryaccumulator_test.cpp):
 *
 *   - the coordinate arithmetic, which converts a voxel in a coarse-magnification block
 *     into the magnification-1 coordinate the rest of the application speaks;
 *   - the choice of *which* voxel to remember per object, which decides where pressing
 *     "next object" actually lands you.
 *
 * On that second point: the mean of an object's voxels is not a place. A centroid is fine
 * for a blob, but a vessel bends and a mitochondrion branches, and the mean of a curved
 * shape sits outside it — jump there and you are looking at empty neuropil next to the
 * thing you wanted. So the stored jump target is a real voxel, picked as described at
 * `ingestCube`. The mean is kept too, but only for display and sorting. */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace objinv {

/* One object. Deliberately a fixed-size POD with no padding surprises: the on-disk cache
 * is a header followed by this array verbatim, in discovery order, which is the simplest
 * possible guarantee that the order a resumed or reloaded scan presents is the order it
 * was discovered in. */
struct Record {
    std::uint64_t id{0};
    std::uint64_t voxels{0};        // at the scan magnification, not mag 1
    std::int64_t sum[3]{};          /* Σ of mag-1 coordinates. Stored rather than a finished
                                     * centroid so a scan resumed from the cache keeps
                                     * accumulating into the same running total. */
    std::int32_t rep[3]{};          // the jump target: a real voxel of this object, mag-1
    std::int32_t bboxMin[3]{};
    std::int32_t bboxMax[3]{};
    std::uint32_t bestCubeVoxels{0};// how good `rep` is; see ingestCube

    // The mean position. Inside the object for a blob, possibly outside it for anything
    // bent — which is why it is not what `rep` holds.
    void centroid(std::int32_t (& out)[3]) const {
        for (int a = 0; a < 3; ++a) {
            out[a] = voxels == 0 ? rep[a] : static_cast<std::int32_t>(sum[a] / static_cast<std::int64_t>(voxels));
        }
    }
};
static_assert(sizeof(Record) == 80, "the cache format stores Record verbatim");
static_assert(alignof(Record) == 8, "");

/* Where a block sits and how big its voxels are.
 *
 * `origin` is the mag-1 coordinate of the block's first voxel and `step` how many mag-1
 * voxels one block voxel spans, per axis — the pyramid is not obliged to downsample z at
 * the same rate as x and y, and a dataset that declares `VoxelSize_nm` per level is free
 * to say so. `nmPerVoxel` is the physical spacing at the scan magnification, used only to
 * measure distances when picking a representative voxel, so that "nearest the middle"
 * means nearest in tissue rather than nearest in array indices. */
struct CubeGeometry {
    std::int32_t origin[3]{};
    std::int32_t step[3]{1, 1, 1};
    std::int32_t shape[3]{128, 128, 128};
    double nmPerVoxel[3]{1.0, 1.0, 1.0};

    std::size_t voxelCount() const {
        return static_cast<std::size_t>(shape[0]) * static_cast<std::size_t>(shape[1]) * static_cast<std::size_t>(shape[2]);
    }
    bool sane() const {
        return shape[0] > 0 && shape[1] > 0 && shape[2] > 0
                && step[0] > 0 && step[1] > 0 && step[2] > 0
                && nmPerVoxel[0] > 0 && nmPerVoxel[1] > 0 && nmPerVoxel[2] > 0;
    }
};

class Accumulator {
public:
    /* Folds one decoded block into the tally.
     *
     * `voxels` is the block's `uint64` ids, indexed [z][y][x] the way KNOSSOS stores a
     * cube. Background is skipped — and the caller passes the id in, because the
     * background id is a user setting and is not always 0.
     *
     * Two passes, both linear, no per-voxel hashing. A naive "look up every voxel" costs
     * 2M hash probes per 128³ block, which for the larger datasets is hours of CPU for
     * work that is almost all redundant: mode-downsampled segmentation is strongly
     * run-coherent along x, so the first pass walks runs of equal id and probes once per
     * run. That leaves the per-block mean known only at the end of the first pass, hence
     * the second one, which costs a 16 MiB sequential re-read (~2 ms) and is cheaper and
     * far simpler than materialising a run list that could, in the pathological case, be
     * larger than the block.
     *
     * The representative voxel is the one nearest the middle of the block in which this
     * object has the most voxels. Picking per-block and then keeping the densest block's
     * candidate makes the answer independent of the order blocks arrive in — ties are
     * broken on the coordinate itself, not on who got there first — so the same dataset
     * always yields the same list, whether scanned in one go, resumed from a checkpoint,
     * or swept in a different order entirely. It also lands you in the object's thickest
     * part rather than on a thin tendril that happens to pass near the mean, which is the
     * better place to start refining. */
    void ingestCube(const std::uint64_t * voxels, const CubeGeometry & geom, const std::uint64_t backgroundId) {
        if (voxels == nullptr || !geom.sane()) {
            return;
        }
        const auto sx = geom.shape[0], sy = geom.shape[1], sz = geom.shape[2];
        const auto rowStride = static_cast<std::size_t>(sx);
        const auto planeStride = rowStride * static_cast<std::size_t>(sy);

        local.clear();
        touchedIdx.clear();
        // pass 1 — runs along x, one probe per run
        for (int z = 0; z < sz; ++z) {
            for (int y = 0; y < sy; ++y) {
                const auto * row = voxels + static_cast<std::size_t>(z) * planeStride + static_cast<std::size_t>(y) * rowStride;
                for (int x = 0; x < sx; ) {
                    const auto id = row[x];
                    int end = x + 1;
                    while (end < sx && row[end] == id) { ++end; }
                    if (id != backgroundId) {
                        const auto len = end - x;
                        auto & s = local[id];
                        s.count += static_cast<std::uint64_t>(len);
                        // Σ over the run of x is len*(first+last)/2; len*(first+last) is always even
                        s.sum[0] += static_cast<std::int64_t>(len) * (x + end - 1) / 2;
                        s.sum[1] += static_cast<std::int64_t>(len) * y;
                        s.sum[2] += static_cast<std::int64_t>(len) * z;
                        s.lo[0] = std::min(s.lo[0], x);
                        s.hi[0] = std::max(s.hi[0], end - 1);
                        s.lo[1] = std::min(s.lo[1], y);
                        s.hi[1] = std::max(s.hi[1], y);
                        s.lo[2] = std::min(s.lo[2], z);
                        s.hi[2] = std::max(s.hi[2], z);
                    }
                    x = end;
                }
            }
        }
        if (local.empty()) {
            return;
        }
        // pass 2 — the voxel of each run nearest that object's block mean
        for (int z = 0; z < sz; ++z) {
            for (int y = 0; y < sy; ++y) {
                const auto * row = voxels + static_cast<std::size_t>(z) * planeStride + static_cast<std::size_t>(y) * rowStride;
                for (int x = 0; x < sx; ) {
                    const auto id = row[x];
                    int end = x + 1;
                    while (end < sx && row[end] == id) { ++end; }
                    if (id != backgroundId) {
                        auto & s = local[id];
                        const auto n = static_cast<double>(s.count);
                        const double cx = s.sum[0] / n, cy = s.sum[1] / n, cz = s.sum[2] / n;
                        // best x within this run is closed form; y and z are fixed by the row
                        const auto bx = std::min(end - 1, std::max(x, static_cast<int>(std::lround(cx))));
                        const auto dx = (bx - cx) * geom.nmPerVoxel[0];
                        const auto dy = (y - cy) * geom.nmPerVoxel[1];
                        const auto dz = (z - cz) * geom.nmPerVoxel[2];
                        const auto d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < s.bestDist2 || (d2 == s.bestDist2 && lexLess(bx, y, z, s.best))) {
                            s.bestDist2 = d2;
                            s.best[0] = bx;
                            s.best[1] = y;
                            s.best[2] = z;
                        }
                    }
                    x = end;
                }
            }
        }
        merge(geom);
    }

    /* Discovery order, append-only. An id's position is fixed the first time it is seen;
     * only its contents keep changing as later blocks arrive. */
    const std::vector<Record> & records() const { return recs; }

    /* Which records the last block changed. The list a scan feeds is live, and a block can
     * revise an object discovered ten minutes ago, so a caller repainting a table needs to
     * know which rows went stale — resending everything would be 40 MB a few times a
     * second on a large volume. Bounded by the number of distinct ids in one block. */
    const std::vector<std::size_t> & touched() const { return touchedIdx; }

    static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();
    std::size_t indexOfId(const std::uint64_t id) const {
        const auto it = index.find(id);
        return it == std::end(index) ? npos : it->second;
    }

    // Rehydrates from the cache. The order in the file is the order that comes back.
    void adopt(std::vector<Record> && loaded) {
        recs = std::move(loaded);
        index.clear();
        index.reserve(recs.size() * 2);
        for (std::size_t i = 0; i < recs.size(); ++i) {
            index.emplace(recs[i].id, i);
        }
        truncatedFlag = false;
        rejectedCount = 0;
        touchedIdx.clear();
    }

    void clear() {
        recs.clear();
        index.clear();
        local.clear();
        touchedIdx.clear();
        truncatedFlag = false;
        rejectedCount = 0;
    }

    /* A ceiling on how many distinct ids to hold, so a pathologically over-segmented
     * volume cannot exhaust memory. Past it the tally keeps *updating* ids it already
     * knows — their centroids stay correct — and only stops taking on new ones, which is
     * reported rather than done quietly. */
    void setIdCap(const std::size_t cap) { idCap = cap; }
    std::size_t idCap_() const { return idCap; }
    bool truncated() const { return truncatedFlag; }
    std::uint64_t rejected() const { return rejectedCount; }

private:
    struct Local {
        std::uint64_t count{0};
        std::int64_t sum[3]{};
        int lo[3]{std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
        int hi[3]{std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
        int best[3]{0, 0, 0};
        double bestDist2{std::numeric_limits<double>::max()};
    };

    // A tiebreak on the coordinate itself, so which block got there first cannot matter.
    template<typename A, typename B>
    static bool lexLess(const A x, const A y, const A z, const B (& other)[3]) {
        if (x != static_cast<A>(other[0])) { return x < static_cast<A>(other[0]); }
        if (y != static_cast<A>(other[1])) { return y < static_cast<A>(other[1]); }
        return z < static_cast<A>(other[2]);
    }

    void merge(const CubeGeometry & geom) {
        for (const auto & pair : local) {
            const auto id = pair.first;
            const auto & s = pair.second;
            auto idx = indexOfId(id);
            if (idx == npos) {
                if (recs.size() >= idCap) {
                    truncatedFlag = true;
                    ++rejectedCount;
                    continue;
                }
                idx = recs.size();
                Record fresh;
                fresh.id = id;
                for (int a = 0; a < 3; ++a) {
                    fresh.bboxMin[a] = std::numeric_limits<std::int32_t>::max();
                    fresh.bboxMax[a] = std::numeric_limits<std::int32_t>::min();
                }
                recs.push_back(fresh);
                index.emplace(id, idx);
            }
            touchedIdx.push_back(idx);
            auto & r = recs[idx];
            r.voxels += s.count;
            std::int32_t candidate[3];
            for (int a = 0; a < 3; ++a) {
                r.sum[a] += static_cast<std::int64_t>(s.count) * geom.origin[a]
                          + static_cast<std::int64_t>(geom.step[a]) * s.sum[a];
                r.bboxMin[a] = std::min(r.bboxMin[a], static_cast<std::int32_t>(geom.origin[a] + geom.step[a] * s.lo[a]));
                r.bboxMax[a] = std::max(r.bboxMax[a], static_cast<std::int32_t>(geom.origin[a] + geom.step[a] * s.hi[a]));
                candidate[a] = static_cast<std::int32_t>(geom.origin[a] + geom.step[a] * s.best[a]);
            }
            const auto cubeVoxels = static_cast<std::uint32_t>(std::min<std::uint64_t>(s.count, std::numeric_limits<std::uint32_t>::max()));
            const bool better = cubeVoxels > r.bestCubeVoxels
                    || (cubeVoxels == r.bestCubeVoxels
                        && lexLess(candidate[0], candidate[1], candidate[2], r.rep));
            if (better) {
                r.bestCubeVoxels = cubeVoxels;
                for (int a = 0; a < 3; ++a) { r.rep[a] = candidate[a]; }
            }
        }
    }

    std::vector<Record> recs;
    std::unordered_map<std::uint64_t, std::size_t> index;
    std::unordered_map<std::uint64_t, Local> local;// reused between blocks, keeps its buckets
    std::vector<std::size_t> touchedIdx;
    std::size_t idCap{500000};
    bool truncatedFlag{false};
    std::uint64_t rejectedCount{0};
};

/* Sweep order over the block grid.
 *
 * The order blocks are visited in *is* the order the list ends up in, so it has to be
 * fixed, resumable, and — because the whole point is to walk the result with a key —
 * spatially coherent. Raster order fails the last one: a row of the largest dataset is 55
 * blocks, so every row wrap is a 14 µm leap and consecutive list entries are nowhere near
 * each other. Z-order gives octree locality in all three axes at once, and collapses the
 * resume point to a single integer.
 *
 * Interleaving three 21-bit axes fits in 63 bits, which is far more grid than any dataset
 * has. The grid is padded to a power of two and out-of-range indices are skipped; that
 * costs a few shifts per skipped index and nothing else. */
inline std::uint64_t mortonEncode3(const std::uint32_t x, const std::uint32_t y, const std::uint32_t z) {
    const auto spread = [](std::uint64_t v) {
        v &= 0x1fffffu;                         // 21 bits
        v = (v | (v << 32)) & 0x1f00000000ffffull;
        v = (v | (v << 16)) & 0x1f0000ff0000ffull;
        v = (v | (v <<  8)) & 0x100f00f00f00f00full;
        v = (v | (v <<  4)) & 0x10c30c30c30c30c3ull;
        v = (v | (v <<  2)) & 0x1249249249249249ull;
        return v;
    };
    return spread(x) | (spread(y) << 1) | (spread(z) << 2);
}

inline void mortonDecode3(const std::uint64_t code, std::uint32_t & x, std::uint32_t & y, std::uint32_t & z) {
    const auto compact = [](std::uint64_t v) {
        v &= 0x1249249249249249ull;
        v = (v | (v >>  2)) & 0x10c30c30c30c30c3ull;
        v = (v | (v >>  4)) & 0x100f00f00f00f00full;
        v = (v | (v >>  8)) & 0x1f0000ff0000ffull;
        v = (v | (v >> 16)) & 0x1f00000000ffffull;
        v = (v | (v >> 32)) & 0x1fffffull;
        return static_cast<std::uint32_t>(v);
    };
    x = compact(code);
    y = compact(code >> 1);
    z = compact(code >> 2);
}

// How far the Z-order walk has to run to have covered a grid of this size.
inline std::uint64_t mortonSpan(const std::uint32_t nx, const std::uint32_t ny, const std::uint32_t nz) {
    const auto bits = [](std::uint32_t n) {
        std::uint32_t b = 0;
        while ((1u << b) < n) { ++b; }
        return b;
    };
    const auto b = std::max(bits(nx), std::max(bits(ny), bits(nz)));
    return b == 0 ? 1 : (1ull << (3 * b));
}

/* How far the sweep has got, in a form a resume can trust.
 *
 * Several blocks are in flight at once and they finish out of order, so "how far have we
 * got" cannot be a count. It is the lowest index not yet accounted for: blocks that
 * finished ahead of it wait in a set until it catches up, and that set is bounded by how
 * many requests are allowed in flight, so it stays tiny.
 *
 * The failure this prevents is the quiet one. If the cursor ran ahead of a block that was
 * still in flight when the sweep stopped — or that failed — then resuming would step over
 * it, and the objects in it would be missing from a list that claims to be complete.
 * Nothing downstream could tell. */
class SweepCursor {
public:
    // `covers` says whether an index is inside the real grid; the walk is over a padded one.
    template<typename Covers>
    void init(const std::uint64_t first, const std::uint64_t span, Covers covers) {
        at = first;
        end = span;
        done.clear();
        skipPadding(covers);
        issued = at;// must follow skipPadding, and must not survive a previous walk
    }

    std::uint64_t position() const { return at; }
    bool finished() const { return at >= end; }
    // Everything has been handed out; whatever is left is outstanding, not unstarted.
    bool exhausted() const { return issued >= end; }
    std::size_t pending() const { return done.size(); }

    // Hands out the next index to fetch, or nothing when the walk has run out.
    template<typename Covers>
    bool next(std::uint64_t & code, Covers covers) {
        if (issued < at) { issued = at; }
        while (issued < end && !covers(issued)) { ++issued; }
        if (issued >= end) {
            return false;
        }
        code = issued++;
        return true;
    }

    template<typename Covers>
    void complete(const std::uint64_t code, Covers covers) {
        if (code < at) {
            return;// already accounted for; a duplicate completion must not move anything
        }
        done.insert(code);
        skipPadding(covers);
        while (at < end) {
            const auto it = done.find(at);
            if (it == std::end(done)) {
                break;
            }
            done.erase(it);
            ++at;
            skipPadding(covers);
        }
    }

private:
    template<typename Covers>
    void skipPadding(Covers covers) {
        while (at < end && !covers(at)) { ++at; }
    }

    std::uint64_t at{0}, end{0}, issued{0};
    std::unordered_set<std::uint64_t> done;
};

// Blocks needed to cover `extent` mag-1 voxels, when one block voxel spans `step` of them.
inline std::uint32_t blocksAlong(const std::int64_t extent, const std::int32_t cubeShape, const std::int32_t step) {
    if (extent <= 0 || cubeShape <= 0 || step <= 0) {
        return 1;
    }
    const auto voxels = extent / step;// the coarse level has this many voxels along the axis
    const auto blocks = (voxels + cubeShape - 1) / cubeShape;
    return static_cast<std::uint32_t>(std::max<std::int64_t>(1, blocks));
}

}
