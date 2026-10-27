// Standalone test for segmentation/blocktour.h — see tests/README.md.
#include "segmentation/blocktour.h"

#include <cstdio>
#include <random>
#include <set>

static int failures = 0;
static void check(const bool ok, const char * what) {
    if (!ok) {
        ++failures;
        std::printf("  FAIL: %s\n", what);
    }
}

// Loader::Worker::DcoiFromPos, transcribed: the window is global2cube(pos ± halfFOV), with
// halfFOV = cube2global({1,1,1}) * (M - 1) / 2 in integer arithmetic
static void dcoiWindow(const blocktour::Block & centre, const int M, const int extent[3], blocktour::Block & lo, blocktour::Block & hi) {
    for (int a = 0; a < 3; ++a) {
        const long long pos = static_cast<long long>(centre[a]) * extent[a] + extent[a] / 2;
        const long long half = static_cast<long long>(extent[a]) * (M - 1) / 2;
        lo[a] = blocktour::floorDiv(pos - half, extent[a]);
        hi[a] = blocktour::floorDiv(pos + half, extent[a]);
    }
}

int main() {
    std::printf("1. the window matches the loader's, and holds M blocks per axis\n");
    for (const int M : {2, 3, 4, 5, 7}) {
        for (const auto & ext : {std::array<int, 3>{128, 128, 128}, std::array<int, 3>{256, 512, 128}}) {
            for (int c = -5; c <= 5; ++c) {
                const blocktour::Block centre{c, 2 * c, -c};
                blocktour::Block lo, hi, rlo, rhi;
                blocktour::windowAround(centre, M, ext.data(), lo, hi);
                dcoiWindow(centre, M, ext.data(), rlo, rhi);
                check(lo == rlo && hi == rhi, "window equals the transcribed DcoiFromPos bounds");
                for (int a = 0; a < 3; ++a) {
                    check(hi[a] - lo[a] + 1 == M, "M blocks per axis");
                }
            }
        }
    }

    std::printf("2. every needed block is in exactly one window, which really holds it\n");
    std::mt19937 rng(7);
    for (int trial = 0; trial < 300; ++trial) {
        const int M = 2 + trial % 6;
        const int ext[3]{128, 128, 128};
        std::uniform_int_distribution<int> span(1, 14), off(-20, 20), keep(0, 3);
        const int sx = span(rng), sy = span(rng), sz = 1 + span(rng) % 3, ox = off(rng), oy = off(rng), oz = off(rng);
        std::vector<blocktour::Block> needed;
        for (int z = 0; z < sz; ++z) for (int y = 0; y < sy; ++y) for (int x = 0; x < sx; ++x) {
            if (trial % 2 == 0 || keep(rng) != 0) {// odd trials: a sparse set with gaps
                needed.push_back({ox + x, oy + y, oz + z});
            }
        }
        const auto tour = blocktour::plan(needed, M, ext);
        std::set<blocktour::Block> seen;
        for (const auto & w : tour) {
            check(!w.blocks.empty(), "no empty window");
            blocktour::Block lo, hi;
            blocktour::windowAround(w.centre, M, ext, lo, hi);
            check(lo == w.lo && hi == w.hi, "stored window is the centre's window");
            for (const auto & b : w.blocks) {
                check(w.contains(b), "window holds its blocks");
                check(seen.insert(b).second, "no block in two windows");
            }
        }
        check(seen.size() == std::set<blocktour::Block>(needed.begin(), needed.end()).size(), "every needed block assigned");
        if (trial % 2 == 0) {// a full box: the window count is the tiling count
            const auto tiles = [M](const int s){ return (s + M - 1) / M; };
            check(tour.size() == static_cast<std::size_t>(tiles(sx) * tiles(sy) * tiles(sz)), "ceil(span/M) windows per axis for a full box");
        }
        for (std::size_t i = 1; i < tour.size(); ++i) {
            int dist = 0;
            for (int a = 0; a < 3; ++a) {
                dist += std::abs(tour[i].centre[a] - tour[i - 1].centre[a]);
            }
            check(trial % 2 == 1 || dist <= M, "a full box is walked one window step at a time");
        }
    }

    std::printf("3. crop 126: 6 x 4 x 1 blocks\n");
    {
        std::vector<blocktour::Block> needed;
        for (int y = 0; y < 6; ++y) for (int z = 0; z < 4; ++z) needed.push_back({0, y, z});
        const int ext[3]{128, 128, 128};
        check(blocktour::plan(needed, 7, ext).size() == 1, "M = 7: one move");
        check(blocktour::plan(needed, 3, ext).size() == 4, "M = 3: four moves");
    }

    std::printf("4. degenerate input\n");
    {
        const int ext[3]{128, 128, 128};
        const int bad[3]{0, 128, 128};
        check(blocktour::plan({}, 3, ext).empty(), "nothing needed, no tour");
        check(blocktour::plan({{0, 0, 0}}, 0, ext).empty(), "M = 0");
        check(blocktour::plan({{0, 0, 0}}, 3, bad).empty(), "zero extent");
        check(blocktour::plan({{0, 0, 0}}, 1, ext).size() == 1, "M = 1");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
