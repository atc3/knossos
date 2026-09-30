// Standalone test for segmentation/planarwrite.h — which blocks a slice of an interpolation
// gets written into, and where in each. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/planarwrite_test planarwrite_test.cpp && /tmp/planarwrite_test
//
// The point of this file is the equivalence check in "skipping blocks changes nothing":
// leaving out blocks the mask does not reach is a large speed-up in a position where a
// mistake does not crash or warn, it quietly leaves part of the object unwritten. So the
// fast plan is checked against the exhaustive one by writing a simulated volume through
// both and comparing, rather than by reasoning about it.
#include "segmentation/planarwrite.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}
void section(const char * title) { std::printf("%s\n", title); }

using Voxel = std::tuple<int, int, int>;

// A mask on a lattice, with a shape the caller paints in.
template<typename Shape>
SISlice makeMask(const int uMin, const int vMin, const int uStep, const int vStep,
                 const int uSize, const int vSize, Shape shape) {
    SISlice m;
    m.uMin = uMin; m.vMin = vMin; m.uStep = uStep; m.vStep = vStep;
    m.uSize = uSize; m.vSize = vSize;
    m.mask.assign(static_cast<std::size_t>(uSize) * vSize, 0);
    for (int v = 0; v < vSize; ++v) {
        for (int u = 0; u < uSize; ++u) {
            if (shape(u, v)) { m.mask[static_cast<std::size_t>(v) * uSize + u] = 1; }
        }
    }
    return m;
}

/* Mirrors what processRegion() does inside one block: it steps the magnification lattice
 * from the *block's* origin, not the region's, and caps each coordinate to the region — see
 * the inner loop in segmentation/cubeloader.cpp. Getting that wrong is the whole reason the
 * mask lattice and the block grid have to be reasoned about together. */
template<typename Visit>
void forEachVoxelInBlock(const planarwrite::Block & block, const int cubeShape[3],
                         const int step[3], Visit visit) {
    int origin[3];
    for (int a = 0; a < 3; ++a) { origin[a] = block.cube[a] * cubeShape[a] * step[a]; }
    for (int lz = 0; lz < cubeShape[2]; ++lz)
    for (int ly = 0; ly < cubeShape[1]; ++ly)
    for (int lx = 0; lx < cubeShape[0]; ++lx) {
        const int local[3]{lx, ly, lz};
        int pos[3];
        bool inRegion = true;
        for (int a = 0; a < 3; ++a) {
            const auto raw = origin[a] + step[a] * local[a];
            pos[a] = std::max(block.first[a], std::min(raw, block.last[a]));
            inRegion = inRegion && raw >= block.first[a] - step[a] && raw <= block.last[a] + step[a];
        }
        if (inRegion) { visit(pos[0], pos[1], pos[2]); }
    }
}

// Writes a simulated volume by running the plan, the way writeMaskAt does.
std::set<Voxel> simulateWrite(const planarwrite::Plan & plan, const SISlice & mask,
                              const planarwrite::Axes & axes,
                              const int cubeShape[3], const int step[3]) {
    std::set<Voxel> written;
    for (const auto & block : plan.blocks) {
        forEachVoxelInBlock(block, cubeShape, step, [&](const int x, const int y, const int z) {
            const int pos[3]{x, y, z};
            if (mask.at(mask.uIndexOf(pos[axes.u]), mask.vIndexOf(pos[axes.v])) != 0) {
                written.emplace(x, y, z);
            }
        });
    }
    return written;
}
}

int main() {
    const planarwrite::Axes xy{2, 0, 1};// slices along z, the usual case

    section("the plan covers the mask");
    {
        const int cubeShape[3]{8, 8, 8};
        const int step[3]{1, 1, 1};
        const int areaMin[3]{0, 0, 0};
        const int areaMax[3]{1000, 1000, 1000};
        // a disc straddling several blocks, deliberately not block-aligned
        const auto mask = makeMask(5, 3, 1, 1, 40, 40, [](int u, int v){
            const auto dx = u - 20, dy = v - 20; return dx*dx + dy*dy <= 15*15; });
        const auto full = planarwrite::plan(mask, 7, xy, cubeShape, step, areaMin, areaMax, false);
        const auto fast = planarwrite::plan(mask, 7, xy, cubeShape, step, areaMin, areaMax, true);

        check(!full.blocks.empty() && !fast.blocks.empty(), "both plans have blocks to write");
        check(fast.blocks.size() < full.blocks.size(), "the fast plan really does leave blocks out");

        // every set voxel of the mask must fall inside exactly one block of the plan
        auto coverage = [&](const planarwrite::Plan & p) {
            std::map<Voxel, int> times;
            for (int v = 0; v < mask.vSize; ++v) {
                for (int u = 0; u < mask.uSize; ++u) {
                    if (mask.at(u, v) == 0) { continue; }
                    int pos[3];
                    pos[xy.depth] = 7;
                    pos[xy.u] = mask.uCoordOf(u);
                    pos[xy.v] = mask.vCoordOf(v);
                    int n = 0;
                    for (const auto & b : p.blocks) {
                        bool in = true;
                        for (int a = 0; a < 3; ++a) { in = in && pos[a] >= b.first[a] && pos[a] <= b.last[a]; }
                        n += in ? 1 : 0;
                    }
                    times[{pos[0], pos[1], pos[2]}] = n;
                }
            }
            return times;
        };
        const auto fullCover = coverage(full);
        const auto fastCover = coverage(fast);
        bool onceEach = !fullCover.empty();
        for (const auto & [voxel, n] : fullCover) { (void)voxel; onceEach = onceEach && n == 1; }
        check(onceEach, "the exhaustive plan covers every painted voxel exactly once");
        check(fastCover == fullCover, "and the fast plan covers exactly the same voxels, also once each");

        // the regions must not overlap, or a voxel would be written twice
        bool disjoint = true;
        for (std::size_t i = 0; i < full.blocks.size() && disjoint; ++i) {
            for (std::size_t j = i + 1; j < full.blocks.size() && disjoint; ++j) {
                bool overlap = true;
                for (int a = 0; a < 3; ++a) {
                    overlap = overlap && full.blocks[i].first[a] <= full.blocks[j].last[a]
                                      && full.blocks[j].first[a] <= full.blocks[i].last[a];
                }
                disjoint = !overlap;
            }
        }
        check(disjoint, "no two blocks claim the same voxel");
    }

    section("skipping blocks changes nothing");
    {
        /* The equivalence check. Randomised over magnifications, block shapes, mask origins
         * and slice orientations, writing a volume through both plans and diffing it. */
        std::mt19937 rng{20260930};
        bool identical = true, fasterOrSame = true, everSkipped = false;
        int trials = 0;
        for (int trial = 0; trial < 300 && identical; ++trial) {
            std::uniform_int_distribution<int> magPick{0, 2};
            std::uniform_int_distribution<int> shapePick{2, 7};
            std::uniform_int_distribution<int> sizePick{1, 34};
            std::uniform_int_distribution<int> originPick{-20, 60};
            std::uniform_int_distribution<int> orient{0, 2};

            const int m = 1 << magPick(rng);
            const int step[3]{m, m, m};
            const int cubeShape[3]{shapePick(rng), shapePick(rng), shapePick(rng)};
            const int areaMin[3]{0, 0, 0};
            const int areaMax[3]{4000, 4000, 4000};
            const planarwrite::Axes axes = orient(rng) == 0 ? planarwrite::Axes{2, 0, 1}
                                         : orient(rng) == 1 ? planarwrite::Axes{0, 1, 2}
                                                            : planarwrite::Axes{1, 0, 2};
            /* Three kinds of shape, because density is what decides whether a mistake
             * here shows up. Blobs leave whole blocks empty, which is what the skip is
             * for. Salt and pepper puts a *single* isolated voxel in a block, which is the
             * case that catches a test for "reached" that is any stricter than "at least
             * one". A solid fill is the control, where nothing may be skipped. */
            std::uniform_int_distribution<int> kind{0, 2};
            std::uniform_int_distribution<int> coin{0, 9};
            const auto blobs = coin(rng);
            const auto shapeKind = kind(rng);
            std::uniform_int_distribution<int> speck{0, 23};
            auto pepper = [&](int, int){ return speck(rng) == 0; };
            const auto mask = makeMask(originPick(rng) * m, originPick(rng) * m, m, m,
                                       sizePick(rng), sizePick(rng), [&](int u, int v){
                if (shapeKind == 0) { return ((u / 5) % 2 == 0) && ((v / 4) % 2 == 0) && (u + v) % (2 + blobs % 3) != 1; }
                if (shapeKind == 1) { return pepper(u, v); }
                return true; });
            std::uniform_int_distribution<int> depthPick{0, 60};
            const int depth = depthPick(rng) * m;

            const auto full = planarwrite::plan(mask, depth, axes, cubeShape, step, areaMin, areaMax, false);
            const auto fast = planarwrite::plan(mask, depth, axes, cubeShape, step, areaMin, areaMax, true);
            everSkipped = everSkipped || fast.blocks.size() < full.blocks.size();
            fasterOrSame = fasterOrSame && fast.blocks.size() <= full.blocks.size();

            identical = simulateWrite(full, mask, axes, cubeShape, step)
                     == simulateWrite(fast, mask, axes, cubeShape, step);
            ++trials;
        }
        check(identical, "300 randomised plans: the simulated volume is byte-identical either way");
        check(fasterOrSame, "the fast plan is never larger than the exhaustive one");
        check(everSkipped, "and the cases really did exercise skipping");
        check(trials > 250, "enough trials ran");
    }

    section("a block holding a single voxel");
    {
        // the least a block can hold and still need writing; a "reached" test that asks for
        // more than one set voxel would skip it and lose that voxel without a word
        const int cubeShape[3]{8, 8, 8};
        const int step[3]{1, 1, 1};
        const int areaMin[3]{0, 0, 0};
        const int areaMax[3]{500, 500, 500};
        for (const int m : {1, 2, 4}) {
            const int stepM[3]{m, m, m};
            // one voxel, alone, in the middle of a mask spanning several blocks
            const auto mask = makeMask(0, 0, m, m, 24, 24, [](int u, int v){ return u == 11 && v == 13; });
            const auto fast = planarwrite::plan(mask, 3 * m, xy, cubeShape, stepM, areaMin, areaMax, true);
            const auto full = planarwrite::plan(mask, 3 * m, xy, cubeShape, stepM, areaMin, areaMax, false);
            const auto wroteFast = simulateWrite(fast, mask, xy, cubeShape, stepM);
            const auto wroteFull = simulateWrite(full, mask, xy, cubeShape, stepM);
            check(fast.blocks.size() == 1, std::string{"exactly one block is planned at "} + std::to_string(m) + "x");
            check(!wroteFast.empty() && wroteFast == wroteFull,
                  std::string{"and the lone voxel is written, same as exhaustively, at "} + std::to_string(m) + "x");
        }
    }

    section("the movement area");
    {
        const int cubeShape[3]{4, 4, 4};
        const int step[3]{1, 1, 1};
        const auto mask = makeMask(0, 0, 1, 1, 32, 32, [](int, int){ return true; });
        // areaMax is exclusive: with 20 the last writable coordinate is 19
        const int areaMin[3]{0, 0, 0};
        const int areaMax[3]{20, 20, 20};
        const auto p = planarwrite::plan(mask, 5, xy, cubeShape, step, areaMin, areaMax, true);
        int maxU = -1, maxV = -1;
        for (const auto & b : p.blocks) {
            maxU = std::max(maxU, b.last[xy.u]);
            maxV = std::max(maxV, b.last[xy.v]);
        }
        check(maxU == 19 && maxV == 19, "the last coordinate written is areaMax - 1, not areaMax");
        check(p.clamped, "and the plan says it was clamped");

        // a plane outside the area entirely must produce nothing, not a stray block
        const int elsewhere[3]{100, 100, 100};
        const int elsewhereMax[3]{200, 200, 200};
        const auto none = planarwrite::plan(mask, 5, xy, cubeShape, step, elsewhere, elsewhereMax, true);
        check(none.blocks.empty(), "a plane the movement area excludes yields no blocks at all");
    }

    section("degenerate input");
    {
        const int cubeShape[3]{4, 4, 4};
        const int step[3]{1, 1, 1};
        const int areaMin[3]{0, 0, 0};
        const int areaMax[3]{100, 100, 100};
        const auto solid = makeMask(0, 0, 1, 1, 8, 8, [](int, int){ return true; });

        SISlice blank;
        check(planarwrite::plan(blank, 0, xy, cubeShape, step, areaMin, areaMax, true).blocks.empty(),
              "an unsized mask yields no blocks");
        const auto empty = makeMask(0, 0, 1, 1, 8, 8, [](int, int){ return false; });
        check(planarwrite::plan(empty, 0, xy, cubeShape, step, areaMin, areaMax, true).blocks.empty(),
              "a mask with nothing painted yields no blocks when skipping");
        check(!planarwrite::plan(empty, 0, xy, cubeShape, step, areaMin, areaMax, false).blocks.empty(),
              "but the exhaustive plan still walks its box");

        const int zeroShape[3]{0, 4, 4};
        check(planarwrite::plan(solid, 0, xy, zeroShape, step, areaMin, areaMax, true).blocks.empty(),
              "a zero block shape is refused");
        const int zeroStep[3]{1, 0, 1};
        check(planarwrite::plan(solid, 0, xy, cubeShape, zeroStep, areaMin, areaMax, true).blocks.empty(),
              "a zero magnification step is refused");
        check(planarwrite::plan(solid, 0, planarwrite::Axes{0, 0, 1}, cubeShape, step, areaMin, areaMax, true).blocks.empty(),
              "a repeated axis is refused rather than producing a nonsense plan");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
