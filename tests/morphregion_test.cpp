// Standalone test for segmentation/morphregion.h — which box a morphology run reads, which
// it writes, and how both sit on the magnification lattice. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/morphregion_test morphregion_test.cpp && /tmp/morphregion_test
//
// The central check is "every reported coordinate is exactly addressable", which simulates
// processRegion()'s own traversal — stepping the lattice from each block's origin and
// capping the reported coordinate to the region, which is what it actually does — and
// requires every coordinate it hands back to land on a distinct, in-range index of the read
// grid. That is the property the first version of the driver broke at magnification 2 and
// above, where it quietly wrote one plane of the region using the neighbouring cell's
// answer. At magnification 1 the step is 1 and the whole class of mistake is unreachable,
// so the magnifications here matter more than the shapes do.
#include "segmentation/morphregion.h"

#include <array>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}
void section(const char * title) { std::printf("%s\n", title); }

using morphregion::Box;
using morphregion::Plan;

int cap(const int v, const int lo, const int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* What processRegion() reports for a region, block by block.
 *
 * Mirrors segmentation/cubeloader.cpp: the cube range comes from global2cube of the
 * region's ends, the lattice is stepped from the *block's* origin rather than the region's,
 * the local start and end are the region's ends capped into the block, and the coordinate
 * handed to the visitor is capped to [first, last + 1]. That last cap is the part that
 * matters — it is why an off-lattice region bound produces an off-lattice report. */
std::vector<std::array<int, 3>> simulateTraversal(const Box & region, const int cubeShape[3], const int step[3]) {
    std::vector<std::array<int, 3>> reported;
    int cubeBegin[3], cubeEnd[3];
    for (int a = 0; a < 3; ++a) {
        const auto extentPerCube = cubeShape[a] * step[a];
        cubeBegin[a] = morphregion::floorDiv(region.first[a], extentPerCube);
        cubeEnd[a] = morphregion::floorDiv(region.last[a], extentPerCube) + 1;
    }
    for (int cz = cubeBegin[2]; cz < cubeEnd[2]; ++cz)
    for (int cy = cubeBegin[1]; cy < cubeEnd[1]; ++cy)
    for (int cx = cubeBegin[0]; cx < cubeEnd[0]; ++cx) {
        const int cube[3]{cx, cy, cz};
        int origin[3], localStart[3], localEnd[3];
        for (int a = 0; a < 3; ++a) {
            origin[a] = cube[a] * cubeShape[a] * step[a];
            const auto cubeLast = origin[a] + cubeShape[a] * step[a] - 1;
            const auto cappedFirst = cap(region.first[a], origin[a], cubeLast);
            const auto cappedLast = cap(region.last[a], origin[a], cubeLast);
            localStart[a] = (cappedFirst - origin[a]) / step[a];
            localEnd[a] = (cappedLast - origin[a]) / step[a];
        }
        for (int lz = localStart[2]; lz <= localEnd[2]; ++lz)
        for (int ly = localStart[1]; ly <= localEnd[1]; ++ly)
        for (int lx = localStart[0]; lx <= localEnd[0]; ++lx) {
            const int local[3]{lx, ly, lz};
            std::array<int, 3> pos{};
            for (int a = 0; a < 3; ++a) {
                const auto onLattice = origin[a] + local[a] * step[a];
                pos[a] = cap(onLattice, region.first[a], region.last[a] + 1);
            }
            reported.push_back(pos);
        }
    }
    return reported;
}
}

int main() {
    section("the read origin is on the global lattice and the write box is on the read grid");
    {
        std::mt19937 rng(20261005);
        bool readSnapped = true, writeOnGrid = true, writeInsideRead = true, ran = false;
        for (int trial = 0; trial < 300; ++trial) {
            const auto mag = 1 << (rng() % 4);// 1, 2, 4, 8
            const int step[3]{mag, mag, static_cast<int>(mag * (1 + rng() % 2))};
            const int centre[3]{static_cast<int>(rng() % 4000), static_cast<int>(rng() % 4000), static_cast<int>(rng() % 4000)};
            const int pad[3]{static_cast<int>(rng() % 6), static_cast<int>(rng() % 6), static_cast<int>(rng() % 6)};
            const int limitMin[3]{static_cast<int>(rng() % 500), static_cast<int>(rng() % 500), static_cast<int>(rng() % 500)};
            const int limitMax[3]{limitMin[0] + static_cast<int>(rng() % 4000), limitMin[1] + static_cast<int>(rng() % 4000), limitMin[2] + static_cast<int>(rng() % 4000)};
            const auto extent = 1 + static_cast<int>(rng() % 200);
            const auto p = morphregion::plan(centre, extent, step, pad, limitMin, limitMax);
            if (p.empty) {
                continue;
            }
            ran = true;
            for (int a = 0; a < 3; ++a) {
                if (p.read.first[a] % step[a] != 0) { readSnapped = false; }
                if ((p.write.first[a] - p.read.first[a]) % step[a] != 0) { writeOnGrid = false; }
                if ((p.write.last[a] - p.read.first[a]) % step[a] != 0) { writeOnGrid = false; }
                if (p.write.first[a] < p.read.first[a] || p.write.last[a] > p.read.last[a]) { writeInsideRead = false; }
                if (p.write.first[a] < limitMin[a] || p.write.last[a] > limitMax[a]) { writeInsideRead = false; }
            }
        }
        check(ran, "the randomised cases produced plans to check");
        check(readSnapped, "the read origin is always a multiple of the step");
        check(writeOnGrid, "both write bounds are whole steps from the read origin");
        check(writeInsideRead, "the write box is inside the read box and inside the limits");
    }

    section("every coordinate the traversal reports is exactly addressable");
    {
        std::mt19937 rng(4242);
        bool allAddressable = true, allDistinct = true, ran = false;
        int casesChecked = 0;
        for (int trial = 0; trial < 400; ++trial) {
            const auto mag = 1 << (rng() % 4);
            const int step[3]{mag, mag, mag};
            const int cubeShape[3]{8, 8, 8};// small, so several blocks are crossed
            const int centre[3]{static_cast<int>(rng() % 400), static_cast<int>(rng() % 400), static_cast<int>(rng() % 400)};
            const int pad[3]{static_cast<int>(rng() % 4), static_cast<int>(rng() % 4), static_cast<int>(rng() % 4)};
            const int limitMin[3]{0, 0, 0};
            const int limitMax[3]{511, 511, 511};
            const auto extent = 1 + static_cast<int>(rng() % 40);
            const auto p = morphregion::plan(centre, extent, step, pad, limitMin, limitMax);
            if (p.empty) {
                continue;
            }
            ran = true;
            ++casesChecked;
            // both boxes are traversed: the read box is what is read into the dense grid,
            // the write box is what is written back out of it
            for (const auto * box : {&p.read, &p.write}) {
                std::set<std::size_t> seen;
                for (const auto & pos : simulateTraversal(*box, cubeShape, step)) {
                    const int at[3]{pos[0], pos[1], pos[2]};
                    if (!p.read.onLattice(at) || !p.read.inRange(at)) {
                        allAddressable = false;
                        continue;
                    }
                    const auto idx = p.read.index(at);
                    if (idx >= p.read.voxels()) {
                        allAddressable = false;
                    }
                    if (!seen.insert(idx).second) {
                        allDistinct = false;
                    }
                }
            }
        }
        check(ran && casesChecked > 200, "the traversal was simulated over " + std::to_string(casesChecked) + " plans");
        check(allAddressable, "every reported coordinate is on the read lattice and in range");
        check(allDistinct, "and no two reported coordinates land on the same index");
    }

    section("the apron");
    {
        const int step[3]{1, 1, 1};
        const int pad[3]{3, 3, 3};
        const int centre[3]{1000, 1000, 1000};
        const int limitMin[3]{0, 0, 0};
        const int limitMax[3]{2000, 2000, 2000};
        const auto roomy = morphregion::plan(centre, 100, step, pad, limitMin, limitMax);
        check(!roomy.apronIncomplete, "with room on every side the apron is complete");
        check(roomy.read.first[0] == roomy.write.first[0] - 3 && roomy.read.last[0] == roomy.write.last[0] + 3,
              "and is exactly the requested depth");
        check(!roomy.clipped, "and the write box is the requested extent");
        check(roomy.write.dim[0] == 101, "which for an extent of 100 is 101 voxels inclusive");

        // hard against the lower limit: the apron cannot be read and that has to be said
        const int corner[3]{1, 1, 1};
        const auto tight = morphregion::plan(corner, 100, step, pad, limitMin, limitMax);
        check(tight.apronIncomplete, "against a limit the apron is reported incomplete");
        check(tight.clipped, "and the write box is reported clipped");
        check(tight.write.first[0] >= limitMin[0], "but it never runs past the limit");
    }

    section("magnification 1 leaves the requested box alone");
    {
        const int step[3]{1, 1, 1};
        const int pad[3]{2, 2, 2};
        const int centre[3]{500, 600, 700};
        const int limitMin[3]{0, 0, 0};
        const int limitMax[3]{2000, 2000, 2000};
        const auto p = morphregion::plan(centre, 64, step, pad, limitMin, limitMax);
        check(p.write.first[0] == 468 && p.write.last[0] == 532, "the write box is the box that was asked for");
        check(p.read.first[1] == 566 && p.read.last[1] == 634, "and the read box is it plus the apron");
    }

    section("degenerate input");
    {
        const int step[3]{1, 1, 1};
        const int pad[3]{1, 1, 1};
        const int centre[3]{10, 10, 10};
        const int lo[3]{0, 0, 0};
        const int hi[3]{100, 100, 100};
        check(morphregion::plan(centre, 0, step, pad, lo, hi).empty, "an extent of zero yields no plan");
        const int zeroStep[3]{1, 0, 1};
        check(morphregion::plan(centre, 10, zeroStep, pad, lo, hi).empty, "a zero step is refused");
        const int negPad[3]{1, -1, 1};
        check(morphregion::plan(centre, 10, step, negPad, lo, hi).empty, "a negative apron is refused");
        const int inverted[3]{5, 5, 5};
        check(morphregion::plan(centre, 10, step, pad, hi, inverted).empty, "limits the wrong way round yield no plan");
        // a single-voxel region is legitimate and must still be addressable
        const auto one = morphregion::plan(centre, 1, step, pad, lo, hi);
        check(!one.empty && one.write.dim[0] == 1 && one.write.dim[1] == 1 && one.write.dim[2] == 1,
              "an extent of one is a single-voxel write box");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
