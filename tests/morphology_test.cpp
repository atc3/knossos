// Standalone test for segmentation/morphology.h — dilate, erode and smooth on a label
// block. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/morphology_test morphology_test.cpp && /tmp/morphology_test
//
// The separable Voronoi distance transform is the part worth testing hardest: it is three
// axis passes that have to compose into a real 3-D answer, it carries an argmin through
// each of them, and when it is wrong it is wrong by a voxel or two in a way that looks
// entirely plausible on screen. So it is checked against brute force — every voxel against
// every seed — over randomised blocks with anisotropic spacing, rather than by reasoning
// about it. Dilate and erode are then checked the same way, and against each other through
// the morphological duality that erosion of a set is the complement of the dilation of its
// complement.
#include "segmentation/morphology.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}
void section(const char * title) { std::printf("%s\n", title); }

using morphology::Block;
using morphology::Params;
using morphology::Operation;
using morphology::Direction;
using morphology::Infill;

constexpr std::uint32_t BG = 0;// palette 0 is the background id throughout

std::size_t flat(const int dim[3], const int x, const int y, const int z) {
    return (static_cast<std::size_t>(z) * dim[1] + y) * dim[0] + x;
}

// A block whose palette is {background, objectA, objectB, otherObject}; `paint` says which.
template<typename Paint>
Block makeBlock(const int w, const int h, const int d, Paint paint) {
    Block b;
    b.dim[0] = w; b.dim[1] = h; b.dim[2] = d;
    b.labels.assign(static_cast<std::size_t>(w) * h * d, BG);
    b.selected = {0, 1, 1, 0};// background no, objects A and B yes, the other object no
    for (int z = 0; z < d; ++z)
    for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
        b.labels[flat(b.dim, x, y, z)] = paint(x, y, z);
    }
    return b;
}

// Brute force: squared physical distance from (x,y,z) to the nearest voxel the predicate
// accepts, and one label achieving it.
template<typename SeedPredicate>
std::pair<double, std::uint32_t> nearest(const Block & b, const double spacing[3],
                                         const int x, const int y, const int z, SeedPredicate seed) {
    double best = 1e30;
    std::uint32_t bestLabel = 0;
    for (int sz = 0; sz < b.dim[2]; ++sz)
    for (int sy = 0; sy < b.dim[1]; ++sy)
    for (int sx = 0; sx < b.dim[0]; ++sx) {
        const auto i = flat(b.dim, sx, sy, sz);
        if (!seed(i)) {
            continue;
        }
        const auto dx = (x - sx) * spacing[0], dy = (y - sy) * spacing[1], dz = (z - sz) * spacing[2];
        const auto dsq = dx * dx + dy * dy + dz * dz;
        if (dsq < best) {
            best = dsq;
            bestLabel = b.labels[i];
        }
    }
    return {best, bestLabel};
}

Params basic(const Operation op, const double radius, const double sx, const double sy, const double sz) {
    Params p;
    p.op = op;
    p.radius = radius;
    p.spacing[0] = sx; p.spacing[1] = sy; p.spacing[2] = sz;
    p.background = BG;
    return p;
}

std::size_t countSelected(const Block & b) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < b.voxels(); ++i) {
        n += b.selectedAt(i) ? 1 : 0;
    }
    return n;
}

std::size_t countSelectedLabels(const Block & b, const std::vector<std::uint32_t> & labels) {
    std::size_t n = 0;
    for (const auto label : labels) {
        n += (label < b.selected.size() && b.selected[label]) ? 1 : 0;
    }
    return n;
}
}

int main() {
    section("the separable Voronoi transform agrees with brute force");
    {
        std::mt19937 rng(20261005);
        bool distancesMatch = true, labelsMatch = true, ran = false;
        for (int trial = 0; trial < 12; ++trial) {
            const int w = 3 + static_cast<int>(rng() % 9);
            const int h = 3 + static_cast<int>(rng() % 9);
            const int d = 3 + static_cast<int>(rng() % 7);
            // deliberately anisotropic, including the EM-typical 4x in z
            const double spacing[3]{1.0 + (rng() % 3), 1.0 + (rng() % 3), 1.0 + (rng() % 5)};
            auto block = makeBlock(w, h, d, [&rng](int, int, int) -> std::uint32_t {
                const auto r = rng() % 10;
                return r < 6 ? BG : (r < 8 ? 1u : (r < 9 ? 2u : 3u));
            });
            if (countSelected(block) == 0) {
                continue;// nothing to be near; the transform has no answer to agree with
            }
            ran = true;
            std::vector<float> dist;
            std::vector<std::int32_t> arg;
            morphology::voronoiSq3d(block.dim, spacing, [&block](const std::size_t i){ return block.selectedAt(i); }, dist, arg);
            for (int z = 0; z < d; ++z)
            for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const auto i = flat(block.dim, x, y, z);
                const auto ref = nearest(block, spacing, x, y, z, [&block](const std::size_t j){ return block.selectedAt(j); });
                if (std::abs(dist[i] - ref.first) > 1e-3 * std::max(1.0, ref.first)) {
                    distancesMatch = false;
                }
                // ties are legitimate, so the argument has to *achieve* the minimum rather
                // than be the particular voxel brute force happened to find first
                if (arg[i] < 0) {
                    labelsMatch = false;
                } else {
                    const auto a = static_cast<std::size_t>(arg[i]);
                    const int ax = static_cast<int>(a % w), ay = static_cast<int>((a / w) % h), az = static_cast<int>(a / (static_cast<std::size_t>(w) * h));
                    const auto dx = (x - ax) * spacing[0], dy = (y - ay) * spacing[1], dz = (z - az) * spacing[2];
                    const auto achieved = dx * dx + dy * dy + dz * dz;
                    if (!block.selectedAt(a) || std::abs(achieved - ref.first) > 1e-6 * std::max(1.0, ref.first)) {
                        labelsMatch = false;
                    }
                }
            }
        }
        check(ran, "the randomised blocks produced cases to check");
        check(distancesMatch, "every distance matches the exhaustive answer");
        check(labelsMatch, "every reported nearest voxel really is a nearest voxel of the object");
    }

    section("dilation adds exactly the voxels within the radius");
    {
        std::mt19937 rng(7);
        bool matches = true;
        for (int trial = 0; trial < 10; ++trial) {
            const int w = 5 + static_cast<int>(rng() % 6), h = 5 + static_cast<int>(rng() % 6), d = 4 + static_cast<int>(rng() % 5);
            const double spacing[3]{1.0, 2.0, 4.0};
            auto block = makeBlock(w, h, d, [&rng](int, int, int) -> std::uint32_t {
                return (rng() % 5 == 0) ? 1u : BG;
            });
            if (countSelected(block) == 0) {
                continue;
            }
            auto p = basic(Operation::Dilate, 2.5, spacing[0], spacing[1], spacing[2]);
            const auto result = morphology::apply(block, p);
            for (int z = 0; z < d; ++z)
            for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const auto i = flat(block.dim, x, y, z);
                const auto ref = nearest(block, spacing, x, y, z, [&block](const std::size_t j){ return block.selectedAt(j); });
                const bool shouldBeIn = block.selectedAt(i) || ref.first <= p.radius * p.radius + 1e-9;
                const bool isIn = result.labels[i] < block.selected.size() && block.selected[result.labels[i]];
                if (shouldBeIn != isIn) {
                    matches = false;
                }
            }
        }
        check(matches, "the dilated set is the set of voxels within the radius, voxel for voxel");
    }

    section("a radius is a length, not a voxel count");
    {
        // one voxel in the middle of a 9x9x9 block, EM-typical 10x10x40 nm sampling
        const int w = 9, h = 9, d = 9;
        auto block = makeBlock(w, h, d, [](const int x, const int y, const int z) -> std::uint32_t {
            return (x == 4 && y == 4 && z == 4) ? 1u : BG;
        });
        auto p = basic(Operation::Dilate, 20.0, 10.0, 10.0, 40.0);
        const auto result = morphology::apply(block, p);
        check(result.labels[flat(block.dim, 6, 4, 4)] == 1, "20 nm reaches two voxels along a 10 nm axis");
        check(result.labels[flat(block.dim, 7, 4, 4)] == BG, "and no further");
        check(result.labels[flat(block.dim, 4, 4, 5)] == BG, "and not one voxel along a 40 nm axis");
        check(result.labels[flat(block.dim, 5, 5, 4)] == 1, "the diagonal at 14.1 nm is inside");
        check(result.labels[flat(block.dim, 6, 5, 4)] == BG, "the one at 22.4 nm is not");
    }

    section("erosion is the dilation of the complement");
    {
        std::mt19937 rng(99);
        bool dual = true;
        for (int trial = 0; trial < 10; ++trial) {
            const int w = 6 + static_cast<int>(rng() % 5), h = 6 + static_cast<int>(rng() % 5), d = 5 + static_cast<int>(rng() % 4);
            const double spacing[3]{1.0, 1.0, 3.0};
            auto block = makeBlock(w, h, d, [&rng](int, int, int) -> std::uint32_t {
                return (rng() % 3 != 0) ? 1u : BG;// mostly object, so erosion has something to eat
            });
            auto p = basic(Operation::Erode, 2.0, spacing[0], spacing[1], spacing[2]);
            const auto eroded = morphology::apply(block, p);
            for (int z = 0; z < d; ++z)
            for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const auto i = flat(block.dim, x, y, z);
                if (!block.selectedAt(i)) {
                    continue;
                }
                // a voxel survives erosion exactly when no non-object voxel is within r
                const auto ref = nearest(block, spacing, x, y, z, [&block](const std::size_t j){ return !block.selectedAt(j); });
                const bool survives = ref.first > p.radius * p.radius + 1e-9;
                const bool kept = eroded.labels[i] < block.selected.size() && block.selected[eroded.labels[i]];
                if (survives != kept) {
                    dual = false;
                }
            }
        }
        check(dual, "a voxel survives exactly when nothing outside the object is within the radius");
    }

    section("neither operation moves the boundary the wrong way");
    {
        std::mt19937 rng(1234);
        bool dilateOnlyGrows = true, erodeOnlyShrinks = true;
        for (int trial = 0; trial < 8; ++trial) {
            const int w = 7, h = 7, d = 6;
            auto block = makeBlock(w, h, d, [&rng](int, int, int) -> std::uint32_t {
                return (rng() % 2) ? 1u : BG;
            });
            const auto before = countSelected(block);
            const auto grown = morphology::apply(block, basic(Operation::Dilate, 1.5, 1, 1, 1));
            const auto shrunk = morphology::apply(block, basic(Operation::Erode, 1.5, 1, 1, 1));
            for (std::size_t i = 0; i < block.voxels(); ++i) {
                if (block.selectedAt(i) && !(grown.labels[i] < block.selected.size() && block.selected[grown.labels[i]])) {
                    dilateOnlyGrows = false;// dilation dropped a voxel
                }
                if (!block.selectedAt(i) && shrunk.labels[i] < block.selected.size() && block.selected[shrunk.labels[i]]) {
                    erodeOnlyShrinks = false;// erosion added one
                }
            }
            if (countSelectedLabels(block, grown.labels) < before) { dilateOnlyGrows = false; }
            if (countSelectedLabels(block, shrunk.labels) > before) { erodeOnlyShrinks = false; }
        }
        check(dilateOnlyGrows, "dilation never takes a voxel away");
        check(erodeOnlyShrinks, "erosion never adds one");
    }

    section("a merged object grows as its own fragments, not as one id");
    {
        // two fragments of the same selection, far apart along x
        const int w = 11, h = 3, d = 3;
        auto block = makeBlock(w, h, d, [](const int x, const int y, const int z) -> std::uint32_t {
            if (y != 1 || z != 1) { return BG; }
            if (x == 1) { return 1u; }
            if (x == 9) { return 2u; }
            return BG;
        });
        const auto result = morphology::apply(block, basic(Operation::Dilate, 1.0, 1, 1, 1));
        check(result.labels[flat(block.dim, 0, 1, 1)] == 1 && result.labels[flat(block.dim, 2, 1, 1)] == 1,
              "the voxels beside the first fragment take its id");
        check(result.labels[flat(block.dim, 8, 1, 1)] == 2 && result.labels[flat(block.dim, 10, 1, 1)] == 2,
              "and the ones beside the second take the second's");
        check(result.labels[flat(block.dim, 1, 0, 1)] == 1 && result.labels[flat(block.dim, 1, 1, 0)] == 1,
              "the fragment grows across all three axes, not just along x");
        // a radius of one at unit spacing is the six face neighbours, twice over
        check(result.added == 12, "twelve voxels in all, six around each fragment");
    }

    section("what an eroded voxel turns into");
    {
        const int w = 5, h = 5, d = 5;
        // a solid object with one neighbouring label pressed against its -x face
        auto block = makeBlock(w, h, d, [](const int x, const int y, const int z) -> std::uint32_t {
            if (x == 0) { return 3u; }// the other object, not selected
            if (x >= 1 && x <= 3 && y >= 1 && y <= 3 && z >= 1 && z <= 3) { return 1u; }
            return BG;
        });
        auto background = basic(Operation::Erode, 1.0, 1, 1, 1);
        background.infill = Infill::Background;
        check(morphology::apply(block, background).labels[flat(block.dim, 1, 2, 2)] == BG,
              "Background hands it to the background id");

        auto nearestLabel = basic(Operation::Erode, 1.0, 1, 1, 1);
        nearestLabel.infill = Infill::NearestLabel;
        check(morphology::apply(block, nearestLabel).labels[flat(block.dim, 1, 2, 2)] == 3,
              "NearestLabel hands it to the neighbour pressed against that face");

        auto replace = basic(Operation::Erode, 1.0, 1, 1, 1);
        replace.infill = Infill::Replace;
        replace.replacement = 3;
        check(morphology::apply(block, replace).labels[flat(block.dim, 2, 2, 1)] == 3,
              "Replace hands every one of them to the chosen label");
        check(morphology::apply(block, background).labels[flat(block.dim, 2, 2, 2)] == 1,
              "and the core, deeper than the radius, is left alone");
    }

    section("smoothing rounds a shape off rather than resizing it");
    {
        /* A 12x12 slab one plane deep with a one-voxel spur sticking out of one face and a
         * one-voxel dent bitten out of the other. Smoothing should take the spur off and
         * fill the dent in, and leave the flat faces and the interior alone — that is the
         * difference between this and a dilate followed by an erode. */
        const int w = 16, h = 16, d = 1;
        const int lo = 4, hi = 11;
        auto block = makeBlock(w, h, d, [&](const int x, const int y, int) -> std::uint32_t {
            if (x >= lo && x <= hi && y >= lo && y <= hi) { return 1u; }
            if (x == hi + 1 && y == 8) { return 1u; }// the spur
            return BG;
        });
        block.labels[flat(block.dim, lo, 8, 0)] = BG;// the dent

        auto p = basic(Operation::Smooth, 2.0, 1, 1, 1);
        p.direction = Direction::Both;
        p.infill = Infill::Background;
        const auto both = morphology::apply(block, p);
        check(both.labels[flat(block.dim, hi + 1, 8, 0)] == BG, "the spur is gone");
        check(both.labels[flat(block.dim, lo, 8, 0)] == 1, "the dent is filled");
        check(both.labels[flat(block.dim, 8, 8, 0)] == 1, "the interior is untouched");
        check(both.labels[flat(block.dim, 8, 2, 0)] == BG, "and so is the background well clear of it");

        p.direction = Direction::Shrink;
        const auto shrink = morphology::apply(block, p);
        check(shrink.labels[flat(block.dim, hi + 1, 8, 0)] == BG && shrink.labels[flat(block.dim, lo, 8, 0)] == BG,
              "Shrink takes the spur and leaves the dent");
        check(shrink.added == 0, "and adds nothing at all");

        p.direction = Direction::Expand;
        const auto expand = morphology::apply(block, p);
        check(expand.labels[flat(block.dim, hi + 1, 8, 0)] == 1 && expand.labels[flat(block.dim, lo, 8, 0)] == 1,
              "Expand fills the dent and leaves the spur");
        check(expand.removed == 0, "and removes nothing at all");
    }

    section("a voxel sitting exactly on the threshold counts as inside");
    {
        /* Paintera's test is `gaussian >= threshold`, and the boundary case is not reachable
         * by picking a threshold and hoping: with a discrete kernel an exact tie is a
         * coincidence. So the threshold is read back off the very blur the operation will
         * compute — mask, then the same sigma — which makes one chosen voxel land on it
         * exactly. With the comparison flipped to `>` that voxel is dropped instead, and
         * every flat face of every smoothed object loses its outermost tie the same way. */
        const int w = 16, h = 16, d = 1;
        const int dim[3]{w, h, d};
        auto block = makeBlock(w, h, d, [](const int x, const int y, int) -> std::uint32_t {
            return (x >= 4 && x <= 11 && y >= 4 && y <= 11) ? 1u : BG;
        });
        const double radius = 2.0;
        std::vector<float> blurred(block.voxels(), 0.f);
        for (std::size_t i = 0; i < block.voxels(); ++i) {
            blurred[i] = block.selectedAt(i) ? 1.f : 0.f;
        }
        const double sigma[3]{radius, radius, radius};// unit spacing, so sigma == radius
        morphology::gaussian3d(dim, sigma, blurred);

        // a background voxel just outside the face: within the radius, so only the
        // threshold decides it
        const auto probe = flat(dim, 12, 8, 0);
        auto p = basic(Operation::Smooth, radius, 1, 1, 1);
        p.direction = Direction::Expand;
        p.threshold = blurred[probe];
        const auto result = morphology::apply(block, p);
        check(!block.selectedAt(probe), "the probe voxel starts outside the object");
        check(result.labels[probe] == 1, "and a voxel exactly on the threshold is taken in");
    }

    section("smoothing a shape with nothing to round off changes nothing");
    {
        const int w = 14, h = 14, d = 14;
        auto block = makeBlock(w, h, d, [](const int x, const int y, const int z) -> std::uint32_t {
            return (x >= 3 && x <= 10 && y >= 3 && y <= 10 && z >= 3 && z <= 10) ? 1u : BG;
        });
        auto p = basic(Operation::Smooth, 1.2, 1, 1, 1);
        const auto result = morphology::apply(block, p);
        // a cube's corners are the one thing a smooth legitimately rounds, so this checks
        // the faces and the body rather than asserting the whole block is identical
        bool facesHeld = true;
        for (int z = 4; z <= 9; ++z)
        for (int y = 4; y <= 9; ++y) {
            if (result.labels[flat(block.dim, 3, y, z)] != 1 || result.labels[flat(block.dim, 10, y, z)] != 1) { facesHeld = false; }
            if (result.labels[flat(block.dim, 2, y, z)] != BG || result.labels[flat(block.dim, 11, y, z)] != BG) { facesHeld = false; }
        }
        check(facesHeld, "flat faces stay where they are");
    }

    section("the result is the input wherever nothing changed");
    {
        std::mt19937 rng(555);
        bool diffMatchesCounts = true, othersUntouched = true;
        for (int trial = 0; trial < 10; ++trial) {
            const int w = 9, h = 8, d = 7;
            auto block = makeBlock(w, h, d, [&rng](int, int, int) -> std::uint32_t {
                const auto r = rng() % 8;
                return r < 4 ? BG : (r < 6 ? 1u : (r < 7 ? 2u : 3u));
            });
            for (const auto op : {Operation::Dilate, Operation::Erode, Operation::Smooth}) {
                auto p = basic(op, 1.6, 1, 1, 2);
                p.infill = Infill::Background;
                const auto result = morphology::apply(block, p);
                std::size_t added = 0, removed = 0;
                for (std::size_t i = 0; i < block.voxels(); ++i) {
                    const bool was = block.selectedAt(i);
                    const bool now = result.labels[i] < block.selected.size() && block.selected[result.labels[i]] != 0;
                    added += (!was && now) ? 1 : 0;
                    removed += (was && !now) ? 1 : 0;
                    // a label that is neither the object nor something the object gave up
                    // must come through untouched
                    if (!was && !now && result.labels[i] != block.labels[i]) {
                        othersUntouched = false;
                    }
                }
                if (added != result.added || removed != result.removed) {
                    diffMatchesCounts = false;
                }
            }
        }
        check(diffMatchesCounts, "the reported counts are the actual difference");
        check(othersUntouched, "a voxel that neither joined nor left keeps its own label");
    }

    section("the Gaussian itself");
    {
        for (const double sigma : {0.5, 1.0, 2.5, 7.0}) {
            const auto kernel = morphology::gaussianKernel(sigma);
            double sum = 0;
            for (const auto w : kernel) { sum += w; }
            check(std::abs(sum - 1.0) < 1e-5, "the kernel for sigma " + std::to_string(sigma) + " sums to one");
        }
        // a field that is 1 everywhere must stay 1 away from the zero-extended border
        const int w = 21, h = 21, d = 21;
        const int dim[3]{w, h, d};
        std::vector<float> img(static_cast<std::size_t>(w) * h * d, 1.f);
        const double sigma[3]{1.5, 1.5, 1.5};
        morphology::gaussian3d(dim, sigma, img);
        check(std::abs(img[flat(dim, 10, 10, 10)] - 1.f) < 1e-4, "a constant field survives the blur in the middle");
        /* At the face the kernel keeps only the half of its mass that falls inside, plus the
         * centre tap: for sigma 1.5 that is 0.633 of the 1.0 it should be. Small enough to
         * push a voxel across a 0.5 threshold the wrong way, which is exactly why the driver
         * reads a margin of real data around the region it writes. */
        const auto atFace = img[flat(dim, 0, 10, 10)];
        check(atFace > 0.60f && atFace < 0.67f, "and is pulled down to ~0.63 at the border, which is why the driver reads a margin");
    }

    section("degenerate input");
    {
        Block empty;
        check(morphology::apply(empty, basic(Operation::Dilate, 2, 1, 1, 1)).labels.empty(), "an empty block yields an empty result");

        auto nothingSelected = makeBlock(5, 5, 5, [](int, int, int) -> std::uint32_t { return BG; });
        const auto result = morphology::apply(nothingSelected, basic(Operation::Dilate, 3, 1, 1, 1));
        check(result.added == 0 && result.labels == nothingSelected.labels, "a block with no object in it is left alone");

        auto onePlane = makeBlock(7, 7, 1, [](const int x, const int y, int) -> std::uint32_t {
            return (x >= 2 && x <= 4 && y >= 2 && y <= 4) ? 1u : BG;
        });
        const auto planeResult = morphology::apply(onePlane, basic(Operation::Smooth, 1.5, 1, 1, 1));
        check(planeResult.labels[flat(onePlane.dim, 3, 3, 0)] == 1,
              "a single-plane block is smoothed in plane rather than blurred away along the flat axis");

        auto zeroRadius = makeBlock(5, 5, 5, [](const int x, const int y, const int z) -> std::uint32_t {
            return (x == 2 && y == 2 && z == 2) ? 1u : BG;
        });
        const auto none = morphology::apply(zeroRadius, basic(Operation::Dilate, 0.0, 1, 1, 1));
        check(none.added == 0, "a radius of zero adds nothing");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
