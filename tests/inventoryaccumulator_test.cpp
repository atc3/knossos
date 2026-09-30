// Standalone test for segmentation/inventoryaccumulator.h — the tally behind the object
// inventory: where each object is, and in what order the list presents them. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/invacc_test inventoryaccumulator_test.cpp && /tmp/invacc_test
#include "segmentation/inventoryaccumulator.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

using objinv::Record;
using objinv::CubeGeometry;
using objinv::Accumulator;

// A block of ids, indexed [z][y][x] the way KNOSSOS stores a cube.
struct Block {
    CubeGeometry geom;
    std::vector<std::uint64_t> voxels;
    std::uint64_t & at(const int x, const int y, const int z) {
        return voxels[(static_cast<std::size_t>(z) * geom.shape[1] + y) * geom.shape[0] + x];
    }
    std::uint64_t at(const int x, const int y, const int z) const {
        return voxels[(static_cast<std::size_t>(z) * geom.shape[1] + y) * geom.shape[0] + x];
    }
};

Block makeBlock(const int side, const std::int32_t origin[3], const std::int32_t step[3],
                const double nm[3] = nullptr) {
    Block b;
    for (int a = 0; a < 3; ++a) {
        b.geom.origin[a] = origin[a];
        b.geom.step[a] = step[a];
        b.geom.shape[a] = side;
        b.geom.nmPerVoxel[a] = nm == nullptr ? static_cast<double>(step[a]) : nm[a];
    }
    b.voxels.assign(b.geom.voxelCount(), 0);
    return b;
}

// What the tally should say, worked out one voxel at a time.
struct Naive {
    std::uint64_t voxels{0};
    std::int64_t sum[3]{};
    std::int32_t lo[3]{INT32_MAX, INT32_MAX, INT32_MAX};
    std::int32_t hi[3]{INT32_MIN, INT32_MIN, INT32_MIN};
};
std::map<std::uint64_t, Naive> naiveTally(const std::vector<Block> & blocks, const std::uint64_t background) {
    std::map<std::uint64_t, Naive> out;
    for (const auto & b : blocks) {
        for (int z = 0; z < b.geom.shape[2]; ++z) {
        for (int y = 0; y < b.geom.shape[1]; ++y) {
        for (int x = 0; x < b.geom.shape[0]; ++x) {
            const auto id = b.at(x, y, z);
            if (id == background) { continue; }
            auto & n = out[id];
            ++n.voxels;
            const int local[3]{x, y, z};
            for (int a = 0; a < 3; ++a) {
                const auto g = b.geom.origin[a] + b.geom.step[a] * local[a];
                n.sum[a] += g;
                n.lo[a] = std::min(n.lo[a], static_cast<std::int32_t>(g));
                n.hi[a] = std::max(n.hi[a], static_cast<std::int32_t>(g));
            }
        }}}
    }
    return out;
}

// Is `rep` really a voxel of that object?
bool repIsRealVoxel(const std::vector<Block> & blocks, const Record & r) {
    for (const auto & b : blocks) {
        for (int z = 0; z < b.geom.shape[2]; ++z) {
        for (int y = 0; y < b.geom.shape[1]; ++y) {
        for (int x = 0; x < b.geom.shape[0]; ++x) {
            if (b.at(x, y, z) != r.id) { continue; }
            const int local[3]{x, y, z};
            bool same = true;
            for (int a = 0; a < 3; ++a) {
                same = same && (b.geom.origin[a] + b.geom.step[a] * local[a]) == r.rep[a];
            }
            if (same) { return true; }
        }}}
    }
    return false;
}

std::vector<Record> run(const std::vector<Block> & blocks, const std::uint64_t background,
                        const std::size_t cap = 500000) {
    Accumulator acc;
    acc.setIdCap(cap);
    for (const auto & b : blocks) {
        acc.ingestCube(b.voxels.data(), b.geom, background);
    }
    return acc.records();
}

// A handful of blobs, deterministic, with a curved object among them.
std::vector<Block> scene(const std::int32_t step[3]) {
    const std::int32_t o0[3]{0, 0, 0};
    const std::int32_t o1[3]{8 * step[0], 0, 0};
    auto a = makeBlock(8, o0, step);
    auto b = makeBlock(8, o1, step);
    // id 7: a solid 3x3x3 cube wholly inside block a
    for (int z = 1; z < 4; ++z) { for (int y = 1; y < 4; ++y) { for (int x = 1; x < 4; ++x) { a.at(x, y, z) = 7; } } }
    // id 9: an L, so its mean sits off the shape
    for (int x = 0; x < 7; ++x) { a.at(x, 6, 6) = 9; }
    for (int y = 0; y < 7; ++y) { a.at(6, y, 6) = 9; }
    // id 11: straddles both blocks, much denser in b
    for (int z = 0; z < 2; ++z) { for (int y = 0; y < 2; ++y) { a.at(7, y, z) = 11; } }
    for (int z = 0; z < 6; ++z) { for (int y = 0; y < 6; ++y) { for (int x = 0; x < 6; ++x) { b.at(x, y, z) = 11; } } }
    // id 13: a single voxel in b
    b.at(7, 7, 7) = 13;
    return {a, b};
}

void section(const char * title) { std::printf("%s\n", title); }
}

int main() {
    section("Z-order sweep");
    {
        bool roundTrips = true, distinct = true;
        std::vector<std::uint64_t> codes;
        for (std::uint32_t z = 0; z < 12; ++z) {
        for (std::uint32_t y = 0; y < 12; ++y) {
        for (std::uint32_t x = 0; x < 12; ++x) {
            const auto code = objinv::mortonEncode3(x, y, z);
            std::uint32_t dx, dy, dz;
            objinv::mortonDecode3(code, dx, dy, dz);
            roundTrips = roundTrips && dx == x && dy == y && dz == z;
            codes.push_back(code);
        }}}
        std::sort(std::begin(codes), std::end(codes));
        distinct = std::adjacent_find(std::begin(codes), std::end(codes)) == std::end(codes);
        check(roundTrips, "encode/decode round-trips over the grid");
        check(distinct, "distinct blocks get distinct codes");

        // the extreme corner, to show 21 bits per axis really are there
        const auto hi = objinv::mortonEncode3(0x1fffff, 0x1fffff, 0x1fffff);
        std::uint32_t hx, hy, hz;
        objinv::mortonDecode3(hi, hx, hy, hz);
        check(hx == 0x1fffff && hy == 0x1fffff && hz == 0x1fffff, "21 bits per axis survive the round-trip");

        // the walk must be long enough to reach every block, including the padding
        const auto span = objinv::mortonSpan(3, 1, 3);
        std::uint64_t reached = 0;
        for (std::uint64_t c = 0; c < span; ++c) {
            std::uint32_t x, y, z;
            objinv::mortonDecode3(c, x, y, z);
            if (x < 3 && y < 1 && z < 3) { ++reached; }
        }
        check(reached == 9, "the walk covers every block of a 3x1x3 grid");
        check(objinv::mortonSpan(1, 1, 1) == 1, "a single-block grid spans one index");
    }

    section("block geometry");
    {
        check(objinv::blocksAlong(632, 128, 8) == 1, "632 mag-1 voxels at 8x is one 128 block");
        check(objinv::blocksAlong(1349, 128, 8) == 2, "1349 at 8x is two blocks");
        check(objinv::blocksAlong(1994, 128, 8) == 2, "1994 at 8x is two blocks");
        // the measured cube counts for the real datasets, which is what sizes the scan
        const auto cubes = [](const std::int64_t e[3], const std::int32_t step) {
            return static_cast<std::uint64_t>(objinv::blocksAlong(e[0], 128, step))
                 * objinv::blocksAlong(e[1], 128, step) * objinv::blocksAlong(e[2], 128, step);
        };
        const std::int64_t biggest[3]{13948, 7211, 12186};
        const std::int64_t crop[3]{632, 1349, 1994};
        check(cubes(biggest, 8) == 1344, "largest dataset is 1344 blocks at 8x");
        check(cubes(biggest, 4) == 10080, "largest dataset is 10080 blocks at 4x");
        check(cubes(biggest, 2) == 76560, "largest dataset is 76560 blocks at 2x");
        check(cubes(crop, 1) == 880, "crop_2 is 880 blocks at mag 1");
        check(cubes(crop, 2) == 144, "crop_2 is 144 blocks at 2x");
        check(objinv::blocksAlong(0, 128, 1) == 1 && objinv::blocksAlong(100, 0, 1) == 1,
              "degenerate extents report one block rather than none");
    }

    section("how far the sweep has got");
    {
        // a 3x1x3 grid inside a padded 4x4x4 walk
        const std::uint32_t g[3]{3, 1, 3};
        const auto span = objinv::mortonSpan(g[0], g[1], g[2]);
        const auto covers = [&](const std::uint64_t code) {
            std::uint32_t x, y, z;
            objinv::mortonDecode3(code, x, y, z);
            return x < g[0] && y < g[1] && z < g[2];
        };

        objinv::SweepCursor cursor;
        cursor.init(0, span, covers);
        std::vector<std::uint64_t> handed;
        std::uint64_t code = 0;
        while (cursor.next(code, covers)) { handed.push_back(code); }
        check(handed.size() == 9, "it hands out every real block and no padding");
        bool allReal = true;
        for (const auto c : handed) { allReal = allReal && covers(c); }
        check(allReal, "and only blocks inside the grid");

        // finishing out of order must not move the cursor past anything unfinished
        cursor.init(0, span, covers);
        cursor.next(code, covers);
        const auto first = code;
        cursor.next(code, covers);
        const auto second = code;
        cursor.complete(second, covers);
        check(cursor.position() == first, "a block finishing early does not carry the cursor over its neighbour");
        check(cursor.pending() == 1, "it waits instead");
        cursor.complete(first, covers);
        check(cursor.position() > second && cursor.pending() == 0,
              "once the straggler lands the cursor takes in both at once");

        // a resume from a recorded position must cover exactly what was left
        cursor.init(0, span, covers);
        std::vector<std::uint64_t> beforeStop;
        for (int i = 0; i < 4; ++i) {
            cursor.next(code, covers);
            cursor.complete(code, covers);
            beforeStop.push_back(code);
        }
        const auto resumeAt = cursor.position();
        objinv::SweepCursor resumed;
        resumed.init(resumeAt, span, covers);
        std::vector<std::uint64_t> after;
        while (resumed.next(code, covers)) { after.push_back(code); }
        auto union_ = beforeStop;
        union_.insert(union_.end(), after.begin(), after.end());
        std::sort(union_.begin(), union_.end());
        check(union_.size() == 9 && std::adjacent_find(union_.begin(), union_.end()) == union_.end(),
              "resuming covers every remaining block exactly once, with none skipped or repeated");

        // a block left in flight when the sweep stops must be fetched again on resume
        cursor.init(0, span, covers);
        cursor.next(code, covers);
        const auto inFlight = code;
        cursor.next(code, covers);
        cursor.complete(code, covers);// the later one lands, the earlier never does
        objinv::SweepCursor afterCrash;
        afterCrash.init(cursor.position(), span, covers);
        afterCrash.next(code, covers);
        check(code == inFlight, "the block that never came back is the first one tried again");

        check(!cursor.finished(), "a partly done walk does not claim to be finished");
        objinv::SweepCursor empty;
        empty.init(0, 0, covers);
        check(empty.finished() && !empty.next(code, covers), "an empty walk is finished and hands out nothing");
        cursor.init(0, span, covers);
        cursor.next(code, covers);
        cursor.complete(code, covers);
        const auto settled = cursor.position();
        cursor.complete(code, covers);
        check(cursor.position() == settled, "completing the same block twice moves nothing");
    }

    section("the tally matches a per-voxel reference");
    for (const auto & named : std::vector<std::pair<std::string, std::vector<std::int32_t>>>{
             {"at magnification 1", {1, 1, 1}},
             {"at 8x", {8, 8, 8}},
             {"with an anisotropic pyramid (z not downsampled)", {4, 4, 1}}}) {
        const std::int32_t step[3]{named.second[0], named.second[1], named.second[2]};
        const auto blocks = scene(step);
        const auto recs = run(blocks, 0);
        const auto want = naiveTally(blocks, 0);
        bool counts = recs.size() == want.size(), fields = true;
        for (const auto & r : recs) {
            const auto it = want.find(r.id);
            if (it == std::end(want)) { fields = false; continue; }
            const auto & n = it->second;
            fields = fields && r.voxels == n.voxels;
            for (int a = 0; a < 3; ++a) {
                fields = fields && r.sum[a] == n.sum[a] && r.bboxMin[a] == n.lo[a] && r.bboxMax[a] == n.hi[a];
            }
        }
        check(counts && fields, "voxel counts, coordinate sums and bounding boxes agree " + named.first);
    }

    section("the jump target");
    {
        const std::int32_t step[3]{4, 4, 1};
        const auto blocks = scene(step);
        const auto recs = run(blocks, 0);
        bool allReal = true;
        for (const auto & r : recs) { allReal = allReal && repIsRealVoxel(blocks, r); }
        check(allReal, "every stored position is a real voxel of its object");

        // The L-shaped object is the case the mean gets wrong.
        const auto ell = std::find_if(std::begin(recs), std::end(recs), [](const Record & r){ return r.id == 9; });
        check(ell != std::end(recs), "the L-shaped object was found");
        if (ell != std::end(recs)) {
            std::int32_t mean[3];
            ell->centroid(mean);
            bool meanIsVoxel = false;
            for (const auto & b : blocks) {
                for (int z = 0; z < b.geom.shape[2]; ++z) {
                for (int y = 0; y < b.geom.shape[1]; ++y) {
                for (int x = 0; x < b.geom.shape[0]; ++x) {
                    if (b.at(x, y, z) != 9) { continue; }
                    bool same = true;
                    for (int a = 0; a < 3; ++a) {
                        const int local[3]{x, y, z};
                        same = same && (b.geom.origin[a] + b.geom.step[a] * local[a]) == mean[a];
                    }
                    meanIsVoxel = meanIsVoxel || same;
                }}}
            }
            check(!meanIsVoxel, "its mean position is NOT on the object — which is why the mean is not the jump target");
            check(repIsRealVoxel(blocks, *ell), "its stored position is on the object anyway");
        }

        // The straddling object must be represented from its dense block, not its sparse one.
        const auto straddler = std::find_if(std::begin(recs), std::end(recs), [](const Record & r){ return r.id == 11; });
        check(straddler != std::end(recs) && straddler->rep[0] >= 8 * step[0],
              "an object spanning two blocks is represented from the block it is dense in");
    }

    section("the list is reproducible");
    {
        const std::int32_t step[3]{2, 2, 2};
        const auto blocks = scene(step);
        const auto reference = run(blocks, 0);

        // Visit order must not change any object's contents. It does fix their order in
        // the list, so compare by id rather than by position.
        std::mt19937 rng{12345};
        bool stable = true;
        for (int trial = 0; trial < 12 && stable; ++trial) {
            auto shuffled = blocks;
            std::shuffle(std::begin(shuffled), std::end(shuffled), rng);
            const auto got = run(shuffled, 0);
            stable = stable && got.size() == reference.size();
            for (const auto & want : reference) {
                const auto it = std::find_if(std::begin(got), std::end(got),
                                             [&](const Record & r){ return r.id == want.id; });
                if (it == std::end(got)) { stable = false; break; }
                stable = stable && it->voxels == want.voxels && it->bestCubeVoxels == want.bestCubeVoxels;
                for (int a = 0; a < 3; ++a) {
                    stable = stable && it->rep[a] == want.rep[a] && it->sum[a] == want.sum[a]
                            && it->bboxMin[a] == want.bboxMin[a] && it->bboxMax[a] == want.bboxMax[a];
                }
            }
        }
        check(stable, "shuffling the order blocks arrive in changes nothing about any object");

        // Ingesting the same block twice must not renumber the list.
        Accumulator acc;
        for (const auto & b : blocks) { acc.ingestCube(b.voxels.data(), b.geom, 0); }
        std::vector<std::uint64_t> order;
        for (const auto & r : acc.records()) { order.push_back(r.id); }
        acc.ingestCube(blocks.front().voxels.data(), blocks.front().geom, 0);
        std::vector<std::uint64_t> after;
        for (const auto & r : acc.records()) { after.push_back(r.id); }
        check(order == after, "re-reading a block appends nothing and reorders nothing");
    }

    section("resuming from the cache");
    {
        const std::int32_t step[3]{4, 4, 4};
        const auto blocks = scene(step);
        const auto whole = run(blocks, 0);

        // Half the blocks, persist, rehydrate, finish. Must equal one uninterrupted pass —
        // this is why the running coordinate sums are stored rather than a finished centroid.
        Accumulator first;
        first.ingestCube(blocks.front().voxels.data(), blocks.front().geom, 0);
        auto carried = first.records();

        Accumulator resumed;
        resumed.adopt(std::move(carried));
        resumed.ingestCube(blocks.back().voxels.data(), blocks.back().geom, 0);
        const auto got = resumed.records();

        bool same = got.size() == whole.size();
        for (std::size_t i = 0; i < got.size() && same; ++i) {
            same = same && got[i].id == whole[i].id && got[i].voxels == whole[i].voxels
                    && got[i].bestCubeVoxels == whole[i].bestCubeVoxels;
            for (int a = 0; a < 3; ++a) {
                same = same && got[i].rep[a] == whole[i].rep[a] && got[i].sum[a] == whole[i].sum[a]
                        && got[i].bboxMin[a] == whole[i].bboxMin[a] && got[i].bboxMax[a] == whole[i].bboxMax[a];
            }
        }
        check(same, "a scan resumed halfway is identical to one that never stopped, order included");
        check(resumed.indexOfId(11) != Accumulator::npos && resumed.indexOfId(4242) == Accumulator::npos,
              "id lookup finds what was adopted and nothing else");
    }

    section("the id ceiling");
    {
        const std::int32_t origin[3]{0, 0, 0};
        const std::int32_t step[3]{1, 1, 1};
        auto b = makeBlock(8, origin, step);
        std::uint64_t id = 1;
        for (int z = 0; z < 8; ++z) { for (int y = 0; y < 8; ++y) { for (int x = 0; x < 8; ++x) { b.at(x, y, z) = id++; } } }

        Accumulator acc;
        acc.setIdCap(10);
        acc.ingestCube(b.voxels.data(), b.geom, 0);
        check(acc.records().size() == 10, "the ceiling stops new ids being taken on");
        check(acc.truncated() && acc.rejected() == 512 - 10, "and says how many it turned away");

        // Past the ceiling, ids already known must keep accumulating.
        const auto known = acc.records().front().id;
        const auto before = acc.records().front().voxels;
        auto b2 = makeBlock(8, origin, step);
        for (int z = 0; z < 8; ++z) { for (int y = 0; y < 8; ++y) { for (int x = 0; x < 8; ++x) { b2.at(x, y, z) = known; } } }
        acc.ingestCube(b2.voxels.data(), b2.geom, 0);
        check(acc.records().front().voxels == before + 512 && acc.records().size() == 10,
              "an id it already knows keeps growing past the ceiling");
    }

    section("degenerate input");
    {
        const std::int32_t origin[3]{0, 0, 0};
        const std::int32_t step[3]{1, 1, 1};
        auto empty = makeBlock(4, origin, step);
        check(run({empty}, 0).empty(), "an all-background block yields nothing");

        Accumulator acc;
        acc.ingestCube(nullptr, empty.geom, 0);
        check(acc.records().empty(), "a null block is refused rather than read");

        auto bad = makeBlock(4, origin, step);
        bad.geom.shape[1] = 0;
        acc.ingestCube(bad.voxels.data(), bad.geom, 0);
        check(acc.records().empty(), "a zero-sized block is refused");

        bad.geom.shape[1] = 4;
        bad.geom.step[2] = 0;
        acc.ingestCube(bad.voxels.data(), bad.geom, 0);
        check(acc.records().empty(), "a zero step is refused");

        // A non-zero background id is a user setting, so it must be honoured.
        auto bg = makeBlock(4, origin, step);
        for (auto & v : bg.voxels) { v = 5; }
        bg.at(1, 1, 1) = 6;
        const auto recs = run({bg}, 5);
        check(recs.size() == 1 && recs.front().id == 6, "a non-zero background id is skipped, not counted");
    }

    section("the cache record layout");
    {
        check(sizeof(Record) == 80, "a record is 80 bytes");
        Record r;
        r.id = 0xdeadbeefcafef00dull;
        r.voxels = 1ull << 40;
        for (int a = 0; a < 3; ++a) {
            r.sum[a] = -(static_cast<std::int64_t>(1) << (40 + a));
            r.rep[a] = 1000 + a;
            r.bboxMin[a] = -7 - a;
            r.bboxMax[a] = 900000 + a;
        }
        r.bestCubeVoxels = 0xffffffffu;
        std::vector<unsigned char> bytes(sizeof(Record));
        std::memcpy(bytes.data(), &r, sizeof(Record));
        Record back;
        std::memcpy(&back, bytes.data(), sizeof(Record));
        bool intact = back.id == r.id && back.voxels == r.voxels && back.bestCubeVoxels == r.bestCubeVoxels;
        for (int a = 0; a < 3; ++a) {
            intact = intact && back.sum[a] == r.sum[a] && back.rep[a] == r.rep[a]
                    && back.bboxMin[a] == r.bboxMin[a] && back.bboxMax[a] == r.bboxMax[a];
        }
        check(intact, "a record survives a byte-for-byte round-trip, negatives and 64-bit ids included");

        Record none;
        none.voxels = 0;
        none.rep[0] = 5; none.rep[1] = 6; none.rep[2] = 7;
        std::int32_t c[3];
        none.centroid(c);
        check(c[0] == 5 && c[1] == 6 && c[2] == 7, "an empty record's mean falls back to its position rather than dividing by zero");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
