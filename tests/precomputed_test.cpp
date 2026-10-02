// Standalone test for segmentation/precomputed.h — reading a Neuroglancer precomputed
// segmentation. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/precomputed_test precomputed_test.cpp && /tmp/precomputed_test
//
// The decoder is checked by round trip against an encoder written here from the spec, over
// every index width the format allows, because the data is bit-packed and an off-by-one in
// the packing produces a plausible-looking volume of wrong labels rather than a failure.
#include "segmentation/precomputed.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
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

std::size_t idx(const std::int32_t e[3], const int x, const int y, const int z) {
    return (static_cast<std::size_t>(z) * e[1] + y) * e[0] + x;
}

/* A compressed_segmentation encoder, from the spec, so the decoder has something
 * independent to be checked against. Lays out: one offset per channel, then per block two
 * words (label table offset + index width, index data offset), then the tables and the
 * packed indices. */
std::vector<std::uint8_t> encode(const std::vector<std::uint64_t> & volume,
                                 const std::int32_t extent[3], const std::int32_t block[3]) {
    const int grid[3]{(extent[0] + block[0] - 1) / block[0],
                      (extent[1] + block[1] - 1) / block[1],
                      (extent[2] + block[2] - 1) / block[2]};
    const std::size_t blocks = static_cast<std::size_t>(grid[0]) * grid[1] * grid[2];
    std::vector<std::uint32_t> words(1 + 2 * blocks, 0);
    words[0] = 1;// one channel, its block table starts right after this offset

    for (int bz = 0; bz < grid[2]; ++bz)
    for (int by = 0; by < grid[1]; ++by)
    for (int bx = 0; bx < grid[0]; ++bx) {
        const std::size_t bi = (static_cast<std::size_t>(bz) * grid[1] + by) * grid[0] + bx;
        // the labels this block uses, in first-seen order of the table we build
        std::vector<std::uint64_t> table;
        const auto indexOf = [&table](const std::uint64_t v) {
            const auto it = std::find(std::begin(table), std::end(table), v);
            if (it != std::end(table)) { return static_cast<std::uint32_t>(it - std::begin(table)); }
            table.push_back(v);
            return static_cast<std::uint32_t>(table.size() - 1);
        };
        const int x1 = std::min(extent[0], (bx + 1) * block[0]);
        const int y1 = std::min(extent[1], (by + 1) * block[1]);
        const int z1 = std::min(extent[2], (bz + 1) * block[2]);
        std::map<std::size_t, std::uint32_t> assigned;// voxel within block -> table index
        for (int z = bz * block[2]; z < z1; ++z)
        for (int y = by * block[1]; y < y1; ++y)
        for (int x = bx * block[0]; x < x1; ++x) {
            const std::size_t within = (static_cast<std::size_t>(z - bz * block[2]) * block[1]
                                        + (y - by * block[1])) * block[0] + (x - bx * block[0]);
            assigned[within] = indexOf(volume[idx(extent, x, y, z)]);
        }
        int indexBits = 0;
        for (const int candidate : {0, 1, 2, 4, 8, 16, 32}) {
            const std::uint64_t room = candidate == 0 ? 1ull : (1ull << candidate);
            if (table.size() <= room) { indexBits = candidate; break; }
        }
        const auto tableAt = static_cast<std::uint32_t>(words.size());
        for (const auto label : table) {
            std::uint32_t lo = 0, hi = 0;
            std::memcpy(&lo, reinterpret_cast<const char *>(&label), 4);
            std::memcpy(&hi, reinterpret_cast<const char *>(&label) + 4, 4);
            words.push_back(lo);
            words.push_back(hi);
        }
        std::uint32_t valuesAt = 0;
        if (indexBits != 0) {
            const auto voxelsPerBlock = static_cast<std::size_t>(block[0]) * block[1] * block[2];
            const auto bitsTotal = voxelsPerBlock * static_cast<std::size_t>(indexBits);
            const auto wordsNeeded = (bitsTotal + 31) / 32;
            valuesAt = static_cast<std::uint32_t>(words.size());
            words.resize(words.size() + wordsNeeded, 0);
            for (const auto & [within, value] : assigned) {
                const auto bitAt = within * static_cast<std::size_t>(indexBits);
                words[valuesAt + bitAt / 32] |= value << (bitAt % 32);
            }
        }
        // offsets in the block table are relative to the channel's start, which is word 1
        words[1 + bi * 2] = (tableAt - 1) | (static_cast<std::uint32_t>(indexBits) << 24);
        words[1 + bi * 2 + 1] = indexBits == 0 ? 0 : (valuesAt - 1);
    }
    std::vector<std::uint8_t> out(words.size() * 4);
    std::memcpy(out.data(), words.data(), out.size());
    return out;
}

std::vector<std::uint8_t> le64(const std::vector<std::uint64_t> & values) {
    std::vector<std::uint8_t> out(values.size() * 8);
    std::memcpy(out.data(), values.data(), out.size());
    return out;
}
}

int main() {
    section("compressed_segmentation round trip");
    {
        const std::int32_t block[3]{8, 8, 8};
        std::mt19937 rng{20261002};
        bool allExact = true;
        int cases = 0;
        /* One distinct label, then 2, 4, 16, 256 and many: the format picks an index width
         * per block from the number of labels in it, so this walks every width including
         * the zero-bit case where a block is one label throughout. */
        for (const int distinct : {1, 2, 3, 5, 17, 300, 4000}) {
            // extents deliberately not multiples of the block size, so blocks get clipped
            for (const auto & e : std::vector<std::vector<std::int32_t>>{{8,8,8},{9,7,5},{17,13,11},{121,54,77}}) {
                const std::int32_t extent[3]{e[0], e[1], e[2]};
                const std::size_t voxels = static_cast<std::size_t>(extent[0]) * extent[1] * extent[2];
                std::vector<std::uint64_t> volume(voxels);
                std::uniform_int_distribution<std::uint64_t> pick{0, static_cast<std::uint64_t>(distinct) - 1};
                for (auto & v : volume) {
                    // large ids, as real segmentations have
                    v = pick(rng) == 0 ? 0 : 1000000ull + pick(rng) * 7919ull;
                }
                const auto encoded = encode(volume, extent, block);
                std::vector<std::uint64_t> decoded;
                const auto ok = precomputed::decodeCompressedSegmentation(
                            encoded.data(), encoded.size(), extent, block, 0, 1, decoded);
                allExact = allExact && ok && decoded == volume;
                ++cases;
            }
        }
        check(allExact, "28 volumes across every index width and clipped blocks decode exactly");
        check(cases == 28, "all the cases ran");

        // a block of one label must use the zero-bit path, which is the whole point of the format
        const std::int32_t extent[3]{8, 8, 8};
        std::vector<std::uint64_t> uniform(512, 42);
        const auto encoded = encode(uniform, extent, block);
        std::vector<std::uint64_t> decoded;
        check(precomputed::decodeCompressedSegmentation(encoded.data(), encoded.size(), extent, block, 0, 1, decoded)
              && decoded == uniform, "a block of a single label round-trips");
        check(encoded.size() < 64, "and costs only the table, not a bit per voxel");
    }

    section("refusing malformed input rather than half-decoding");
    {
        const std::int32_t extent[3]{8, 8, 8};
        const std::int32_t block[3]{8, 8, 8};
        std::vector<std::uint64_t> volume(512);
        for (std::size_t i = 0; i < volume.size(); ++i) { volume[i] = i % 7; }
        auto encoded = encode(volume, extent, block);
        std::vector<std::uint64_t> decoded;
        check(precomputed::decodeCompressedSegmentation(encoded.data(), encoded.size(), extent, block, 0, 1, decoded),
              "the intact chunk decodes");
        auto truncated = encoded;
        truncated.resize(truncated.size() / 2);
        check(!precomputed::decodeCompressedSegmentation(truncated.data(), truncated.size(), extent, block, 0, 1, decoded),
              "a chunk cut in half is refused");
        check(!precomputed::decodeCompressedSegmentation(encoded.data(), 0, extent, block, 0, 1, decoded),
              "an empty chunk is refused");
        const std::int32_t zero[3]{0, 8, 8};
        check(!precomputed::decodeCompressedSegmentation(encoded.data(), encoded.size(), zero, block, 0, 1, decoded),
              "a zero extent is refused");
        check(!precomputed::decodeCompressedSegmentation(encoded.data(), encoded.size(), extent, block, 9, 1, decoded),
              "a channel index past the channel count is refused");
    }

    section("raw encoding");
    {
        const std::int32_t extent[3]{4, 3, 2};
        std::vector<std::uint64_t> volume(24);
        for (std::size_t i = 0; i < volume.size(); ++i) { volume[i] = 1ull << (i % 60); }
        const auto bytes = le64(volume);
        std::vector<std::uint64_t> decoded;
        check(precomputed::decodeRaw(bytes.data(), bytes.size(), extent, decoded) && decoded == volume,
              "a raw chunk is read straight through, x fastest");
        check(!precomputed::decodeRaw(bytes.data(), bytes.size() - 1, extent, decoded),
              "one byte short is refused");
    }

    section("addressing a chunk in a shard");
    {
        // the interleave gives each axis only as many bits as its grid needs
        const std::uint64_t gridSize[3]{16, 7, 10};
        const auto code = [&](std::uint64_t x, std::uint64_t y, std::uint64_t z) {
            const std::uint64_t g[3]{x, y, z};
            return precomputed::compressedMorton(g, gridSize);
        };
        check(code(0,0,0) == 0, "the first chunk is zero");
        check(code(1,0,0) == 1 && code(0,1,0) == 2 && code(0,0,1) == 4,
              "the low bit of each axis interleaves in order");
        check(code(0,0,2) == 32, "an axis keeps its place as the others contribute");
        // once y is exhausted at 3 bits, x and z carry on without it
        check(code(8,0,0) == (1ull << 9), "past an exhausted axis the rest close up");
        bool distinct = true;
        std::set<std::uint64_t> seen;
        for (std::uint64_t z = 0; z < gridSize[2]; ++z)
        for (std::uint64_t y = 0; y < gridSize[1]; ++y)
        for (std::uint64_t x = 0; x < gridSize[0]; ++x) {
            distinct = distinct && seen.insert(code(x, y, z)).second;
        }
        check(distinct && seen.size() == 16 * 7 * 10, "every chunk of the grid gets its own number");

        /* Filenames, against what a real store was observed to use: 8 and 5 shard bits give
         * two hex digits, 2 and 0 bits give one. */
        check(precomputed::shardName(0, 0) == "0.shard", "no shard bits: one digit");
        check(precomputed::shardName(0, 2) == "0.shard" && precomputed::shardName(2, 2) == "2.shard",
              "two shard bits: one digit");
        check(precomputed::shardName(1, 5) == "01.shard" && precomputed::shardName(16, 5) == "10.shard",
              "five shard bits: two digits, as observed");
        check(precomputed::shardName(15, 8) == "0f.shard", "eight shard bits: two digits, lowercase hex");
        check(precomputed::shardName(255, 8) == "ff.shard", "and the last of them");

        precomputed::Scale scale;
        scale.size[0] = 1928; scale.size[1] = 853; scale.size[2] = 1220;
        scale.chunk[0] = scale.chunk[1] = scale.chunk[2] = 128;
        scale.preshiftBits = 3; scale.minishardBits = 0; scale.shardBits = 8;
        const std::uint64_t origin[3]{0, 0, 0};
        const auto target = precomputed::shardFor(scale, origin);
        check(target.chunkId == 0 && target.shard == 0 && target.minishard == 0,
              "the first chunk lands in shard zero");
        check(scale.gridSize(0) == 16 && scale.gridSize(1) == 7 && scale.gridSize(2) == 10,
              "the grid is as the volume and chunk sizes imply");
        check(scale.clippedExtent(0, 15) == 1928 - 15 * 128 && scale.clippedExtent(0, 0) == 128,
              "a chunk on the far face is clipped, not padded");
    }

    section("the two levels of index inside a shard");
    {
        precomputed::Scale scale;
        scale.minishardBits = 0;
        // the shard opens with 16 bytes per minishard: where that minishard's index sits
        const auto header = le64({40, 64});
        const auto range = precomputed::minishardIndexRange(header, 0, 0);
        check(range.begin == 16 + 40 && range.end == 16 + 64, "the minishard index is found past the header");
        check(precomputed::minishardIndexRange(header, 3, 0).empty(), "a minishard that is not there is empty");
        check(precomputed::minishardIndexRange({}, 0, 0).empty(), "so is a truncated header");

        /* Three delta-encoded arrays: ids, then offsets against the end of the previous
         * chunk, then sizes. Walked from the start, because nothing can be seeked to. */
        const auto index = le64({ 5, 2, 1,      // ids: 5, 7, 8
                                  0, 10, 0,     // offsets
                                  100, 50, 25});// sizes
        const auto first = precomputed::chunkRange(index, 5, 1000);
        const auto second = precomputed::chunkRange(index, 7, 1000);
        const auto third = precomputed::chunkRange(index, 8, 1000);
        check(first.begin == 1000 && first.size() == 100, "the first entry");
        check(second.begin == 1000 + 100 + 10 && second.size() == 50, "the second, offset past the first");
        check(third.begin == second.end && third.size() == 25, "the third, with a zero gap");
        check(precomputed::chunkRange(index, 6, 1000).empty(),
              "an id that is not in the index is empty — an absent chunk is an empty one");
        check(precomputed::chunkRange({}, 5, 1000).empty(), "and an empty index finds nothing");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
