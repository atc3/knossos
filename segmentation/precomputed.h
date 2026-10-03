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

/* Reading a Neuroglancer "precomputed" segmentation.
 *
 * KNOSSOS stores a segmentation as one file per block, snappy-compressed, named after its
 * cube coordinate. Neuroglancer's precomputed format stores the same thing quite
 * differently, and a dataset in it currently loads as an empty layer because every request
 * for a KNOSSOS-style cube path misses. Two things have to be understood to read one:
 *
 *   The *addressing*. Chunks may be stored individually, at
 *   `<key>/<x0>-<x1>_<y0>-<y1>_<z0>-<z1>`, or — far more common for segmentation, because
 *   a volume has millions of mostly-empty chunks — gathered into shard files with two
 *   levels of index inside them. Finding one chunk in a sharded store means a compressed
 *   Morton code, a shard number, a minishard number, and two index lookups before the data.
 *
 *   The *encoding*. `raw` is a plain array. `compressed_segmentation` splits the chunk into
 *   small blocks, each with its own table of the labels appearing in it and its voxels
 *   stored as bit-packed indices into that table. For a segmentation this is a large win —
 *   a block of one label costs 8 bytes — which is exactly why it is what segmentations use.
 *
 * All of it is integer arithmetic over bit-packed data, so it lives here, free of Qt and of
 * the loader, and is tested on its own against vectors built from the spec and against a
 * chunk from a real dataset. See tests/precomputed_test.cpp.
 *
 * Spec: https://github.com/google/neuroglancer/blob/master/src/datasource/precomputed/
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace precomputed {

enum class Encoding { Raw, CompressedSegmentation, Unsupported };

/* One resolution level of a precomputed volume.
 *
 * `key` is the directory it lives in — arbitrary text, often the resolution ("8_8_8") but
 * just as often "mag1". Nothing may be inferred from it. */
struct Scale {
    std::string key;
    std::int64_t size[3]{0, 0, 0};
    std::int32_t chunk[3]{0, 0, 0};
    std::int64_t voxelOffset[3]{0, 0, 0};
    double resolution[3]{1, 1, 1};
    Encoding encoding{Encoding::Unsupported};
    std::int32_t csegBlock[3]{8, 8, 8};// compressed_segmentation block size

    bool sharded{false};
    int preshiftBits{0};
    int minishardBits{0};
    int shardBits{0};
    bool shardedIndexGzip{false};// minishard_index_encoding == "gzip"
    bool shardedDataGzip{false}; // data_encoding == "gzip"

    // blocks of `chunk` needed to cover `size`, per axis
    std::int64_t gridSize(const int axis) const {
        if (axis < 0 || axis > 2 || chunk[axis] <= 0) {
            return 0;
        }
        return (size[axis] + chunk[axis] - 1) / chunk[axis];
    }
    /* A chunk on the far face is clipped to the volume rather than padded, so its decoded
     * extent is smaller than `chunk` — which the decoder has to be told, since the bit
     * packing is laid out for the clipped size. */
    std::int32_t clippedExtent(const int axis, const std::int64_t gridPos) const {
        if (axis < 0 || axis > 2) {
            return 0;
        }
        const auto begin = gridPos * chunk[axis];
        return static_cast<std::int32_t>(std::max<std::int64_t>(0, std::min<std::int64_t>(chunk[axis], size[axis] - begin)));
    }
};

/* The interleaving precomputed uses to number chunks within a shard.
 *
 * Not plain Morton: each axis contributes only as many bits as its grid actually needs, and
 * once an axis runs out the remaining bits of the others carry on without it. A plain
 * three-way interleave gives the wrong number for any grid that is not a cube, which for a
 * volume like 16x7x10 blocks is every grid. */
inline std::uint64_t compressedMorton(const std::uint64_t grid[3], const std::uint64_t gridSize[3]) {
    int bits[3]{0, 0, 0};
    for (int a = 0; a < 3; ++a) {
        while ((1ull << bits[a]) < gridSize[a]) { ++bits[a]; }
    }
    std::uint64_t code = 0;
    int outBit = 0;
    for (int bit = 0; bit < 64; ++bit) {
        bool any = false;
        for (int a = 0; a < 3; ++a) {
            if (bit < bits[a]) {
                any = true;
                if ((grid[a] >> bit) & 1ull) {
                    code |= 1ull << outBit;
                }
                ++outBit;
            }
        }
        if (!any) {
            break;
        }
    }
    return code;
}

struct ShardTarget {
    std::uint64_t chunkId{0};
    std::uint64_t shard{0};
    std::uint64_t minishard{0};
};

// Which shard and minishard hold a chunk. `hash` is only ever "identity" in practice.
inline ShardTarget shardFor(const Scale & scale, const std::uint64_t grid[3]) {
    const std::uint64_t gridSize[3]{static_cast<std::uint64_t>(scale.gridSize(0)),
                                    static_cast<std::uint64_t>(scale.gridSize(1)),
                                    static_cast<std::uint64_t>(scale.gridSize(2))};
    ShardTarget out;
    out.chunkId = compressedMorton(grid, gridSize);
    const auto shifted = out.chunkId >> scale.preshiftBits;
    out.minishard = scale.minishardBits >= 64 ? shifted : (shifted & ((1ull << scale.minishardBits) - 1));
    const auto rest = shifted >> scale.minishardBits;
    out.shard = scale.shardBits >= 64 ? rest : (rest & ((1ull << scale.shardBits) - 1));
    return out;
}

/* The shard's filename: the number in lowercase hex, padded to ceil(shardBits/4) digits and
 * at least one. Confirmed against a real store, where 8 and 5 shard bits give two digits
 * ("0f.shard", "01.shard") and 2 and 0 bits give one ("0.shard"). */
inline std::string shardName(const std::uint64_t shard, const int shardBits) {
    const auto digits = std::max(1, (shardBits + 3) / 4);
    std::string out(static_cast<std::size_t>(digits), '0');
    auto value = shard;
    for (int i = digits - 1; i >= 0 && value != 0; --i) {
        out[static_cast<std::size_t>(i)] = "0123456789abcdef"[value & 0xf];
        value >>= 4;
    }
    return out + ".shard";
}

// Byte range of a shard file, half open.
struct Range {
    std::uint64_t begin{0}, end{0};
    std::uint64_t size() const { return end > begin ? end - begin : 0; }
    bool empty() const { return size() == 0; }
};

// Where the index for a minishard sits. The shard begins with 16 bytes per minishard,
// holding that minishard's index as offsets past the end of this header.
inline Range minishardIndexRange(const std::vector<std::uint8_t> & shardHeader,
                                 const std::uint64_t minishard, const int minishardBits) {
    Range out;
    const auto count = 1ull << minishardBits;
    const auto headerBytes = count * 16;
    if (minishard >= count || shardHeader.size() < headerBytes) {
        return out;
    }
    std::uint64_t begin = 0, end = 0;
    std::memcpy(&begin, shardHeader.data() + minishard * 16, 8);
    std::memcpy(&end, shardHeader.data() + minishard * 16 + 8, 8);
    out.begin = headerBytes + begin;
    out.end = headerBytes + end;
    return out;
}

/* Finds one chunk's bytes in a minishard index.
 *
 * The index is three arrays of the same length — chunk ids, then offsets, then sizes — and
 * all three are delta encoded, the offsets against the end of the previous chunk. So it has
 * to be walked from the start; there is no seeking to an entry.
 *
 * `indexBase` is where the data region begins, which is the end of the shard's minishard
 * header. Returns an empty range when the chunk is not in this minishard, which is normal:
 * an absent chunk is an empty one. */
inline Range chunkRange(const std::vector<std::uint8_t> & index, const std::uint64_t wanted,
                        const std::uint64_t indexBase) {
    Range out;
    const auto entries = index.size() / 24;
    if (entries == 0) {
        return out;
    }
    const auto at = [&index](const std::size_t i) {
        std::uint64_t v = 0;
        std::memcpy(&v, index.data() + i * 8, 8);
        return v;
    };
    std::uint64_t id = 0, cursor = 0;
    for (std::size_t k = 0; k < entries; ++k) {
        id += at(k);
        const auto begin = cursor + at(entries + k);
        const auto size = at(2 * entries + k);
        cursor = begin + size;
        if (id == wanted) {
            out.begin = indexBase + begin;
            out.end = out.begin + size;
            return out;
        }
    }
    return out;
}

/* Expands one compressed_segmentation chunk into `out`, indexed [z][y][x] as KNOSSOS holds
 * a cube, with `extent` the chunk's clipped size.
 *
 * The chunk opens with one offset per channel. A channel is a table of two words per block —
 * where that block's label table is, how many bits each voxel index takes, and where the
 * packed indices are. Zero bits means the block is one label throughout, which is the case
 * that makes the format worth using.
 *
 * Returns false rather than reading past the end: a truncated or malformed chunk is
 * something to report, not to half-decode. */
inline bool decodeCompressedSegmentation(const std::uint8_t * data, const std::size_t bytes,
                                         const std::int32_t extent[3], const std::int32_t block[3],
                                         const int channel, const int numChannels,
                                         std::vector<std::uint64_t> & out) {
    const auto words = bytes / 4;
    const auto word = [data](const std::size_t i) {
        std::uint32_t v = 0;
        std::memcpy(&v, data + i * 4, 4);
        return v;
    };
    for (int a = 0; a < 3; ++a) {
        if (extent[a] <= 0 || block[a] <= 0) {
            return false;
        }
    }
    const std::size_t voxels = static_cast<std::size_t>(extent[0]) * extent[1] * extent[2];
    out.assign(voxels, 0);
    /* How many channels there are is in the volume's info, not in the chunk — the chunk
     * opens with one offset per channel and nothing that says how many that is. So it has
     * to be passed in; reading word[channel] without it would happily treat some block's
     * table offset as a channel offset and decode nonsense. */
    if (channel < 0 || numChannels <= 0 || channel >= numChannels
            || words < static_cast<std::size_t>(numChannels)) {
        return false;
    }
    const std::size_t channelBase = word(static_cast<std::size_t>(channel));
    const int grid[3]{(extent[0] + block[0] - 1) / block[0],
                      (extent[1] + block[1] - 1) / block[1],
                      (extent[2] + block[2] - 1) / block[2]};
    const std::size_t blocks = static_cast<std::size_t>(grid[0]) * grid[1] * grid[2];
    if (channelBase + 2 * blocks > words) {
        return false;
    }
    for (int bz = 0; bz < grid[2]; ++bz)
    for (int by = 0; by < grid[1]; ++by)
    for (int bx = 0; bx < grid[0]; ++bx) {
        const std::size_t bi = (static_cast<std::size_t>(bz) * grid[1] + by) * grid[0] + bx;
        const auto w0 = word(channelBase + bi * 2);
        const auto w1 = word(channelBase + bi * 2 + 1);
        const std::size_t lookupAt = channelBase + (w0 & 0xffffffu);
        const auto indexBits = static_cast<int>((w0 >> 24) & 0xffu);
        const std::size_t valuesAt = channelBase + w1;
        if (indexBits != 0 && indexBits != 1 && indexBits != 2 && indexBits != 4
                && indexBits != 8 && indexBits != 16 && indexBits != 32) {
            return false;
        }
        /* The label table holds as many labels as this block actually uses, not 2^bits of
         * them — the index width is rounded up to a power of two, the table is not. So each
         * label is bounds-checked where it is read rather than the table being required to
         * have a size it does not have. */
        const std::uint64_t indexLimit = indexBits == 0 ? 1ull : (1ull << indexBits);
        bool inBounds = true;
        const auto label = [&](const std::size_t k) -> std::uint64_t {
            if (lookupAt + 2 * k + 2 > words) {
                inBounds = false;
                return 0;
            }
            std::uint64_t v = 0;
            std::memcpy(&v, data + (lookupAt + 2 * k) * 4, 8);
            return v;
        };
        const int x1 = std::min(extent[0], (bx + 1) * block[0]);
        const int y1 = std::min(extent[1], (by + 1) * block[1]);
        const int z1 = std::min(extent[2], (bz + 1) * block[2]);
        for (int z = bz * block[2]; z < z1; ++z)
        for (int y = by * block[1]; y < y1; ++y)
        for (int x = bx * block[0]; x < x1; ++x) {
            std::uint64_t value = 0;
            if (indexBits == 0) {
                value = label(0);
            } else {
                // voxel order inside a block is x fastest, over the block's full size
                const std::size_t within = (static_cast<std::size_t>(z - bz * block[2]) * block[1]
                                            + (y - by * block[1])) * block[0] + (x - bx * block[0]);
                const std::size_t bitAt = within * static_cast<std::size_t>(indexBits);
                const std::size_t wordAt = valuesAt + bitAt / 32;
                if (wordAt >= words) {
                    return false;
                }
                const auto shift = bitAt % 32;
                const auto mask = indexBits == 32 ? 0xffffffffu : ((1u << indexBits) - 1u);
                const auto idx = (word(wordAt) >> shift) & mask;
                if (idx >= indexLimit) {
                    return false;
                }
                value = label(idx);
            }
            if (!inBounds) {
                return false;
            }
            out[(static_cast<std::size_t>(z) * extent[1] + y) * extent[0] + x] = value;
        }
    }
    return true;
}

// `raw`: a plain little-endian array, x fastest, already in the order KNOSSOS wants.
inline bool decodeRaw(const std::uint8_t * data, const std::size_t bytes,
                      const std::int32_t extent[3], std::vector<std::uint64_t> & out) {
    const std::size_t voxels = static_cast<std::size_t>(extent[0]) * extent[1] * extent[2];
    if (extent[0] <= 0 || extent[1] <= 0 || extent[2] <= 0 || bytes < voxels * 8) {
        return false;
    }
    out.assign(voxels, 0);
    std::memcpy(out.data(), data, voxels * 8);
    return true;
}

}
