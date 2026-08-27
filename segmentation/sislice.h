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

/* One painted key slice of a shape-interpolation chain.
 *
 * Deliberately free of Qt and of any KNOSSOS type, so the index arithmetic can be
 * exercised on its own — see tests/sislice_test.cpp. */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

// integer floor division, so negative mask indices round the right way
inline int siFloorDiv(const int a, const int b) {
    const auto q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

// One painted key slice: a dense 2D mask over its own tight bounding box.
//
// The slice owns the whole coordinate<->index mapping, including the voxel step, so that
// growing the bounding box can shift the origin without any caller having to know about it.
class SISlice {
public:
    int depth{0};                       // global mag1 coordinate along the normal axis
    int uMin{0}, vMin{0};               // global mag1 coordinate of mask index 0
    int uStep{1}, vStep{1};             // mag1 units per mask index (Dataset::scaleFactor)
    int uSize{0}, vSize{0};             // mask extent, in mask indices
    std::vector<std::uint8_t> mask;     // uSize*vSize, row major: mask[v * uSize + u]

    bool empty() const { return voxelCount == 0; }
    std::size_t count() const { return voxelCount; }

    int uIndexOf(const int globalU) const { return siFloorDiv(globalU - uMin, uStep); }
    int vIndexOf(const int globalV) const { return siFloorDiv(globalV - vMin, vStep); }
    int uCoordOf(const int u) const { return uMin + u * uStep; }
    int vCoordOf(const int v) const { return vMin + v * vStep; }

    std::uint8_t at(const int u, const int v) const {
        return (u < 0 || v < 0 || u >= uSize || v >= vSize) ? 0 : mask[static_cast<std::size_t>(v) * uSize + u];
    }
    void set(int u, int v, std::uint8_t value);
    // replace the whole mask (used for computed, rather than painted, masks)
    void adoptMask(std::vector<std::uint8_t> && newMask);
    // grow the bounding box so that mask index (u,v) becomes representable, preserving
    // content and shifting uMin/vMin to match when padding the low side
    void reserveIndex(int u, int v);
    void shrinkToFit();

private:
    std::size_t voxelCount{0};
};


namespace {
// grow the mask bounding box in blocks so a drag doesn’t reallocate on every stamp
constexpr int GROW_BLOCK = 64;
}

inline void SISlice::reserveIndex(const int u, const int v) {
    if (u >= 0 && v >= 0 && u < uSize && v < vSize) {
        return;
    }
    const auto padLow = [](const int idx){ return idx < 0 ? ((-idx + GROW_BLOCK - 1) / GROW_BLOCK) * GROW_BLOCK : 0; };
    const auto padU = padLow(u);
    const auto padV = padLow(v);
    const auto padHigh = [](const int idx, const int size){ return idx >= size ? ((idx - size) / GROW_BLOCK + 1) * GROW_BLOCK : 0; };
    const auto newUSize = uSize + padU + padHigh(u + padU, uSize + padU);
    const auto newVSize = vSize + padV + padHigh(v + padV, vSize + padV);

    std::vector<std::uint8_t> grown(static_cast<std::size_t>(newUSize) * newVSize, 0);
    for (int y = 0; y < vSize; ++y) {
        const auto * src = mask.data() + static_cast<std::size_t>(y) * uSize;
        auto * dst = grown.data() + static_cast<std::size_t>(y + padV) * newUSize + padU;
        std::copy(src, src + uSize, dst);
    }
    mask = std::move(grown);
    uSize = newUSize;
    vSize = newVSize;
    // padding the low side moves mask index 0 to a smaller global coordinate
    uMin -= padU * uStep;
    vMin -= padV * vStep;
}

inline void SISlice::set(const int u, const int v, const std::uint8_t value) {
    if (value == 0 && (u < 0 || v < 0 || u >= uSize || v >= vSize)) {
        return; // clearing outside the box is a no-op; don’t grow for it
    }
    auto uu = u, vv = v;
    if (uu < 0 || vv < 0 || uu >= uSize || vv >= vSize) {
        const auto oldUMin = uMin, oldVMin = vMin;
        reserveIndex(uu, vv);
        uu += (oldUMin - uMin) / uStep; // reserveIndex may have shifted the origin
        vv += (oldVMin - vMin) / vStep;
    }
    auto & cell = mask[static_cast<std::size_t>(vv) * uSize + uu];
    if (cell != value) {
        voxelCount += (value != 0) ? 1 : -1;
        cell = value;
    }
}

inline void SISlice::adoptMask(std::vector<std::uint8_t> && newMask) {
    mask = std::move(newMask);
    voxelCount = 0;
    for (const auto cell : mask) {
        voxelCount += (cell != 0) ? 1 : 0;
    }
}

inline void SISlice::shrinkToFit() {
    if (voxelCount == 0) {
        mask.clear();
        uSize = vSize = 0;
        return;
    }
    int minU = uSize, maxU = -1, minV = vSize, maxV = -1;
    for (int v = 0; v < vSize; ++v) {
        for (int u = 0; u < uSize; ++u) {
            if (mask[static_cast<std::size_t>(v) * uSize + u] != 0) {
                minU = std::min(minU, u); maxU = std::max(maxU, u);
                minV = std::min(minV, v); maxV = std::max(maxV, v);
            }
        }
    }
    const auto newUSize = maxU - minU + 1;
    const auto newVSize = maxV - minV + 1;
    if (newUSize == uSize && newVSize == vSize) {
        return;
    }
    std::vector<std::uint8_t> tight(static_cast<std::size_t>(newUSize) * newVSize, 0);
    for (int v = 0; v < newVSize; ++v) {
        const auto * src = mask.data() + static_cast<std::size_t>(v + minV) * uSize + minU;
        std::copy(src, src + newUSize, tight.data() + static_cast<std::size_t>(v) * newUSize);
    }
    mask = std::move(tight);
    uSize = newUSize;
    vSize = newVSize;
    uMin += minU * uStep;
    vMin += minV * vStep;
}


