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

#include "coordinate.h"
#include "segmentation/segmentationsplit.h"// brush_t, needed by FloodFillOptions

#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using CubeCoordSet = std::unordered_set<CoordOfCube>;
using subobjectRetrievalMap = std::unordered_map<uint64_t, Coordinate>;

bool isInsideSphere(const double xi, const double yi, const double zi, const double radius);

void coordCubesMarkChanged(const CubeCoordSet & cubeChangeSet);
std::optional<std::uint64_t> readLayerVoxel(const Coordinate & pos, const std::size_t layerIdx);
std::uint64_t readVoxel(const Coordinate & pos);
subobjectRetrievalMap readVoxels(const Coordinate & centerPos, const brush_t &);
void collectFromMovementArea();
void assignNewIdInMovementArea(const std::uint64_t newId);
bool writeVoxel(const Coordinate & pos, const uint64_t value, bool isMarkChanged = true);
void writeVoxels(const Coordinate & centerPos, const uint64_t value, const brush_t &, bool isMarkChanged = true);
void coordCubesMarkChanged(const CubeCoordSet & cubeChangeSet);
CubeCoordSet processRegionByStridedBuf(const Coordinate & globalFirst, const Coordinate &  globalLast, char * data, const Coordinate & strides, bool isWrite, bool markChanged);
void listFill(const Coordinate & centerPos, const brush_t & brush, const uint64_t fillsoid, const std::unordered_set<Coordinate> & voxels);

// Callback-driven region access. Both reuse the same cube-outer/voxel-inner traversal as
// writeVoxels(), so they inherit its semantics exactly: cubes that are not currently
// resident are skipped, and the returned CubeCoordSet lists only the cubes actually
// visited — compare it against the intended cube set to detect dropped writes.
using VoxelVisitor = std::function<void(std::uint64_t, const Coordinate &)>;
using VoxelPredicate = std::function<bool(const Coordinate &)>;

CubeCoordSet readRegion(const Coordinate & globalFirst, const Coordinate & globalLast, const VoxelVisitor &);
// Reads back exactly the voxels a brush stamp covers, i.e. the same AABB writeVoxels() uses.
CubeCoordSet readBrushRegion(const Coordinate & centerPos, const brush_t &, const VoxelVisitor &);
// listFill() generalised: writes `value` wherever `inside` returns true. Unlike
// processRegionByStridedBuf(isWrite=true) this leaves non-matching voxels untouched,
// so it will not erase other labels sharing the region.
CubeCoordSet writeVoxelsWhere(const Coordinate & globalFirst, const Coordinate & globalLast, const VoxelPredicate & inside, const std::uint64_t value, bool markChanged = true);
// Splits the cubes covering a region into (resident, missing).
std::pair<CubeCoordSet, CubeCoordSet> regionCubeResidency(const Coordinate & globalFirst, const Coordinate & globalLast);

/* The box around `pos` that can possibly be in memory: the M³ supercube the loader keeps
 * resident, clipped to the movement area. Anything outside it is guaranteed absent, so a
 * scan that wants "everything currently available" should bound itself by this rather
 * than by the movement area, which may be the whole dataset. */
std::pair<Coordinate, Coordinate> residentBoxAround(const Coordinate & pos);

// Raw pointer to a resident overlay cube, or nullptr — for snapshotting a cube whole.
void * residentCubePointer(std::size_t layerId, const CoordOfCube &);
// Uncompressed size of one overlay cube, in bytes.
std::size_t overlayCubeBytes(std::size_t layerId);

// Replaces every occurrence of `from` with `to` in a region, returning how many voxels
// changed. A voxel-level relabel, leaving the object graph alone.
std::size_t processRegionReplacing(const Coordinate & globalFirst, const Coordinate & globalLast, std::uint64_t from, std::uint64_t to, bool markChanged = true);

/* Flood fill that treats a non-resident cube as a wall rather than as background.
 *
 * This is the whole point of it. readVoxel() returns the background id for a cube that is
 * not in memory, which is indistinguishable from real background, so a naive flood spills
 * into unloaded blocks and every write there is silently dropped. This one checks
 * residency before believing a read, stops at the boundary, and hands back the frontier
 * voxels it refused to cross so a caller can decide whether to load and continue. */
struct FloodFillOptions {
    bool threeDimensional{false};       // 6-connected 3D, else 2D within `view`'s plane
    brush_t::view_t view{brush_t::view_t::xy};
    std::size_t maxVoxels{20000000};    // safety cap; a runaway 3D fill can be enormous
};

struct FloodFillResult {
    std::size_t voxelsFilled{0};
    std::size_t blockedAtBoundary{0};   // frontier voxels sitting in a non-resident cube
    bool seedNotLoaded{false};
    bool seedAlreadyFilled{false};
    bool hitCap{false};
    CubeCoordSet cubes;                 // cubes actually written
    Coordinate filledMin, filledMax;    // bounding box of the filled voxels
    CubeCoordSet pendingCubes;          // non-resident cubes the fill wanted to enter
    std::unordered_set<Coordinate> deferred;// frontier voxels in those cubes, to resume from
};

// Fills from `seeds`, replacing `targetSoid` with `fillsoid`. Marks the written cubes.
FloodFillResult floodFillFrom(const std::unordered_set<Coordinate> & seeds, std::uint64_t targetSoid, std::uint64_t fillsoid,
                              const FloodFillOptions &, const Coordinate & areaMin, const Coordinate & areaMax);
// Convenience: takes the value to replace from the seed voxel itself.
FloodFillResult floodFill(const Coordinate & seed, std::uint64_t fillsoid,
                          const FloodFillOptions &, const Coordinate & areaMin, const Coordinate & areaMax);
