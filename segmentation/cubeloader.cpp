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

#include "cubeloader.h"

#include "annotation/annotation.h"
#include "loader.h"
#include "segmentation.h"
#include "segmentationsplit.h"
#include "stateInfo.h"

#include <QMutex>

#include <boost/multi_array.hpp>

#include <cstdint>
#include <limits>

std::pair<bool, void *> getRawCube(const Coordinate & pos, const std::size_t layerIdx = Segmentation::singleton().layerId) {
    QMutexLocker locker(&state->protectCube2Pointer);
    auto * rawcube = cubeQuery(state->cube2Pointer, layerIdx, Dataset::datasets[layerIdx].magIndex, Dataset::datasets[layerIdx].global2cube(pos));
    return std::make_pair(rawcube != nullptr, rawcube);
}

template<typename T = std::uint64_t>
boost::multi_array_ref<T, 3> getCubeRef(void * const rawcube, const std::size_t layerIdx = Segmentation::singleton().layerId) {
    const auto cubeShape = Dataset::datasets[layerIdx].cubeShape;
    const auto dims = boost::extents[cubeShape.z][cubeShape.y][cubeShape.x];
    return boost::multi_array_ref<T, 3>(reinterpret_cast<T *>(rawcube), dims);
}

// can hold ids as well as raw data
std::optional<std::uint64_t> readLayerVoxel(const Coordinate & pos, const std::size_t layerIdx) {
    auto cubeIt = getRawCube(pos, layerIdx);
    if (!cubeIt.first || (Dataset::datasets[layerIdx].isOverlay() && Annotation::singleton().outsideMovementArea(pos))) {
        return std::nullopt;
    }
    const auto inCube = pos.insideCube(Dataset::datasets[layerIdx].cubeShape, Dataset::datasets[layerIdx].scaleFactor);
    const auto access = [&](auto arg){
        return getCubeRef<decltype(arg)>(cubeIt.second, layerIdx)[inCube.z][inCube.y][inCube.x];
    };
    return Dataset::datasets[layerIdx].isOverlay() ? access(std::uint64_t{}) : access(std::uint8_t{});
}

std::uint64_t readVoxel(const Coordinate & pos) {
    if (!Segmentation::singleton().enabled) {
        return Segmentation::singleton().getBackgroundId();
    }
    return readLayerVoxel(pos, Segmentation::singleton().layerId).value_or(Segmentation::singleton().getBackgroundId());
}

bool writeVoxel(const Coordinate & pos, const uint64_t value, bool isMarkChanged) {
    auto cubeIt = getRawCube(pos);
    if (Annotation::singleton().outsideMovementArea(pos) || !cubeIt.first) {
        return false;
    }
    const auto inCube = pos.insideCube(Dataset::current().cubeShape, Dataset::current().scaleFactor);
    getCubeRef<std::uint64_t>(cubeIt.second)[inCube.z][inCube.y][inCube.x] = value;
    if (isMarkChanged) {
        Loader::Controller::singleton().markCubeAsModified(Segmentation::singleton().layerId, pos.cube(Dataset::current().cubeShape, Dataset::current().scaleFactor), Dataset::current().magnification);
    }
    return true;
}

namespace {
/* Which voxels a write is allowed to replace. Resolved once per operation into a plain
 * pair of values so the per-voxel test is two integer compares, not a singleton lookup —
 * writeVoxels() is deliberately written to inline hard and this sits in its inner loop. */
struct PaintGuard {
    Segmentation::PaintTarget target{Segmentation::PaintTarget::Anything};
    std::uint64_t background{0};

    bool allows(const std::uint64_t voxel, const std::uint64_t value) const {
        if (value == background) {
            return true;// an erase is not a paint; protecting labels must not block it
        }
        switch (target) {
        case Segmentation::PaintTarget::OnlyBackground: return voxel == background || voxel == value;
        case Segmentation::PaintTarget::OnlyExisting: return voxel != background;
        case Segmentation::PaintTarget::Anything: break;
        }
        return true;
    }
    bool unrestricted() const { return target == Segmentation::PaintTarget::Anything; }
};

PaintGuard currentPaintGuard() {
    const auto & seg = Segmentation::singleton();
    // Mode_OverPaint has always meant "only over existing segmentation"; keep it winning
    // over the toggle rather than having two settings disagree
    const auto overPaint = Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_OverPaint);
    return {overPaint ? Segmentation::PaintTarget::OnlyExisting : seg.paintTarget, seg.getBackgroundId()};
}
}

bool isInsideSphere(const double xi, const double yi, const double zi, const double radius) {
    const auto x = xi * Dataset::current().scales[0].x;
    const auto y = yi * Dataset::current().scales[0].y;
    const auto z = zi * Dataset::current().scales[0].z;
    const auto sqdistance = x*x + y*y + z*z;
    return sqdistance < radius * radius;
}

std::pair<Coordinate, Coordinate> getRegion(const floatCoordinate & centerPos, const brush_t & brush) { // calcs global AABB of local coordinate system's region
    const auto posArb = centerPos.toLocal(brush.v1, brush.v2, brush.n);
    const auto width = brush.radius / Dataset::current().scales[0].componentMul(brush.v1).length();
    const auto height = brush.radius / Dataset::current().scales[0].componentMul(brush.v2).length();
    const auto depth = (brush.mode == brush_t::mode_t::three_dim) ? brush.radius / Dataset::current().scales[0].componentMul(brush.n).length() : 0;
    std::vector<floatCoordinate> localPoints(8);
    for (std::size_t i = 0; i < localPoints.size(); ++i) { // all possible combinations. Ordering like in a truth table (with - and + instead of false and true). I just didn't want to type it all out…
        localPoints[i].x = (i < 4) ? posArb.x - width : posArb.x + width;
        localPoints[i].y = (i % 4 < 2) ? posArb.y - height : posArb.y + height;
        localPoints[i].z = (i % 2 == 0) ? posArb.z - depth : posArb.z + depth;
    }
    auto min = floatCoordinate(1, 1, 1) * std::numeric_limits<float>::max();
    floatCoordinate max{0, 0, 0};
    for (const auto localCoord : localPoints) {
        const auto worldCoord = localCoord.toWorldFrom(brush.v1, brush.v2, brush.n).capped(Annotation::singleton().movementAreaMin, Annotation::singleton().movementAreaMax);
        min = {std::min(worldCoord.x, min.x), std::min(worldCoord.y, min.y), std::min(worldCoord.z, min.z)};
        max = {std::max(worldCoord.x, max.x), std::max(worldCoord.y, max.y), std::max(worldCoord.z, max.z)};
    }
    return std::make_pair(min, max);
}

void coordCubesMarkChanged(const CubeCoordSet & cubeChangeSet) {
    for (auto &cubeCoord : cubeChangeSet) {
        Loader::Controller::singleton().markCubeAsModified(Segmentation::singleton().layerId, cubeCoord, Dataset::current().magnification);
    }
}

auto wholeCubes = [](const Coordinate & globalFirst, const Coordinate & globalLast, const uint64_t value, CubeCoordSet & cubeChangeSet) {
    const auto wholeCubeBegin = Dataset::current().global2cube(globalFirst + Dataset::current().cubeShape - 1);
    const auto wholeCubeEnd = Dataset::current().global2cube(globalLast);

    //fill all whole cubes
    for (int z = wholeCubeBegin.z; z < wholeCubeEnd.z; ++z)
    for (int y = wholeCubeBegin.y; y < wholeCubeEnd.y; ++y)
    for (int x = wholeCubeBegin.x; x < wholeCubeEnd.x; ++x) {
        const auto cubeCoord = CoordOfCube(x, y, z);
        const auto globalCoord = Dataset::current().cube2global(cubeCoord);
        auto rawcube = getRawCube(globalCoord);
        if (rawcube.first) {
            auto cubeRef = getCubeRef(rawcube.second);
            std::fill(cubeRef.data(), cubeRef.data() + cubeRef.num_elements(), value);
            cubeChangeSet.emplace(cubeCoord);
        } else {
            qCritical() << x << y << z << "cube missing for (complete) writeVoxels";
        }
    }
    //returns the skip function for the region traversal
    return [wholeCubeBegin, wholeCubeEnd](int & x, int y, int z){
        if (x == wholeCubeBegin.x && y >= wholeCubeBegin.y && y < wholeCubeEnd.y && z >= wholeCubeBegin.z && z < wholeCubeEnd.z) {
            x = wholeCubeEnd.x;
        }
    };
};

template<typename Func, typename Skip>
CubeCoordSet processRegion(const Coordinate & globalFirst, const Coordinate &  globalLast, Func func, Skip skip) {
    const auto & cubeShape = Dataset::current().cubeShape;
    const auto cubeBegin = Dataset::current().global2cube(globalFirst);
    const auto cubeEnd = Dataset::current().global2cube(globalLast) + 1;
    CubeCoordSet cubeCoords;

    //traverse all remaining cubes
    for (int z = cubeBegin.z; z < cubeEnd.z; ++z)
    for (int y = cubeBegin.y; y < cubeEnd.y; ++y)
    for (int x = cubeBegin.x; x < cubeEnd.x; ++x) {
        skip(x, y, z);//skip cubes which got processed before
        const auto cubeCoord = CoordOfCube(x, y, z);
        const auto globalCubeBegin = Dataset::current().cube2global(cubeCoord);
        auto rawcube = getRawCube(globalCubeBegin);
        if (rawcube.first) {
            auto cubeRef = getCubeRef(rawcube.second);
            const auto globalCubeEnd = globalCubeBegin + Dataset::current().scaleFactor.componentMul(cubeShape);
            const auto localStart = globalFirst.capped(globalCubeBegin, globalCubeEnd).insideCube(cubeShape, Dataset::current().scaleFactor);
            const auto localEnd = globalLast.capped(globalCubeBegin, globalCubeEnd).insideCube(cubeShape, Dataset::current().scaleFactor);

            for (int z = localStart.z; z <= localEnd.z; ++z)
            for (int y = localStart.y; y <= localEnd.y; ++y)
            for (int x = localStart.x; x <= localEnd.x; ++x) {
                const Coordinate globalFromVoxelCoord{globalCubeBegin + Dataset::current().scaleFactor.componentMul(Coordinate{x, y, z})};
                const auto adjustedGlobalCoord = globalFromVoxelCoord.capped(globalFirst, globalLast + 1);// fit to region boundaries that don’t exactly match mag2+ voxel coords
                func(cubeRef[z][y][x], adjustedGlobalCoord);
            }
            cubeCoords.emplace(cubeCoord);
        }
    }
    return cubeCoords;
}

template<typename Func>//wrapper without Skip
CubeCoordSet processRegion(const Coordinate & globalFirst, const Coordinate &  globalLast, Func func) {
    return processRegion(globalFirst, globalLast, func, [](int &, int, int){});
}

void collectFromMovementArea() {
    QElapsedTimer t;
    t.start();
    std::unordered_map<std::uint64_t, Coordinate> ids;
    const auto cubeChangeSet = processRegion(Annotation::singleton().movementAreaMin, Annotation::singleton().movementAreaMax, [&ids](uint64_t & voxel, const Coordinate & pos){
        ids.try_emplace(voxel, pos);
    });
    ids.erase(Segmentation::singleton().getBackgroundId());
    Segmentation::singleton().bulkOperation([&ids](){
        for (const auto & [id, pos] : ids) {
            Segmentation::singleton().selectObjectFromSubObject(id, pos);
        }
    });
    coordCubesMarkChanged(cubeChangeSet);
    qDebug() << "collected ids in" << t.nsecsElapsed()/1e9 << 's';
}

void assignNewIdInMovementArea(const std::uint64_t newId) {
    const auto cubeChangeSet = processRegion(Annotation::singleton().movementAreaMin, Annotation::singleton().movementAreaMax, [newId](uint64_t & voxel, Coordinate){
        if (Segmentation::singleton().isSubObjectIdSelected(voxel)) {
            voxel = newId;
        }
    });
    coordCubesMarkChanged(cubeChangeSet);
}

subobjectRetrievalMap readVoxels(const Coordinate & centerPos, const brush_t &brush) {
    subobjectRetrievalMap subobjects;
    const auto region = getRegion(centerPos, brush);
    processRegion(region.first, region.second, [&subobjects](uint64_t & voxel, Coordinate position){
        if (voxel != 0) {//don’t select the unsegmented area as object
            subobjects.emplace(std::piecewise_construct, std::make_tuple(voxel), std::make_tuple(position));
        }
    });
    return subobjects;
}

void writeVoxels(const Coordinate & centerPos, const uint64_t value, const brush_t & brush, bool isMarkChanged) {
    //all the different invocations here are listed explicitly so the compiler can inline the fuck out of it
    //the brush differentiations were moved outside the core lambda which is called for every voxel
    CubeCoordSet cubeChangeSet;
    CubeCoordSet cubeChangeSetWholeCube;
    if (Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_Paint) || Annotation::singleton().annotationMode.testFlag(AnnotationMode::Mode_OverPaint)) {
        const auto region = getRegion(centerPos, brush);
        const auto guard = currentPaintGuard();
        if (brush.shape == brush_t::shape_t::angular) {
            if (!brush.inverse || Segmentation::singleton().selectedObjectsCount() == 0) {
                //for rectangular brushes no further range checks are needed
                if (brush.mode == brush_t::mode_t::three_dim && brush.shape == brush_t::shape_t::angular && guard.unrestricted()) {
                    //rarest special case: processes completely exclosed cubes first
                    //(only when nothing is being protected — the whole-cube fill can’t look at what it overwrites)
                    cubeChangeSet = processRegion(region.first, region.second, [value](uint64_t & voxel, Coordinate){
                        voxel = value;
                    }, wholeCubes(region.first, region.second, value, cubeChangeSetWholeCube));
                } else {
                    cubeChangeSet = processRegion(region.first, region.second, [value, guard](uint64_t & voxel, Coordinate){
                        if (guard.allows(voxel, value)) {
                            voxel = value;
                        }
                    });
                }
            } else {//inverse but selected
                cubeChangeSet = processRegion(region.first, region.second, [](uint64_t & voxel, Coordinate){
                    if (Segmentation::singleton().isSubObjectIdSelected(voxel)) {//if there’re selected objects, we only want to erase these
                        voxel = 0;
                    }
                });
            }
        } else {
            if (!brush.inverse || Segmentation::singleton().selectedObjectsCount() == 0) {
                //voxel need to check if they are inside the circle
                cubeChangeSet = processRegion(region.first, region.second, [&brush, centerPos, value, guard](uint64_t & voxel, Coordinate globalPos){
                    if (isInsideSphere(globalPos.x - centerPos.x, globalPos.y - centerPos.y, globalPos.z - centerPos.z, brush.radius)
                            && guard.allows(voxel, value)) {
                        voxel = value;
                    }
                });
            } else {//circle, inverse and selected
                cubeChangeSet = processRegion(region.first, region.second, [&brush, centerPos](uint64_t & voxel, Coordinate globalPos){
                    if (isInsideSphere(globalPos.x - centerPos.x, globalPos.y - centerPos.y, globalPos.z - centerPos.z, brush.radius)
                            && Segmentation::singleton().isSubObjectIdSelected(voxel)) {
                        voxel = 0;
                    }
                });
            }
        }
    }
    if (isMarkChanged) {
        for (auto &elem : cubeChangeSetWholeCube) {
            cubeChangeSet.emplace(elem);
        }
        coordCubesMarkChanged(cubeChangeSet);
    }
}

CubeCoordSet processRegionByStridedBuf(const Coordinate & globalFirst, const Coordinate &  globalLast, char * data, const Coordinate & strides, bool isWrite, bool markChanged) {
    CubeCoordSet cubeChangeSet;
    if (isWrite) {
        cubeChangeSet = processRegion(globalFirst, globalLast,
                [globalFirst,data,strides](uint64_t & voxel, Coordinate globalPos){
                voxel = reinterpret_cast<const uint64_t &>(data[(globalPos - globalFirst).componentMul(strides).sum()]);
            });
        if (markChanged) {
            coordCubesMarkChanged(cubeChangeSet);
        }
    }
    else {
        cubeChangeSet = processRegion(globalFirst, globalLast,
                [globalFirst,data,strides](uint64_t & voxel, Coordinate globalPos){
                reinterpret_cast<uint64_t &>(data[(globalPos - globalFirst).componentMul(strides).sum()]) = voxel;
            });
    }
    return cubeChangeSet;
}

void listFill(const Coordinate & centerPos, const brush_t & brush, const uint64_t fillsoid, const std::unordered_set<Coordinate> & voxels) {
    const auto region = getRegion(centerPos, brush);
    auto cubeChangeSet = processRegion(region.first, region.second, [fillsoid, &voxels](uint64_t & voxel, Coordinate globalPos){
        if (voxels.find(globalPos) != std::end(voxels)) {
            voxel = fillsoid;
        }
    });
    coordCubesMarkChanged(cubeChangeSet);
}

CubeCoordSet readRegion(const Coordinate & globalFirst, const Coordinate & globalLast, const VoxelVisitor & visit) {
    return processRegion(globalFirst, globalLast, [&visit](uint64_t & voxel, Coordinate globalPos){
        visit(voxel, globalPos);
    });
}

CubeCoordSet readBrushRegion(const Coordinate & centerPos, const brush_t & brush, const VoxelVisitor & visit) {
    const auto region = getRegion(centerPos, brush);
    return readRegion(region.first, region.second, visit);
}

CubeCoordSet writeVoxelsWhere(const Coordinate & globalFirst, const Coordinate & globalLast, const VoxelPredicate & inside, const std::uint64_t value, const bool markChanged) {
    const auto guard = currentPaintGuard();
    const auto cubeChangeSet = processRegion(globalFirst, globalLast, [&inside, value, guard](uint64_t & voxel, Coordinate globalPos){
        if (guard.allows(voxel, value) && inside(globalPos)) {
            voxel = value;
        }
    });
    if (markChanged) {
        coordCubesMarkChanged(cubeChangeSet);
    }
    return cubeChangeSet;
}

std::size_t processRegionReplacing(const Coordinate & globalFirst, const Coordinate & globalLast, const std::uint64_t from, const std::uint64_t to, const bool markChanged) {
    std::size_t changed{0};
    const auto cubeChangeSet = processRegion(globalFirst, globalLast, [from, to, &changed](uint64_t & voxel, Coordinate){
        if (voxel == from) {
            voxel = to;
            ++changed;
        }
    });
    if (markChanged && changed != 0) {
        coordCubesMarkChanged(cubeChangeSet);
    }
    return changed;
}

std::pair<Coordinate, Coordinate> residentBoxAround(const Coordinate & pos) {
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    const auto cubeExtent = dataset.scaleFactor.componentMul(dataset.cubeShape);
    const auto centre = dataset.global2cube(pos);
    const auto half = std::max(0, (state->M - 1) / 2);
    const auto first = dataset.cube2global({centre.x - half, centre.y - half, centre.z - half});
    const auto last = dataset.cube2global({centre.x + half, centre.y + half, centre.z + half}) + cubeExtent - 1;
    const auto & areaMin = Annotation::singleton().movementAreaMin;
    const auto & areaMax = Annotation::singleton().movementAreaMax;
    return {first.capped(areaMin, areaMax + 1), last.capped(areaMin, areaMax + 1)};
}

std::pair<CubeCoordSet, CubeCoordSet> regionCubeResidency(const Coordinate & globalFirst, const Coordinate & globalLast) {
    const auto cubeBegin = Dataset::current().global2cube(globalFirst);
    const auto cubeEnd = Dataset::current().global2cube(globalLast) + 1;
    CubeCoordSet resident, missing;
    for (int z = cubeBegin.z; z < cubeEnd.z; ++z)
    for (int y = cubeBegin.y; y < cubeEnd.y; ++y)
    for (int x = cubeBegin.x; x < cubeEnd.x; ++x) {
        const auto cubeCoord = CoordOfCube(x, y, z);
        (getRawCube(Dataset::current().cube2global(cubeCoord)).first ? resident : missing).emplace(cubeCoord);
    }
    return {resident, missing};
}

FloodFillResult floodFillFrom(const std::unordered_set<Coordinate> & seeds, const std::uint64_t targetSoid, const std::uint64_t fillsoid,
                              const FloodFillOptions & options, const Coordinate & areaMin, const Coordinate & areaMax) {
    FloodFillResult result;
    if (targetSoid == fillsoid) {
        result.seedAlreadyFilled = true;
        return result;
    }
    const auto layerId = Segmentation::singleton().layerId;
    const auto & dataset = Dataset::datasets[layerId];
    const auto scaleFactor = dataset.scaleFactor;// float, for insideCube()
    // integer voxel step for the walk; scaleFactor is 2^magIndex, so this is exact
    const Coordinate step{std::max(1, static_cast<int>(scaleFactor.x)), std::max(1, static_cast<int>(scaleFactor.y)), std::max(1, static_cast<int>(scaleFactor.z))};
    const auto cubeShape = dataset.cubeShape;

    // Flood fills have strong spatial locality, so caching the cube the last voxel landed
    // in turns one mutex acquisition per voxel into one per cube. Safe only because the
    // caller suspends the loader for the duration — nothing can evict underneath us.
    CoordOfCube cachedCoord{std::numeric_limits<int>::min(), 0, 0};
    void * cachedCube{nullptr};
    bool cacheValid{false};
    const auto cubeAt = [&](const Coordinate & pos) -> void * {
        const auto cubeCoord = dataset.global2cube(pos);
        if (!cacheValid || !(cubeCoord == cachedCoord)) {
            cachedCoord = cubeCoord;
            cachedCube = getRawCube(pos, layerId).second;
            cacheValid = true;
        }
        return cachedCube;
    };
    const auto voxelAt = [&](void * const cube, const Coordinate & pos) -> std::uint64_t & {
        const auto inCube = pos.insideCube(cubeShape, scaleFactor);
        return getCubeRef<std::uint64_t>(cube, layerId)[inCube.z][inCube.y][inCube.x];
    };

    std::vector<Coordinate> work;
    const auto claim = [&](const Coordinate & pos){
        auto * cube = cubeAt(pos);
        if (cube == nullptr) {
            // A cube that is not in memory is a wall, not background. Without this check
            // readVoxel() would report background here and the fill would spill onwards,
            // writing nothing — silently leaving a hole.
            ++result.blockedAtBoundary;
            result.pendingCubes.insert(cachedCoord);
            result.deferred.insert(pos);// a set: the same frontier voxel is reached from several sides
            return;
        }
        auto & voxel = voxelAt(cube, pos);
        if (voxel != targetSoid) {
            return;
        }
        voxel = fillsoid;
        if (result.voxelsFilled == 0) {
            result.filledMin = result.filledMax = pos;
        } else {
            result.filledMin = {std::min(result.filledMin.x, pos.x), std::min(result.filledMin.y, pos.y), std::min(result.filledMin.z, pos.z)};
            result.filledMax = {std::max(result.filledMax.x, pos.x), std::max(result.filledMax.y, pos.y), std::max(result.filledMax.z, pos.z)};
        }
        ++result.voxelsFilled;
        result.cubes.insert(cachedCoord);
        work.push_back(pos);
    };

    for (const auto & seed : seeds) {
        if (seed.x < areaMin.x || seed.y < areaMin.y || seed.z < areaMin.z
                || seed.x > areaMax.x || seed.y > areaMax.y || seed.z > areaMax.z) {
            continue;
        }
        claim(seed);
    }
    if (work.empty() && result.voxelsFilled == 0 && result.blockedAtBoundary == 0) {
        result.seedNotLoaded = seeds.size() == 1 && cubeAt(*std::begin(seeds)) == nullptr;
        coordCubesMarkChanged(result.cubes);
        return result;
    }

    while (!work.empty()) {
        if (result.voxelsFilled >= options.maxVoxels) {
            result.hitCap = true;
            break;
        }
        const auto pos = work.back();
        work.pop_back();

        const auto visit = [&](const Coordinate & next){
            if (next.x < areaMin.x || next.y < areaMin.y || next.z < areaMin.z
                    || next.x > areaMax.x || next.y > areaMax.y || next.z > areaMax.z) {
                return;
            }
            claim(next);
        };
        // in 2D only the two axes spanned by the viewport plane are walked
        if (options.threeDimensional || options.view != brush_t::view_t::zy) {
            visit({pos.x + step.x, pos.y, pos.z});
            visit({pos.x - step.x, pos.y, pos.z});
        }
        if (options.threeDimensional || options.view != brush_t::view_t::xz) {
            visit({pos.x, pos.y + step.y, pos.z});
            visit({pos.x, pos.y - step.y, pos.z});
        }
        if (options.threeDimensional || options.view != brush_t::view_t::xy) {
            visit({pos.x, pos.y, pos.z + step.z});
            visit({pos.x, pos.y, pos.z - step.z});
        }
    }

    coordCubesMarkChanged(result.cubes);
    return result;
}

FloodFillResult floodFill(const Coordinate & seed, const std::uint64_t fillsoid,
                          const FloodFillOptions & options, const Coordinate & areaMin, const Coordinate & areaMax) {
    const auto seedCube = getRawCube(seed, Segmentation::singleton().layerId);
    if (!seedCube.first) {
        FloodFillResult result;
        result.seedNotLoaded = true;
        return result;
    }
    return floodFillFrom({seed}, readVoxel(seed), fillsoid, options, areaMin, areaMax);
}
