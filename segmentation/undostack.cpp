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

#include "undostack.h"

#include "annotation/annotation.h"
#include "dataset.h"
#include "loader.h"
#include "segmentation/cubeloader.h"
#include "segmentation/segmentation.h"
#include "segmentation/shapeinterpolation.h"
#include "stateInfo.h"
#include "widgets/historytimeline.h"
#include "viewer.h"

#include <QApplication>
#include <QBuffer>
#include <QSignalBlocker>
#include <QTextStream>

#include <snappy.h>

#include <algorithm>

namespace {
constexpr std::size_t MAX_ENTRIES = 10;
// Compressed ceiling across the whole stack. Label data is mostly runs of one id, so a
// 16 MiB cube usually lands well under 1 MiB — this is a lot of history in practice.
constexpr std::size_t MAX_BYTES = 2ull * 1024 * 1024 * 1024;

QByteArray serializeMergelist() {
    QByteArray out;
    QTextStream stream(&out, QIODevice::WriteOnly);
    Segmentation::singleton().mergelistSave(stream);
    stream.flush();
    return out;
}

// The mergelist is megabytes of text on a large annotation and most operations only move
// voxels, so it is re-serialised only when the object graph has actually changed since
// last time. QByteArray is implicitly shared, so entries taken between two graph changes
// all reference one buffer.
QByteArray cachedMergelist;
std::vector<std::uint64_t> cachedSelection;
std::uint64_t cachedRevision = std::numeric_limits<std::uint64_t>::max();

}

std::vector<std::uint64_t> UndoStack::selectedObjectIds() {
    std::vector<std::uint64_t> ids;
    const auto & seg = Segmentation::singleton();
    for (const auto index : seg.selectedObjectIndices) {
        ids.push_back(seg.objects[index].id);
    }
    return ids;
}

void UndoStack::refreshGraphSnapshot() {
    const auto revision = Segmentation::singleton().graphRevision;
    if (revision != cachedRevision) {
        cachedMergelist = serializeMergelist();
        cachedSelection = selectedObjectIds();
        cachedRevision = revision;
    }
}

/* Replaces the object graph wholesale. mergelistLoad appends rather than replacing, and
 * createObjectFromSubobjectId silently renumbers an object whose id already exists, so
 * clearing first is not optional. Selection is not part of the mergelist format at all and
 * has to be re-applied by id — indices are not stable across object removal. */
void UndoStack::restoreGraph(const UndoEntry & entry) {
    auto & seg = Segmentation::singleton();
    seg.bulkOperation([&entry, &seg](){
        seg.mergelistClear();
        QByteArray copy = entry.mergelist;
        QTextStream stream(&copy, QIODevice::ReadOnly);
        seg.mergelistLoad(stream);
        for (const auto id : entry.selectedObjectIds) {
            if (const auto it = seg.objectIdToIndex.find(id); it != std::end(seg.objectIdToIndex)) {
                seg.selectObject(it->second);
            }
        }
    });
    // restore the id watermarks too, so redo hands out the same ids the original did
    Segmentation::SubObject::highestId = entry.subObjectHighestId;
    Segmentation::Object::highestId = entry.objectHighestId;
    Segmentation::Object::highestIndex = entry.objectHighestIndex;
}

/* Undo rolls back voxels; it must not also move the brush.
 *
 * restoreGraph() re-selects whatever was selected when the edit was *made*, which is right
 * for the object graph — objects have to come back selected as they were — but wrong for
 * the brush: whichever id the user was painting with when they pressed Ctrl+Z is still the
 * one they mean to paint with, and jumping them onto some earlier object means the next
 * stroke lands on the wrong label without any of that being visible.
 *
 * The one case where the earlier selection has to stand is an undo that removed the very
 * object the brush was pointing at — there is nothing to go back to then. */
void UndoStack::reinstatePaintTarget(const bool background, const std::vector<std::uint64_t> & objectIds) {
    auto & seg = Segmentation::singleton();
    if (background) {
        seg.setPaintingBackground(true);
        return;
    }
    std::vector<std::uint64_t> indices;
    for (const auto id : objectIds) {
        if (const auto it = seg.objectIdToIndex.find(id); it != std::end(seg.objectIdToIndex)) {
            indices.push_back(it->second);
        }
    }
    if (indices.empty()) {
        return;// the object the brush was on is gone; leave the entry's own selection
    }
    seg.bulkOperation([&seg, &indices](){
        seg.clearObjectSelection();
        for (const auto index : indices) {
            seg.selectObject(index);
        }
    });
}

std::size_t UndoStack::totalBytes() const {
    std::size_t bytes{0};
    for (const auto * stack : {&past, &future}) {
        for (const auto & entry : *stack) {
            bytes += entry.bytes;
        }
    }
    return bytes;
}

void UndoStack::beginScope(const QString & description) {
    if (depth++ != 0) {
        return;// nested scopes collapse into the outermost operation
    }
    const auto & seg = Segmentation::singleton();
    pending = UndoEntry{};
    pending.description = description;
    pending.when = QDateTime::currentDateTime();
    pending.layerId = seg.layerId;
    pending.magIndex = Dataset::datasets[seg.layerId].magIndex;
    refreshGraphSnapshot();
    pending.mergelist = cachedMergelist;
    pending.selectedObjectIds = cachedSelection;
    pending.subObjectHighestId = seg.highestSubobjectId();
    pending.objectHighestId = Segmentation::Object::highestId;
    pending.objectHighestIndex = Segmentation::Object::highestIndex;
    pending.shapeInterpolation = ShapeInterpolation::singleton().saveState();
    graphRevisionAtScopeStart = seg.graphRevision;
}

void UndoStack::recordCube(const std::size_t layerId, const CoordOfCube & cubeCoord, const void * const rawCube) {
    if (depth == 0 || rawCube == nullptr || layerId != pending.layerId) {
        return;
    }
    if (pending.cubes.find(cubeCoord) != std::end(pending.cubes)) {
        return;// already have this cube's "before" state for this operation
    }
    auto & compressed = pending.cubes[cubeCoord];
    snappy::Compress(reinterpret_cast<const char *>(rawCube), overlayCubeBytes(layerId), &compressed);
    pending.bytes += compressed.size();
}

void UndoStack::endScope() {
    if (--depth != 0 || pending.cubes.empty()) {
        return;
    }
    if (pending.bytes > MAX_BYTES) {
        // Being honest beats pretending: one operation too big to hold means the history
        // is gone, and the History window says so rather than showing stale entries.
        past.clear();
        future.clear();
        droppedBecauseTooLarge = true;
        pending = UndoEntry{};
        emit changed();
        return;
    }
    droppedBecauseTooLarge = false;
    past.push_back(std::move(pending));
    pending = UndoEntry{};
    future.clear();// a fresh edit truncates the redo tail
    enforceBudget();
    emit changed();
}

void UndoStack::enforceBudget() {
    std::vector<std::size_t> sizes;
    sizes.reserve(past.size());
    for (const auto & entry : past) {
        sizes.push_back(entry.bytes);
    }
    for (auto evict = entriesToEvict(sizes, MAX_ENTRIES, MAX_BYTES); evict != 0; --evict) {
        past.pop_front();
    }
}

/* Restores `entry`, and fills `opposite` with what was there just before — so undo and
 * redo are the same operation pointed in different directions. */
void UndoStack::applyEntry(UndoEntry & entry, std::deque<UndoEntry> & opposite, QWidget * const) {
    const auto & seg = Segmentation::singleton();
    const auto layerId = entry.layerId;
    const auto cubeBytes = overlayCubeBytes(layerId);

    UndoEntry inverse;
    inverse.description = entry.description;
    inverse.when = QDateTime::currentDateTime();
    inverse.layerId = layerId;
    inverse.magIndex = entry.magIndex;
    refreshGraphSnapshot();
    inverse.mergelist = cachedMergelist;
    inverse.selectedObjectIds = cachedSelection;
    inverse.subObjectHighestId = seg.highestSubobjectId();
    inverse.objectHighestId = Segmentation::Object::highestId;
    inverse.objectHighestIndex = Segmentation::Object::highestIndex;
    inverse.shapeInterpolation = ShapeInterpolation::singleton().saveState();
    /* What the brush is pointing at right now, to be put back after the graph is restored.
     *
     * Read live rather than from cachedSelection: that only refreshes when graphRevision
     * moves, and adding to a selection with Ctrl + click emits changedRowSelection, which
     * does not move it. */
    const bool paintingBackground = seg.paintingBackground();
    const auto paintSelection = selectedObjectIds();

    // Cubes that are still in memory are restored in place, which is instant and covers
    // the common case of undoing what you are looking at. Everything else goes through the
    // snappy cache, from which it re-hydrates — that path is why undo still works after
    // you have scrolled far enough away for the cubes to be evicted.
    Loader::Worker::SnappySet evicted;
    std::string scratch;
    CubeCoordSet restoredResident;
    for (auto & [cubeCoord, before] : entry.cubes) {
        auto * slot = residentCubePointer(layerId, cubeCoord);
        if (slot != nullptr) {
            scratch.clear();
            snappy::Compress(reinterpret_cast<const char *>(slot), cubeBytes, &scratch);
            inverse.bytes += scratch.size();
            inverse.cubes[cubeCoord] = scratch;
            snappy::RawUncompress(before.c_str(), before.size(), reinterpret_cast<char *>(slot));
            restoredResident.insert(cubeCoord);
        } else {
            evicted[cubeCoord] = before;
        }
    }

    if (!evicted.empty()) {
        // read the current contents of the evicted cubes back out of the snappy cache so
        // the inverse entry is complete; the flush inside makes sure it is up to date
        const auto guard = Loader::Controller::singleton().getAllModifiedCubes(layerId);
        if (entry.magIndex < guard.cubes.size()) {
            const auto & cached = guard.cubes[entry.magIndex];
            for (const auto & [cubeCoord, before] : evicted) {
                if (const auto it = cached.find(cubeCoord); it != std::end(cached)) {
                    inverse.bytes += it->second.size();
                    inverse.cubes[cubeCoord] = it->second;
                }
            }
        }
    }

    {
        QSignalBlocker blockAutosave(Annotation::singleton().autoSaveTimer);
        if (!evicted.empty()) {
            Loader::Controller::singleton().snappyCacheReplace(layerId, static_cast<quint64>(entry.magIndex), evicted);
        }
        if (!restoredResident.empty()) {
            coordCubesMarkChanged(restoredResident);
        }
        // the object graph rides along, otherwise objects whose voxels just vanished stay
        // in the segmentation table
        restoreGraph(entry);
        reinstatePaintTarget(paintingBackground, paintSelection);
        ShapeInterpolation::singleton().restoreState(entry.shapeInterpolation);
        cachedRevision = std::numeric_limits<std::uint64_t>::max();// force a re-snapshot
    }

    // snappyCacheReplace notifies nobody, and a bulkOperation signal blocker suppresses the
    // dirty flag — both have to be done by hand
    state->viewer->loader_notify();
    state->viewer->reslice_notify();
    Annotation::singleton().setUnsavedChanges(true);

    opposite.push_back(std::move(inverse));
    while (opposite.size() > MAX_ENTRIES) {
        opposite.pop_front();
    }
}

void UndoStack::undo(QWidget * const parent) {
    if (past.empty()) {
        return;
    }
    auto entry = std::move(past.back());
    past.pop_back();
    state->viewer->suspend([&]{ applyEntry(entry, future, parent); return 0; });
    emit changed();
    state->viewer->run();
}

void UndoStack::redo(QWidget * const parent) {
    if (future.empty()) {
        return;
    }
    auto entry = std::move(future.back());
    future.pop_back();
    state->viewer->suspend([&]{ applyEntry(entry, past, parent); return 0; });
    emit changed();
    state->viewer->run();
}

void UndoStack::moveTo(const int stepsBack, QWidget * const parent) {
    for (int i = 0; i < stepsBack && canUndo(); ++i) {
        undo(parent);
    }
    for (int i = 0; i < -stepsBack && canRedo(); ++i) {
        redo(parent);
    }
}

void UndoStack::clear() {
    past.clear();
    future.clear();
    droppedBecauseTooLarge = false;
    emit changed();
}
