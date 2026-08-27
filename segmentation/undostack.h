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

/* Undo for segmentation edits, at cube granularity.
 *
 * Not a command model: nothing has to be expressed as an invertible operation, so a flood
 * fill that escaped through a one-voxel gap and swallowed a whole plane is no harder to
 * undo than a single brush stamp. An operation declares a scope; the first time it touches
 * a cube, that cube's current contents are snappy-compressed into the entry. Undo puts
 * them back.
 *
 * Cubes are the right unit because they are already what KNOSSOS compresses, evicts and
 * saves — and because restoring one does not require it to be resident, which matters when
 * the thing you want to undo is now hundreds of sections behind you.
 *
 * The object graph rides along: voxels alone would leave objects whose voxels are gone
 * still sitting in the segmentation table. */

#include "coordinate.h"
#include "segmentation/shapeinterpolation.h"

#include <QByteArray>
#include <QDateTime>
#include <QObject>
#include <QString>

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

class QWidget;

struct UndoEntry {
    QString description;
    QDateTime when;
    std::size_t layerId{0};
    std::size_t magIndex{0};// the snappy cache is per magnification
    std::unordered_map<CoordOfCube, std::string> cubes;// compressed contents *before* the edit
    QByteArray mergelist;  // empty when the operation left the object graph alone
    std::vector<std::uint64_t> selectedObjectIds;// not part of the mergelist format
    std::uint64_t subObjectHighestId{0}, objectHighestId{0}, objectHighestIndex{0};
    // key slices live in memory, not in the overlay, so they roll back with the voxels
    ShapeInterpolation::State shapeInterpolation;
    std::size_t bytes{0};
};

class UndoStack : public QObject {
    Q_OBJECT
public:
    static UndoStack & singleton() {
        static UndoStack instance;
        return instance;
    }

    bool canUndo() const { return !past.empty(); }
    bool canRedo() const { return !future.empty(); }
    const std::deque<UndoEntry> & undoEntries() const { return past; }
    const std::deque<UndoEntry> & redoEntries() const { return future; }
    std::size_t totalBytes() const;
    bool droppedForSize() const { return droppedBecauseTooLarge; }

    void undo(QWidget * parent);
    void redo(QWidget * parent);
    // Rewind or replay until `stepsBack` entries remain undone (0 = newest state).
    void moveTo(int stepsBack, QWidget * parent);
    void clear();

    // called by the recording hooks; no-ops when no scope is open
    void beginScope(const QString & description);
    void endScope();
    void recordCube(std::size_t layerId, const CoordOfCube &, const void * rawCube);
    bool scopeOpen() const { return depth != 0; }

signals:
    void changed();

private:
    UndoStack() = default;
    void applyEntry(UndoEntry & entry, std::deque<UndoEntry> & opposite, QWidget * parent);
    // these reach into Segmentation's internals, so they have to be members —
    // friendship does not extend to free functions
    static std::vector<std::uint64_t> selectedObjectIds();
    static void refreshGraphSnapshot();
    static void restoreGraph(const UndoEntry &);
    void enforceBudget();

    std::deque<UndoEntry> past, future;
    UndoEntry pending;
    int depth{0};
    std::uint64_t graphRevisionAtScopeStart{0};
    bool droppedBecauseTooLarge{false};
};

/* RAII scope. Nested scopes collapse into the outermost one, so a helper that opens its
 * own scope inside a larger operation does not split it into two undo steps. */
class UndoScope {
public:
    explicit UndoScope(const QString & description) { UndoStack::singleton().beginScope(description); }
    ~UndoScope() { UndoStack::singleton().endScope(); }
    UndoScope(const UndoScope &) = delete;
    UndoScope & operator=(const UndoScope &) = delete;
};
