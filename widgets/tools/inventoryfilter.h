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

/* Which rows of the object inventory are shown, and where "next object" goes.
 *
 * A scan of a large volume finds hundreds of thousands of ids, most of them specks. So the
 * table shows a subset, and the navigation keys walk that subset. Both are a vector of
 * indices into the scan's records — deliberately not a QSortFilterProxyModel, which would
 * evaluate a QVariant per cell per pass and compare voxel counts as strings.
 *
 * The one guarantee the whole feature rests on is that the walk is repeatable: press the
 * key twenty times and you have seen twenty different objects, once each. That needs the
 * row order never to be rearranged, only appended to. Filtering hides rows without
 * reordering them, so it is safe; live sorting is not, and is therefore not done. Whatever
 * the filter, `rows` stays strictly increasing, which `appendRows` maintains by only ever
 * pushing at the end.
 *
 * Kept free of Qt so the two things here that are easy to get wrong can be tested on their
 * own (see tests/inventoryfilter_test.cpp): that appending incrementally during a scan
 * gives exactly what a full rebuild would — anything else means the key silently skips
 * objects — and that changing a filter mid-walk resumes rather than restarting. */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace objinv {

struct Filter {
    std::uint64_t minVoxels{0};
    std::uint64_t maxVoxels{std::numeric_limits<std::uint64_t>::max()};
    bool hideKnown{false};  // hide ids the annotation already has an object for
    bool hideVisited{false};// hide ids already stepped through this session
};

/* `known` and `visited` are called with a record index. They are predicates rather than
 * containers so the caller can answer from whatever it has — a bitset for visited, a
 * lookup into the segmentation for known — without this header knowing about either. */
template<typename Record, typename Known, typename Visited>
bool accepts(const Record & r, const std::size_t i, const Filter & f, Known known, Visited visited) {
    if (r.voxels < f.minVoxels || r.voxels > f.maxVoxels) {
        return false;
    }
    if (f.hideKnown && known(i)) {
        return false;
    }
    if (f.hideVisited && visited(i)) {
        return false;
    }
    return true;
}

template<typename Records, typename Known, typename Visited>
std::vector<std::uint32_t> buildRows(const Records & records, const Filter & f, Known known, Visited visited) {
    std::vector<std::uint32_t> rows;
    rows.reserve(records.size());
    for (std::size_t i = 0; i < records.size(); ++i) {
        if (accepts(records[i], i, f, known, visited)) {
            rows.push_back(static_cast<std::uint32_t>(i));
        }
    }
    return rows;
}

/* Takes on the records discovered since the last call, appending only at the end.
 *
 * Returns how many rows were added, so a caller can skip emitting a row-insertion signal
 * altogether — most batches during a sweep of empty tissue add nothing, and a view does
 * layout work on every signal it gets. */
template<typename Records, typename Known, typename Visited>
std::size_t appendRows(std::vector<std::uint32_t> & rows, const Records & records,
                       const std::size_t firstNew, const Filter & f, Known known, Visited visited) {
    const auto before = rows.size();
    for (std::size_t i = firstNew; i < records.size(); ++i) {
        if (accepts(records[i], i, f, known, visited)) {
            rows.push_back(static_cast<std::uint32_t>(i));
        }
    }
    return rows.size() - before;
}

/* The row to put the cursor on after a filter change, given the record it was on before.
 *
 * Rebuilding the row list drops the view's current row, which would send the next keypress
 * back to the top of the list. So the walk is re-anchored on the record itself: the first
 * row at or after where it was. If everything after it has just been filtered away, the
 * last row; if the list is now empty, -1. Without this, nudging the minimum-size box
 * quietly restarts the walk from the beginning. */
inline int reanchor(const std::vector<std::uint32_t> & rows, const std::uint32_t wantedRecord) {
    if (rows.empty()) {
        return -1;
    }
    // rows is strictly increasing, so this is a binary search
    const auto it = std::lower_bound(std::begin(rows), std::end(rows), wantedRecord);
    if (it == std::end(rows)) {
        return static_cast<int>(rows.size()) - 1;
    }
    return static_cast<int>(it - std::begin(rows));
}

/* Where a next/previous keypress lands. -1 means there is nowhere to go.
 *
 * Caps at both ends rather than wrapping, matching how the skeleton table's next/previous
 * keys behave. Wrapping would be worse than it sounds: at the end of a long list, silently
 * jumping back to the first object is indistinguishable from the key having done nothing,
 * whereas capping lets the caller say so. With no current row, the first or last row
 * depending on direction, so the first keypress enters the list rather than doing nothing. */
inline int step(const int currentRow, const int rowCount, const bool forward) {
    if (rowCount <= 0) {
        return -1;
    }
    if (currentRow < 0 || currentRow >= rowCount) {
        return forward ? 0 : rowCount - 1;
    }
    const auto next = currentRow + (forward ? 1 : -1);
    if (next < 0 || next >= rowCount) {
        return currentRow;// already at the end; the caller reports that rather than wrapping
    }
    return next;
}

}
