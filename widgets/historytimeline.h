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

/* Arithmetic behind the History window and the undo budget.
 *
 * Kept free of Qt and of any KNOSSOS type so it can be exercised on its own — see
 * tests/undohistory_test.cpp. Both of these are the sort of off-by-one that only shows up
 * as "the wrong edit got undone", which is not a thing to discover by hand. */

#include <cstddef>
#include <vector>

/* The history list runs newest-first: the redo entries (furthest-ahead first), then the
 * current position, then the undo entries (most recent first).
 *
 * Returns the argument for UndoStack::moveTo — positive undoes that many steps, negative
 * redoes that many, zero is already where we are. */
inline int historyRowToSteps(const int row, const int futureCount) {
    return row < futureCount ? -(futureCount - row) : row - futureCount;
}

/* How many entries to drop from the front of the undo stack.
 *
 * Oldest first, until both the count and the total compressed size are within budget —
 * but never the last remaining entry, so a single edit larger than the whole budget is
 * still undoable once. */
inline std::size_t entriesToEvict(const std::vector<std::size_t> & entryBytes, const std::size_t maxEntries, const std::size_t maxBytes) {
    std::size_t evicted{0};
    std::size_t remaining = entryBytes.size();
    std::size_t total{0};
    for (const auto bytes : entryBytes) {
        total += bytes;
    }
    while (remaining > maxEntries) {
        total -= entryBytes[evicted++];
        --remaining;
    }
    while (total > maxBytes && remaining > 1) {
        total -= entryBytes[evicted++];
        --remaining;
    }
    return evicted;
}
