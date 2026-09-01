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

#include "dataset.h"

#include <vector>

/* Stops the image layers being fetched while a bulk label operation walks the volume.
 *
 * Interpolating or filling only ever touches the overlay, but moving the position to make
 * a block resident pulls the whole supercube for every layer — so the raw EM data gets
 * downloaded and decompressed for regions nobody is going to look at. On a remote dataset
 * that is the bulk of the wait. Slot allocation is deliberately left alone: freeing and
 * reallocating the raw layer's cube slots would be far more disruptive than skipping a
 * few downloads. */
class LabelOnlyLoading {
    std::vector<bool> previous;

public:
    LabelOnlyLoading() {
        previous.reserve(Dataset::datasets.size());
        for (auto & dataset : Dataset::datasets) {
            previous.push_back(dataset.loadingEnabled);
            if (!dataset.isOverlay()) {
                dataset.loadingEnabled = false;
            }
        }
    }
    ~LabelOnlyLoading() {
        for (std::size_t i = 0; i < Dataset::datasets.size() && i < previous.size(); ++i) {
            Dataset::datasets[i].loadingEnabled = previous[i];
        }
    }
    LabelOnlyLoading(const LabelOnlyLoading &) = delete;
    LabelOnlyLoading & operator=(const LabelOnlyLoading &) = delete;
};
