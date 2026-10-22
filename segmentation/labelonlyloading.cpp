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

#include "segmentation/labelonlyloading.h"

#include "loader.h"
#include "stateInfo.h"
#include "viewer.h"

LabelOnlyLoading::LabelOnlyLoading() : loadsAtStart{Loader::Controller::singleton().loadingNr.load()} {
    previous.reserve(Dataset::datasets.size());
    for (auto & dataset : Dataset::datasets) {
        previous.push_back(dataset.loadingEnabled);
        if (!dataset.isOverlay()) {
            switchedOff = switchedOff || dataset.loadingEnabled;
            dataset.loadingEnabled = false;
        }
    }
}

LabelOnlyLoading::~LabelOnlyLoading() {
    for (std::size_t i = 0; i < Dataset::datasets.size() && i < previous.size(); ++i) {
        Dataset::datasets[i].loadingEnabled = previous[i];
    }
    // every move of the loader goes through startLoading(), which bumps loadingNr
    const bool loaderMoved = Loader::Controller::singleton().loadingNr.load() != loadsAtStart;
    if (switchedOff && loaderMoved && state != nullptr && state->viewer != nullptr) {
        state->viewer->loader_notify();// back around the crosshair, image layers included
    }
}
