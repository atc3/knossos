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
    bool switchedOff{false};// whether any layer actually stopped loading, so restoring matters
    unsigned int loadsAtStart{0};// the loader's request count, to tell whether it was moved

public:
    LabelOnlyLoading();
    /* Restores each layer's loading and then restarts loading at the crosshair.
     *
     * The restart is the half that was missing. Every user of this drives the loader
     * somewhere else — to the blocks an adopted slice, a fill or an erase reaches — and the
     * loader evicts whatever lies outside the area it is pointed at, the image blocks around
     * the crosshair included. Moving the view back was not enough either: the users that do
     * it do so while this is still in scope, so that reload ran with the image layers
     * switched off. The image then stayed blank, with nothing pending, until a pan happened
     * to ask for it.
     *
     * Only when the loader was in fact moved: a brush stroke reads its plane under this too,
     * loading nothing, and restarting the loader after every stroke would abort whatever it
     * had in flight. Defined in the .cpp so this header need not include the loader. */
    ~LabelOnlyLoading();
    LabelOnlyLoading(const LabelOnlyLoading &) = delete;
    LabelOnlyLoading & operator=(const LabelOnlyLoading &) = delete;
};
