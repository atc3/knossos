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

#include <QLineEdit>

/* Toolbar field showing — and setting — the subobject id the brush is painting with.
 *
 * A plain QLineEdit rather than a QSpinBox because subobject ids are 64 bit and QSpinBox
 * tops out at 32. Typing an id selects the object holding it, creating that object if the
 * id is not in the mergelist yet, which is how you deliberately paint into a specific
 * label rather than whatever happens to be selected.
 *
 * The background id (0) is accepted too, and means erase — see Segmentation::paintsBackground.
 * That is a big enough change in what the brush does that the field colours itself for it;
 * a bare `0` is far too easy to miss. */
class SubobjectIdEdit : public QLineEdit {
    Q_OBJECT
    bool editing{false};
    bool showingErase{false};

public:
    explicit SubobjectIdEdit(QWidget * parent = nullptr);
    void refresh();// pull the current id back out of Segmentation

private:
    void commit();
    void showErasing(const bool erasing);
};
