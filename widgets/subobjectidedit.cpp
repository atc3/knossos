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

#include "subobjectidedit.h"

#include "segmentation/segmentation.h"
#include "stateInfo.h"
#include "viewer.h"

#include <QFontMetrics>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

SubobjectIdEdit::SubobjectIdEdit(QWidget * const parent) : QLineEdit(parent) {
    setValidator(new QRegularExpressionValidator(QRegularExpression(R"(\d{0,20})"), this));
    setPlaceholderText(tr("id"));
    setToolTip(tr("<b>Subobject id being painted.</b><br/>Type an id to paint into that label instead; "
                  "an id that isn’t in the mergelist yet becomes a new object."));
    setFixedWidth(QFontMetrics(font()).horizontalAdvance(QStringLiteral("id: 000000000")) + 16);

    QObject::connect(this, &QLineEdit::returnPressed, this, &SubobjectIdEdit::commit);
    QObject::connect(this, &QLineEdit::editingFinished, this, &SubobjectIdEdit::commit);
    QObject::connect(this, &QLineEdit::textEdited, this, [this](){ editing = true; });

    auto & seg = Segmentation::singleton();
    for (const auto signal : {&Segmentation::resetData, &Segmentation::resetSelection, &Segmentation::selectionChanged, &Segmentation::appendedRow, &Segmentation::removedRow}) {
        QObject::connect(&seg, signal, this, &SubobjectIdEdit::refresh);
    }
    QObject::connect(&seg, &Segmentation::changedRowSelection, this, [this](int){ refresh(); });
    refresh();
}

void SubobjectIdEdit::refresh() {
    if (editing && hasFocus()) {
        return;// don’t yank the field out from under a half-typed id
    }
    const auto id = Segmentation::singleton().currentPaintSubobjectId();
    const auto text = id ? QString::number(*id) : QString{};
    if (text != this->text()) {
        setText(text);
    }
    editing = false;
}

void SubobjectIdEdit::commit() {
    if (!editing) {
        return;// editingFinished also fires on plain focus loss
    }
    editing = false;
    auto & seg = Segmentation::singleton();
    if (text().isEmpty()) {
        seg.clearObjectSelection();
        refresh();
        return;
    }
    bool ok{false};
    const auto id = text().toULongLong(&ok);
    if (!ok || id == seg.getBackgroundId()) {
        refresh();// not a usable id; put the real one back
        return;
    }
    if (id == seg.currentPaintSubobjectId().value_or(seg.getBackgroundId())) {
        return;// already painting with it
    }
    seg.clearObjectSelection();
    seg.selectObjectFromSubObject(id, state->viewerState->currentPosition);
    state->viewer->run();
    refresh();
}
