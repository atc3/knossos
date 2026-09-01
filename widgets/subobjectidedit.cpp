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
    showErasing(false);// also installs the normal tooltip
    setFixedWidth(QFontMetrics(font()).horizontalAdvance(QStringLiteral("id: 000000000")) + 16);

    QObject::connect(this, &QLineEdit::returnPressed, this, &SubobjectIdEdit::commit);
    QObject::connect(this, &QLineEdit::editingFinished, this, &SubobjectIdEdit::commit);
    QObject::connect(this, &QLineEdit::textEdited, this, [this](){ editing = true; });

    auto & seg = Segmentation::singleton();
    for (const auto signal : {&Segmentation::resetData, &Segmentation::resetSelection, &Segmentation::selectionChanged, &Segmentation::appendedRow, &Segmentation::removedRow}) {
        QObject::connect(&seg, signal, this, &SubobjectIdEdit::refresh);
    }
    QObject::connect(&seg, &Segmentation::changedRowSelection, this, [this](int){ refresh(); });
    QObject::connect(&seg, &Segmentation::paintingBackgroundChanged, this, [this](bool){ refresh(); });
    refresh();
}

/* Erasing is signalled by colour, not just by the `0` in the field.
 *
 * Selecting background changes what every brush stroke and every fill does, and the
 * difference between an id of 0 and an id of 10 is one glyph in a small box — not enough
 * warning for an operation that rubs out labels. */
void SubobjectIdEdit::showErasing(const bool erasing) {
    showingErase = erasing;
    setStyleSheet(erasing ? QStringLiteral("QLineEdit { background-color: #7d2727; color: #ffe4e4; }") : QString{});
    setToolTip(erasing ? tr("<b>Painting background (0) — the brush and the fills erase.</b><br/>"
                            "Fill with G to rub out the connected region under the pointer. "
                            "Type another id, or click a label, to paint again.")
                       : tr("<b>Subobject id being painted.</b><br/>Type an id to paint into that label instead; "
                            "an id that isn’t in the mergelist yet becomes a new object.<br/>"
                            "Type <b>0</b> to paint the background instead — the brush and the fills then erase."));
}

void SubobjectIdEdit::refresh() {
    if (editing && hasFocus()) {
        return;// don’t yank the field out from under a half-typed id
    }
    const auto & seg = Segmentation::singleton();
    const auto id = seg.currentPaintSubobjectId();
    const auto text = id ? QString::number(*id) : QString{};
    if (text != this->text()) {
        setText(text);
    }
    if (seg.paintingBackground() != showingErase) {
        showErasing(seg.paintingBackground());
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
        seg.setPaintingBackground(false);
        seg.clearObjectSelection();
        refresh();
        return;
    }
    bool ok{false};
    const auto id = text().toULongLong(&ok);
    if (!ok) {
        refresh();// not a usable id; put the real one back
        return;
    }
    if (id == seg.getBackgroundId()) {
        // 0 is not an object but the absence of one, so it cannot be selected the way a
        // label is. It arms erasing instead, which is what makes the fill rub a region out.
        if (!seg.paintingBackground()) {
            seg.setPaintingBackground(true);
            state->viewer->run();
        }
        refresh();
        return;
    }
    if (id == seg.currentPaintSubobjectId().value_or(seg.getBackgroundId())) {
        return;// already painting with it
    }
    seg.setPaintingBackground(false);
    seg.clearObjectSelection();
    seg.selectObjectFromSubObject(id, state->viewerState->currentPosition);
    state->viewer->run();
    refresh();
}
