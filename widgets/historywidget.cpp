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

#include "historywidget.h"

#include "segmentation/undostack.h"
#include "widgets/GuiConstants.h"
#include "widgets/historytimeline.h"

#include <QLocale>

HistoryWidget::HistoryWidget(QWidget * const parent) : DialogVisibilityNotify(HISTORY_WIDGET, parent) {
    setWindowTitle(tr("History"));

    list.setToolTip(tr("Segmentation edits, newest first. Click one to go back — or forward — to that point."));
    footer.setWordWrap(true);
    buttonLayout.addWidget(&undoButton);
    buttonLayout.addWidget(&redoButton);
    buttonLayout.addWidget(&clearButton);

    mainLayout.addWidget(&list);
    mainLayout.addWidget(&footer);
    mainLayout.addLayout(&buttonLayout);
    setLayout(&mainLayout);

    QObject::connect(&undoButton, &QPushButton::clicked, [this](){ UndoStack::singleton().undo(this); });
    QObject::connect(&redoButton, &QPushButton::clicked, [this](){ UndoStack::singleton().redo(this); });
    QObject::connect(&clearButton, &QPushButton::clicked, [](){ UndoStack::singleton().clear(); });
    QObject::connect(&list, &QListWidget::itemActivated, [this](QListWidgetItem * item){
        const auto steps = item->data(Qt::UserRole).toInt();
        if (steps != 0) {
            UndoStack::singleton().moveTo(steps, this);
        }
    });
    QObject::connect(&list, &QListWidget::itemClicked, [this](QListWidgetItem * item){
        const auto steps = item->data(Qt::UserRole).toInt();
        if (steps != 0) {
            UndoStack::singleton().moveTo(steps, this);
        }
    });
    QObject::connect(&UndoStack::singleton(), &UndoStack::changed, this, &HistoryWidget::refresh);

    refresh();
}

void HistoryWidget::refresh() {
    const auto & undoStack = UndoStack::singleton();
    const auto & past = undoStack.undoEntries();
    const auto & future = undoStack.redoEntries();

    const QSignalBlocker blocker(list);
    list.clear();
    const auto describe = [](const UndoEntry & entry){
        return QStringLiteral("%1  ·  %2  ·  %3")
                .arg(entry.description)
                .arg(entry.when.toString(QStringLiteral("HH:mm:ss")))
                .arg(QLocale::system().formattedDataSize(static_cast<qint64>(entry.bytes)));
    };

    // newest first: the states you could redo into sit above the current one
    const auto futureCount = static_cast<int>(future.size());
    int row{0};
    for (auto it = std::begin(future); it != std::end(future); ++it, ++row) {
        auto * item = new QListWidgetItem(tr("↷ %1").arg(describe(*it)), &list);
        item->setData(Qt::UserRole, historyRowToSteps(row, futureCount));
    }
    auto * current = new QListWidgetItem(tr("— current —"), &list);
    current->setData(Qt::UserRole, 0);
    ++row;
    auto font = current->font();
    font.setBold(true);
    current->setFont(font);

    for (auto it = std::rbegin(past); it != std::rend(past); ++it, ++row) {
        auto * item = new QListWidgetItem(describe(*it), &list);
        item->setData(Qt::UserRole, historyRowToSteps(row, futureCount));
    }

    if (undoStack.droppedForSize()) {
        footer.setText(tr("The last edit was too large to keep — history was dropped."));
    } else if (past.empty() && future.empty()) {
        footer.setText(tr("Nothing to undo yet."));
    } else {
        footer.setText(tr("%1 held across %n step(s).", "", static_cast<int>(past.size() + future.size()))
                           .arg(QLocale::system().formattedDataSize(static_cast<qint64>(undoStack.totalBytes()))));
    }
    undoButton.setEnabled(undoStack.canUndo());
    redoButton.setEnabled(undoStack.canRedo());
    clearButton.setEnabled(!past.empty() || !future.empty());
}
