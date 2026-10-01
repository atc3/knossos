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

#include "widgets/tools/objectinventoryview.h"

#include "dataset.h"
#include "segmentation/segmentation.h"
#include "stateInfo.h"
#include "viewer.h"
#include "widgets/GuiConstants.h"

#include <QBrush>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QSettings>

#include <algorithm>
#include <cmath>

namespace {
Coordinate positionOf(const objinv::Record & r) {
    return Coordinate(r.rep[0], r.rep[1], r.rep[2]);
}
}

// ------------------------------------------------------------------------------- model

ObjectInventoryModel::ObjectInventoryModel(QObject * parent) : QAbstractTableModel(parent) {}

int ObjectInventoryModel::rowCount(const QModelIndex &) const { return static_cast<int>(rows.size()); }
int ObjectInventoryModel::columnCount(const QModelIndex &) const { return ColumnCount; }

QVariant ObjectInventoryModel::headerData(const int section, const Qt::Orientation orientation, const int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
        return {};
    }
    switch (section) {
    case Ordinal: return tr("#");
    case Id: return tr("Subobject ID");
    case Voxels: return tr("Voxels");
    case Position: return tr("Position");
    case StateCol: return tr("State");
    }
    return {};
}

bool ObjectInventoryModel::accepts(const std::size_t recordIndex) const {
    const auto & records = objinv::Inventory::singleton().records();
    if (recordIndex >= records.size()) {
        return false;
    }
    return objinv::accepts(records[recordIndex], recordIndex, activeFilter,
                           [&records](const std::size_t i) {
                               // read-only: subobjectExists() is a plain find and creates nothing
                               return Segmentation::singleton().subobjectExists(records[i].id);
                           },
                           [this](const std::size_t i) { return i < visited.size() && visited[i]; });
}

void ObjectInventoryModel::rebuild() {
    const auto & records = objinv::Inventory::singleton().records();
    beginResetModel();
    visited.resize(records.size(), false);
    rows = objinv::buildRows(records, activeFilter,
                             [&records](const std::size_t i) { return Segmentation::singleton().subobjectExists(records[i].id); },
                             [this](const std::size_t i) { return i < visited.size() && visited[i]; });
    if (activeOrder != Order::Scan) {
        sortRows();
    }
    staleUnderOrder = false;
    endResetModel();
}

void ObjectInventoryModel::onAppended(const std::size_t first, const std::size_t count) {
    const auto & records = objinv::Inventory::singleton().records();
    if (first == 0 || count == 0 || first + count != records.size()) {
        rebuild();
        return;
    }
    visited.resize(records.size(), false);
    std::vector<std::uint32_t> passing;
    for (std::size_t i = first; i < records.size(); ++i) {
        if (accepts(i)) {
            passing.push_back(static_cast<std::uint32_t>(i));
        }
    }
    if (passing.empty()) {
        return;// no signal at all: a view does layout work on every insertion it is told about
    }
    /* Appended at the tail whatever order is active, so the rows a walk has already passed
     * never renumber. When an order other than scan order is active that leaves the new ones
     * out of place, which `reapplyOrder` fixes on request and the status line admits to. */
    beginInsertRows({}, static_cast<int>(rows.size()), static_cast<int>(rows.size() + passing.size()) - 1);
    rows.insert(std::end(rows), std::begin(passing), std::end(passing));
    endInsertRows();
    if (activeOrder != Order::Scan) {
        staleUnderOrder = true;
    }
}

void ObjectInventoryModel::onRevised(const std::size_t first, const std::size_t last) {
    if (rows.empty()) {
        return;
    }
    /* The sweep keeps refining objects it found earlier, so their voxel counts and positions
     * go stale. Repainting the whole table for that would be tens of millions of cells; the
     * view asks for what it can see, so a blanket dataChanged over the row range it might
     * touch is enough and costs nothing for rows that are scrolled away. */
    if (activeOrder != Order::Scan) {
        // rows are not in record order under any other one, so there is no range to narrow to
        emit dataChanged(index(0, Voxels), index(static_cast<int>(rows.size()) - 1, StateCol));
        return;
    }
    const auto lo = std::lower_bound(std::begin(rows), std::end(rows), static_cast<std::uint32_t>(first));
    const auto hi = std::upper_bound(std::begin(rows), std::end(rows), static_cast<std::uint32_t>(last));
    if (lo == std::end(rows) || lo == hi) {
        return;
    }
    const auto from = static_cast<int>(lo - std::begin(rows));
    const auto to = static_cast<int>(hi - std::begin(rows)) - 1;
    emit dataChanged(index(from, Voxels), index(std::max(from, to), StateCol));
}

void ObjectInventoryModel::repaintRows(const int first, const int last) {
    if (first < 0 || last < first || last >= static_cast<int>(rows.size())) {
        return;
    }
    emit dataChanged(index(first, Ordinal), index(last, StateCol));
}

void ObjectInventoryModel::setFilter(const objinv::Filter & f) {
    activeFilter = f;
    rebuild();
}

void ObjectInventoryModel::setOrder(const Order o) {
    activeOrder = o;
    rebuild();
}

void ObjectInventoryModel::reapplyOrder() {
    if (activeOrder == Order::Scan) {
        rebuild();
        return;
    }
    beginResetModel();
    sortRows();
    staleUnderOrder = false;
    endResetModel();
}

void ObjectInventoryModel::sortRows() {
    const auto & records = objinv::Inventory::singleton().records();
    if (activeOrder == Order::LargestFirst) {
        std::sort(std::begin(rows), std::end(rows), [&records](const std::uint32_t a, const std::uint32_t b) {
            if (records[a].voxels != records[b].voxels) { return records[a].voxels > records[b].voxels; }
            return a < b;// a total order, so the same list always sorts the same way
        });
    } else if (activeOrder == Order::NearestFirst) {
        const auto here = state->viewerState->currentPosition;
        const auto scale = Dataset::current().scales.empty() ? floatCoordinate{1, 1, 1} : Dataset::current().scales.front();
        const auto distance = [&](const std::uint32_t i) {
            const auto & r = records[i];
            const double dx = (r.rep[0] - here.x) * scale.x;
            const double dy = (r.rep[1] - here.y) * scale.y;
            const double dz = (r.rep[2] - here.z) * scale.z;
            return dx * dx + dy * dy + dz * dz;
        };
        std::sort(std::begin(rows), std::end(rows), [&](const std::uint32_t a, const std::uint32_t b) {
            const auto da = distance(a), db = distance(b);
            if (da != db) { return da < db; }
            return a < b;
        });
    }
}

const objinv::Record * ObjectInventoryModel::recordAt(const int row) const {
    const auto & records = objinv::Inventory::singleton().records();
    if (row < 0 || row >= static_cast<int>(rows.size()) || rows[row] >= records.size()) {
        return nullptr;
    }
    return &records[rows[row]];
}

std::size_t ObjectInventoryModel::recordIndexAt(const int row) const {
    if (row < 0 || row >= static_cast<int>(rows.size())) {
        return objinv::Accumulator::npos;
    }
    return rows[row];
}

int ObjectInventoryModel::rowOfRecord(const std::size_t recordIndex) const {
    if (activeOrder == Order::Scan) {
        return objinv::reanchor(rows, static_cast<std::uint32_t>(recordIndex));
    }
    const auto it = std::find(std::begin(rows), std::end(rows), static_cast<std::uint32_t>(recordIndex));
    return it == std::end(rows) ? (rows.empty() ? -1 : 0) : static_cast<int>(it - std::begin(rows));
}

bool ObjectInventoryModel::visitedAt(const int row) const {
    const auto i = recordIndexAt(row);
    return i != objinv::Accumulator::npos && i < visited.size() && visited[i];
}

void ObjectInventoryModel::markVisited(const std::size_t recordIndex, const bool on) {
    if (recordIndex >= visited.size()) {
        visited.resize(objinv::Inventory::singleton().records().size(), false);
    }
    if (recordIndex < visited.size()) {
        visited[recordIndex] = on;
        const auto row = rowOfRecord(recordIndex);
        if (row >= 0 && row < static_cast<int>(rows.size()) && rows[row] == recordIndex) {
            emit dataChanged(index(row, StateCol), index(row, StateCol));
        }
    }
}

std::size_t ObjectInventoryModel::visitedCount() const {
    return static_cast<std::size_t>(std::count(std::begin(visited), std::end(visited), true));
}

std::vector<std::uint64_t> ObjectInventoryModel::visitedIds() const {
    const auto & records = objinv::Inventory::singleton().records();
    std::vector<std::uint64_t> ids;
    for (std::size_t i = 0; i < visited.size() && i < records.size(); ++i) {
        if (visited[i]) {
            ids.push_back(records[i].id);
        }
    }
    return ids;
}

void ObjectInventoryModel::adoptVisitedIds(const std::vector<std::uint64_t> & ids) {
    auto & inv = objinv::Inventory::singleton();
    visited.assign(inv.records().size(), false);
    for (const auto id : ids) {
        if (const auto i = inv.indexOfId(id)) {
            if (*i < visited.size()) {
                visited[*i] = true;
            }
        }
    }
    rebuild();
}

QVariant ObjectInventoryModel::data(const QModelIndex & modelIndex, const int role) const {
    const auto * r = recordAt(modelIndex.row());
    if (r == nullptr) {
        return {};
    }
    const auto annotated = Segmentation::singleton().subobjectExists(r->id);
    const auto seen = visitedAt(modelIndex.row());

    if (role == Qt::TextAlignmentRole) {
        return modelIndex.column() == Position || modelIndex.column() == StateCol
                ? QVariant{} : QVariant{static_cast<int>(Qt::AlignRight | Qt::AlignVCenter)};
    }
    if (role == Qt::ForegroundRole && annotated) {
        return QBrush(Qt::darkGray);// already has an object, so it is not the thing to go and draw
    }
    if (role == Qt::ToolTipRole) {
        return tr("%1 voxels at %2× · bounding box %3,%4,%5 to %6,%7,%8")
                .arg(r->voxels).arg(objinv::Inventory::singleton().scanMag())
                .arg(r->bboxMin[0]).arg(r->bboxMin[1]).arg(r->bboxMin[2])
                .arg(r->bboxMax[0]).arg(r->bboxMax[1]).arg(r->bboxMax[2]);
    }
    if (role != Qt::DisplayRole && role != Qt::UserRole) {
        return {};
    }
    // Qt::UserRole is what the shared copy action reads; without it copying yields blank rows
    switch (modelIndex.column()) {
    case Ordinal: return static_cast<qulonglong>(recordIndexAt(modelIndex.row()) + 1);
    case Id: return static_cast<qulonglong>(r->id);
    case Voxels: return role == Qt::UserRole ? QVariant{static_cast<qulonglong>(r->voxels)}
                                             : QVariant{QLocale{}.toString(static_cast<qulonglong>(r->voxels))};
    case Position: {
        const auto p = positionOf(*r);
        return QString("%1, %2, %3").arg(p.x).arg(p.y).arg(p.z);
    }
    case StateCol:
        if (annotated && seen) { return tr("annotated · visited"); }
        if (annotated) { return tr("annotated"); }
        if (seen) { return tr("visited"); }
        return QString{};
    }
    return {};
}

// -------------------------------------------------------------------------------- view

ObjectInventoryView::ObjectInventoryView(QWidget * parent) : QWidget(parent) {
    auto & inv = objinv::Inventory::singleton();

    scanButton.setToolTip(tr("Read the segmentation layer in the background and list every object it finds."));
    rescanButton.setToolTip(tr("Throw the list away and read the layer again."));
    annotationButton.setToolTip(tr("List what this annotation has painted, rather than what the dataset stores.\n"
                                   "Reads the blocks you have edited — no network, and it sees unsaved work,\n"
                                   "but nothing that was already baked into the volume."));
    magCombo.setToolTip(tr("How much detail to read. Coarser is much faster but misses small objects."));
    minVoxelsSpin.setRange(0, 1000000000);
    minVoxelsSpin.setSingleStep(50);
    minVoxelsSpin.setToolTip(tr("Hide anything smaller than this, so the walk is not mostly specks."));
    hideKnownCheck.setToolTip(tr("Hide objects the annotation already has, so the list drains as you work."));
    hideVisitedCheck.setToolTip(tr("Hide objects you have already stepped to."));
    orderCombo.addItem(tr("scan order"), static_cast<int>(ObjectInventoryModel::Order::Scan));
    orderCombo.addItem(tr("largest first"), static_cast<int>(ObjectInventoryModel::Order::LargestFirst));
    orderCombo.addItem(tr("nearest first"), static_cast<int>(ObjectInventoryModel::Order::NearestFirst));
    orderCombo.setToolTip(tr("The order the table shows, which is also the order the keys walk.\n"
                             "Applied once when you pick it — newly found objects are added at the end\n"
                             "rather than slotted in, so a walk in progress never renumbers."));
    reorderButton.setToolTip(tr("Apply the order again, taking in everything found since."));
    reorderButton.setMaximumWidth(30);
    progressBar.setTextVisible(false);
    progressBar.setMaximumWidth(140);
    progressBar.hide();
    statusLabel.setWordWrap(true);

    controlLayout.addWidget(&scanButton);
    controlLayout.addWidget(&rescanButton);
    controlLayout.addWidget(&annotationButton);
    controlLayout.addWidget(&magLabel);
    controlLayout.addWidget(&magCombo);
    controlLayout.addStretch();
    controlLayout.addWidget(&progressBar);

    filterLayout.addWidget(&minVoxelsLabel);
    filterLayout.addWidget(&minVoxelsSpin);
    filterLayout.addWidget(&hideKnownCheck);
    filterLayout.addWidget(&hideVisitedCheck);
    filterLayout.addStretch();
    filterLayout.addWidget(&orderCombo);
    filterLayout.addWidget(&reorderButton);

    table.setModel(&model);
    table.setAllColumnsShowFocus(true);
    table.setContextMenuPolicy(Qt::CustomContextMenu);
    table.setUniformRowHeights(true);// perf hint from the doc, and it matters at this row count
    table.setRootIsDecorated(false);
    /* Single selection on purpose. An extended selection over a few hundred thousand rows
     * builds an item selection with that many ranges, and any future code that resolved the
     * selected ids would create an object for each. */
    table.setSelectionMode(QAbstractItemView::SingleSelection);
    table.setSelectionBehavior(QAbstractItemView::SelectRows);
    /* No live sorting, which is what lets the keys follow the visible order without the
     * order shifting under them. The order combo does it once instead. */
    table.setSortingEnabled(false);
    table.header()->setSectionsClickable(false);
    table.setColumnWidth(ObjectInventoryModel::Ordinal, 70);
    table.setColumnWidth(ObjectInventoryModel::Id, 110);
    table.setColumnWidth(ObjectInventoryModel::Voxels, 90);
    table.setColumnWidth(ObjectInventoryModel::Position, 150);

    layout.addLayout(&controlLayout);
    layout.addLayout(&filterLayout);
    layout.addWidget(&table);
    bottomLayout.addWidget(&statusLabel);
    layout.addLayout(&bottomLayout);
    setLayout(&layout);

    jumpAction = contextMenu.addAction(tr("Jump to object"));
    contextMenu.addSeparator();
    markVisitedAction = contextMenu.addAction(tr("Mark as visited"));
    unmarkVisitedAction = contextMenu.addAction(tr("Mark as not visited"));
    markAboveAction = contextMenu.addAction(tr("Mark everything above as visited"));
    contextMenu.setDefaultAction(jumpAction);

    QObject::connect(&table, &QTreeView::customContextMenuRequested, this, &ObjectInventoryView::showContextMenu);
    QObject::connect(&table, &QTreeView::doubleClicked, this, [this](const QModelIndex & i) { jumpToRow(i.row()); });
    QObject::connect(jumpAction, &QAction::triggered, this, [this]() { jumpToRow(currentRow()); });
    QObject::connect(markVisitedAction, &QAction::triggered, this, [this]() {
        model.markVisited(model.recordIndexAt(currentRow()), true);
        refreshStatus();
    });
    QObject::connect(unmarkVisitedAction, &QAction::triggered, this, [this]() {
        model.markVisited(model.recordIndexAt(currentRow()), false);
        refreshStatus();
    });
    QObject::connect(markAboveAction, &QAction::triggered, this, [this]() {
        const auto row = currentRow();
        for (int r = 0; r <= row; ++r) {
            model.markVisited(model.recordIndexAt(r), true);
        }
        emit message(tr("Marked %n object(s) as visited.", "", row + 1));
        refreshStatus();
    });

    QObject::connect(&scanButton, &QPushButton::clicked, this, [this, &inv]() {
        switch (inv.state()) {
        case objinv::State::Scanning: inv.pause(); break;
        case objinv::State::Paused:
        case objinv::State::Stalled: inv.resume(); break;
        default: inv.startScan(magCombo.currentData().toInt()); break;
        }
    });
    QObject::connect(&rescanButton, &QPushButton::clicked, this, [&inv]() { inv.rescan(); });
    QObject::connect(&annotationButton, &QPushButton::clicked, this, [this, &inv]() { inv.scanAnnotation(this); });
    QObject::connect(&magCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() { refreshStatus(); });

    QObject::connect(&minVoxelsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() { applyFilterFromControls(); });
    QObject::connect(&hideKnownCheck, &QCheckBox::toggled, this, [this]() { applyFilterFromControls(); });
    QObject::connect(&hideVisitedCheck, &QCheckBox::toggled, this, [this]() { applyFilterFromControls(); });
    QObject::connect(&orderCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        model.setOrder(static_cast<ObjectInventoryModel::Order>(orderCombo.currentData().toInt()));
        refreshStatus();
    });
    QObject::connect(&reorderButton, &QPushButton::clicked, this, [this]() {
        model.reapplyOrder();
        refreshStatus();
    });

    QObject::connect(&inv, &objinv::Inventory::recordsAppended, this, [this](const quint64 first, const quint64 count) {
        model.onAppended(first, count);
        refreshStatus();
    });
    QObject::connect(&inv, &objinv::Inventory::recordsRevised, this, [this](const quint64 first, const quint64 last) {
        model.onRevised(first, last);
    });
    QObject::connect(&inv, &objinv::Inventory::stateChanged, this, [this]() {
        refreshControls();
        refreshStatus();
    });
    QObject::connect(&inv, &objinv::Inventory::progressChanged, this, [this](quint64 done, quint64 total, quint64) {
        if (total == 0) {
            progressBar.hide();
            return;
        }
        const auto percent = static_cast<int>(100 * done / total);
        if (percent != progressBar.value()) {
            progressBar.setRange(0, 100);
            progressBar.setValue(percent);
        }
        progressBar.setVisible(objinv::Inventory::singleton().state() == objinv::State::Scanning);
        refreshStatus();
    });
    QObject::connect(&inv, &objinv::Inventory::magOptionsChanged, this, [this]() {
        rebuildMagCombo();
        refreshControls();
        refreshStatus();
    });
    /* The "annotated" marks come from Segmentation, which changes as the user works. Painted
     * over a coalescing timer and only over what is on screen — a blanket repaint here would
     * be the one thing that makes a large table feel slow. */
    QObject::connect(&Segmentation::singleton(), &Segmentation::appendedRow, this, [this]() { repaintTimer.start(); });
    QObject::connect(&Segmentation::singleton(), &Segmentation::resetData, this, [this]() { repaintTimer.start(); });
    repaintTimer.setSingleShot(true);
    repaintTimer.setInterval(400);
    QObject::connect(&repaintTimer, &QTimer::timeout, this, [this]() { refreshVisibleRows(); });

    rebuildMagCombo();
    refreshControls();
    refreshStatus();
}

void ObjectInventoryView::showEvent(QShowEvent * event) {
    QWidget::showEvent(event);
    // the magnification choices need probing, which is network work — so it waits until the
    // tab is actually looked at rather than happening on every dataset load
    objinv::Inventory::singleton().ensureProbed();
}

int ObjectInventoryView::currentRow() const {
    return table.currentIndex().isValid() ? table.currentIndex().row() : -1;
}

void ObjectInventoryView::applyFilterFromControls() {
    // hold on to where the walk was, so raising the minimum does not send it back to the top
    const auto anchor = model.recordIndexAt(currentRow());
    objinv::Filter f;
    f.minVoxels = static_cast<std::uint64_t>(minVoxelsSpin.value());
    f.hideKnown = hideKnownCheck.isChecked();
    f.hideVisited = hideVisitedCheck.isChecked();
    model.setFilter(f);
    if (anchor != objinv::Accumulator::npos) {
        const auto row = model.rowOfRecord(anchor);
        if (row >= 0) {
            table.setCurrentIndex(model.index(row, ObjectInventoryModel::Id));
            table.scrollTo(model.index(row, 0), QAbstractItemView::PositionAtCenter);
        }
    }
    refreshStatus();
}

void ObjectInventoryView::rebuildMagCombo() {
    const auto & inv = objinv::Inventory::singleton();
    const auto previous = magCombo.currentData().toInt();
    QSignalBlocker blocker{magCombo};
    magCombo.clear();
    magCombo.addItem(tr("auto"), 0);
    auto options = inv.magOptions();
    std::sort(std::begin(options), std::end(options),
              [](const objinv::MagOption & a, const objinv::MagOption & b) { return a.mag < b.mag; });
    for (const auto & option : options) {
        // every declared level is offered. A sparse segmentation can have nothing at any
        // sampled block and still be full of objects, so "not sampled" is a note, not a bar.
        const auto label = option.sampled
                ? tr("%1× — %2 blocks, about %3 MB")
                  .arg(option.mag).arg(option.cubes).arg(option.estBytes / (1024 * 1024))
                : tr("%1× — %2 blocks, about %3 MB (nothing at the sampled blocks)")
                  .arg(option.mag).arg(option.cubes).arg(option.estBytes / (1024 * 1024));
        magCombo.addItem(label, option.mag);
    }
    const auto restore = magCombo.findData(previous);
    magCombo.setCurrentIndex(restore >= 0 ? restore : 0);
}

void ObjectInventoryView::refreshControls() {
    const auto & inv = objinv::Inventory::singleton();
    const auto scanning = inv.state() == objinv::State::Scanning;
    const auto resumable = inv.state() == objinv::State::Paused || inv.state() == objinv::State::Stalled;
    const auto usable = inv.state() != objinv::State::Unsupported;
    scanButton.setText(scanning ? tr("Pause") : resumable ? tr("Resume") : tr("Scan"));
    scanButton.setEnabled(usable);
    rescanButton.setEnabled(usable && !inv.records().empty());
    annotationButton.setEnabled(!scanning);
    magCombo.setEnabled(usable && !scanning);
    progressBar.setVisible(scanning);
}

void ObjectInventoryView::refreshStatus() {
    auto line = objinv::Inventory::singleton().statusLine();
    const auto shown = model.shownCount();
    const auto held = objinv::Inventory::singleton().records().size();
    if (shown != held) {
        line += tr(" · %1 shown").arg(shown);
    }
    const auto seen = model.visitedCount();
    if (seen != 0) {
        line += tr(" · %1 visited").arg(seen);
    }
    if (model.orderIsStale()) {
        line += tr(" · newly found objects are at the end, press ↻ to reorder");
    }
    statusLabel.setText(line);
}

void ObjectInventoryView::refreshVisibleRows() {
    if (model.rowCount() == 0) {
        return;
    }
    const auto top = table.indexAt(table.viewport()->rect().topLeft());
    const auto bottom = table.indexAt(table.viewport()->rect().bottomLeft());
    const auto first = top.isValid() ? top.row() : 0;
    const auto last = bottom.isValid() ? bottom.row() : std::min(model.rowCount() - 1, first + 60);
    if (last >= first) {
        model.repaintRows(first, last);
    }
    refreshStatus();
}

void ObjectInventoryView::showContextMenu(const QPoint & pos) {
    const auto row = currentRow();
    const auto have = row >= 0 && model.recordAt(row) != nullptr;
    // enabled by pointer, not by position in the menu, so inserting an item cannot break this
    jumpAction->setEnabled(have);
    markVisitedAction->setEnabled(have && !model.visitedAt(row));
    unmarkVisitedAction->setEnabled(have && model.visitedAt(row));
    markAboveAction->setEnabled(have);
    contextMenu.exec(table.viewport()->mapToGlobal(pos));
}

void ObjectInventoryView::jumpToNextEntry(const bool forward) {
    auto & inv = objinv::Inventory::singleton();
    if (inv.state() == objinv::State::Unsupported) {
        emit message(tr("No object inventory here: %1").arg(inv.detail()));
        return;
    }
    if (inv.records().empty()) {
        emit message(inv.state() == objinv::State::Scanning
                     ? tr("Scanning at %1× — %2 of %3 blocks, nothing found yet")
                       .arg(inv.scanMag()).arg(inv.cubesDone()).arg(inv.cubesTotal())
                     : tr("No object inventory yet — build one in Annotation ▸ Inventory"));
        return;
    }
    if (model.rowCount() == 0) {
        emit message(tr("All %n object(s) are filtered out — lower the minimum size or clear a filter.",
                        "", static_cast<int>(inv.records().size())));
        return;
    }
    const auto from = currentRow();
    const auto to = objinv::step(from, model.rowCount(), forward);
    if (to < 0) {
        return;
    }
    if (to == from && (forward ? from == model.rowCount() - 1 : from == 0)) {
        emit message(forward
                     ? tr("End of the inventory — %1 shown of %2. Lower the minimum size for more.")
                       .arg(model.shownCount()).arg(inv.records().size())
                     : tr("Start of the inventory — %1 shown of %2.")
                       .arg(model.shownCount()).arg(inv.records().size()));
        return;
    }
    jumpToRow(to);
}

void ObjectInventoryView::jumpToRow(const int row) {
    const auto * found = model.recordAt(row);
    if (found == nullptr) {
        return;
    }
    /* Taken by value. The sweep appends to that vector from the event loop, and what
     * follows moves the crosshair and loads blocks — holding a pointer into a container
     * that can reallocate underneath is the one mistake worth pre-empting here. */
    const auto record = *found;
    const auto position = positionOf(record);
    auto & seg = Segmentation::singleton();
    seg.clearObjectSelection();
    /* Selected before moving, so the object is already coloured as the new blocks arrive.
     *
     * The branch matters: selectMergedObjectFromSubObject() sets the object's location as a
     * side effect, which for an object the user has already been painting would replace the
     * spot they last worked at with this sweep's coarse estimate. For an id that has no
     * object yet there is nothing to lose and one gets created — the same thing clicking
     * that voxel would have done, and the only place in this feature that creates anything. */
    if (const auto objectIndex = seg.objectIndexOfSubobject(record.id)) {
        seg.selectObject(*objectIndex);
    } else {
        seg.selectMergedObjectFromSubObject(record.id, position);
    }
    state->viewer->setPositionWithRecentering(position);

    /* Marked, but the row is not re-filtered away underneath the cursor even when "hide
     * visited" is on — it disappears at the next rebuild instead. Having the row you are
     * looking at vanish as you arrive at it would be disorienting. */
    model.markVisited(model.recordIndexAt(row), true);
    table.setCurrentIndex(model.index(row, ObjectInventoryModel::Id));
    table.scrollTo(model.index(row, 0), QAbstractItemView::PositionAtCenter);
    emit message(tr("Object %1 — %2 voxels — %3 of %4 shown")
                 .arg(record.id).arg(record.voxels).arg(row + 1).arg(model.shownCount()));
    refreshStatus();
}

void ObjectInventoryView::loadSettings() {
    QSettings settings;
    settings.beginGroup(OBJECT_INVENTORY_TAB);
    minVoxelsSpin.setValue(settings.value(INVENTORY_MIN_VOXELS, 0).toInt());
    hideKnownCheck.setChecked(settings.value(INVENTORY_HIDE_KNOWN, false).toBool());
    hideVisitedCheck.setChecked(settings.value(INVENTORY_HIDE_VISITED, false).toBool());
    const auto order = settings.value(INVENTORY_ORDER, 0).toInt();
    const auto orderIndex = orderCombo.findData(order);
    orderCombo.setCurrentIndex(orderIndex >= 0 ? orderIndex : 0);
    const auto header = settings.value(HEADER).toByteArray();
    if (!header.isEmpty()) {
        table.header()->restoreState(header);
    }
    settings.endGroup();
    applyFilterFromControls();
}

void ObjectInventoryView::saveSettings() {
    QSettings settings;
    settings.beginGroup(OBJECT_INVENTORY_TAB);
    settings.setValue(INVENTORY_MIN_VOXELS, minVoxelsSpin.value());
    settings.setValue(INVENTORY_HIDE_KNOWN, hideKnownCheck.isChecked());
    settings.setValue(INVENTORY_HIDE_VISITED, hideVisitedCheck.isChecked());
    settings.setValue(INVENTORY_ORDER, orderCombo.currentData().toInt());
    settings.setValue(HEADER, table.header()->saveState());
    settings.endGroup();
}

/* Which objects have been walked belongs to the *annotation* — it is a record of work done,
 * not a property of the dataset — so it rides along inside the .k.zip. Stored as ids rather
 * than list positions, so a rescan that discovers things in a different order keeps it. */
QByteArray ObjectInventoryView::visitedJson() const {
    const auto ids = model.visitedIds();
    if (ids.empty()) {
        return {};
    }
    QJsonArray array;
    for (const auto id : ids) {
        array.append(QString::number(id));// as text: a quint64 does not survive a JSON double
    }
    QJsonObject root;
    root["mag"] = objinv::Inventory::singleton().scanMag();
    root["visited"] = array;
    return QJsonDocument{root}.toJson(QJsonDocument::Compact);
}

void ObjectInventoryView::importVisitedJson(const QByteArray & json) {
    const auto document = QJsonDocument::fromJson(json);
    if (!document.isObject()) {
        return;
    }
    std::vector<std::uint64_t> ids;
    for (const auto value : document.object()["visited"].toArray()) {
        bool ok = false;
        const auto id = value.toString().toULongLong(&ok);
        if (ok) {
            ids.push_back(id);
        }
    }
    model.adoptVisitedIds(ids);
    refreshStatus();
}
