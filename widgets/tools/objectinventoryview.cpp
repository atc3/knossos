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
#include "skeleton/skeletonizer.h"
#include "stateInfo.h"
#include "viewer.h"
#include "widgets/GuiConstants.h"

#include <QBrush>
#include <QHeaderView>
#include <QKeySequence>
#include <QShortcut>
#include <QMessageBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QStringList>
#include <QSettings>

#include <algorithm>
#include <unordered_map>
#include <cmath>

namespace {
/* Nanometres per voxel at the magnification the inventory was scanned at — the voxels its
 * counts are in, so the size sphericity has to be computed in. */
void scanVoxelNm(double (& nm)[3]) {
    nm[0] = nm[1] = nm[2] = 1;
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return;
    }
    const auto & layer = Dataset::datasets[layerId];
    const auto magIndex = static_cast<std::size_t>(std::lround(std::log2(std::max(1, objinv::Inventory::singleton().scanMag()))));
    if (layer.scales.empty()) {
        return;
    }
    const auto & s = layer.scales[std::min(magIndex, layer.scales.size() - 1)];
    nm[0] = s.x; nm[1] = s.y; nm[2] = s.z;
}

/* How many magnification-1 voxels one voxel of the scanned magnification stands for: the
 * ratio of their volumes. 8 per halving for an isotropic pyramid, but taken from the
 * declared voxel sizes, so a pyramid that does not downsample z gets 4 rather than 8. */
double mag1VoxelsPerScanVoxel() {
    const auto layerId = Segmentation::singleton().layerId;
    if (layerId >= Dataset::datasets.size()) {
        return 1;
    }
    const auto & layer = Dataset::datasets[layerId];
    if (layer.scales.empty()) {
        return 1;
    }
    double nm[3];
    scanVoxelNm(nm);
    const auto & s = layer.scales.front();
    const auto mag1Volume = static_cast<double>(s.x) * s.y * s.z;
    return mag1Volume > 0 ? nm[0] * nm[1] * nm[2] / mag1Volume : 1;
}

Coordinate positionOf(const objinv::Record & r) {
    return Coordinate(r.rep[0], r.rep[1], r.rep[2]);
}
}

// ------------------------------------------------------------------------------- model

ObjectInventoryModel::ObjectInventoryModel(QObject * parent) : QAbstractTableModel(parent) {}

int ObjectInventoryModel::rowCount(const QModelIndex &) const { return static_cast<int>(rows.size()); }
int ObjectInventoryModel::columnCount(const QModelIndex &) const { return ColumnCount; }

QVariant ObjectInventoryModel::headerData(const int section, const Qt::Orientation orientation, const int role) const {
    if (orientation != Qt::Horizontal) {
        return {};
    }
    if (role == Qt::ToolTipRole) {
        switch (section) {
        case Ordinal: return tr("Scan order. Sorted ascending, newly found objects join at the end in place.");
        case Sphericity: return tr("How compact the object is: 1 for a perfect ball, lower for flat, thin or ragged shapes.\n"
                                   "From its voxel faces, so a voxelised ball scores about 0.66 — compare objects with each\n"
                                   "other rather than with 1. Blank for objects scanned before this was measured: rescan.");
        case Mag1Voxels: return tr("Approximate size at magnification 1: the scanned voxel count times how many\n"
                                   "magnification-1 voxels one scanned voxel covers (8 per halving). An estimate —\n"
                                   "nothing at magnification 1 is read. The same as Voxels for a 1× scan.");
        case Distance: return tr("Distance from where the crosshair was when the list was last sorted.\n"
                                 "Sort by it to list what is nearest; sort again after moving to update it.");
        default: return tr("Click to sort by this column; click again to reverse.");
        }
    }
    if (role != Qt::DisplayRole) {
        return {};
    }
    switch (section) {
    case Ordinal: return tr("#");
    case Id: return tr("Subobject ID");
    case Voxels: return tr("Voxels");
    case Mag1Voxels: return tr("≈ Voxels at 1×");
    case Sphericity: return tr("Sphericity");
    case Distance: return tr("Distance");
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
                               /* Dealt with, by any route. subobjectExists() alone misses a
                                * false positive removed with the bucket on background, which
                                * never selects anything — hence the flags too. Read-only:
                                * subobjectExists() is a plain find and creates nothing. */
                               const auto bits = objinv::Inventory::singleton().flagsFor(records[i].id);
                               return (bits & (objinv::Flag::Painted | objinv::Flag::Erased)) != 0
                                       || Segmentation::singleton().subobjectExists(records[i].id);
                           },
                           [](const std::size_t i) {
                               const auto & recs = objinv::Inventory::singleton().records();
                               return i < recs.size()
                                       && (objinv::Inventory::singleton().flagsFor(recs[i].id) & objinv::Flag::Visited) != 0;
                           });
}

void ObjectInventoryModel::rebuild() {
    const auto & records = objinv::Inventory::singleton().records();
    beginResetModel();
    rows = objinv::buildRows(records, activeFilter,
                             [&records](const std::size_t i) {
                                 const auto bits = objinv::Inventory::singleton().flagsFor(records[i].id);
                                 return (bits & (objinv::Flag::Painted | objinv::Flag::Erased)) != 0
                                         || Segmentation::singleton().subobjectExists(records[i].id);
                             },
                             [](const std::size_t i) {
                                 const auto & r = objinv::Inventory::singleton().records();
                                 return i < r.size()
                                         && (objinv::Inventory::singleton().flagsFor(r[i].id) & objinv::Flag::Visited) != 0;
                             });
    sortRows();
    staleUnderOrder = false;
    endResetModel();
}

void ObjectInventoryModel::onAppended(const std::size_t first, const std::size_t count) {
    const auto & records = objinv::Inventory::singleton().records();
    if (first == 0 || count == 0 || first + count != records.size()) {
        rebuild();
        return;
    }
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
     * out of place until the header is clicked again, which the status line admits to. */
    beginInsertRows({}, static_cast<int>(rows.size()), static_cast<int>(rows.size() + passing.size()) - 1);
    rows.insert(std::end(rows), std::begin(passing), std::end(passing));
    endInsertRows();
    if (!natural()) {
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
    if (!natural()) {
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

double ObjectInventoryModel::distanceOf(const objinv::Record & r) const {
    const auto scale = Dataset::current().scales.empty() ? floatCoordinate{1, 1, 1} : Dataset::current().scales.front();
    const double dx = (r.rep[0] - distanceFrom.x) * scale.x;
    const double dy = (r.rep[1] - distanceFrom.y) * scale.y;
    const double dz = (r.rep[2] - distanceFrom.z) * scale.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/* Orders `rows` by the active column.
 *
 * Keys are worked out once per row rather than inside the comparison — sphericity is a cube
 * root and a power, and at a few hundred thousand rows a comparator recomputing it would do
 * that tens of millions of times. Objects whose value is unknown (sphericity from a scan made
 * before it was measured) go last in both directions, so "lowest first" is
 * not a screenful of blanks. Ties fall back to scan order, so the same list always sorts the
 * same way. */
void ObjectInventoryModel::sortRows() {
    distanceFrom = state->viewerState->currentPosition;
    if (natural()) {
        std::sort(std::begin(rows), std::end(rows));
        return;
    }
    const auto & records = objinv::Inventory::singleton().records();
    double nm[3];
    scanVoxelNm(nm);
    const auto mag1Factor = mag1VoxelsPerScanVoxel();
    struct Key { double value; bool known; std::uint32_t row; };
    std::vector<Key> keys;
    keys.reserve(rows.size());
    for (const auto i : rows) {
        const auto & r = records[i];
        double value = 0;
        bool known = true;
        switch (sortColumn) {
        case Ordinal: value = i; break;
        case Id: value = static_cast<double>(r.id); break;
        case Voxels: value = static_cast<double>(r.voxels); break;
        case Mag1Voxels: value = static_cast<double>(r.voxels) * mag1Factor; break;
        case Sphericity: value = objinv::sphericity(r, nm); known = value >= 0; break;
        case Distance: value = distanceOf(r); break;
        case Position: {
            const auto p = positionOf(r);
            value = (static_cast<double>(p.z) * 1048576.0 + p.y) * 1048576.0 + p.x;
            break;
        }
        case StateCol: {
            const auto bits = objinv::Inventory::singleton().flagsFor(r.id);
            const auto worked = (bits & (objinv::Flag::Painted | objinv::Flag::Erased | objinv::Flag::Skeleton)) != 0
                    || Segmentation::singleton().subobjectExists(r.id);
            value = (worked ? 2 : 0) + ((bits & objinv::Flag::Visited) != 0 ? 1 : 0);
            break;
        }
        default: break;
        }
        keys.push_back({value, known, i});
    }
    const bool descending = sortOrder == Qt::DescendingOrder;
    std::sort(std::begin(keys), std::end(keys), [descending](const Key & a, const Key & b) {
        if (a.known != b.known) { return a.known; }// unknown last, whichever way
        if (a.value != b.value) { return descending ? a.value > b.value : a.value < b.value; }
        return a.row < b.row;
    });
    for (std::size_t k = 0; k < keys.size(); ++k) {
        rows[k] = keys[k].row;
    }
}

/* A header click. Done as a layout change with the persistent indexes moved along with
 * their rows, so the current row stays on the same object and . and , carry on from it. */
void ObjectInventoryModel::sort(const int column, const Qt::SortOrder order) {
    if (column < 0 || column >= ColumnCount) {
        return;
    }
    sortColumn = column;
    sortOrder = order;
    emit layoutAboutToBeChanged({}, QAbstractItemModel::VerticalSortHint);
    const auto before = rows;
    sortRows();
    std::unordered_map<std::uint32_t, int> newRowOf;
    newRowOf.reserve(rows.size() * 2);
    for (int k = 0; k < static_cast<int>(rows.size()); ++k) {
        newRowOf.emplace(rows[k], k);
    }
    const auto persistent = persistentIndexList();
    QModelIndexList moved;
    moved.reserve(persistent.size());
    for (const auto & old : persistent) {
        const auto it = old.row() >= 0 && old.row() < static_cast<int>(before.size()) ? newRowOf.find(before[old.row()]) : newRowOf.end();
        moved.append(it == newRowOf.end() ? QModelIndex{} : index(it->second, old.column()));
    }
    changePersistentIndexList(persistent, moved);
    staleUnderOrder = false;
    emit layoutChanged({}, QAbstractItemModel::VerticalSortHint);
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
    if (natural()) {
        return objinv::reanchor(rows, static_cast<std::uint32_t>(recordIndex));
    }
    const auto it = std::find(std::begin(rows), std::end(rows), static_cast<std::uint32_t>(recordIndex));
    return it == std::end(rows) ? (rows.empty() ? -1 : 0) : static_cast<int>(it - std::begin(rows));
}

bool ObjectInventoryModel::visitedAt(const int row) const {
    const auto * r = recordAt(row);
    return r != nullptr && (objinv::Inventory::singleton().flagsFor(r->id) & objinv::Flag::Visited) != 0;
}

void ObjectInventoryModel::markVisited(const std::size_t recordIndex, const bool on) {
    const auto & recs = objinv::Inventory::singleton().records();
    if (recordIndex >= recs.size()) {
        return;
    }
    objinv::Inventory::singleton().setFlag(recs[recordIndex].id, objinv::Flag::Visited, on);
    const auto row = rowOfRecord(recordIndex);
    if (row >= 0 && row < static_cast<int>(rows.size()) && rows[row] == recordIndex) {
        emit dataChanged(index(row, StateCol), index(row, StateCol));
    }
}

std::size_t ObjectInventoryModel::visitedCount() const {
    return objinv::Inventory::singleton().countWith(objinv::Flag::Visited);
}

QVariant ObjectInventoryModel::data(const QModelIndex & modelIndex, const int role) const {
    const auto * r = recordAt(modelIndex.row());
    if (r == nullptr) {
        return {};
    }
    const auto bitsNow = objinv::Inventory::singleton().flagsFor(r->id);
    const auto annotated = Segmentation::singleton().subobjectExists(r->id)
            || (bitsNow & (objinv::Flag::Painted | objinv::Flag::Erased)) != 0;
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
    case Mag1Voxels: {
        const auto estimate = static_cast<qulonglong>(std::llround(static_cast<double>(r->voxels) * mag1VoxelsPerScanVoxel()));
        return role == Qt::UserRole ? QVariant{estimate} : QVariant{QLocale{}.toString(estimate)};
    }
    case Sphericity: {
        double nm[3];
        scanVoxelNm(nm);
        const auto psi = objinv::sphericity(*r, nm);
        return psi < 0 ? QVariant{QStringLiteral("—")} : QVariant{QString::number(psi, 'f', 2)};
    }
    case Distance: {
        const auto nm = distanceOf(*r);
        return nm >= 1000 ? tr("%1 µm").arg(nm / 1000, 0, 'f', 1) : tr("%1 nm").arg(nm, 0, 'f', 0);
    }
    case Position: {
        const auto p = positionOf(*r);
        return QString("%1, %2, %3").arg(p.x).arg(p.y).arg(p.z);
    }
    case StateCol: {
        /* What is known to have happened to it, in the order it is useful to scan: whether
         * it has been worked on at all, then whether it has merely been looked at. */
        const auto bits = objinv::Inventory::singleton().flagsFor(r->id);
        QStringList what;
        if ((bits & objinv::Flag::Painted) != 0) { what << tr("painted"); }
        if ((bits & objinv::Flag::Erased) != 0) { what << tr("erased"); }
        if ((bits & objinv::Flag::Skeleton) != 0) { what << tr("skeleton"); }
        if (annotated && what.isEmpty()) { what << tr("annotated"); }
        if (seen) { what << tr("visited"); }
        return what.join(" · ");
    }
    }
    return {};
}

// -------------------------------------------------------------------------------- view

ObjectInventoryView::ObjectInventoryView(QWidget * parent) : QWidget(parent) {
    auto & inv = objinv::Inventory::singleton();

    scanButton.setToolTip(tr("Read the segmentation layer in the background and list every object it finds."));
    rescanButton.setToolTip(tr("Throw the list away and read the layer again."));
    deleteButton.setToolTip(tr("Set every voxel of the selected object to background, for removing a\n"
                               "false positive outright. Walks the object's own bounding box and is\n"
                               "one undo step. Delete or Backspace does it while this list has focus, and\n"
                               "%1 from anywhere, on the object . and , last moved to.").arg(QKeySequence(Qt::CTRL + Qt::Key_Backspace).toString(QKeySequence::NativeText)));
    annotationButton.setToolTip(tr("List what this annotation has painted, rather than what the dataset stores.\n"
                                   "Reads the blocks you have edited — no network, and it sees unsaved work,\n"
                                   "but nothing that was already baked into the volume."));
    magCombo.setToolTip(tr("How much detail to read. Coarser is much faster but misses small objects."));
    minVoxelsSpin.setRange(0, 1000000000);
    minVoxelsSpin.setSingleStep(50);
    minVoxelsSpin.setToolTip(tr("Hide anything smaller than this, so the walk is not mostly specks."));
    hideKnownCheck.setToolTip(tr("Hide objects the annotation already has, so the list drains as you work."));
    hideVisitedCheck.setToolTip(tr("Hide objects you have already stepped to."));
    progressBar.setTextVisible(false);
    progressBar.setMaximumWidth(140);
    progressBar.hide();
    statusLabel.setWordWrap(true);

    controlLayout.addWidget(&scanButton);
    controlLayout.addWidget(&rescanButton);
    controlLayout.addWidget(&annotationButton);
    controlLayout.addWidget(&deleteButton);
    controlLayout.addWidget(&magLabel);
    controlLayout.addWidget(&magCombo);
    controlLayout.addStretch();
    controlLayout.addWidget(&progressBar);

    filterLayout.addWidget(&minVoxelsLabel);
    filterLayout.addWidget(&minVoxelsSpin);
    filterLayout.addWidget(&hideKnownCheck);
    filterLayout.addWidget(&hideVisitedCheck);
    filterLayout.addStretch();

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
    /* Click a header to sort by it, click again to reverse; the caret says which. Each click
     * sorts once (see ObjectInventoryModel::sort) — rows found afterwards join at the end —
     * which is what lets the keys walk the visible order without it shifting under them.
     * Scan order ascending is the starting point, and the only order new rows never break. */
    table.header()->setSortIndicator(ObjectInventoryModel::Ordinal, Qt::AscendingOrder);
    table.header()->setSortIndicatorShown(true);
    table.header()->setSectionsClickable(true);
    table.setSortingEnabled(true);
    table.setColumnWidth(ObjectInventoryModel::Ordinal, 70);
    table.setColumnWidth(ObjectInventoryModel::Id, 110);
    table.setColumnWidth(ObjectInventoryModel::Voxels, 90);
    table.setColumnWidth(ObjectInventoryModel::Mag1Voxels, 110);
    table.setColumnWidth(ObjectInventoryModel::Sphericity, 80);
    table.setColumnWidth(ObjectInventoryModel::Distance, 80);
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
    QObject::connect(table.selectionModel(), &QItemSelectionModel::currentRowChanged, this, [this]() { refreshControls(); });
    QObject::connect(&inv, &objinv::Inventory::flagsChanged, this, [this]() { repaintTimer.start(); });
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
    QObject::connect(&deleteButton, &QPushButton::clicked, this, &ObjectInventoryView::eraseCurrentEntry);
    /* Delete and Backspace erase the highlighted object while the list has focus.
     *
     * Scoped to the list rather than the whole pane or the application: in the pane they
     * would take Backspace away from the minimum-size box, and in the viewports Delete
     * already deletes the active node and Backspace a key slice. Cmd+Backspace is the
     * application-wide form, for when focus is in a viewport after pressing . and ,. */
    for (const auto key : {Qt::Key_Delete, Qt::Key_Backspace}) {
        auto * shortcut = new QShortcut(QKeySequence(key), &table, nullptr, nullptr, Qt::WidgetWithChildrenShortcut);
        shortcut->setAutoRepeat(false);
        QObject::connect(shortcut, &QShortcut::activated, this, &ObjectInventoryView::eraseCurrentEntry);
    }
    QObject::connect(&magCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() { refreshStatus(); });

    QObject::connect(&minVoxelsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() { applyFilterFromControls(); });
    QObject::connect(&hideKnownCheck, &QCheckBox::toggled, this, [this]() { applyFilterFromControls(); });
    QObject::connect(&hideVisitedCheck, &QCheckBox::toggled, this, [this]() { applyFilterFromControls(); });
    QObject::connect(table.header(), &QHeaderView::sortIndicatorChanged, this, [this]() { refreshStatus(); });

    QObject::connect(&inv, &objinv::Inventory::recordsAppended, this, [this](const quint64 first, const quint64 count) {
        model.onAppended(first, count);
        refreshStatus();
    });
    QObject::connect(&inv, &objinv::Inventory::recordsRevised, this, [this](const quint64 first, const quint64 last) {
        model.onRevised(first, last);
    });
    QObject::connect(&inv, &objinv::Inventory::stateChanged, this, [this, &inv]() {
        /* Rows are filtered as records arrive, and a record's voxel count keeps growing as
         * later blocks find more of it — so an object first met as a 20-voxel sliver stays
         * filtered out by a 500-voxel minimum even once it has grown to 50 000. Re-filtering
         * on every revision would reshuffle the list under the walk; doing it when a scan
         * stops (finished, paused, stalled) catches every such object, and the rebuild keeps
         * the current row, so . and , carry on from where they were. */
        const auto scanningNow = inv.state() == objinv::State::Scanning;
        if (sawScanning && !scanningNow && minVoxelsSpin.value() > 0) {
            applyFilterFromControls();
        }
        sawScanning = scanningNow;
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
    QObject::connect(&repaintTimer, &QTimer::timeout, this, [this]() {
        objinv::Inventory::singleton().refreshSkeletonFlags();
        refreshVisibleRows();
    });
    QObject::connect(&Skeletonizer::singleton(), &Skeletonizer::resetData, this, [this]() { repaintTimer.start(); });
    QObject::connect(&Skeletonizer::singleton(), &Skeletonizer::nodeAddedSignal, this, [this]() { repaintTimer.start(); });

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
    const auto probing = inv.state() == objinv::State::Probing;
    const auto resumable = inv.state() == objinv::State::Paused || inv.state() == objinv::State::Stalled;
    const auto usable = inv.state() != objinv::State::Unsupported;
    scanButton.setText(scanning ? tr("Pause") : probing ? tr("Checking…") : resumable ? tr("Resume") : tr("Scan"));
    scanButton.setEnabled(usable && !probing);
    rescanButton.setEnabled(usable && !inv.records().empty());
    annotationButton.setEnabled(!scanning);
    deleteButton.setEnabled(!scanning && table.currentIndex().isValid());
    magCombo.setEnabled(usable && !scanning);
    progressBar.setVisible(scanning);
    if (scanning && progressBar.maximum() == 0) {
        progressBar.setRange(0, 100);
        progressBar.setValue(0);
    }
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
        line += tr(" · newly found objects are at the end; click the column header to re-sort");
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

/* Erase the object on the current row.
 *
 * Reached from the button and from an application-wide shortcut, so that the false-positive
 * loop — press . to go to the next object, look, erase it if it is not real — never needs the
 * mouse. Focus is in a viewport for that whole loop, which is why the key is not scoped to
 * this list: it acts on the row the list is on, which is the row . and , moved to.
 *
 * Still asks first, with Erase as the default button, so the keyboard path is the shortcut
 * then Return. It is a chord rather than a bare key, it is confirmed, and it is one undo
 * step — three separate chances to stop a stray keypress taking an object with it. */
void ObjectInventoryView::eraseCurrentEntry() {
    auto & inv = objinv::Inventory::singleton();
    const auto * record = model.recordAt(currentRow());
    if (record == nullptr) {
        emit message(tr("Erase: select an object in the inventory first, or move to one with . and ,"));
        return;
    }
    if (inv.state() == objinv::State::Scanning) {
        emit message(tr("Erase: pause the inventory scan first."));
        return;
    }
    const auto soid = record->id;
    const auto voxels = record->voxels;
    QMessageBox ask{QMessageBox::Question, tr("Erase object"),
                    tr("Set every voxel of object %1 to background?").arg(soid),
                    QMessageBox::Cancel, this};
    ask.setInformativeText(tr("About %1 voxels at the scanned magnification. This is one undo step.").arg(voxels));
    auto * erase = ask.addButton(tr("Erase"), QMessageBox::DestructiveRole);
    ask.setDefaultButton(erase);
    ask.setEscapeButton(QMessageBox::Cancel);
    ask.exec();
    if (ask.clickedButton() == erase) {
        inv.eraseObject(soid, this);
        refreshVisibleRows();
    }
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
    /* Instant, not animated.
     *
     * setPositionWithRecentering() hands over to Remote::process(), which only teleports
     * past roughly half a supercube — 448 voxels at magnification 1 — and animates anything
     * closer. Stepping through a list is the one case where that is pure delay: you already
     * know where you are going, and neighbouring entries in scan order are usually near
     * each other, which is exactly the range that animates. This is what jumpToNode() does. */
    state->viewer->setPosition(position, USERMOVE_NEUTRAL);

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
    // the header state carries the sort column and direction; keyed afresh whenever a column
    // is added, since a saved state describes the table as it was
    const auto header = settings.value(INVENTORY_HEADER_V3).toByteArray();
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
    settings.setValue(INVENTORY_HEADER_V3, table.header()->saveState());
    settings.endGroup();
}

QByteArray ObjectInventoryView::stateJson() const {
    return objinv::Inventory::singleton().flagsJson();
}

void ObjectInventoryView::importStateJson(const QByteArray & json) {
    objinv::Inventory::singleton().importStateJson(json);
    refreshStatus();
}
