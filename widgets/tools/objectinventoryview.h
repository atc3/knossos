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

/* The Inventory tab: what the background sweep found, and the keys that walk it.
 *
 * This is a separate table from the Segmentation tab's on purpose. That model indexes
 * Segmentation::objects directly, row by object index, and its selection machinery is built
 * on that identity; a sweep produces hundreds of thousands of *subobject* ids, most of which
 * have no object at all, and materialising one each would be catastrophic — looking an id up
 * in Segmentation creates it. So nothing here resolves an id except the jump itself, one id
 * per keypress, which is exactly what clicking that voxel would have done.
 *
 * The selection flow is one-way, inventory row to Segmentation, and this widget subscribes
 * to none of Segmentation's selection signals. The Segmentation tab guards its own
 * two-way sync with re-entrancy flags; not joining that conversation is simpler and also
 * more honest, since several inventory rows can belong to one merged object. */

#include "segmentation/objectinventory.h"
#include "widgets/tools/inventoryfilter.h"

#include <QAbstractTableModel>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QTreeView>
#include <QVBoxLayout>
#include <QShowEvent>
#include <QWidget>

#include <cstdint>
#include <vector>

class ObjectInventoryView;

class ObjectInventoryModel : public QAbstractTableModel {
    Q_OBJECT
    friend class ObjectInventoryView;
public:
    enum Column { Ordinal, Id, Voxels, Position, StateCol, ColumnCount };
    enum class Order { Scan, LargestFirst, NearestFirst };

    explicit ObjectInventoryModel(QObject * parent = nullptr);
    int rowCount(const QModelIndex & = {}) const override;
    int columnCount(const QModelIndex & = {}) const override;
    QVariant headerData(int section, Qt::Orientation, int role) const override;
    QVariant data(const QModelIndex &, int role = Qt::DisplayRole) const override;

    const objinv::Record * recordAt(int row) const;
    std::size_t recordIndexAt(int row) const;
    int rowOfRecord(std::size_t recordIndex) const;
    bool visitedAt(int row) const;
    void markVisited(std::size_t recordIndex, bool);
    std::vector<std::uint64_t> visitedIds() const;
    void adoptVisitedIds(const std::vector<std::uint64_t> &);

    void setFilter(const objinv::Filter &);
    const objinv::Filter & filter() const { return activeFilter; }
    void setOrder(Order);
    Order order() const { return activeOrder; }
    /* Re-applies the current order to everything discovered since it was last applied.
     * Deliberately manual: re-sorting on every batch would renumber the list under a walk
     * in progress, which is the one thing the navigation keys cannot tolerate. */
    void reapplyOrder();
    std::size_t shownCount() const { return rows.size(); }
    std::size_t visitedCount() const;
    bool orderIsStale() const { return staleUnderOrder; }

    void rebuild();
    // repaint a row range, for values that go stale without the row set changing
    void repaintRows(int first, int last);
    void onAppended(std::size_t first, std::size_t count);
    void onRevised(std::size_t first, std::size_t last);

private:
    bool accepts(std::size_t recordIndex) const;
    void sortRows();

    std::vector<std::uint32_t> rows;// view row -> record index; strictly increasing under Order::Scan
    std::vector<bool> visited;
    objinv::Filter activeFilter;
    Order activeOrder{Order::Scan};
    bool staleUnderOrder{false};
};

class ObjectInventoryView : public QWidget {
    Q_OBJECT
    QVBoxLayout layout;
    QHBoxLayout controlLayout, filterLayout, bottomLayout;

    QPushButton scanButton{tr("Scan")};
    QPushButton rescanButton{tr("Rescan")};
    QPushButton annotationButton{tr("Scan annotation")};
    QComboBox magCombo;
    QLabel magLabel{tr("detail")};
    QProgressBar progressBar;

    QLabel minVoxelsLabel{tr("min voxels")};
    QSpinBox minVoxelsSpin;
    QCheckBox hideKnownCheck{tr("hide annotated")};
    QCheckBox hideVisitedCheck{tr("hide visited")};
    QComboBox orderCombo;
    QPushButton reorderButton{tr("↻")};

    QTreeView table;
    ObjectInventoryModel model;
    QLabel statusLabel;
    QMenu contextMenu;
    QAction * jumpAction{nullptr};
    QAction * markVisitedAction{nullptr};
    QAction * unmarkVisitedAction{nullptr};
    QAction * markAboveAction{nullptr};
    QTimer repaintTimer;// coalesces the State column refresh, never a full-table repaint

public:
    explicit ObjectInventoryView(QWidget * parent = nullptr);
    /* Next / previous object. Walks the rows the filter leaves visible, in the order the
     * table shows them — which, because sorting here is one-shot, is a fixed order that only
     * ever grows at the end. Caps at both ends rather than wrapping. */
    void jumpToNextEntry(bool forward);
    void jumpToRow(int row);

    void loadSettings();
    void saveSettings();
    // The visited set travels with the annotation, keyed by id so a rescan does not lose it.
    QByteArray visitedJson() const;
    void importVisitedJson(const QByteArray &);

signals:
    void message(const QString &);

protected:
    void showEvent(QShowEvent *) override;

private:
    void applyFilterFromControls();
    void refreshControls();
    void refreshStatus();
    void refreshVisibleRows();
    void rebuildMagCombo();
    void showContextMenu(const QPoint &);
    int currentRow() const;
};
