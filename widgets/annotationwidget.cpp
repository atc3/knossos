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

#include "annotationwidget.h"

#include "GuiConstants.h"
#include "skeleton/skeletonizer.h"
#include "viewer.h"

#include <QLineEdit>
#include <QCheckBox>
#include <QSettings>

#include <vector>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QApplication>
#include <QDesktopWidget>
#include <QSplitter>

AnnotationWidget::AnnotationWidget(QWidget *parent) : DialogVisibilityNotify(ANNOTATION_WIDGET, parent) {
    setWindowIcon(QIcon(":/resources/icons/annotation.png"));
    setWindowTitle("Annotation");

    tabs.addTab(&skeletonTab, "Skeleton");
    tabs.addTab(&segmentationTab, "Segmentation");
    tabs.addTab(&commentsTab, "Comments");
    tabs.addTab(&inventoryTab, "Inventory");

    mainLayout.addWidget(&tabs);
    mainLayout.setContentsMargins({});
    setLayout(&mainLayout);
}

void AnnotationWidget::setSegmentationVisibility(const bool visible) {
    const QString tooltip = "Enable the segmentation overlay when loading a dataset";
    // the inventory is a list of segmentation objects, so it is meaningless without the layer
    for (auto * tab : std::vector<QWidget *>{&segmentationTab, &inventoryTab}) {
        const auto index = tabs.indexOf(tab);
        tabs.setTabEnabled(index, visible);
        tabs.setTabToolTip(index, visible ? "" : tooltip);
    }
}

void AnnotationWidget::loadSettings() {
    QSettings settings;
    settings.beginGroup(ANNOTATION_WIDGET);
    restoreGeometry(settings.value(GEOMETRY).toByteArray());

    skeletonTab.treeCommentFilter.setText(settings.value(SEARCH_FOR_TREE, "").toString());
    skeletonTab.nodeCommentFilter.setText(settings.value(SEARCH_FOR_NODE, "").toString());
    settings.endGroup();

    commentsTab.loadSettings();
    inventoryTab.loadSettings();
}

void AnnotationWidget::saveSettings() {
    QSettings settings;
    settings.beginGroup(ANNOTATION_WIDGET);
    settings.setValue(VISIBLE, isVisible());

    settings.setValue(SEARCH_FOR_TREE, skeletonTab.treeCommentFilter.text());
    settings.setValue(SEARCH_FOR_NODE, skeletonTab.nodeCommentFilter.text());
    settings.endGroup();

    commentsTab.saveSettings();
    inventoryTab.saveSettings();
}
