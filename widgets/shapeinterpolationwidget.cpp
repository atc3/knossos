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

#include "shapeinterpolationwidget.h"

#include "segmentation/shapeinterpolation.h"
#include "widgets/GuiConstants.h"

ShapeInterpolationWidget::ShapeInterpolationWidget(QWidget * const parent) : DialogVisibilityNotify(SHAPE_INTERPOLATION_WIDGET, parent) {
    setWindowTitle(tr("Shape Interpolation"));

    statusLabel.setWordWrap(true);
    hintLabel.setWordWrap(true);
    hintLabel.setText(tr("Paint the object on a slice, move to another slice, paint it again. "
                         "Everything between is interpolated. Add as many key slices as you like."));
    auto hintFont = hintLabel.font();
    hintFont.setItalic(true);
    hintLabel.setFont(hintFont);

    sliceList.setToolTip(tr("Key slices in this chain. Double click one to jump to it."));
    alignCombo.addItem(tr("by best overlap"), static_cast<int>(ShapeInterpolation::Alignment::Overlap));
    alignCombo.addItem(tr("on their centroids"), static_cast<int>(ShapeInterpolation::Alignment::Centroids));
    alignCombo.addItem(tr("don't align"), static_cast<int>(ShapeInterpolation::Alignment::None));
    alignCombo.setToolTip(tr("How each pair of key slices is lined up before interpolating, by translation only.\n"
                             "Best overlap: whatever shift makes the two outlines overlap the most — follows the bulk\n"
                             "of an irregular shape. Centroids: their centres of mass, which a branch on one side pulls\n"
                             "off the body. Without either, a structure drifting sideways between slices pinches."));
    alignLayout.addWidget(&alignLabel);
    alignLayout.addWidget(&alignCombo, 1);

    buttonLayout.addWidget(&acceptButton);
    buttonLayout.addWidget(&discardButton);

    mainLayout.addWidget(&hintLabel);
    mainLayout.addWidget(&statusLabel);
    mainLayout.addWidget(&sliceList);
    mainLayout.addWidget(&previewCheck);
    mainLayout.addLayout(&alignLayout);
    mainLayout.addLayout(&buttonLayout);
    setLayout(&mainLayout);

    auto & si = ShapeInterpolation::singleton();
    previewCheck.setChecked(si.previewEnabled());
    alignCombo.setCurrentIndex(alignCombo.findData(static_cast<int>(si.alignment())));

    QObject::connect(&previewCheck, &QCheckBox::toggled, [](const bool on){ ShapeInterpolation::singleton().setPreviewEnabled(on); });
    QObject::connect(&alignCombo, QOverload<int>::of(&QComboBox::activated), [this](const int index){
        ShapeInterpolation::singleton().setAlignment(static_cast<ShapeInterpolation::Alignment>(alignCombo.itemData(index).toInt()));
    });
    QObject::connect(&acceptButton, &QPushButton::clicked, this, &ShapeInterpolationWidget::acceptRequested);
    QObject::connect(&discardButton, &QPushButton::clicked, this, &ShapeInterpolationWidget::discardRequested);
    QObject::connect(&sliceList, &QListWidget::itemDoubleClicked, [this](QListWidgetItem * item){
        emit jumpToDepthRequested(item->data(Qt::UserRole).toInt());
    });
    QObject::connect(&si, &ShapeInterpolation::changed, this, &ShapeInterpolationWidget::updateFromController);

    updateFromController();
}

void ShapeInterpolationWidget::updateFromController() {
    const auto & si = ShapeInterpolation::singleton();
    const auto count = si.sliceCount();

    sliceList.clear();
    std::size_t voxels = 0;
    for (const auto & [depth, slice] : si.sliceMap()) {
        auto * item = new QListWidgetItem(tr("depth %1  ·  %2 voxels").arg(depth).arg(slice.count()), &sliceList);
        item->setData(Qt::UserRole, depth);
        voxels += slice.count();
    }

    statusLabel.setText(si.summary() + (count != 0 ? tr("\n%1 painted voxels.").arg(voxels) : QString{}));
    if (!si.lastError().isEmpty()) {
        statusLabel.setText(statusLabel.text() + "\n" + si.lastError());
    }

    previewCheck.setChecked(si.previewEnabled());
    alignCombo.setCurrentIndex(alignCombo.findData(static_cast<int>(si.alignment())));
    acceptButton.setEnabled(count >= 2);
    discardButton.setEnabled(count >= 1);
}
