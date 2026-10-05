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

#include "morphologywidget.h"

#include "dataset.h"
#include "segmentation/objectmorphology.h"
#include "segmentation/segmentation.h"
#include "widgets/GuiConstants.h"

#include <QSettings>

#include <cmath>
#include <limits>

MorphologyWidget::MorphologyWidget(QWidget * const parent) : DialogVisibilityNotify(MORPHOLOGY_WIDGET, parent) {
    setWindowTitle(tr("Dilate, Erode, Smooth"));

    hintLabel.setWordWrap(true);
    hintLabel.setText(tr("Works on the selected object, over a region about the crosshair — not over the whole "
                         "object, which for anything long would mean loading the entire dataset. Move the "
                         "crosshair onto the part you want changed."));
    auto hintFont = hintLabel.font();
    hintFont.setItalic(true);
    hintLabel.setFont(hintFont);

    radiusSpin.setRange(0.0, 100000.0);
    radiusSpin.setDecimals(1);
    radiusSpin.setSuffix(tr(" nm"));
    radiusSpin.setToolTip(tr("How far the boundary moves, as a physical distance. A length rather than a voxel "
                             "count because the voxels are not cubes: the same radius reaches further along a "
                             "finely sampled axis than along z, which is what you want and is how Paintera does "
                             "it too."));
    reachLabel.setToolTip(radiusSpin.toolTip());

    directionCombo.addItem(tr("Both — round the shape off"));
    directionCombo.addItem(tr("In — only shave spurs off"));
    directionCombo.addItem(tr("Out — only fill dents in"));
    directionCombo.setToolTip(tr("Which way a smooth may move the boundary. Dilate and erode ignore this; they "
                                 "are each one direction by definition."));

    infillCombo.addItem(tr("Background"));
    infillCombo.addItem(tr("Nearest label"));
    infillCombo.addItem(tr("A specific id"));
    infillCombo.setToolTip(tr("What a voxel the object gives up becomes. Background rubs it out. Nearest label "
                              "hands it to whichever label is closest, which keeps a dense segmentation dense but "
                              "does grow a neighbouring object — Paintera's default, not this one, because "
                              "KNOSSOS annotations are usually sparse and that would be a surprise."));

    replacementSpin.setRange(0, std::numeric_limits<int>::max());
    replacementSpin.setToolTip(tr("The id to hand given-up voxels to."));

    thresholdSpin.setRange(0.01, 0.99);
    thresholdSpin.setSingleStep(0.05);
    thresholdSpin.setDecimals(2);
    thresholdSpin.setToolTip(tr("Where the blurred mask counts as inside the object. 0.5 is the neutral choice and "
                                "keeps the object about its own size; lower grows it, higher shrinks it."));

    fragmentsCheck.setToolTip(tr("A merged object is several ids that render alike. By default all of them count as "
                                 "the object, so the seams between them are left alone; with this on only the id you "
                                 "clicked counts, and its merge partners are treated as foreign tissue."));

    extentSpin.setRange(8, 1024);
    extentSpin.setSingleStep(16);
    extentSpin.setSuffix(tr(" voxels"));
    extentSpin.setToolTip(tr("The side of the region worked on, about the crosshair, in voxels of the current "
                             "magnification. Reduced automatically if it would not fit in memory."));

    form.addRow(tr("Radius"), &radiusSpin);
    form.addRow(QString(), &reachLabel);
    form.addRow(tr("Smooth direction"), &directionCombo);
    form.addRow(tr("Give-up infill"), &infillCombo);
    form.addRow(tr("Infill id"), &replacementSpin);
    form.addRow(tr("Smooth threshold"), &thresholdSpin);
    form.addRow(tr("Region"), &extentSpin);

    buttonLayout.addWidget(&dilateButton);
    buttonLayout.addWidget(&erodeButton);
    buttonLayout.addWidget(&smoothButton);

    statusLabel.setWordWrap(true);

    mainLayout.addWidget(&hintLabel);
    mainLayout.addLayout(&form);
    mainLayout.addWidget(&fragmentsCheck);
    mainLayout.addLayout(&buttonLayout);
    mainLayout.addWidget(&statusLabel);
    setLayout(&mainLayout);

    QSettings settings;
    settings.beginGroup(MORPHOLOGY_WIDGET);
    radiusSpin.setValue(settings.value(MORPH_RADIUS, defaultMorphRadius()).toDouble());
    directionCombo.setCurrentIndex(settings.value(MORPH_DIRECTION, 0).toInt());
    infillCombo.setCurrentIndex(settings.value(MORPH_INFILL, 0).toInt());
    replacementSpin.setValue(settings.value(MORPH_REPLACEMENT, 0).toInt());
    thresholdSpin.setValue(settings.value(MORPH_THRESHOLD, 0.5).toDouble());
    fragmentsCheck.setChecked(settings.value(MORPH_FRAGMENTS_ONLY, false).toBool());
    extentSpin.setValue(settings.value(MORPH_EXTENT, 128).toInt());
    settings.endGroup();

    const auto onChange = [this](){ persist(); updateReach(); };
    QObject::connect(&radiusSpin, &QDoubleSpinBox::editingFinished, onChange);
    QObject::connect(&radiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [onChange](double){ onChange(); });
    QObject::connect(&directionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [onChange](int){ onChange(); });
    QObject::connect(&infillCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this, onChange](const int index){
        replacementSpin.setEnabled(index == 2);
        onChange();
    });
    QObject::connect(&replacementSpin, QOverload<int>::of(&QSpinBox::valueChanged), [onChange](int){ onChange(); });
    QObject::connect(&thresholdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [onChange](double){ onChange(); });
    QObject::connect(&fragmentsCheck, &QCheckBox::toggled, [onChange](bool){ onChange(); });
    QObject::connect(&extentSpin, QOverload<int>::of(&QSpinBox::valueChanged), [onChange](int){ onChange(); });

    QObject::connect(&dilateButton, &QPushButton::clicked, [this](){ apply(morphology::Operation::Dilate); });
    QObject::connect(&erodeButton, &QPushButton::clicked, [this](){ apply(morphology::Operation::Erode); });
    QObject::connect(&smoothButton, &QPushButton::clicked, [this](){ apply(morphology::Operation::Smooth); });

    replacementSpin.setEnabled(infillCombo.currentIndex() == 2);
    updateReach();
}

void MorphologyWidget::persist() const {
    QSettings settings;
    settings.beginGroup(MORPHOLOGY_WIDGET);
    settings.setValue(MORPH_RADIUS, radiusSpin.value());
    settings.setValue(MORPH_DIRECTION, directionCombo.currentIndex());
    settings.setValue(MORPH_INFILL, infillCombo.currentIndex());
    settings.setValue(MORPH_REPLACEMENT, replacementSpin.value());
    settings.setValue(MORPH_THRESHOLD, thresholdSpin.value());
    settings.setValue(MORPH_FRAGMENTS_ONLY, fragmentsCheck.isChecked());
    settings.setValue(MORPH_EXTENT, extentSpin.value());
    settings.endGroup();
}

/* Say what the radius actually reaches, per axis.
 *
 * The number that matters is the same one the operation uses — the count of whole voxels
 * within the radius along each axis — so this is deliberately computed the same way rather
 * than rounded for display. An axis the radius does not reach at all reads "0", which is
 * the single most useful thing this label does: on 40 nm sections the default radius is a
 * purely in-plane operation and that is not otherwise apparent. */
void MorphologyWidget::updateReach() {
    if (Dataset::datasets.empty() || Segmentation::singleton().layerId >= Dataset::datasets.size()) {
        reachLabel.setText(tr("Reach: no dataset open."));
        return;
    }
    const auto & dataset = Dataset::datasets[Segmentation::singleton().layerId];
    if (dataset.scales.empty()) {
        reachLabel.setText(tr("Reach: the layer declares no voxel size."));
        return;
    }
    const auto spacing = dataset.magIndex < dataset.scales.size()
            ? dataset.scales[dataset.magIndex]
            : floatCoordinate{dataset.scales[0].x * dataset.scaleFactor.x,
                              dataset.scales[0].y * dataset.scaleFactor.y,
                              dataset.scales[0].z * dataset.scaleFactor.z};
    const double per[3]{spacing.x, spacing.y, spacing.z};
    int reach[3]{0, 0, 0};
    for (int a = 0; a < 3; ++a) {
        reach[a] = per[a] > 0 ? static_cast<int>(std::floor(radiusSpin.value() / per[a])) : 0;
    }
    reachLabel.setText(tr("Reach: %1 × %2 × %3 voxels at %4 × %5 × %6 nm")
                       .arg(reach[0]).arg(reach[1]).arg(reach[2])
                       .arg(per[0]).arg(per[1]).arg(per[2]));
}

MorphologyWidget::Settings MorphologyWidget::settings() const {
    Settings s;
    s.radius = radiusSpin.value();
    s.direction = directionCombo.currentIndex() == 1 ? morphology::Direction::Shrink
                : directionCombo.currentIndex() == 2 ? morphology::Direction::Expand
                : morphology::Direction::Both;
    s.infill = infillCombo.currentIndex() == 1 ? morphology::Infill::NearestLabel
             : infillCombo.currentIndex() == 2 ? morphology::Infill::Replace
             : morphology::Infill::Background;
    s.replacement = static_cast<std::uint64_t>(replacementSpin.value());
    s.threshold = thresholdSpin.value();
    s.fragmentsOnly = fragmentsCheck.isChecked();
    s.extent = extentSpin.value();
    return s;
}

QString MorphologyWidget::apply(const morphology::Operation op) {
    const auto s = settings();
    MorphRequest request;
    request.op = op;
    request.radius = s.radius;
    request.direction = s.direction;
    request.infill = s.infill;
    request.replacement = s.replacement;
    request.threshold = s.threshold;
    request.fragmentsOnly = s.fragmentsOnly;
    request.extent = s.extent;
    const auto message = runObjectMorphology(request, this).message;
    showReport(message);
    return message;
}

void MorphologyWidget::showReport(const QString & message) {
    statusLabel.setText(message);
}

void MorphologyWidget::loadSettings() {
    DialogVisibilityNotify::loadSettings();
    // the default radius depends on the dataset, which may not have been open last time
    if (radiusSpin.value() <= 0) {
        radiusSpin.setValue(defaultMorphRadius());
    }
    updateReach();
}
