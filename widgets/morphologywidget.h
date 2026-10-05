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

/* Settings for dilate, erode and smooth, and the place smoothing is driven from.
 *
 * One panel for all three because they share every parameter that matters — Paintera does
 * the same, with one MorphCommonModel behind its three actions. Dilate and erode also have
 * Action-menu entries that apply straight away with whatever is set here, because at a
 * fixed radius they are the kind of thing you want to press repeatedly rather than
 * configure; smoothing has more to decide and so lives here.
 *
 * The radius is shown in nanometres *and* as the number of voxels it reaches along each
 * axis, which on anisotropic data are three different numbers. Without that the control is
 * close to unusable: on 10×10×40 nm voxels the default radius reaches two voxels sideways
 * and none at all in z, and nothing on screen would otherwise say so. */

#include "widgets/DialogVisibilityNotify.h"
#include "segmentation/morphology.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

class MorphologyWidget : public DialogVisibilityNotify {
    Q_OBJECT

    QVBoxLayout mainLayout;
    QLabel hintLabel;
    QFormLayout form;
    QDoubleSpinBox radiusSpin;
    QLabel reachLabel;
    QComboBox directionCombo;
    QComboBox infillCombo;
    QSpinBox replacementSpin;
    QDoubleSpinBox thresholdSpin;
    QCheckBox fragmentsCheck{tr("Only the clicked fragment, not the whole merged object")};
    QSpinBox extentSpin;
    QHBoxLayout buttonLayout;
    QPushButton dilateButton{tr("Dilate")};
    QPushButton erodeButton{tr("Erode")};
    QPushButton smoothButton{tr("Smooth")};
    QLabel statusLabel;

    void persist() const;
    void updateReach();

public:
    explicit MorphologyWidget(QWidget * parent = nullptr);
    // the settings as the panel currently has them, for the Action-menu entries
    struct Settings {
        double radius{0};
        morphology::Direction direction{morphology::Direction::Both};
        morphology::Infill infill{morphology::Infill::Background};
        std::uint64_t replacement{0};
        double threshold{0.5};
        bool fragmentsOnly{false};
        int extent{128};
    };
    Settings settings() const;
    // runs the operation, shows the outcome on its own status line, and hands it back for the status bar
    QString apply(morphology::Operation);
    void showReport(const QString & message);
    virtual void loadSettings() override;
};
