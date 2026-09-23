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

#include "widgets/DialogVisibilityNotify.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

/* Control panel for the shape-interpolation chain: which slices are keyed, how big the
 * interpolated region is, and the accept/discard buttons. Everything here is also
 * reachable from the Action menu and its shortcuts; this just makes the state visible. */
class ShapeInterpolationWidget : public DialogVisibilityNotify {
    Q_OBJECT

    QVBoxLayout mainLayout;
    QLabel statusLabel;
    QListWidget sliceList;
    QCheckBox previewCheck{tr("Preview interpolation")};
    QCheckBox alignCheck{tr("Align slices on their centroids")};
    QHBoxLayout buttonLayout;
    QPushButton acceptButton{tr("Accept (Enter)")};
    QPushButton discardButton{tr("Discard slices")};
    QLabel hintLabel;

public:
    explicit ShapeInterpolationWidget(QWidget * parent = nullptr);
    void updateFromController();

signals:
    void acceptRequested();
    void discardRequested();
    void jumpToDepthRequested(int depth);
};
