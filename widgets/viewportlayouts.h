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

#include "widgets/viewportlayoutgeometry.h"

#include <QByteArray>
#include <QJsonObject>
#include <QObject>
#include <QString>

#include <map>
#include <vector>

/* Named viewport arrangements; see viewportlayoutgeometry.h for the placement maths. */
struct ViewportLayout {
    QString name;
    bool builtin{false};
    // the stock 2×2 arrangement, which has its own handling for proof-reading mode,
    // flat datasets and the arbitrary viewport, so it defers to MainWindow rather than
    // being described as placements
    bool stockArrangement{false};
    // keyed by ViewportType; a viewport missing from the map is hidden by this layout
    std::map<int, ViewportPlacement> placements;
    // extent this arrangement was designed for; left at zero by the built-ins, which
    // simply fill whatever space they are given
    LayoutCanvas canvas;

    QJsonObject toJson() const;
    static bool fromJson(const QJsonObject &, ViewportLayout & out);
};

class ViewportLayouts : public QObject {
    Q_OBJECT
public:
    static ViewportLayouts & singleton() {
        static ViewportLayouts instance;
        return instance;
    }

    const std::vector<ViewportLayout> & builtinLayouts() const { return builtins; }
    const std::vector<ViewportLayout> & userLayouts() const { return user; }
    const ViewportLayout * find(const QString & name) const;

    void addOrReplaceUser(const ViewportLayout & layout);
    bool removeUser(const QString & name);

    void loadSettings();
    void saveSettings() const;

    // round trip through the annotation file, so layouts travel with the work
    QByteArray userLayoutsJson() const;
    // adds layouts whose names are not already taken; returns how many were added
    int importJson(const QByteArray & json, QStringList & skipped);

signals:
    void changed();

private:
    ViewportLayouts();
    std::vector<ViewportLayout> builtins;
    std::vector<ViewportLayout> user;
};
