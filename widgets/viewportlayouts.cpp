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

#include "viewportlayouts.h"

#include "widgets/GuiConstants.h"
#include "widgets/viewports/viewportbase.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QSettings>
#include <QStringList>

#include <algorithm>

QJsonObject ViewportLayout::toJson() const {
    QJsonArray entries;
    for (const auto & [viewportType, placement] : placements) {
        QJsonObject entry;
        entry["viewport"] = viewportType;
        entry["x"] = placement.x;
        entry["y"] = placement.y;
        entry["side"] = placement.side;
        entries.append(entry);
    }
    QJsonObject object;
    object["name"] = name;
    object["canvasWidth"] = canvas.width;
    object["canvasHeight"] = canvas.height;
    object["placements"] = entries;
    return object;
}

bool ViewportLayout::fromJson(const QJsonObject & object, ViewportLayout & out) {
    const auto name = object["name"].toString();
    if (name.isEmpty() || !object["placements"].isArray()) {
        return false;
    }
    out = ViewportLayout{};
    out.name = name;
    out.canvas = {object["canvasWidth"].toDouble(), object["canvasHeight"].toDouble()};
    for (const auto entry : object["placements"].toArray()) {
        const auto fields = entry.toObject();
        const auto viewportType = fields["viewport"].toInt(-1);
        if (viewportType < 0 || viewportType >= ViewportBase::numberViewports) {
            continue;
        }
        ViewportPlacement placement;
        placement.x = fields["x"].toDouble();
        placement.y = fields["y"].toDouble();
        placement.side = fields["side"].toDouble();
        if (placement.side <= 0) {
            continue;
        }
        out.placements[viewportType] = placement;
    }
    return !out.placements.empty();
}

ViewportLayouts::ViewportLayouts() {
    ViewportLayout grid;
    grid.name = tr("Grid (2×2)");
    grid.builtin = grid.stockArrangement = true;
    builtins.push_back(grid);

    // One plane at full height with the other two stacked beside it at half size, and no
    // 3D viewport. 1.5 units across by 1 down, so it favours wide windows.
    const auto sideBySide = [this](const QString & name, const int mainVP, const int upperVP, const int lowerVP){
        ViewportLayout layout;
        layout.name = name;
        layout.builtin = true;
        layout.placements[mainVP] = {0.0, 0.0, 1.0};
        layout.placements[upperVP] = {1.0, 0.0, 0.5};
        layout.placements[lowerVP] = {1.0, 0.5, 0.5};
        builtins.push_back(layout);
    };
    sideBySide(tr("xy large + xz/zy"), VIEWPORT_XY, VIEWPORT_XZ, VIEWPORT_ZY);
    sideBySide(tr("xz large + xy/zy"), VIEWPORT_XZ, VIEWPORT_XY, VIEWPORT_ZY);
    sideBySide(tr("zy large + xy/xz"), VIEWPORT_ZY, VIEWPORT_XY, VIEWPORT_XZ);

    const auto focus = [this](const QString & name, const int viewportType){
        ViewportLayout layout;
        layout.name = name;
        layout.builtin = true;
        layout.placements[viewportType] = {0.0, 0.0, 1.0};
        builtins.push_back(layout);
    };
    focus(tr("focus-xy"), VIEWPORT_XY);
    focus(tr("focus-xz"), VIEWPORT_XZ);
    focus(tr("focus-zy"), VIEWPORT_ZY);
    focus(tr("focus-arb"), VIEWPORT_ARBITRARY);
    focus(tr("focus-3d"), VIEWPORT_SKELETON);
}

const ViewportLayout * ViewportLayouts::find(const QString & name) const {
    for (const auto * list : {&builtins, &user}) {
        const auto it = std::find_if(std::begin(*list), std::end(*list), [&name](const auto & layout){ return layout.name == name; });
        if (it != std::end(*list)) {
            return &*it;
        }
    }
    return nullptr;
}

void ViewportLayouts::addOrReplaceUser(const ViewportLayout & layout) {
    auto copy = layout;
    copy.builtin = copy.stockArrangement = false;
    const auto it = std::find_if(std::begin(user), std::end(user), [&copy](const auto & existing){ return existing.name == copy.name; });
    if (it != std::end(user)) {
        *it = copy;
    } else {
        user.push_back(copy);
    }
    saveSettings();
    emit changed();
}

bool ViewportLayouts::removeUser(const QString & name) {
    const auto it = std::find_if(std::begin(user), std::end(user), [&name](const auto & layout){ return layout.name == name; });
    if (it == std::end(user)) {
        return false;
    }
    user.erase(it);
    saveSettings();
    emit changed();
    return true;
}

QByteArray ViewportLayouts::userLayoutsJson() const {
    QJsonArray entries;
    for (const auto & layout : user) {
        entries.append(layout.toJson());
    }
    QJsonObject root;
    root["viewport_layouts"] = entries;
    return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

int ViewportLayouts::importJson(const QByteArray & json, QStringList & skipped) {
    const auto document = QJsonDocument::fromJson(json);
    if (!document.isObject()) {
        return 0;
    }
    int added{0};
    for (const auto entry : document.object()["viewport_layouts"].toArray()) {
        ViewportLayout layout;
        if (!ViewportLayout::fromJson(entry.toObject(), layout)) {
            continue;
        }
        // never clobber a layout the user already has under that name — an annotation
        // from someone else should not silently redefine your own arrangement
        if (find(layout.name) != nullptr) {
            skipped.append(layout.name);
            continue;
        }
        user.push_back(layout);
        ++added;
    }
    if (added != 0) {
        saveSettings();
        emit changed();
    }
    return added;
}

void ViewportLayouts::loadSettings() {
    QSettings settings;
    settings.beginGroup(VIEWPORT_LAYOUTS);
    const auto json = settings.value("layouts").toByteArray();
    settings.endGroup();
    user.clear();
    const auto document = QJsonDocument::fromJson(json);
    for (const auto entry : document.object()["viewport_layouts"].toArray()) {
        ViewportLayout layout;
        if (ViewportLayout::fromJson(entry.toObject(), layout)) {
            user.push_back(layout);
        }
    }
    emit changed();
}

void ViewportLayouts::saveSettings() const {
    QSettings settings;
    settings.beginGroup(VIEWPORT_LAYOUTS);
    settings.setValue("layouts", userLayoutsJson());
    settings.endGroup();
}
