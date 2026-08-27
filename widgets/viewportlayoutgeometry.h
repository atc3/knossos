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

/* Geometry behind the named viewport arrangements.
 *
 * Kept free of Qt and of any KNOSSOS type so the arithmetic can be exercised on its own —
 * see tests/viewportlayout_test.cpp. Viewports are square by construction (ViewportBase
 * carries a single edgeLength and the screen-to-dataset mapping assumes it), so a
 * placement is a position plus one side length rather than a rectangle.
 *
 * Placements live in units of a reference length rather than pixels, which is what lets a
 * layout describe proportions and survive any window size. */

#include <algorithm>
#include <cmath>
#include <map>

struct ViewportPlacement {
    double x{0}, y{0}, side{1};
};

struct ViewportGeometry {
    int x{0}, y{0}, side{0};
};

constexpr int MIN_LAYOUT_VP_SIZE = 50;

/* The extent a layout wants, in units.
 *
 * Zero means "derive it from the viewports", which is what the built-in arrangements do —
 * they are designed to fill whatever space they are given. A captured layout instead
 * records the window it came from, so that empty space the user deliberately left is part
 * of the arrangement and survives being re-applied. */
struct LayoutCanvas {
    double width{0}, height{0};
};

/* The reference length, chosen so the whole arrangement fits the space available: a layout
 * needing 1.5 units across and 1 down gets whichever of width/1.5 and height/1 is smaller.
 * Returns 0 for an empty layout. */
inline double layoutUnit(const std::map<int, ViewportPlacement> & placements, const LayoutCanvas & canvas, const int availableWidth, const int availableHeight, const int margin) {
    double neededWidth{canvas.width}, neededHeight{canvas.height};
    if (neededWidth <= 0 || neededHeight <= 0) {
        neededWidth = neededHeight = 0;
        for (const auto & entry : placements) {
            neededWidth = std::max(neededWidth, entry.second.x + entry.second.side);
            neededHeight = std::max(neededHeight, entry.second.y + entry.second.side);
        }
    }
    if (neededWidth <= 0 || neededHeight <= 0) {
        return 0;
    }
    const auto usableWidth = std::max(1, availableWidth - 2 * margin);
    const auto usableHeight = std::max(1, availableHeight - 2 * margin);
    return std::min(usableWidth / neededWidth, usableHeight / neededHeight);
}

inline ViewportGeometry placementGeometry(const ViewportPlacement & placement, const double unit, const int margin) {
    return {margin + static_cast<int>(std::lround(placement.x * unit)),
            margin + static_cast<int>(std::lround(placement.y * unit)),
            std::max(MIN_LAYOUT_VP_SIZE, static_cast<int>(std::lround(placement.side * unit)) - margin)};
}

/* Inverse of placementGeometry(): turns a viewport's pixel geometry back into a placement.
 * The reference length is the usable height, which is what layoutUnit() converges on for a
 * layout whose height requirement is the binding one. */
inline ViewportPlacement capturePlacement(const int x, const int y, const int width, const int referenceLength, const int margin) {
    const auto unit = std::max(1, referenceLength);
    return {static_cast<double>(x - margin) / unit,
            static_cast<double>(y - margin) / unit,
            static_cast<double>(width + margin) / unit};
}

/* The canvas to record alongside a captured arrangement: the usable area expressed in the
 * same reference length capturePlacement() used. */
inline LayoutCanvas captureCanvas(const int availableWidth, const int availableHeight, const int margin) {
    const auto usableWidth = std::max(1, availableWidth - 2 * margin);
    const auto usableHeight = std::max(1, availableHeight - 2 * margin);
    return {static_cast<double>(usableWidth) / usableHeight, 1.0};
}
