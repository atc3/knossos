// Exercises widgets/viewportlayoutgeometry.h — the placement maths behind the named
// viewport arrangements.
#include "widgets/viewportlayoutgeometry.h"
#include <cstdio>
#include <cstdlib>
#include <map>

enum { XY = 0, XZ = 1, ZY = 2, ARB = 3, SKEL = 4 };
constexpr int M = 5;// DEFAULT_VP_MARGIN

static int failures = 0;
static void check(const bool ok, const char * what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) { ++failures; }
}

static std::map<int, ViewportPlacement> sideBySide() {
    return {{XY, {0.0, 0.0, 1.0}}, {XZ, {1.0, 0.0, 0.5}}, {ZY, {1.0, 0.5, 0.5}}};
}
static std::map<int, ViewportPlacement> focus() { return {{XY, {0.0, 0.0, 1.0}}}; }

int main() {
    std::printf("1. the unit fits the arrangement into the space available\n");
    {
        // needs 1.5 units across, 1 down. In a 1600x900 window height is the binding one:
        // 1.5 * 890 = 1335 <= 1590, so unit == usable height
        const auto unit = layoutUnit(sideBySide(), {}, 1600, 900, M);
        check(std::abs(unit - 890.0) < 0.5, "wide window: limited by height");

        // in a narrow window width binds instead: 1000-10 = 990, /1.5 = 660
        const auto narrow = layoutUnit(sideBySide(), {}, 1000, 900, M);
        check(std::abs(narrow - 660.0) < 0.5, "narrow window: limited by width");

        check(layoutUnit({}, {}, 1600, 900, M) == 0, "empty layout yields no unit");
    }

    std::printf("2. the arrangement always fits inside the window\n");
    {
        for (const int w : {600, 1000, 1600, 2560}) {
            for (const int h : {400, 900, 1440}) {
                const auto unit = layoutUnit(sideBySide(), {}, w, h, M);
                bool inside = true;
                for (const auto & entry : sideBySide()) {
                    const auto box = placementGeometry(entry.second, unit, M);
                    inside = inside && box.x >= 0 && box.y >= 0
                          && (box.x + box.side <= w || box.side == MIN_LAYOUT_VP_SIZE)
                          && (box.y + box.side <= h || box.side == MIN_LAYOUT_VP_SIZE);
                }
                char label[80];
                std::snprintf(label, sizeof label, "%dx%d stays in bounds", w, h);
                check(inside, label);
            }
        }
    }

    std::printf("3. the side-by-side layout has the shape it claims\n");
    {
        const auto unit = layoutUnit(sideBySide(), {}, 1600, 900, M);
        const auto main = placementGeometry(sideBySide()[XY], unit, M);
        const auto upper = placementGeometry(sideBySide()[XZ], unit, M);
        const auto lower = placementGeometry(sideBySide()[ZY], unit, M);
        check(main.x == M && main.y == M, "main plane sits at the origin");
        check(std::abs(main.side - (890 - M)) <= 1, "main plane is as tall as the window allows");
        check(upper.x > main.x + main.side - 2 * M, "side planes are to the right of it");
        check(std::abs(upper.side * 2 + M - main.side) <= 3, "two side planes plus the gutter equal the main plane");
        check(lower.y > upper.y + upper.side - 2 * M, "the two side planes are stacked");
        check(upper.x == lower.x, "side planes share a column");
    }

    std::printf("4. focus layouts fill the window\n");
    {
        const auto unit = layoutUnit(focus(), {}, 1600, 900, M);
        const auto box = placementGeometry(focus()[XY], unit, M);
        check(std::abs(box.side - (890 - M)) <= 1, "square fills the smaller dimension");
        const auto tall = layoutUnit(focus(), {}, 700, 1200, M);
        const auto tallBox = placementGeometry(focus()[XY], tall, M);
        check(std::abs(tallBox.side - (690 - M)) <= 1, "in a tall window width binds instead");
    }

    std::printf("5. capture and re-apply round-trips\n");
    {
        // pretend the user arranged three viewports by hand in a 1600x900 window
        const int refLength = 900 - 2 * M;
        struct { int vp, x, y, w; } arranged[] = {{XY, 5, 5, 500}, {XZ, 515, 5, 240}, {ZY, 515, 255, 240}};
        std::map<int, ViewportPlacement> captured;
        for (const auto & a : arranged) {
            captured[a.vp] = capturePlacement(a.x, a.y, a.w, refLength, M);
        }
        const auto canvas = captureCanvas(1600, 900, M);
        const auto unit = layoutUnit(captured, canvas, 1600, 900, M);
        bool same = true;
        for (const auto & a : arranged) {
            const auto box = placementGeometry(captured[a.vp], unit, M);
            same = same && std::abs(box.x - a.x) <= 2 && std::abs(box.y - a.y) <= 2 && std::abs(box.side - a.w) <= 2;
        }
        check(same, "same window size reproduces the arrangement");

        // and at double the size everything scales by two
        const auto bigUnit = layoutUnit(captured, canvas, 3200, 1800, M);
        bool scaled = true;
        for (const auto & a : arranged) {
            const auto box = placementGeometry(captured[a.vp], bigUnit, M);
            scaled = scaled && std::abs(box.side - (2 * a.w + M)) <= 6;
        }
        check(scaled, "double the window, double the viewports");

        // the arrangement above leaves the right third of the window empty; re-applying
        // must not stretch the viewports across it
        double rightmost = 0;
        for (const auto & entry : captured) {
            const auto box = placementGeometry(entry.second, unit, M);
            rightmost = std::max<double>(rightmost, box.x + box.side);
        }
        check(rightmost < 1600 * 0.55, "empty space the user left is preserved");
    }

    std::printf("6. a viewport never collapses below the minimum\n");
    {
        const auto unit = layoutUnit(sideBySide(), {}, 60, 60, M);
        for (const auto & entry : sideBySide()) {
            const auto box = placementGeometry(entry.second, unit, M);
            check(box.side >= MIN_LAYOUT_VP_SIZE, "tiny window still yields a usable viewport");
        }
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
