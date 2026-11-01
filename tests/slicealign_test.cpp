// Standalone test for segmentation/slicealign.h — see tests/README.md.
#include "segmentation/slicealign.h"

#include <chrono>
#include <cstdio>
#include <functional>
#include <random>

static int failures = 0;
static void check(const bool ok, const char * what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what);
    failures += ok ? 0 : 1;
}

using Mask = std::vector<std::uint8_t>;

static Mask paint(const int w, const int h, const std::function<bool(int, int)> & inside) {
    Mask m(static_cast<std::size_t>(w) * h, 0);
    for (int v = 0; v < h; ++v) for (int u = 0; u < w; ++u) m[static_cast<std::size_t>(v) * w + u] = inside(u, v) ? 1 : 0;
    return m;
}

// an irregular body: a lumpy ellipse with a notch, nothing symmetric about it
static bool body(const int u, const int v, const int cu, const int cv) {
    const double x = u - cu, y = v - cv;
    const double r = 30 + 8 * std::sin(3 * std::atan2(y, x)) + 4 * std::cos(5 * std::atan2(y, x));
    const bool notch = x > 5 && std::abs(y) < 6;
    return (x * x) / 1.6 + y * y < r * r && !notch;
}

int main() {
    std::printf("1. a pure translation of an irregular shape is recovered exactly\n");
    {
        const int w = 200, h = 160;
        const auto a = paint(w, h, [](int u, int v){ return body(u, v, 80, 80); });
        const auto b = paint(w, h, [](int u, int v){ return body(u, v, 97, 71); });
        const auto s = slicealign::bestTranslation(a, b, w, h);
        std::printf("     got (%d, %d), level %d\n", s.du, s.dv, s.levelReached);
        check(s.found && s.du == 17 && s.dv == -9 && s.levelReached == 0, "(17, -9)");
    }

    std::printf("2. a branch growing out of one side does not drag the alignment with it\n");
    {
        const int w = 260, h = 200;
        const auto a = paint(w, h, [](int u, int v){ return body(u, v, 90, 100); });
        // the body moved (5, 3), and slice 2 also has a long arm reaching off to the right
        const auto b = paint(w, h, [](int u, int v){
            return body(u, v, 95, 103) || (u > 120 && u < 240 && std::abs(v - 110) < 9);
        });
        double c1u = 0, c1v = 0, c2u = 0, c2v = 0;
        slicealign::detail::centroid(a, w, c1u, c1v);
        slicealign::detail::centroid(b, w, c2u, c2v);
        const auto s = slicealign::bestTranslation(a, b, w, h);
        std::printf("     centroids say (%.1f, %.1f); overlap says (%d, %d)\n", c2u - c1u, c2v - c1v, s.du, s.dv);
        check(std::abs(s.du - 5) <= 1 && std::abs(s.dv - 3) <= 1, "follows the body, within a pixel of (5, 3)");
        check(c2u - c1u > 15, "(and the centroid really is pulled well off by the arm)");
    }

    std::printf("3. a shape that moved further than its own width is still found\n");
    {
        const int w = 300, h = 300;
        const auto a = paint(w, h, [](int u, int v){ return (u - 40) * (u - 40) + (v - 50) * (v - 50) < 15 * 15; });
        const auto b = paint(w, h, [](int u, int v){ return (u - 230) * (u - 230) + (v - 210) * (v - 210) < 15 * 15; });
        const auto s = slicealign::bestTranslation(a, b, w, h);
        std::printf("     got (%d, %d)\n", s.du, s.dv);
        check(s.du == 190 && s.dv == 160, "(190, 160)");
    }

    std::printf("4. symmetric shapes fall back to the centroid offset rather than an arbitrary tie\n");
    {
        const int w = 120, h = 120;
        // a ring inside a ring: overlap is the same for many small shifts
        const auto a = paint(w, h, [](int u, int v){ return (u - 60) * (u - 60) + (v - 60) * (v - 60) < 40 * 40; });
        const auto b = paint(w, h, [](int u, int v){ return (u - 62) * (u - 62) + (v - 59) * (v - 59) < 10 * 10; });
        const auto s = slicealign::bestTranslation(a, b, w, h);
        std::printf("     got (%d, %d)\n", s.du, s.dv);
        check(s.du == 2 && s.dv == -1, "a small disk anywhere inside a large one: the centroid offset, (2, -1)");
    }

    std::printf("5. degenerate input\n");
    {
        const Mask empty(100, 0), one(100, 1);
        check(!slicealign::bestTranslation(empty, one, 10, 10).found, "an empty mask finds nothing");
        const auto s = slicealign::bestTranslation(one, one, 10, 10);
        check(s.found && s.du == 0 && s.dv == 0, "identical masks: no shift");
    }

    std::printf("6. fast enough to sit under the preview delay\n");
    for (const int size : {512, 1500, 4096}) {
        const auto a = paint(size, size, [size](int u, int v){ return body(u * 120 / size, v * 120 / size, 50, 60); });
        const auto b = paint(size, size, [size](int u, int v){ return body(u * 120 / size, v * 120 / size, 63, 52); });
        const auto t0 = std::chrono::steady_clock::now();
        const auto s = slicealign::bestTranslation(a, b, size, size);
        const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const auto expectU = 13 * size / 120.0, expectV = -8 * size / 120.0;
        std::printf("     %d × %d: %.0f ms, (%d, %d) vs about (%.0f, %.0f), level %d\n", size, size, ms, s.du, s.dv, expectU, expectV, s.levelReached);
        check(ms < 1000, "under a second");
        check(std::abs(s.du - expectU) <= 2 + (1 << s.levelReached) && std::abs(s.dv - expectV) <= 2 + (1 << s.levelReached), "and right");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
