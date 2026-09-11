// Standalone test for segmentation/holefill.h — the connectivity behind "close an outline
// and the middle fills in". Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/holefill_test holefill_test.cpp && /tmp/holefill_test
#include "segmentation/holefill.h"

#include <cstdio>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

// Builds a grid from rows of text: '#' is the id being drawn, anything else is not.
struct Grid {
    int w{0}, h{0};
    std::vector<std::uint8_t> solid;
};
Grid parse(const std::vector<std::string> & rows) {
    Grid g;
    g.h = static_cast<int>(rows.size());
    g.w = g.h == 0 ? 0 : static_cast<int>(rows.front().size());
    g.solid.assign(static_cast<std::size_t>(g.w) * g.h, 0);
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            g.solid[static_cast<std::size_t>(y) * g.w + x] = rows[y][x] == '#' ? 1 : 0;
        }
    }
    return g;
}
std::size_t holesIn(const std::vector<std::string> & rows, std::vector<std::uint8_t> * out = nullptr) {
    const auto g = parse(rows);
    std::vector<std::uint8_t> filled;
    const auto n = holefill::enclosed(g.solid, g.w, g.h, filled);
    if (out != nullptr) {
        *out = filled;
    }
    return n;
}
}

int main() {
    std::printf("1. an open outline encloses nothing\n");
    {
        // a C: the right side is missing, so the middle reaches the outside
        check(holesIn({"......",
                       ".####.",
                       ".#....",
                       ".#....",
                       ".####.",
                       "......"}) == 0, "a C fills nothing");
        check(holesIn({"......",
                       "......",
                       "..##..",
                       "..##..",
                       "......",
                       "......"}) == 0, "a solid blob has no interior to fill");
        check(holesIn({"......",
                       "......",
                       "......",
                       "......",
                       "......",
                       "......"}) == 0, "an empty plane fills nothing");
    }

    std::printf("2. a closed ring fills its middle, and only its middle\n");
    {
        std::vector<std::uint8_t> filled;
        const auto n = holesIn({"......",
                                ".####.",
                                ".#..#.",
                                ".#..#.",
                                ".####.",
                                "......"}, &filled);
        check(n == 4, "the four interior cells are filled");
        const auto at = [&filled](const int x, const int y){ return filled[static_cast<std::size_t>(y) * 6 + x] != 0; };
        check(at(2, 2) && at(3, 2) && at(2, 3) && at(3, 3), "the interior cells are the right ones");
        check(!at(0, 0) && !at(5, 5), "the outside is untouched");
        check(!at(1, 1), "the wall itself is not reported as a hole");
    }

    std::printf("3. several holes at once, which is the point of doing it by connectivity\n");
    {
        check(holesIn({"...........",
                       ".####.####.",
                       ".#..#.#..#.",
                       ".####.####.",
                       "..........."}) == 4, "two separate rings, two cells each");
        check(holesIn({"...........",
                       ".#########.",
                       ".#..#..#..#",
                       ".#########.",
                       "..........."}) > 0, "adjoining compartments each count");
    }

    std::printf("4. a diagonal wall seals — walls 8-connected, space 4-connected\n");
    {
        /* A round brush dragged diagonally leaves voxels touching only at the corners. The
         * eye reads that as closed, so the escape walk must not slip between them. */
        const auto n = holesIn({".....#...",
                                "....#.#..",
                                "...#...#.",
                                "..#.....#",
                                "...#...#.",
                                "....#.#..",
                                ".....#..."});
        check(n > 0, "a diamond drawn with corner-touching voxels still encloses");
    }

    std::printf("5. the region border is the outside\n");
    {
        // the same ring, but flush against the edge: its wall is cut by the border, so the
        // middle escapes. This is why the caller grows the region until nothing touches.
        check(holesIn({"####",
                       "#..#",
                       "#..#",
                       "####"}) == 4, "a ring that fits exactly still encloses");
        check(holesIn({"#..#",
                       "#..#",
                       "####",
                       "...."}) == 0, "a shape open at the border encloses nothing");
    }

    std::printf("6. touchesBorder reports the sides that need growing\n");
    {
        auto g = parse({".....",
                        ".###.",
                        ".#.#.",
                        ".###.",
                        "....."});
        auto contact = holefill::touchesBorder(g.solid, g.w, g.h);
        check(!contact.any(), "a shape clear of the edge touches nothing");

        g = parse({"..#..",
                   ".###.",
                   ".#.#.",
                   ".###.",
                   "....."});
        contact = holefill::touchesBorder(g.solid, g.w, g.h);
        check(contact.top && !contact.bottom && !contact.left && !contact.right, "only the top is reported");

        g = parse({".....",
                   "####.",
                   "#.##.",
                   "####.",
                   "....."});
        contact = holefill::touchesBorder(g.solid, g.w, g.h);
        check(contact.left && !contact.right, "only the left is reported");
    }

    std::printf("7. degenerate input is refused rather than read out of bounds\n");
    {
        std::vector<std::uint8_t> out;
        check(holefill::enclosed({}, 0, 0, out) == 0, "an empty grid");
        check(holefill::enclosed({1, 0, 0, 1}, 3, 3, out) == 0, "a size that disagrees with the data");
        check(holefill::enclosed({1}, 1, 1, out) == 0, "a single solid cell");
        check(holefill::enclosed({0}, 1, 1, out) == 0, "a single open cell is the border, so not enclosed");
    }

    std::printf("8. a big region stays linear and does not blow the stack\n");
    {
        // an explicit stack, not recursion: a 1000x1000 open region is a million cells and
        // recursing over it would overflow long before it finished
        const int n = 1000;
        std::vector<std::uint8_t> solid(static_cast<std::size_t>(n) * n, 0);
        for (int i = 200; i < 800; ++i) {// a hollow square well inside the region
            solid[static_cast<std::size_t>(200) * n + i] = 1;
            solid[static_cast<std::size_t>(799) * n + i] = 1;
            solid[static_cast<std::size_t>(i) * n + 200] = 1;
            solid[static_cast<std::size_t>(i) * n + 799] = 1;
        }
        std::vector<std::uint8_t> out;
        const auto count = holefill::enclosed(solid, n, n, out);
        check(count == 598u * 598u, "the whole interior of a 600-wide square");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
