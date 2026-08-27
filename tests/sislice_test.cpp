// Exercises segmentation/sislice.h — the 2D key-slice mask behind shape interpolation.
// The interesting part is that growing the bounding box must shift the origin so that a
// global coordinate keeps mapping to the same content.
#include "segmentation/sislice.h"
#include <cstdio>
#include <vector>

static int failures = 0;
static void check(const bool ok, const char * what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) { ++failures; }
}

// paint a global coordinate into the slice
static void paint(SISlice & s, const int gu, const int gv, const std::uint8_t v = 1) {
    s.set(s.uIndexOf(gu), s.vIndexOf(gv), v);
}
static std::uint8_t read(const SISlice & s, const int gu, const int gv) {
    return s.at(s.uIndexOf(gu), s.vIndexOf(gv));
}

int main() {
    std::printf("1. coordinate round-trip at step 1\n");
    {
        SISlice s; s.uMin = 100; s.vMin = 200; s.uStep = 1; s.vStep = 1;
        check(s.uIndexOf(100) == 0 && s.vIndexOf(200) == 0, "origin maps to index 0");
        check(s.uIndexOf(107) == 7, "global 107 -> index 7");
        check(s.uCoordOf(7) == 107, "index 7 -> global 107");
        check(s.uIndexOf(93) == -7, "below the origin gives a negative index");
    }

    std::printf("2. coordinate round-trip at step 4 (mag 4)\n");
    {
        SISlice s; s.uMin = 400; s.vMin = 800; s.uStep = 4; s.vStep = 4;
        check(s.uIndexOf(400) == 0, "origin maps to index 0");
        check(s.uIndexOf(412) == 3, "global 412 -> index 3");
        check(s.uCoordOf(3) == 412, "index 3 -> global 412");
        check(s.uIndexOf(396) == -1, "one voxel below the origin is index -1");
        check(s.uIndexOf(393) == -2, "floor division rounds away from zero below the origin");
    }

    std::printf("3. growing upward preserves content\n");
    {
        SISlice s; s.uMin = 1000; s.vMin = 1000; s.uStep = 1; s.vStep = 1;
        paint(s, 1000, 1000);
        paint(s, 1005, 1003);
        paint(s, 1300, 1200);   // far outside, forces growth
        check(read(s, 1000, 1000) == 1, "original voxel survives the growth");
        check(read(s, 1005, 1003) == 1, "second voxel survives the growth");
        check(read(s, 1300, 1200) == 1, "the new far voxel is set");
        check(read(s, 1001, 1000) == 0, "an unpainted neighbour stays clear");
        check(s.count() == 3, "exactly three voxels are counted");
    }

    std::printf("4. growing downward shifts the origin and preserves content\n");
    {
        SISlice s; s.uMin = 1000; s.vMin = 1000; s.uStep = 1; s.vStep = 1;
        paint(s, 1000, 1000);
        paint(s, 1010, 1010);
        const auto originBefore = s.uMin;
        paint(s, 700, 640);     // well below the origin
        check(s.uMin < originBefore, "the origin moved down");
        check(read(s, 1000, 1000) == 1, "first voxel still reads back at its global coord");
        check(read(s, 1010, 1010) == 1, "second voxel still reads back at its global coord");
        check(read(s, 700, 640) == 1, "the new low voxel is set");
        check(s.count() == 3, "exactly three voxels are counted");
    }

    std::printf("5. growing downward at step 4 keeps the origin on the lattice\n");
    {
        SISlice s; s.uMin = 4000; s.vMin = 4000; s.uStep = 4; s.vStep = 4;
        paint(s, 4000, 4000);
        paint(s, 4040, 4040);
        paint(s, 3000, 3000);
        check(s.uMin % 4 == 0 && s.vMin % 4 == 0, "origin stays a multiple of the step");
        check(read(s, 4000, 4000) == 1, "first voxel survives");
        check(read(s, 4040, 4040) == 1, "second voxel survives");
        check(read(s, 3000, 3000) == 1, "the new low voxel is set");
        check(s.count() == 3, "exactly three voxels are counted");
    }

    std::printf("6. erasing\n");
    {
        SISlice s; s.uMin = 0; s.vMin = 0; s.uStep = 1; s.vStep = 1;
        paint(s, 10, 10); paint(s, 11, 10); paint(s, 12, 10);
        check(s.count() == 3, "three painted");
        paint(s, 11, 10, 0);
        check(s.count() == 2 && read(s, 11, 10) == 0, "erasing clears the voxel and the count");
        paint(s, 11, 10, 0);
        check(s.count() == 2, "erasing twice does not double-count");
        const auto sizeBefore = s.uSize * s.vSize;
        paint(s, -5000, -5000, 0);
        check(s.uSize * s.vSize == sizeBefore, "erasing far outside does not grow the mask");
        check(s.empty() == false, "not empty");
    }

    std::printf("7. shrinkToFit trims the box and keeps global coordinates\n");
    {
        SISlice s; s.uMin = 0; s.vMin = 0; s.uStep = 1; s.vStep = 1;
        paint(s, 500, 400);
        paint(s, 503, 402);
        const auto before = s.uSize * s.vSize;
        s.shrinkToFit();
        check(s.uSize * s.vSize < before, "the box got smaller");
        check(s.uSize == 4 && s.vSize == 3, "tight box is 4x3");
        check(read(s, 500, 400) == 1 && read(s, 503, 402) == 1, "both voxels still at their global coords");
        check(read(s, 501, 400) == 0, "gap stays clear");
        check(s.count() == 2, "count unchanged");
    }

    std::printf("8. adoptMask recounts\n");
    {
        SISlice s; s.uSize = 4; s.vSize = 2;
        s.adoptMask({0,1,1,0, 1,0,0,1});
        check(s.count() == 4, "counted 4 set pixels");
        s.adoptMask(std::vector<std::uint8_t>(8, 0));
        check(s.count() == 0 && s.empty(), "empty after adopting a blank mask");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
