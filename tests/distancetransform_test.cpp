// Exercises the real segmentation/distancetransform.h used by shape interpolation.
#include "segmentation/distancetransform.h"
#include <cstdio>
#include <cmath>

using namespace distance_transform;

static int failures = 0;
static void check(const bool ok, const char * what, const double got, const double want) {
    if (!ok) { std::printf("  FAIL %-52s got %8.3f want %8.3f\n", what, got, want); ++failures; }
    else     { std::printf("  ok   %-52s      %8.3f\n", what, got); }
}

static std::vector<std::uint8_t> disc(int w, int h, double cx, double cy, double r, double su = 1, double sv = 1) {
    std::vector<std::uint8_t> m(std::size_t(w) * h, 0);
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        const auto dx = (x - cx) * su, dy = (y - cy) * sv;
        m[std::size_t(y) * w + x] = (dx*dx + dy*dy <= r*r) ? 1 : 0;
    }
    return m;
}

// radius of a mask about its centroid, from its area
static double areaRadius(const std::vector<std::uint8_t> & m, double su = 1, double sv = 1) {
    std::size_t n = 0;
    for (auto c : m) n += (c != 0);
    return std::sqrt(n * su * sv / M_PI);
}
static void centroid(const std::vector<std::uint8_t> & m, int w, double & cx, double & cy) {
    double sx = 0, sy = 0; std::size_t n = 0;
    for (std::size_t i = 0; i < m.size(); ++i) if (m[i]) { sx += i % w; sy += i / w; ++n; }
    cx = n ? sx / n : 0; cy = n ? sy / n : 0;
}

int main() {
    const int W = 200, H = 200;

    std::printf("1. signed EDT of a single pixel (isotropic 1nm)\n");
    {
        std::vector<std::uint8_t> m(std::size_t(W) * H, 0);
        m[std::size_t(100) * W + 100] = 1;
        std::vector<float> d;
        signedEdt(m, d, W, H, 1.f, 1.f);
        check(std::fabs(d[std::size_t(100)*W + 100] + 1.f) < 1e-4, "lone pixel is -1 (1px to background)", d[std::size_t(100)*W+100], -1);
        check(std::fabs(d[std::size_t(100)*W + 103] - 3.f) < 1e-3, "3 px along u", d[std::size_t(100)*W+103], 3);
        check(std::fabs(d[std::size_t(104)*W + 100] - 4.f) < 1e-3, "4 px along v", d[std::size_t(104)*W+100], 4);
        check(std::fabs(d[std::size_t(103)*W + 104] - 5.f) < 1e-3, "3-4-5 diagonal", d[std::size_t(103)*W+104], 5);
    }

    std::printf("2. anisotropic spacing (u=2nm, v=10nm) is measured in nm\n");
    {
        std::vector<std::uint8_t> m(std::size_t(W) * H, 0);
        m[std::size_t(100) * W + 100] = 1;
        std::vector<float> d;
        signedEdt(m, d, W, H, 2.f, 10.f);
        check(std::fabs(d[std::size_t(100)*W + 103] - 6.f) < 1e-3, "3 px along u at 2nm", d[std::size_t(100)*W+103], 6);
        check(std::fabs(d[std::size_t(103)*W + 100] - 30.f) < 1e-3, "3 px along v at 10nm", d[std::size_t(103)*W+100], 30);
    }

    std::printf("3. sign convention: negative inside, positive outside\n");
    {
        const auto m = disc(W, H, 100, 100, 20);
        std::vector<float> d;
        signedEdt(m, d, W, H, 1.f, 1.f);
        check(d[std::size_t(100)*W + 100] < -19.f, "centre of r=20 disc is about -20", d[std::size_t(100)*W+100], -20);
        check(d[std::size_t(100)*W + 150] > 29.f,  "50px from centre is about +30", d[std::size_t(100)*W+150], 30);
        check(std::fabs(d[std::size_t(100)*W + 120]) <= 1.5f, "near zero on the boundary", d[std::size_t(100)*W+120], 0);
    }

    std::printf("4. interpolation between concentric discs r=10 and r=40\n");
    {
        std::vector<float> d1, d2;
        signedEdt(disc(W, H, 100, 100, 10), d1, W, H, 1.f, 1.f);
        signedEdt(disc(W, H, 100, 100, 40), d2, W, H, 1.f, 1.f);
        for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            std::vector<std::uint8_t> out(d1.size(), 0);
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] = ((1 - t) * d1[i] + t * d2[i]) <= 0 ? 1 : 0;
            const auto want = 10 + t * 30;
            char label[80]; std::snprintf(label, sizeof label, "t=%.2f radius", t);
            check(std::fabs(areaRadius(out) - want) < 1.0, label, areaRadius(out), want);
        }
    }

    std::printf("5. offset discs: plain blend vs centroid-aligned blend\n");
    {
        std::printf("  %-10s %-22s %s\n", "separation", "plain area", "aligned area  (want ~1257)");
        for (int sep : {0, 20, 30, 38, 42, 60, 100}) {
            const auto m1 = disc(W, H, 100 - sep/2.0, 100, 20);
            const auto m2 = disc(W, H, 100 + sep/2.0, 100, 20);
            std::vector<float> d1, d2;
            signedEdt(m1, d1, W, H, 1.f, 1.f);
            signedEdt(m2, d2, W, H, 1.f, 1.f);
            float c1u, c1v, c2u, c2v;
            maskCentroid(m1, W, c1u, c1v);
            maskCentroid(m2, W, c2u, c2v);
            std::vector<std::uint8_t> plain, aligned;
            blendSigned(d1, d2, W, H, 0.5f, 0.f, 0.f, plain);
            blendSigned(d1, d2, W, H, 0.5f, c2u - c1u, c2v - c1v, aligned);
            std::size_t np = 0, na = 0;
            for (auto c : plain) np += (c != 0);
            for (auto c : aligned) na += (c != 0);
            std::printf("  %-10d %-22zu %zu\n", sep, np, na);
            char label[80];
            std::snprintf(label, sizeof label, "sep=%d aligned area holds up", sep);
            check(na > 1100 && na < 1400, label, double(na), 1257);
            double cx, cy; centroid(aligned, W, cx, cy);
            std::snprintf(label, sizeof label, "sep=%d aligned centroid is the midpoint", sep);
            check(std::fabs(cx - 100) < 1.5, label, cx, 100);
        }
    }

    std::printf("6. key slices reproduce exactly at t=0 and t=1 (alignment on)\n");
    {
        const auto m1 = disc(W, H, 70, 100, 12);
        const auto m2 = disc(W, H, 130, 100, 28);
        std::vector<float> d1, d2;
        signedEdt(m1, d1, W, H, 1.f, 1.f);
        signedEdt(m2, d2, W, H, 1.f, 1.f);
        float c1u, c1v, c2u, c2v;
        maskCentroid(m1, W, c1u, c1v);
        maskCentroid(m2, W, c2u, c2v);
        std::vector<std::uint8_t> at0, at1;
        blendSigned(d1, d2, W, H, 0.f, c2u - c1u, c2v - c1v, at0);
        blendSigned(d1, d2, W, H, 1.f, c2u - c1u, c2v - c1v, at1);
        std::size_t diff0 = 0, diff1 = 0;
        for (std::size_t i = 0; i < m1.size(); ++i) { diff0 += (at0[i] != m1[i]); diff1 += (at1[i] != m2[i]); }
        check(diff0 == 0, "t=0 reproduces slice 1 exactly", double(diff0), 0);
        check(diff1 == 0, "t=1 reproduces slice 2 exactly", double(diff1), 0);
    }

    std::printf("7. shrinking to a point still behaves\n");
    {
        std::vector<float> d1, d2;
        const auto big = disc(W, H, 100, 100, 30);
        std::vector<std::uint8_t> tiny(std::size_t(W) * H, 0);
        tiny[std::size_t(100) * W + 100] = 1;
        signedEdt(big, d1, W, H, 1.f, 1.f);
        signedEdt(tiny, d2, W, H, 1.f, 1.f);
        float c1u, c1v, c2u, c2v;
        maskCentroid(big, W, c1u, c1v);
        maskCentroid(tiny, W, c2u, c2v);
        std::vector<std::uint8_t> out;
        blendSigned(d1, d2, W, H, 0.5f, c2u - c1u, c2v - c1v, out);
        check(areaRadius(out) > 13.0 && areaRadius(out) < 17.0, "halfway to a point is about r=15", areaRadius(out), 15);
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
