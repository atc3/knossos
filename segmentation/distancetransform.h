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

/* Exact 2D Euclidean distance transform, used by shape interpolation.
 *
 * Deliberately free of Qt and of any KNOSSOS type, so the numerics can be exercised on
 * their own. Sample spacing is passed in nanometres, which is how anisotropic voxels are
 * handled — the transform measures physical distance, not pixel counts. */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace distance_transform {

constexpr float SI_INF = 1e20f;

/* Felzenszwalb & Huttenlocher's O(n) squared distance transform of a sampled function.
 * `sq` is the squared sample spacing, which is how anisotropic voxels are handled: the
 * transform is computed in nanometres rather than in pixels. */
inline void dt1d(const float * f, float * d, int * v, float * z, const int n, const float sq) {
    int k = 0;
    v[0] = 0;
    z[0] = -SI_INF;
    z[1] = SI_INF;
    for (int q = 1; q < n; ++q) {
        auto s = ((f[q] + sq * q * q) - (f[v[k]] + sq * v[k] * v[k])) / (2 * sq * q - 2 * sq * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + sq * q * q) - (f[v[k]] + sq * v[k] * v[k])) / (2 * sq * q - 2 * sq * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = SI_INF;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) {
            ++k;
        }
        d[q] = sq * (q - v[k]) * (q - v[k]) + f[v[k]];
    }
}

inline void edt2d(std::vector<float> & f, const int w, const int h, const float su, const float sv) {
    const auto maxDim = std::max(w, h);
    std::vector<float> in(maxDim), out(maxDim), z(maxDim + 1);
    std::vector<int> v(maxDim);
    for (int y = 0; y < h; ++y) {
        auto * row = f.data() + static_cast<std::size_t>(y) * w;
        std::copy(row, row + w, in.data());
        dt1d(in.data(), out.data(), v.data(), z.data(), w, su * su);
        std::copy(out.data(), out.data() + w, row);
    }
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            in[y] = f[static_cast<std::size_t>(y) * w + x];
        }
        dt1d(in.data(), out.data(), v.data(), z.data(), h, sv * sv);
        for (int y = 0; y < h; ++y) {
            f[static_cast<std::size_t>(y) * w + x] = out[y];
        }
    }
}

/* Signed distance: negative inside the shape, positive outside. Matches Paintera's
 * sqrt(distance to foreground) − sqrt(distance to background). */
inline void signedEdt(const std::vector<std::uint8_t> & bin, std::vector<float> & result, const int w, const int h, const float su, const float sv) {
    std::vector<float> outside(bin.size()), inside(bin.size());
    for (std::size_t i = 0; i < bin.size(); ++i) {
        outside[i] = bin[i] != 0 ? 0.f : SI_INF;// distance to the nearest foreground pixel
        inside[i] = bin[i] != 0 ? SI_INF : 0.f; // distance to the nearest background pixel
    }
    edt2d(outside, w, h, su, sv);
    edt2d(inside, w, h, su, sv);
    result.resize(bin.size());
    for (std::size_t i = 0; i < bin.size(); ++i) {
        result[i] = std::sqrt(outside[i]) - std::sqrt(inside[i]);
    }
}


/* Mix two signed distance fields into a binary mask at parameter t ∈ [0,1].
 *
 * (du, dv) is the centroid offset of field 2 relative to field 1, in pixels; pass (0,0)
 * for a plain blend. Shifting both fields onto the interpolated centroid before mixing
 * is what keeps a laterally drifting shape from pinching — and, once the two shapes stop
 * overlapping at all, from vanishing entirely. The shifts vanish at t=0 and t=1, so the
 * key slices always reproduce exactly. */
inline void blendSigned(const std::vector<float> & d1, const std::vector<float> & d2, const int w, const int h,
                        const float t, const float du, const float dv, std::vector<std::uint8_t> & out) {
    out.assign(static_cast<std::size_t>(w) * h, 0);
    const auto shift1u = static_cast<int>(std::lround(-t * du));
    const auto shift1v = static_cast<int>(std::lround(-t * dv));
    const auto shift2u = static_cast<int>(std::lround((1.f - t) * du));
    const auto shift2v = static_cast<int>(std::lround((1.f - t) * dv));

    // sampling outside the grid means "far outside the shape", which is what the padding
    // around the union bounding box already represents
    constexpr float FAR = 1e20f;
    const auto sample = [&](const std::vector<float> & d, const int u, const int v){
        return (u < 0 || v < 0 || u >= w || v >= h) ? FAR : d[static_cast<std::size_t>(v) * w + u];
    };
    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            const auto a = sample(d1, u + shift1u, v + shift1v);
            const auto b = sample(d2, u + shift2u, v + shift2v);
            out[static_cast<std::size_t>(v) * w + u] = ((1.f - t) * a + t * b) <= 0.f ? 1 : 0;
        }
    }
}

/* Centroid of a binary mask, in pixel coordinates. Returns false for an empty mask. */
inline bool maskCentroid(const std::vector<std::uint8_t> & bin, const int w, float & cu, float & cv) {
    double su{0}, sv{0};
    std::size_t n{0};
    for (std::size_t i = 0; i < bin.size(); ++i) {
        if (bin[i] != 0) {
            su += static_cast<double>(i % w);
            sv += static_cast<double>(i / w);
            ++n;
        }
    }
    cu = n != 0 ? static_cast<float>(su / n) : 0.f;
    cv = n != 0 ? static_cast<float>(sv / n) : 0.f;
    return n != 0;
}
}
