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

/* Dilate, erode and smooth one object in 3D.
 *
 * Qt-free and free of any KNOSSOS type, so the numerics can be exercised on their own the
 * way sislice.h, holefill.h and distancetransform.h already are. The driver in
 * objectmorphology.cpp does the reading, writing and block loading; everything that decides
 * what the result *is* lives here.
 *
 * Distances are physical. The sample spacing arrives in nanometres per axis, and the kernel
 * radius is a length rather than a voxel count — which is the only way one number means the
 * same thing along z as along x on EM data that is anisotropic by a factor of four. A
 * radius of 20 nm on 10×10×40 nm voxels reaches two voxels sideways and none at all in z,
 * and that is the correct answer, not a rounding problem to paper over.
 *
 * The algorithm is Paintera's (saalfeldlab/paintera, control/actions/paint/morph), which is
 * Kotlin over imglib2 — so this is a port, not a copy. What is carried over is every part
 * that decides the outcome:
 *
 *   - dilation adds the voxels whose squared physical distance to the object is <= r², from
 *     a Voronoi distance transform that also reports *which* label was nearest, so each
 *     fragment of a merged object grows its own id instead of the whole group collapsing
 *     onto one;
 *   - erosion is the same transform seeded on everything that is not the object;
 *   - smoothing blurs the object's binary mask with a Gaussian of sigma = r / spacing per
 *     axis and thresholds it, and the two transforms above then say what each voxel that
 *     crossed the threshold should become.
 *
 * One deliberate divergence: Paintera defaults the infill for voxels an erode gives up to
 * NearestLabel, which hands them to whatever label is nearest. That suits a dense
 * segmentation where every voxel belongs to something. KNOSSOS annotations are usually
 * sparse — objects floating in unlabelled space — so quietly growing a *neighbouring*
 * object to take up the slack would be a surprise, and the default here is the background
 * id. NearestLabel is still offered, and behaves identically to Background when the object
 * is surrounded by background anyway.
 */

#include "segmentation/distancetransform.h"// dt1d's constants and its structure

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace morphology {

enum class Operation { Dilate, Erode, Smooth };
/* Which way a smooth is allowed to move the boundary. Paintera's In/Out/Both.
 *
 * Dilate and erode imply their own direction; this only has teeth for Smooth, where
 * restricting it to one side turns "round off this object" into "fill in its dents" or
 * "shave off its spurs". */
enum class Direction { Shrink, Expand, Both };
// What a voxel the object gives up turns into.
enum class Infill { Background, Replace, NearestLabel };

/* Labels are carried as indices into a palette the caller owns, not as raw 64-bit ids.
 *
 * A region holds a handful of distinct ids — background plus whatever objects pass through
 * — so an index costs 4 bytes a voxel where the id costs 8, and the whole working set of a
 * morphology run is several arrays this size. It also means `selected` is one byte per
 * *label* rather than a set lookup per voxel. */
struct Block {
    std::vector<std::uint32_t> labels;// palette index per voxel, x fastest, then y, then z
    std::vector<char> selected;       // one entry per palette index: is it part of the object
    int dim[3]{0, 0, 0};

    std::size_t voxels() const {
        return static_cast<std::size_t>(dim[0]) * dim[1] * dim[2];
    }
    bool selectedAt(const std::size_t i) const {
        const auto label = labels[i];
        return label < selected.size() && selected[label] != 0;
    }
};

struct Params {
    Operation op{Operation::Dilate};
    double radius{0};            // in the same units as `spacing`; Paintera's "kernel size"
    double spacing[3]{1, 1, 1};  // nanometres per voxel along x, y, z
    Direction direction{Direction::Both};
    Infill infill{Infill::Background};
    std::uint32_t replacement{0};// palette index written for Infill::Replace
    std::uint32_t background{0}; // palette index of the background id
    double threshold{0.5};       // Smooth only: where the blurred mask counts as inside
};

/* Felzenszwalb & Huttenlocher's 1-D squared distance transform, carrying the argument of
 * the minimum alongside the distance.
 *
 * distance_transform::dt1d() throws that away, and the nearest label is the whole reason
 * this exists: it is what tells a newly dilated voxel which fragment of a merged object it
 * joined, and an eroded one which neighbour it fell back to. `fArg` comes in holding each
 * sample's current answer and `dArg` goes out holding the answer of whichever sample won,
 * which is what makes the three axis passes compose into a real 3-D Voronoi diagram.
 *
 * SI_INF is 1e20f rather than a true infinity for the reason it is in dt1d: the parabola
 * intersection subtracts two f values, and inf - inf is NaN, which would walk `k` off the
 * bottom of the stack. 1e20 - 1e20 is 0 and the first comparison against z[0] = -1e20
 * fails, so an entirely unseeded line is handled by the ordinary path. */
inline void dt1dArg(const float * f, const std::int32_t * fArg, float * d, std::int32_t * dArg,
                    int * v, float * z, const int n, const float sq) {
    int k = 0;
    v[0] = 0;
    z[0] = -distance_transform::SI_INF;
    z[1] = distance_transform::SI_INF;
    for (int q = 1; q < n; ++q) {
        auto s = ((f[q] + sq * q * q) - (f[v[k]] + sq * v[k] * v[k])) / (2 * sq * q - 2 * sq * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + sq * q * q) - (f[v[k]] + sq * v[k] * v[k])) / (2 * sq * q - 2 * sq * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = distance_transform::SI_INF;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) {
            ++k;
        }
        d[q] = sq * (q - v[k]) * (q - v[k]) + f[v[k]];
        dArg[q] = fArg[v[k]];
    }
}

/* Squared physical distance from every voxel to the nearest seed, and the flat index of
 * that seed. Exact, O(voxels), and separable — one pass per axis, each weighted by that
 * axis's spacing, which is where the anisotropy enters.
 *
 * A voxel with no seed anywhere in the block comes back at roughly SI_INF with an argument
 * of -1, so callers must not dereference the argument without checking the distance first. */
template<typename SeedPredicate>
inline void voronoiSq3d(const int dim[3], const double spacing[3], const SeedPredicate & isSeed,
                        std::vector<float> & dist, std::vector<std::int32_t> & arg) {
    const auto w = dim[0], h = dim[1], d = dim[2];
    const auto n = static_cast<std::size_t>(w) * h * d;
    dist.assign(n, 0.f);
    arg.assign(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const bool seed = isSeed(i);
        dist[i] = seed ? 0.f : distance_transform::SI_INF;
        arg[i] = seed ? static_cast<std::int32_t>(i) : -1;
    }
    if (n == 0) {
        return;
    }
    const auto maxDim = std::max(std::max(w, h), d);
    std::vector<float> in(maxDim), out(maxDim), z(maxDim + 1);
    std::vector<std::int32_t> inArg(maxDim), outArg(maxDim);
    std::vector<int> v(maxDim);

    const auto sq = [&spacing](const int axis){ return static_cast<float>(spacing[axis] * spacing[axis]); };

    for (int zz = 0; zz < d; ++zz) {
        for (int yy = 0; yy < h; ++yy) {
            const auto base = (static_cast<std::size_t>(zz) * h + yy) * w;
            for (int xx = 0; xx < w; ++xx) { in[xx] = dist[base + xx]; inArg[xx] = arg[base + xx]; }
            dt1dArg(in.data(), inArg.data(), out.data(), outArg.data(), v.data(), z.data(), w, sq(0));
            for (int xx = 0; xx < w; ++xx) { dist[base + xx] = out[xx]; arg[base + xx] = outArg[xx]; }
        }
    }
    for (int zz = 0; zz < d; ++zz) {
        for (int xx = 0; xx < w; ++xx) {
            for (int yy = 0; yy < h; ++yy) {
                const auto i = (static_cast<std::size_t>(zz) * h + yy) * w + xx;
                in[yy] = dist[i]; inArg[yy] = arg[i];
            }
            dt1dArg(in.data(), inArg.data(), out.data(), outArg.data(), v.data(), z.data(), h, sq(1));
            for (int yy = 0; yy < h; ++yy) {
                const auto i = (static_cast<std::size_t>(zz) * h + yy) * w + xx;
                dist[i] = out[yy]; arg[i] = outArg[yy];
            }
        }
    }
    for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
            for (int zz = 0; zz < d; ++zz) {
                const auto i = (static_cast<std::size_t>(zz) * h + yy) * w + xx;
                in[zz] = dist[i]; inArg[zz] = arg[i];
            }
            dt1dArg(in.data(), inArg.data(), out.data(), outArg.data(), v.data(), z.data(), d, sq(2));
            for (int zz = 0; zz < d; ++zz) {
                const auto i = (static_cast<std::size_t>(zz) * h + yy) * w + xx;
                dist[i] = out[zz]; arg[i] = outArg[zz];
            }
        }
    }
}

// Half-width of the sampled Gaussian. Three sigma carries 99.7% of the mass; past that the
// weights are below the rounding error of the 0/1 mask being blurred.
inline int gaussianRadius(const double sigma) {
    return sigma <= 0 ? 0 : std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
}

inline std::vector<float> gaussianKernel(const double sigma) {
    const auto r = gaussianRadius(sigma);
    std::vector<float> k(2 * r + 1, 0.f);
    if (r == 0) {
        k[0] = 1.f;
        return k;
    }
    double sum = 0;
    for (int i = -r; i <= r; ++i) {
        const auto w = std::exp(-(i * i) / (2.0 * sigma * sigma));
        k[i + r] = static_cast<float>(w);
        sum += w;
    }
    for (auto & w : k) {
        w = static_cast<float>(w / sum);
    }
    return k;
}

/* Separable Gaussian blur, zero-extended at the block border.
 *
 * Zero extension matches Paintera's Views.extendValue(mask, 0.0) and says "there is no
 * object out there", which is wrong at a border the object actually crosses — it would pull
 * the blurred mask down and shave the object off along that face. The driver's answer is
 * the same as Paintera's PaddedCell: read a margin of real data around the region that will
 * be written and only write the inside, so the border never falls where anything is
 * committed. When the margin could not be read in full, that is reported rather than
 * silently accepted. */
inline void gaussian3d(const int dim[3], const double sigma[3], std::vector<float> & img) {
    const auto w = dim[0], h = dim[1], d = dim[2];
    if (img.empty()) {
        return;
    }
    const auto maxDim = std::max(std::max(w, h), d);
    std::vector<float> line(maxDim), blurred(maxDim);

    const auto pass = [&](const int axis, const int n, auto && index){
        if (n <= 1) {
            return;// nothing to convolve along; a single plane stays as it is
        }
        const auto kernel = gaussianKernel(sigma[axis]);
        const auto r = static_cast<int>(kernel.size() / 2);
        if (r == 0) {
            return;
        }
        const auto outerA = (axis == 0) ? d : (axis == 1) ? d : h;
        const auto outerB = (axis == 0) ? h : (axis == 1) ? w : w;
        for (int a = 0; a < outerA; ++a) {
            for (int b = 0; b < outerB; ++b) {
                for (int t = 0; t < n; ++t) { line[t] = img[index(a, b, t)]; }
                for (int t = 0; t < n; ++t) {
                    float acc = 0.f;
                    for (int k = -r; k <= r; ++k) {
                        const auto s = t + k;
                        if (s >= 0 && s < n) {// outside the block the mask is taken as 0
                            acc += kernel[k + r] * line[s];
                        }
                    }
                    blurred[t] = acc;
                }
                for (int t = 0; t < n; ++t) { img[index(a, b, t)] = blurred[t]; }
            }
        }
    };

    pass(0, w, [&](const int zz, const int yy, const int xx){ return (static_cast<std::size_t>(zz) * h + yy) * w + xx; });
    pass(1, h, [&](const int zz, const int xx, const int yy){ return (static_cast<std::size_t>(zz) * h + yy) * w + xx; });
    pass(2, d, [&](const int yy, const int xx, const int zz){ return (static_cast<std::size_t>(zz) * h + yy) * w + xx; });
}

struct Result {
    std::vector<std::uint32_t> labels;// palette index per voxel after the operation
    std::size_t added{0};             // voxels that joined the object
    std::size_t removed{0};           // voxels the object gave up
};

/* Run one operation over a block. `out.labels` comes back the same size as the input, equal
 * to it wherever nothing changed, so the caller can diff the two and write only the
 * difference — which is what keeps a 2-voxel dilate from marking every cube in the region
 * as modified. */
inline Result apply(const Block & block, const Params & p) {
    Result result;
    result.labels = block.labels;
    const auto n = block.voxels();
    if (n == 0 || block.labels.size() != n) {
        return result;
    }

    const auto radiusSq = static_cast<float>(p.radius * p.radius);
    const auto grows = p.op == Operation::Dilate
            || (p.op == Operation::Smooth && (p.direction == Direction::Expand || p.direction == Direction::Both));
    const auto shrinks = p.op == Operation::Erode
            || (p.op == Operation::Smooth && (p.direction == Direction::Shrink || p.direction == Direction::Both));

    /* For a smooth, whether a voxel ends up inside is decided by the blurred mask, not by a
     * distance — that is the entire difference between rounding a shape off and growing it.
     * Paintera's sigma is the radius divided by the spacing, i.e. the same physical length
     * expressed in voxels along each axis. */
    std::vector<float> blurred;
    if (p.op == Operation::Smooth) {
        blurred.assign(n, 0.f);
        for (std::size_t i = 0; i < n; ++i) {
            blurred[i] = block.selectedAt(i) ? 1.f : 0.f;
        }
        const double sigma[3]{p.radius / p.spacing[0], p.radius / p.spacing[1], p.radius / p.spacing[2]};
        gaussian3d(block.dim, sigma, blurred);
    }
    const auto insideAfter = [&](const std::size_t i){
        return p.op == Operation::Smooth ? blurred[i] >= p.threshold : false;
    };

    std::vector<float> dist;
    std::vector<std::int32_t> arg;

    if (grows) {
        // seeded on the object: distance out from it, and which fragment is nearest
        voronoiSq3d(block.dim, p.spacing, [&block](const std::size_t i){ return block.selectedAt(i); }, dist, arg);
        for (std::size_t i = 0; i < n; ++i) {
            if (block.selectedAt(i)) {
                continue;
            }
            /* The distance cap applies to a smooth too, and not only as a safety net: it is
             * what Paintera's dilated image does by falling back to the original label
             * beyond r. A voxel further than the radius from the object cannot sensibly be
             * said to have joined it however the blurred mask came out, and inside a
             * concavity the blur does creep slightly past the boundary. */
            const bool takeIt = dist[i] <= radiusSq && (p.op != Operation::Smooth || insideAfter(i));
            if (!takeIt || arg[i] < 0) {
                continue;// no object anywhere in the block to join
            }
            result.labels[i] = block.labels[static_cast<std::size_t>(arg[i])];
            ++result.added;
        }
    }

    if (shrinks) {
        // seeded on everything that is not the object: how deep each voxel sits, and which
        // neighbour it would fall back to
        voronoiSq3d(block.dim, p.spacing, [&block](const std::size_t i){ return !block.selectedAt(i); }, dist, arg);
        for (std::size_t i = 0; i < n; ++i) {
            if (!block.selectedAt(i)) {
                continue;
            }
            const bool dropIt = dist[i] <= radiusSq && (p.op != Operation::Smooth || !insideAfter(i));
            if (!dropIt) {
                continue;
            }
            const auto fallback = [&]() -> std::uint32_t {
                switch (p.infill) {
                case Infill::Replace: return p.replacement;
                case Infill::NearestLabel: return arg[i] >= 0 ? block.labels[static_cast<std::size_t>(arg[i])] : p.background;
                case Infill::Background: break;
                }
                return p.background;
            }();
            if (fallback == result.labels[i]) {
                continue;// the nearest neighbour is this very label; nothing to do
            }
            result.labels[i] = fallback;
            ++result.removed;
        }
    }
    return result;
}

}// namespace morphology
