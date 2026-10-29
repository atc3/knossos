// Standalone test for segmentation/inventoryroute.h — see tests/README.md.
#include "segmentation/inventoryroute.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <tuple>

using namespace invroute;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("FAIL %s:%d: ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static Window windowAt(const Int3 & lo, const int M) {
    return {lo, {lo[0] + M - 1, lo[1] + M - 1, lo[2] + M - 1}};
}

static std::int64_t totalCost(const std::vector<Window> & w, const std::vector<std::uint32_t> & order, Window from) {
    std::int64_t total = 0;
    for (const auto i : order) {
        total += newBlocks(from, w[i]);
        from = w[i];
    }
    return total;
}

// the same greedy rule, by brute force over every remaining window
static std::vector<std::uint32_t> reference(const std::vector<Window> & w, Window current) {
    std::map<Int3, std::vector<std::uint32_t>> buckets;
    for (std::uint32_t i = 0; i < w.size(); ++i) {
        buckets[w[i].lo].push_back(i);
    }
    std::vector<std::uint32_t> order;
    while (!buckets.empty()) {
        bool anyOverlap = false;
        for (const auto & b : buckets) {
            anyOverlap |= overlap(current, w[b.second.front()]) > 0;
        }
        const std::map<Int3, std::vector<std::uint32_t>>::value_type * best = nullptr;
        std::array<std::int64_t, 3> bestKey{};
        for (const auto & b : buckets) {
            const auto & bw = w[b.second.front()];
            std::array<std::int64_t, 3> key;
            if (anyOverlap) {
                if (overlap(current, bw) == 0) { continue; }
                key = {newBlocks(current, bw), detail::squaredDistance(current.lo, b.first), b.second.front()};
            } else {
                key = {detail::chebyshev(current.lo, b.first), detail::squaredDistance(current.lo, b.first), b.second.front()};
            }
            if (best == nullptr || key < bestKey) {
                best = &b;
                bestKey = key;
            }
        }
        order.insert(std::end(order), std::begin(best->second), std::end(best->second));
        current = w[best->second.front()];
        buckets.erase(best->first);
    }
    return order;
}

int main() {
    std::mt19937 rng{7};

    // the obvious case: two clusters, interleaved in the input
    {
        const int M = 3;
        std::vector<Window> w;
        for (int i = 0; i < 20; ++i) {
            w.push_back(windowAt(i % 2 ? Int3{0, 0, 0} : Int3{40, 0, 0}, M));
        }
        const auto order = plan(w, windowAt({0, 0, 0}, M), M);
        CHECK(order.size() == w.size(), "size %zu", order.size());
        for (std::size_t k = 0; k < 10; ++k) {
            CHECK(order[k] % 2 == 1, "first cluster first, got %u at %zu", order[k], k);
        }
        CHECK(totalCost(w, order, windowAt({0, 0, 0}, M)) == 27, "one whole new window, got %lld",
              static_cast<long long>(totalCost(w, order, windowAt({0, 0, 0}, M))));
    }

    // a neighbour one block over beats a further one, regardless of input order
    {
        const int M = 3;
        std::vector<Window> w{windowAt({2, 0, 0}, M), windowAt({1, 0, 0}, M)};
        const auto order = plan(w, windowAt({0, 0, 0}, M), M);
        CHECK(order[0] == 1 && order[1] == 0, "stepwise, got %u %u", order[0], order[1]);
    }

    // against the brute-force greedy, and never worse than scan order on clustered data
    for (int trial = 0; trial < 300; ++trial) {
        const int M = 2 + trial % 4;
        const int span = 3 + trial % 15;
        std::uniform_int_distribution<int> coord{0, span}, count{1, 120};
        std::vector<Window> w;
        const int n = count(rng);
        for (int i = 0; i < n; ++i) {
            w.push_back(windowAt({coord(rng), coord(rng), coord(rng) / (1 + trial % 3)}, M));
        }
        const auto start = windowAt({coord(rng), coord(rng), coord(rng)}, M);
        const auto order = plan(w, start, M);
        std::set<std::uint32_t> seen(std::begin(order), std::end(order));
        CHECK(order.size() == w.size() && seen.size() == w.size(), "trial %d: not a permutation", trial);
        const auto expected = reference(w, start);
        CHECK(order == expected, "trial %d (M=%d, n=%d): differs from the brute-force greedy", trial, M, n);
    }

    /* What the inventory actually produces: scanned at 8x, viewed at 1x. The sweep visits
     * scan chunks in Z-order and appends each chunk's objects in voxel order, so one scan
     * chunk's objects arrive together but spread over 8x8x8 viewing blocks, and the chunk
     * after can be far off in the curve. */
    {
        const int M = 3;
        const auto morton = [](const Int3 & c) {
            std::uint64_t m = 0;
            for (int bit = 0; bit < 10; ++bit) {
                for (int a = 0; a < 3; ++a) {
                    m |= static_cast<std::uint64_t>((c[a] >> bit) & 1) << (3 * bit + a);
                }
            }
            return m;
        };
        std::uniform_int_distribution<int> coord{0, 63};
        std::vector<Int3> points;
        for (int i = 0; i < 20000; ++i) {
            points.push_back({coord(rng), coord(rng), coord(rng) / 4});
        }
        std::sort(std::begin(points), std::end(points), [&](const Int3 & a, const Int3 & b) {
            const Int3 ca{a[0] / 8, a[1] / 8, a[2] / 8}, cb{b[0] / 8, b[1] / 8, b[2] / 8};
            if (ca != cb) { return morton(ca) < morton(cb); }
            return std::tie(a[2], a[1], a[0]) < std::tie(b[2], b[1], b[0]);
        });
        std::vector<Window> w;
        for (const auto & p : points) {
            w.push_back(windowAt({p[0] - 1, p[1] - 1, p[2] - 1}, M));
        }
        std::vector<std::uint32_t> scan(w.size());
        for (std::uint32_t i = 0; i < scan.size(); ++i) { scan[i] = i; }
        const auto order = plan(w, w.front(), M);
        const auto before = totalCost(w, scan, w.front());
        const auto after = totalCost(w, order, w.front());
        std::printf("20000 objects scanned at 8x, viewed at 1x: scan order %lld block loads, route %lld (%.1fx fewer)\n",
                    static_cast<long long>(before), static_cast<long long>(after), static_cast<double>(before) / after);
        CHECK(after * 4 < before * 3, "route should save at least a quarter");
    }

    // speed, dense and sparse, at the size an inventory reaches
    for (const int extent : {200, 2000}) {
        const int M = 5;
        std::uniform_int_distribution<int> coord{0, extent};
        std::vector<Window> w;
        for (int i = 0; i < 300000; ++i) {
            w.push_back(windowAt({coord(rng), coord(rng), coord(rng) / 8}, M));
        }
        const auto t0 = std::chrono::steady_clock::now();
        const auto order = plan(w, w.front(), M);
        const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("300000 objects over %d blocks a side, M=5: %.0f ms\n", extent, ms);
        CHECK(order.size() == w.size(), "size");
        CHECK(ms < 5000, "too slow: %.0f ms", ms);
    }

    /* The cache-aware choice. A dense inventory over a volume much larger than the cache:
     * the sweep has to win on downloads. The same inventory with a cache that holds all of
     * it: every order downloads each block once, so the greedy route, with fewer loads,
     * has to be chosen. */
    {
        const int M = 3;
        std::uniform_int_distribution<int> a{0, 63}, z{0, 15};
        std::vector<Window> w;
        for (int i = 0; i < 20000; ++i) {
            w.push_back(windowAt({a(rng) - 1, a(rng) - 1, z(rng) - 1}, M));
        }
        const auto start = w.front();
        const auto swept = sweep(w, start, M);
        std::set<std::uint32_t> seen(std::begin(swept), std::end(swept));
        CHECK(swept.size() == w.size() && seen.size() == w.size(), "sweep is not a permutation");
        const auto greedyOrder = plan(w, start, M);
        for (const std::size_t cache : {1000ul, 1000000ul}) {
            const auto g = simulate(w, greedyOrder, start, cache);
            const auto s2 = simulate(w, swept, start, cache);
            const auto t0 = std::chrono::steady_clock::now();
            const auto choice = planForCache(w, start, M, cache);
            const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            std::printf("cache %zu blocks: greedy %lld loads / %lld downloads, sweep %lld / %lld -> %s (%.0f ms)\n", cache,
                        static_cast<long long>(g.loads), static_cast<long long>(g.downloads),
                        static_cast<long long>(s2.loads), static_cast<long long>(s2.downloads),
                        choice.swept ? "sweep" : "greedy", ms);
            CHECK(choice.swept == (cache == 1000), "wrong choice for a %zu-block cache", cache);
            CHECK(s2.downloads <= g.downloads, "the sweep should never download more here");
        }
        // with everything cached, downloads are exactly the distinct blocks, whatever the order
        const auto all = simulate(w, swept, start, 1000000);
        const auto all2 = simulate(w, greedyOrder, start, 1000000);
        CHECK(all.downloads == all2.downloads, "unbounded cache: %lld vs %lld", static_cast<long long>(all.downloads), static_cast<long long>(all2.downloads));
    }

    // the curve itself: consecutive indices are adjacent cells, and every cell has one index
    {
        const int bits = 3;
        std::vector<std::pair<std::uint64_t, Int3>> cells;
        for (int x = 0; x < 8; ++x) for (int y = 0; y < 8; ++y) for (int z = 0; z < 8; ++z) {
            cells.push_back({detail::hilbertIndex({std::uint32_t(x), std::uint32_t(y), std::uint32_t(z)}, bits), {x, y, z}});
        }
        std::sort(std::begin(cells), std::end(cells));
        for (std::size_t k = 0; k < cells.size(); ++k) {
            CHECK(cells[k].first == k, "index %zu missing", k);
            if (k > 0) {
                const auto d = detail::squaredDistance(cells[k].second, cells[k - 1].second);
                CHECK(d == 1, "step %zu is not to a neighbour", k);
            }
        }
    }

    // and fast enough at the inventory's largest
    {
        const int M = 3;
        std::uniform_int_distribution<int> coord{0, 200};
        std::vector<Window> w;
        for (int i = 0; i < 300000; ++i) {
            w.push_back(windowAt({coord(rng), coord(rng), coord(rng) / 8}, M));
        }
        const auto t0 = std::chrono::steady_clock::now();
        const auto choice = planForCache(w, w.front(), M, 2000);
        const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("planForCache, 300000 objects: %.0f ms (%s)\n", ms, choice.swept ? "sweep" : "greedy");
        CHECK(choice.order.size() == w.size(), "size");
        CHECK(ms < 8000, "too slow: %.0f ms", ms);
    }

    std::printf(failures ? "%d failure(s)\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
