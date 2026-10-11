// Standalone test for widgets/tools/inventoryfilter.h — which objects the inventory shows
// and where the next/previous keys go. Compile and run:
//   c++ -std=c++17 -O2 -I .. -o /tmp/invfilter_test inventoryfilter_test.cpp && /tmp/invfilter_test
#include "widgets/tools/inventoryfilter.h"

#include <cstdio>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(const bool ok, const std::string & what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

struct Rec { std::uint64_t voxels{0}; };

std::vector<Rec> records(const std::vector<std::uint64_t> & sizes) {
    std::vector<Rec> out;
    for (const auto s : sizes) { out.push_back(Rec{s}); }
    return out;
}

const auto nothing = [](std::size_t){ return false; };

void section(const char * title) { std::printf("%s\n", title); }
}

int main() {
    using objinv::Filter;

    section("filtering hides rows without reordering them");
    {
        const auto recs = records({500, 3, 1200, 7, 90, 40000, 1});
        Filter f;
        f.minVoxels = 100;
        const auto rows = objinv::buildRows(recs, f, nothing, nothing);
        check((rows == std::vector<std::uint32_t>{0, 2, 5}), "a minimum size keeps the big ones, in scan order");

        bool increasing = true;
        for (std::size_t i = 1; i < rows.size(); ++i) { increasing = increasing && rows[i] > rows[i - 1]; }
        check(increasing, "the row list is strictly increasing — the guarantee the walk rests on");

        Filter none;
        check(objinv::buildRows(recs, none, nothing, nothing).size() == recs.size(),
              "an empty filter shows everything");

        Filter all;
        all.minVoxels = 1000000;
        check(objinv::buildRows(recs, all, nothing, nothing).empty(), "a filter can hide everything");

        Filter band;
        band.minVoxels = 10;
        band.maxVoxels = 1000;
        check((objinv::buildRows(recs, band, nothing, nothing) == std::vector<std::uint32_t>{0, 4}),
              "an upper bound catches the over-merged blobs");

        Filter zero;
        check(objinv::buildRows(recs, zero, nothing, nothing).size() == recs.size(),
              "a minimum of zero excludes nothing, not even single voxels");
    }

    section("hiding what is already dealt with");
    {
        const auto recs = records({100, 200, 300, 400});
        const std::set<std::size_t> annotated{1, 3};
        const std::set<std::size_t> seen{0, 1};
        const auto known = [&](const std::size_t i){ return annotated.count(i) != 0; };
        const auto beenThere = [&](const std::size_t i){ return seen.count(i) != 0; };

        Filter hk; hk.hideKnown = true;
        check((objinv::buildRows(recs, hk, known, beenThere) == std::vector<std::uint32_t>{0, 2}),
              "hiding already annotated objects drains the queue as work gets done");

        Filter hv; hv.hideVisited = true;
        check((objinv::buildRows(recs, hv, known, beenThere) == std::vector<std::uint32_t>{2, 3}),
              "hiding visited objects stops the walk being a treadmill");

        Filter both; both.hideKnown = both.hideVisited = true;
        check((objinv::buildRows(recs, both, known, beenThere) == std::vector<std::uint32_t>{2}),
              "both at once");

        const auto huge = std::numeric_limits<std::uint64_t>::max();
        const auto extreme = records({huge, 0});
        Filter def;
        check(objinv::buildRows(extreme, def, nothing, nothing).size() == 2,
              "the default filter admits both a zero-voxel and a 64-bit-maximum record");
    }

    section("appending during a scan matches a full rebuild");
    {
        // The failure this pins down is the worst one this feature has: if the incremental
        // path ever diverges from a rebuild, the key silently skips objects.
        std::mt19937 rng{20260923};
        std::uniform_int_distribution<std::uint64_t> size{0, 5000};
        std::uniform_int_distribution<int> chunk{0, 40};

        bool identical = true, neverShrinks = true, alwaysIncreasing = true;
        for (int trial = 0; trial < 200 && identical; ++trial) {
            Filter f;
            f.minVoxels = trial % 3 == 0 ? 0 : 250 * (trial % 7);
            f.hideKnown = trial % 5 == 0;
            const auto known = [](const std::size_t i){ return i % 4 == 0; };

            std::vector<Rec> grown;
            std::vector<std::uint32_t> incremental;
            for (int batch = 0; batch < 8; ++batch) {
                const auto firstNew = grown.size();
                const auto n = chunk(rng);
                for (int k = 0; k < n; ++k) { grown.push_back(Rec{size(rng)}); }
                const auto was = incremental.size();
                const auto added = objinv::appendRows(incremental, grown, firstNew, f, known, nothing);
                neverShrinks = neverShrinks && incremental.size() == was + added;
                for (std::size_t i = 1; i < incremental.size(); ++i) {
                    alwaysIncreasing = alwaysIncreasing && incremental[i] > incremental[i - 1];
                }
                const auto rebuilt = objinv::buildRows(grown, f, known, nothing);
                identical = identical && incremental == rebuilt;
            }
        }
        check(identical, "200 randomised scans: appending batch by batch equals rebuilding from scratch");
        check(neverShrinks, "the reported count is exactly what was added");
        check(alwaysIncreasing, "the row list stays strictly increasing through every batch");

        // A batch that contributes nothing must report nothing, so no signal is emitted.
        const auto recs = records({5000});
        std::vector<Rec> grown = recs;
        Filter f; f.minVoxels = 1000;
        auto rows = objinv::buildRows(grown, f, nothing, nothing);
        const auto snapshot = rows;
        const auto firstNew = grown.size();
        grown.push_back(Rec{1});
        grown.push_back(Rec{2});
        const auto added = objinv::appendRows(rows, grown, firstNew, f, nothing, nothing);
        check(added == 0 && rows == snapshot, "a batch of specks adds no rows and leaves the list untouched");
    }

    section("a filter change resumes the walk rather than restarting it");
    {
        const auto recs = records({100, 5, 200, 6, 300, 7, 400});
        Filter loose;
        const auto wide = objinv::buildRows(recs, loose, nothing, nothing);
        check(wide.size() == 7, "everything shown to begin with");

        // walking, currently on record 3 (a speck) — then the minimum is raised
        Filter tight; tight.minVoxels = 100;
        const auto narrow = objinv::buildRows(recs, tight, nothing, nothing);
        check((narrow == std::vector<std::uint32_t>{0, 2, 4, 6}), "raising the minimum drops the specks");
        check(objinv::reanchor(narrow, 3) == 2, "the walk resumes at the next surviving object, not at the top");
        check(objinv::reanchor(narrow, 4) == 2, "a record that survived is landed on exactly");
        check(objinv::reanchor(narrow, 0) == 0, "re-anchoring at the very first record stays there");
        check(objinv::reanchor(narrow, 6) == 3, "re-anchoring at the last record stays there");
        check(objinv::reanchor(narrow, 99) == 3, "if everything after it was filtered away, the last row");
        check(objinv::reanchor({}, 3) == -1, "nowhere to go in an empty list");
    }

    section("next and previous");
    {
        check(objinv::step(-1, 5, true) == 0, "the first press with no current row enters at the top");
        check(objinv::step(-1, 5, false) == 4, "and going backwards enters at the bottom");
        check(objinv::step(0, 5, true) == 1, "forward");
        check(objinv::step(3, 5, false) == 2, "back");
        check(objinv::step(4, 5, true) == 4, "the end caps rather than wrapping, so the caller can say so");
        check(objinv::step(0, 5, false) == 0, "and so does the start");
        check(objinv::step(0, 0, true) == -1, "an empty list has nowhere to go");
        check(objinv::step(-1, 0, false) == -1, "in either direction");
        check(objinv::step(9, 5, true) == 0, "a stale row out of range re-enters the list rather than reading past it");
        check(objinv::step(0, 1, true) == 0 && objinv::step(0, 1, false) == 0,
              "a single-row list stays put both ways");

        // Walking a list end to end visits every row exactly once.
        const int n = 6;
        std::vector<int> visits;
        int row = -1;
        for (int i = 0; i < n; ++i) {
            row = objinv::step(row, n, true);
            visits.push_back(row);
        }
        bool once = visits.size() == static_cast<std::size_t>(n);
        for (int i = 0; i < n; ++i) { once = once && visits[i] == i; }
        check(once, "walking forward from nothing visits every row once, in order");
        check(objinv::step(row, n, true) == n - 1, "and then stops");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : (std::to_string(failures) + " failed").c_str());
    return failures == 0 ? 0 : 1;
}
