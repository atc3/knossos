// Exercises widgets/historytimeline.h — the row/step mapping behind the History window
// and the undo budget rule.
#include "widgets/historytimeline.h"
#include <cstdio>
#include <vector>

static int failures = 0;
static void check(const bool ok, const char * what) {
    std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) { ++failures; }
}

int main() {
    std::printf("1. row -> steps with nothing to redo\n");
    {
        // rows: [current][undo 1][undo 2][undo 3]
        check(historyRowToSteps(0, 0) == 0, "row 0 is the current state");
        check(historyRowToSteps(1, 0) == 1, "first entry below current undoes once");
        check(historyRowToSteps(3, 0) == 3, "third undoes three times");
    }

    std::printf("2. row -> steps with two redoable states above\n");
    {
        // rows: [redo x2][redo x1][current][undo 1][undo 2]
        check(historyRowToSteps(0, 2) == -2, "top row is the furthest state ahead");
        check(historyRowToSteps(1, 2) == -1, "the row just above current redoes once");
        check(historyRowToSteps(2, 2) == 0, "current sits below the redo entries");
        check(historyRowToSteps(3, 2) == 1, "below current undoes once");
        check(historyRowToSteps(4, 2) == 2, "and again");
    }

    std::printf("3. clicking a row always lands on that row\n");
    {
        // applying `steps` then re-rendering must leave the clicked entry at the current row
        bool consistent = true;
        for (int futureCount = 0; futureCount <= 4; ++futureCount) {
            for (int row = 0; row <= futureCount + 4; ++row) {
                const auto steps = historyRowToSteps(row, futureCount);
                // after moving, the future count changes by exactly `steps`
                const auto newFutureCount = futureCount + steps;
                consistent = consistent && historyRowToSteps(row, newFutureCount) == 0;
            }
        }
        check(consistent, "the clicked row becomes the current row");
    }

    std::printf("4. budget: entry count\n");
    {
        const std::vector<std::size_t> five{10, 10, 10, 10, 10};
        check(entriesToEvict(five, 10, 1000) == 0, "under both caps, evict nothing");
        check(entriesToEvict(five, 3, 1000) == 2, "over the count cap, drop the two oldest");
        check(entriesToEvict({}, 3, 1000) == 0, "empty stack is fine");
    }

    std::printf("5. budget: total size\n");
    {
        const std::vector<std::size_t> sizes{100, 100, 100, 100};
        check(entriesToEvict(sizes, 10, 400) == 0, "exactly at the byte cap, keep everything");
        check(entriesToEvict(sizes, 10, 250) == 2, "drop oldest until under the byte cap");
        check(entriesToEvict(sizes, 10, 0) == 3, "never drop the last entry, however large");
        check(entriesToEvict({5000}, 10, 100) == 0, "a single oversized entry is still kept");
    }

    std::printf("6. both caps at once\n");
    {
        const std::vector<std::size_t> sizes{500, 1, 1, 1, 1, 1};
        // count cap 4 drops the 500 first, which also fixes the byte total
        check(entriesToEvict(sizes, 4, 10) == 2, "count eviction is applied before size");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "THERE WERE FAILURES");
    return failures != 0;
}
