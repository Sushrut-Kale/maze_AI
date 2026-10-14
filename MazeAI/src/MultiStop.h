#pragma once
// ============================================================
// MultiStop.h  —  SmartRoute AI Multi-Customer Delivery Routing
// ============================================================
// Educational Last-Mile Delivery Multi-Stop Route Planning
// Supports Single-Customer and User-Ordered Multi-Stop Routing.
// Concatenates valid route segments without duplicating shared
// segment endpoints in route metrics or visual ribbons.
// Provides high-resolution steady_clock benchmarking & evaluation.
// ============================================================
#include "Cell.h"
#include "Maze.h"
#include "SearchAlgorithm.h"
#include "Algorithms.h"
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>

struct MultiLegStats {
    int legIndex = 0;
    std::string fromLabel;
    std::string toLabel;
    bool found = false;
    int moves = 0;
    float cost = 0.0f;
    int nodesExplored = 0;
    double timeMs = 0.0;
    std::vector<Position> path;
};

// ── Repeated Benchmark Statistics ─────────────────────────────
struct BenchmarkStats {
    bool   ran           = false;
    int    sampleCount   = 0;
    double medianMs      = 0.0;
    double meanMs        = 0.0;
    double minMs         = 0.0;
    double maxMs         = 0.0;
    double singleRunMs   = 0.0;
    std::vector<double> samplesMs;
};

struct MultiStopResult {
    bool fullyCompleted = false;
    int stopsCompleted = 0;
    int totalStops = 0;
    int totalMoves = 0;
    float totalCost = 0.0f;
    int totalNodesExplored = 0;
    double totalTimeMs = 0.0;
    int uniqueRouteCells = 0;
    int rewardsTraversed = 0;
    int penaltiesTraversed = 0;
    std::vector<Position> fullPath;      // Concatenated path with shared endpoints deduplicated
    std::vector<int> stopPathIndices;   // Index in fullPath where each customer stop is reached
    std::vector<MultiLegStats> legs;
    std::string failedDestination = "";
    std::string lastReachedStop = "";
    std::string stopSequenceStr = "";
    BenchmarkStats benchmark;
};

inline std::unique_ptr<SearchAlgorithm> createSearchAlgo(const std::string& name) {
    if (name == "BFS") return std::unique_ptr<SearchAlgorithm>(new BFSAlgorithm());
    if (name == "DFS") return std::unique_ptr<SearchAlgorithm>(new DFSAlgorithm());
    if (name == "A*" || name == "ASTAR") return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
    return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
}

// ════════════════════════════════════════════════════════════
// benchmarkMultiStopSearch — Repeated Independent Benchmarking
// ════════════════════════════════════════════════════════════
// Runs algorithm searches repeatedly on identical immutable maze snapshots
// using std::chrono::steady_clock.
// Avoids measuring UI, animation, logging, or rendering.
// Guarantees every repetition starts with fresh search structures.
// ════════════════════════════════════════════════════════════
inline BenchmarkStats benchmarkMultiStopSearch(const Maze& network,
                                              const std::vector<Position>& stopPositions,
                                              const std::vector<std::string>& stopLabels,
                                              const std::string& algoName,
                                              int repetitions = 25,
                                              int warmups = 3) {
    BenchmarkStats stats;
    if (stopPositions.empty()) return stats;

    // Warm-up runs to prime CPU caches, branch prediction, and memory allocator
    for (int w = 0; w < warmups; ++w) {
        Position curStart = network.start;
        for (size_t legIdx = 0; legIdx < stopPositions.size(); ++legIdx) {
            Position legGoal = stopPositions[legIdx];
            Maze legMaze = network;
            legMaze.resetAllFlags();
            auto algo = createSearchAlgo(algoName);
            algo->initialize(legMaze, curStart, legGoal);
            algo->runToCompletion();
            if (!algo->foundPath()) break;
            curStart = legGoal;
        }
    }

    std::vector<double> samples;
    samples.reserve(repetitions);

    auto benchStart = std::chrono::steady_clock::now();
    for (int rep = 0; rep < repetitions; ++rep) {
        auto t0 = std::chrono::steady_clock::now();
        Position curStart = network.start;
        for (size_t legIdx = 0; legIdx < stopPositions.size(); ++legIdx) {
            Position legGoal = stopPositions[legIdx];
            Maze legMaze = network;
            legMaze.resetAllFlags();
            auto algo = createSearchAlgo(algoName);
            algo->initialize(legMaze, curStart, legGoal);
            algo->runToCompletion();
            if (!algo->foundPath()) break;
            curStart = legGoal;
        }
        auto t1 = std::chrono::steady_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        samples.push_back(elapsedMs);

        // Bounded workload: If at least 5 runs are recorded and elapsed time exceeds 50ms,
        // stop to prevent any GUI freezing on massive networks
        if (rep >= 4) {
            double totalSoFar = std::chrono::duration<double, std::milli>(t1 - benchStart).count();
            if (totalSoFar > 50.0) break;
        }
    }

    if (samples.empty()) return stats;

    std::vector<double> sorted = samples;
    std::sort(sorted.begin(), sorted.end());

    size_t n = sorted.size();
    double median = (n % 2 == 1) ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    double sum = 0.0;
    for (double s : sorted) sum += s;

    stats.ran = true;
    stats.sampleCount = (int)n;
    stats.medianMs = median;
    stats.meanMs = sum / (double)n;
    stats.minMs = sorted.front();
    stats.maxMs = sorted.back();
    stats.samplesMs = samples;
    return stats;
}

// ════════════════════════════════════════════════════════════
// runMultiStopSearch — Execute ordered multi-stop route search
// ════════════════════════════════════════════════════════════
// Starts at Store (network.start), visits each customer in stopPositions
// in the specified order. Each leg uses the designated search algorithm.
// Reports partial progress and unreachable destinations cleanly.
// Also executes repeated benchmarking on identical immutable maze snapshots.
// ════════════════════════════════════════════════════════════
inline MultiStopResult runMultiStopSearch(const Maze& network,
                                         const std::vector<Position>& stopPositions,
                                         const std::vector<std::string>& stopLabels,
                                         const std::string& algoName,
                                         bool runBenchmark = true,
                                         int benchmarkReps = 25) {
    MultiStopResult res;
    res.totalStops = (int)stopPositions.size();

    // Build human-readable stop sequence
    std::string seq = "Store";
    for (size_t i = 0; i < stopPositions.size(); ++i) {
        std::string lbl = (i < stopLabels.size()) ? stopLabels[i] : ("C" + std::to_string(i + 1));
        seq += " -> " + lbl;
    }
    res.stopSequenceStr = seq;

    if (stopPositions.empty()) {
        res.fullyCompleted = false;
        return res;
    }

    Position currentLegStart = network.start;
    std::string currentLegStartLabel = "Store";

    std::vector<Position> combinedPath;

    for (size_t legIdx = 0; legIdx < stopPositions.size(); ++legIdx) {
        Position legGoal = stopPositions[legIdx];
        std::string legGoalLabel = (legIdx < stopLabels.size()) ? stopLabels[legIdx] : ("C" + std::to_string(legIdx + 1));

        MultiLegStats leg;
        leg.legIndex = (int)legIdx + 1;
        leg.fromLabel = currentLegStartLabel;
        leg.toLabel = legGoalLabel;

        // Clean copy of maze for this leg
        Maze legMaze = network;
        legMaze.resetAllFlags();

        auto algo = createSearchAlgo(algoName);
        algo->initialize(legMaze, currentLegStart, legGoal);

        auto t0 = std::chrono::steady_clock::now();
        SearchStats st = algo->runToCompletion();
        auto t1 = std::chrono::steady_clock::now();
        leg.timeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        leg.nodesExplored = st.nodesExplored;
        res.totalNodesExplored += leg.nodesExplored;
        res.totalTimeMs += leg.timeMs;

        std::vector<Position> lPath = algo->getPath();
        std::string valErr;
        bool valid = algo->foundPath() && validatePath(network, lPath, currentLegStart, legGoal, &valErr);
        leg.found = valid;

        if (!valid) {
            leg.moves = 0;
            leg.cost = 0.0f;
            res.legs.push_back(leg);
            res.fullyCompleted = false;
            res.failedDestination = legGoalLabel;
            res.lastReachedStop = currentLegStartLabel;
            break;
        }

        leg.path = lPath;
        leg.moves = (int)leg.path.size() > 0 ? (int)leg.path.size() - 1 : 0;
        leg.cost = network.pathCostWeighted(leg.path);

        res.totalMoves += leg.moves;
        res.totalCost += leg.cost;
        res.stopsCompleted++;
        res.legs.push_back(leg);

        // Concatenate without duplicating shared endpoint
        if (combinedPath.empty()) {
            combinedPath = leg.path;
        } else {
            for (size_t pIdx = 1; pIdx < leg.path.size(); ++pIdx) {
                combinedPath.push_back(leg.path[pIdx]);
            }
        }
        res.stopPathIndices.push_back((int)combinedPath.size() - 1);

        currentLegStart = legGoal;
        currentLegStartLabel = legGoalLabel;
    }

    res.fullyCompleted = (res.stopsCompleted == res.totalStops && res.totalStops > 0);
    res.fullPath = combinedPath;
    res.uniqueRouteCells = (int)combinedPath.size();

    // Count rewards and penalties traversed along the combined path
    for (size_t i = 1; i < combinedPath.size(); ++i) {
        const auto& pos = combinedPath[i];
        if (network.inBounds(pos)) {
            const Cell& c = network.at(pos);
            if (c.special == SpecialType::REWARD) res.rewardsTraversed++;
            else if (c.special == SpecialType::PENALTY) res.penaltiesTraversed++;
        }
    }

    // Benchmark execution
    if (runBenchmark) {
        res.benchmark = benchmarkMultiStopSearch(network, stopPositions, stopLabels, algoName, benchmarkReps);
        res.benchmark.singleRunMs = res.totalTimeMs;
    } else {
        res.benchmark.ran = true;
        res.benchmark.sampleCount = 1;
        res.benchmark.singleRunMs = res.totalTimeMs;
        res.benchmark.medianMs = res.totalTimeMs;
        res.benchmark.meanMs = res.totalTimeMs;
        res.benchmark.minMs = res.totalTimeMs;
        res.benchmark.maxMs = res.totalTimeMs;
    }

    return res;
}

// ════════════════════════════════════════════════════════════
// Duration & Timing Formatting
// ════════════════════════════════════════════════════════════
// Formats durations cleanly without misleading integer truncation or "0.00 ms".
// Supports microseconds (µs) and milliseconds (ms).
// ════════════════════════════════════════════════════════════
inline std::string formatSearchDuration(double timeMs, bool includeBothUnits = false) {
    if (timeMs <= 0.0) {
        return "Below measurement resolution";
    }
    if (timeMs < 0.001) {
        return "< 1 \xc2\xb5s";
    }
    std::ostringstream ss;
    if (timeMs >= 10.0) {
        ss << std::fixed << std::setprecision(3) << timeMs << " ms";
    } else if (timeMs >= 1.0) {
        ss << std::fixed << std::setprecision(3) << timeMs << " ms";
    } else {
        double us = timeMs * 1000.0;
        if (includeBothUnits) {
            if (us >= 100.0) {
                ss << (int)std::round(us) << " \xc2\xb5s ("
                   << std::fixed << std::setprecision(3) << timeMs << " ms)";
            } else {
                ss << std::fixed << std::setprecision(1) << us << " \xc2\xb5s ("
                   << std::setprecision(3) << timeMs << " ms)";
            }
        } else {
            if (us >= 100.0) {
                ss << (int)std::round(us) << " \xc2\xb5s";
            } else if (us >= 10.0) {
                ss << (int)std::round(us) << " \xc2\xb5s";
            } else {
                ss << std::fixed << std::setprecision(1) << us << " \xc2\xb5s";
            }
        }
    }
    return ss.str();
}

// ════════════════════════════════════════════════════════════
// Algorithm Evaluation & Winner Determination
// ════════════════════════════════════════════════════════════
struct AlgoEvalData {
    std::string name;
    bool hasRun = false;
    bool foundPath = false;
    int totalMoves = 0;
    float totalCost = 0.0f;
    int totalNodesExplored = 0;
    double medianTimeMs = 0.0;
    double singleRunTimeMs = 0.0;
    int stopsCompleted = 0;
    int totalStops = 0;
};

struct WinnerResult {
    std::vector<std::string> winners; // e.g. {"BFS", "A*"}
    bool isTie = false;
    bool isAvailable = false;
    std::string summary;      // e.g. "BFS, A* (tie)" or "DFS"
    std::string explanation;  // Educational context
};

inline WinnerResult computeRouteMovesWinner(const std::vector<AlgoEvalData>& algos) {
    WinnerResult res;
    int minMoves = 99999999;
    for (const auto& a : algos) {
        if (a.hasRun && a.foundPath) {
            if (a.totalMoves < minMoves) minMoves = a.totalMoves;
        }
    }
    if (minMoves == 99999999) {
        res.isAvailable = false;
        res.summary = "None (no valid routes)";
        res.explanation = "No algorithm found a successful delivery route.";
        return res;
    }
    for (const auto& a : algos) {
        if (a.hasRun && a.foundPath && a.totalMoves == minMoves) {
            res.winners.push_back(a.name);
        }
    }
    res.isAvailable = true;
    res.isTie = (res.winners.size() > 1);
    std::string names;
    for (size_t i = 0; i < res.winners.size(); ++i) {
        if (i > 0) names += ", ";
        names += res.winners[i];
    }
    if (res.isTie) names += " (tie)";
    res.summary = names;
    res.explanation = "Unweighted hop count: BFS guarantees fewest moves in unweighted graphs.";
    return res;
}

inline WinnerResult computeWeightedCostWinner(const std::vector<AlgoEvalData>& algos) {
    WinnerResult res;
    float minCost = 1e9f;
    for (const auto& a : algos) {
        if (a.hasRun && a.foundPath) {
            if (a.totalCost < minCost) minCost = a.totalCost;
        }
    }
    if (minCost > 9e8f) {
        res.isAvailable = false;
        res.summary = "None (no valid routes)";
        res.explanation = "No algorithm found a successful delivery route.";
        return res;
    }
    for (const auto& a : algos) {
        if (a.hasRun && a.foundPath && std::abs(a.totalCost - minCost) <= 0.001f) {
            res.winners.push_back(a.name);
        }
    }
    res.isAvailable = true;
    res.isTie = (res.winners.size() > 1);
    std::string names;
    for (size_t i = 0; i < res.winners.size(); ++i) {
        if (i > 0) names += ", ";
        names += res.winners[i];
    }
    if (res.isTie) names += " (tie)";
    res.summary = names;
    res.explanation = "Weighted terrain cost: A* guarantees lowest weighted cost with admissible heuristic.";
    return res;
}

inline WinnerResult computeFewestNodesWinner(const std::vector<AlgoEvalData>& algos) {
    WinnerResult res;
    int minNodes = 99999999;
    for (const auto& a : algos) {
        if (a.hasRun) {
            if (a.totalNodesExplored < minNodes) minNodes = a.totalNodesExplored;
        }
    }
    if (minNodes == 99999999) {
        res.isAvailable = false;
        res.summary = "Not executed";
        res.explanation = "No algorithm has executed yet.";
        return res;
    }
    for (const auto& a : algos) {
        if (a.hasRun && a.totalNodesExplored == minNodes) {
            res.winners.push_back(a.name);
        }
    }
    res.isAvailable = true;
    res.isTie = (res.winners.size() > 1);
    std::string names;
    for (size_t i = 0; i < res.winners.size(); ++i) {
        if (i > 0) names += ", ";
        names += res.winners[i];
    }
    if (res.isTie) names += " (tie)";
    res.summary = names;
    res.explanation = "Empirical state exploration on current maze; varies by maze structure & order.";
    return res;
}

inline WinnerResult computeExecutionTimeWinner(const std::vector<AlgoEvalData>& algos) {
    WinnerResult res;
    double minTime = 1e9;
    bool anyRan = false;
    bool anyAboveResolution = false;

    for (const auto& a : algos) {
        if (a.hasRun) {
            anyRan = true;
            if (a.medianTimeMs > 0.0005) { // at least 0.5 µs
                anyAboveResolution = true;
                if (a.medianTimeMs < minTime) minTime = a.medianTimeMs;
            }
        }
    }
    if (!anyRan) {
        res.isAvailable = false;
        res.summary = "Not executed";
        res.explanation = "No benchmark has run yet.";
        return res;
    }
    if (!anyAboveResolution) {
        res.isAvailable = false;
        res.summary = "Inconclusive";
        res.explanation = "Search completed faster than monotonic clock resolution on this platform.";
        return res;
    }

    // Tie policy: within 2 µs (0.002 ms) or within 5% of minTime
    double tieTol = std::max(0.002, 0.05 * minTime);
    for (const auto& a : algos) {
        if (a.hasRun && a.medianTimeMs > 0.0005) {
            if (std::abs(a.medianTimeMs - minTime) <= tieTol) {
                res.winners.push_back(a.name);
            }
        }
    }

    res.isAvailable = true;
    res.isTie = (res.winners.size() > 1);
    std::string names;
    for (size_t i = 0; i < res.winners.size(); ++i) {
        if (i > 0) names += ", ";
        names += res.winners[i];
    }
    if (res.isTie) names += " (tie)";
    res.summary = names;
    res.explanation = "Fastest median execution across repeated benchmark runs on identical snapshot.";
    return res;
}

inline bool checkAllEqualRouteQuality(const std::vector<AlgoEvalData>& algos) {
    if (algos.size() < 2) return false;
    for (const auto& a : algos) {
        if (!a.hasRun || !a.foundPath) return false;
    }
    int firstMoves = algos[0].totalMoves;
    float firstCost = algos[0].totalCost;
    for (size_t i = 1; i < algos.size(); ++i) {
        if (algos[i].totalMoves != firstMoves) return false;
        if (std::abs(algos[i].totalCost - firstCost) > 0.001f) return false;
    }
    return true;
}
