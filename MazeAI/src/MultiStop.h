#pragma once
// ============================================================
// MultiStop.h  —  SmartRoute AI Multi-Customer Delivery Routing
// ============================================================
// Educational Last-Mile Delivery Multi-Stop Route Planning
// Supports Single-Customer and User-Ordered Multi-Stop Routing.
// Concatenates valid route segments without duplicating shared
// segment endpoints in route metrics or visual ribbons.
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
    std::vector<MultiLegStats> legs;
    std::string failedDestination = "";
    std::string lastReachedStop = "";
    std::string stopSequenceStr = "";
};

inline std::unique_ptr<SearchAlgorithm> createSearchAlgo(const std::string& name) {
    if (name == "BFS") return std::unique_ptr<SearchAlgorithm>(new BFSAlgorithm());
    if (name == "DFS") return std::unique_ptr<SearchAlgorithm>(new DFSAlgorithm());
    if (name == "A*" || name == "ASTAR") return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
    return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
}

// ════════════════════════════════════════════════════════════
// runMultiStopSearch — Execute ordered multi-stop route search
// ════════════════════════════════════════════════════════════
// Starts at Store (network.start), visits each customer in stopPositions
// in the specified order. Each leg uses the designated search algorithm.
// Reports partial progress and unreachable destinations cleanly.
// ════════════════════════════════════════════════════════════
inline MultiStopResult runMultiStopSearch(const Maze& network,
                                         const std::vector<Position>& stopPositions,
                                         const std::vector<std::string>& stopLabels,
                                         const std::string& algoName) {
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
        res.fullyCompleted = true;
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

        auto t0 = std::chrono::high_resolution_clock::now();
        SearchStats st = algo->runToCompletion();
        auto t1 = std::chrono::high_resolution_clock::now();
        leg.timeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        leg.found = algo->foundPath();
        leg.nodesExplored = st.nodesExplored;
        res.totalNodesExplored += leg.nodesExplored;
        res.totalTimeMs += leg.timeMs;

        if (!leg.found) {
            leg.moves = 0;
            leg.cost = 0.0f;
            res.legs.push_back(leg);
            res.fullyCompleted = false;
            res.failedDestination = legGoalLabel;
            res.lastReachedStop = currentLegStartLabel;
            break;
        }

        leg.path = algo->getPath();
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

        currentLegStart = legGoal;
        currentLegStartLabel = legGoalLabel;
    }

    res.fullyCompleted = (res.stopsCompleted == res.totalStops);
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

    return res;
}
