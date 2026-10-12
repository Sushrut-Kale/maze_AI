#pragma once
// ============================================================
// SearchAlgorithm.h  —  SmartRoute AI Abstract Search Interface
// ============================================================
// All algorithms operate on the same Maze grid.
// g(n) uses float traversal costs from Cell::traversalCost().
// h(n) uses Manhattan distance × min_cost (admissible).
// ============================================================
#include "Maze.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>
#include <algorithm>
#include <cmath>

// ── Statistics collected per algorithm run ────────────────────
struct SearchStats {
    int    nodesExplored  = 0;
    int    pathLength     = 0;  // number of cells in route (pathCells)
    int    routeMoves     = 0;  // number of moves (edges): max(0, pathLength - 1)
    float  pathCost       = 0.f; // weighted traversal cost (sum of entered cells, excluding start)
    double timeMs         = 0.0;
    int    maxFrontier    = 0;
    int    backtracks     = 0;  // DFS-specific
    int    rewardsOnPath  = 0;
    int    penaltiesOnPath= 0;
    bool   foundPath      = false;

    // Live display for A* / Dijkstra
    float currentG = 0.f;
    float currentH = 0.f;
    float currentF = 0.f;
};

// ════════════════════════════════════════════════════════════
// SearchAlgorithm — Abstract base class
// ════════════════════════════════════════════════════════════
class SearchAlgorithm {
public:
    SearchStats stats;
    Position    currentPos = Position::invalid();

    // ── Interface ────────────────────────────────────────────
    virtual void                  initialize(Maze& maze,
                                             Position start,
                                             Position goal) = 0;
    virtual bool                  step()                    = 0;
    virtual bool                  isFinished()        const = 0;
    virtual bool                  foundPath()         const = 0;
    virtual std::vector<Position> getPath()           const = 0;
    virtual std::string           getName()           const = 0;
    virtual ~SearchAlgorithm()                              = default;

    // Run to completion instantly (comparison mode)
    SearchStats runToCompletion() {
        auto t0 = std::chrono::high_resolution_clock::now();
        while (!isFinished()) step();
        auto t1 = std::chrono::high_resolution_clock::now();
        stats.timeMs =
            std::chrono::duration<double, std::milli>(t1-t0).count();
        return stats;
    }

protected:
    Maze*       pMaze     = nullptr;
    Position    goalPos   = Position::invalid();
    bool        finished_ = false;
    bool        found_    = false;
    std::vector<Position>               path;
    std::unordered_map<Position,Position> parent; // shared parent map

    // Admissible Manhattan heuristic.
    // h(n) = min_cost × (|dr| + |dc|)
    // With min_cost = reward_cost = 0.5, this is a lower bound
    // even when reward roads (cost 0.5) exist.
    float h(Position a) const {
        if (!goalPos.isValid()) return 0.f;
        float minCost = CostConfig::get().rewardCost; // 0.5
        return minCost * (float)a.manhattan(goalPos);
    }

    // Reconstruct path from parent map, mark cells in maze
    void reconstructPath(Position goal) {
        path.clear();
        Position cur = goal;
        int safety = pMaze->rows * pMaze->cols + 5;
        while (cur.isValid() && safety-- > 0) {
            path.push_back(cur);
            auto it = parent.find(cur);
            if (it == parent.end() || !it->second.isValid()) break;
            cur = it->second;
        }
        std::reverse(path.begin(), path.end());

        pMaze->markPath(path);

        stats.pathLength      = (int)path.size();
        stats.routeMoves      = stats.pathLength > 0 ? std::max(0, stats.pathLength - 1) : 0;
        stats.pathCost        = pMaze->pathCostWeighted(path);
        stats.rewardsOnPath   = pMaze->countRewardsOnPath();
        stats.penaltiesOnPath = pMaze->countPenaltiesOnPath();
        stats.foundPath       = true;
    }
};
