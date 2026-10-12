#pragma once
// ============================================================
// Algorithms.h  —  SmartRoute AI Pathfinding Algorithm Implementations
//   BFS, DFS (search), Dijkstra, Greedy Best-First, A*
//   Compatible with GCC 6.3 / MinGW
// ============================================================
// KEY NOTES ON CORRECTNESS:
//   BFS    : Uses FIFO queue. Treats all edges equal cost = 1.
//            Guarantees fewest-MOVES path, NOT minimum traversal cost.
//            Reward/penalty cells do NOT affect BFS route selection.
//
//   DFS    : Uses explicit stack. Does NOT guarantee any optimality.
//            May find long paths. Deterministic neighbor order.
//
//   Dijkstra: Priority queue on g-cost. Guarantees minimum
//            traversal-cost path using float cell costs.
//
//   Greedy : Priority queue on h(n) only. Fast but NOT optimal.
//
//   A*     : f(n)=g(n)+h(n). Guaranteed optimal with admissible h.
//            h(n) = min_cost * manhattan (min_cost = 0.5 for reward roads).
//            Finds minimum weighted cost path including reward shortcuts.
// ============================================================
#include "SearchAlgorithm.h"
#include <queue>
#include <stack>
#include <unordered_set>
#include <functional>

// ════════════════════════════════════════════════════════════
// BFS  —  Breadth-First Search
// IMPORTANT: BFS ignores terrain weights. It finds the path with
// the fewest road segments, not the lowest travel cost.
// Reward and penalty roads affect COST calculation but NOT which
// path BFS selects. This is intentional and educationally correct.
// ════════════════════════════════════════════════════════════
class BFSAlgorithm : public SearchAlgorithm {
    std::queue<Position>           frontier;
    std::unordered_set<Position>   visited;

public:
    std::string getName()      const override { return "BFS"; }
    bool isFinished()          const override { return finished_; }
    bool foundPath()           const override { return found_;    }
    std::vector<Position> getPath() const override { return path; }

    void initialize(Maze& maze, Position start, Position goal) override {
        pMaze    = &maze;
        goalPos  = goal;
        finished_= found_ = false;
        stats    = SearchStats{};
        path.clear(); parent.clear();
        frontier = std::queue<Position>(); visited.clear();

        frontier.push(start);
        visited.insert(start);
        maze.at(start).inFrontier = true;
        currentPos = start;
    }

    bool step() override {
        if (finished_) return true;
        if (frontier.empty()) {
            finished_ = true; stats.foundPath = false; return true;
        }

        Position cur = frontier.front(); frontier.pop();
        pMaze->at(cur).inFrontier = false;
        pMaze->at(cur).explored   = true;
        currentPos = cur;
        stats.nodesExplored++;

        if (cur == goalPos) {
            reconstructPath(goalPos);
            finished_ = found_ = true;
            return true;
        }

        for (const Position& nb : pMaze->getNeighbors(cur)) {
            if (!visited.count(nb)) {
                visited.insert(nb);
                parent[nb] = cur;
                frontier.push(nb);
                pMaze->at(nb).inFrontier = true;
            }
        }
        stats.maxFrontier =
            std::max(stats.maxFrontier, (int)frontier.size());
        return false;
    }
};

// ════════════════════════════════════════════════════════════
// DFSAlgorithm  —  Depth-First Search (iterative with stack)
// IMPORTANT: DFS explores deep branches first. It does NOT
// guarantee shortest path or lowest cost. Neighbor exploration
// order: TOP, RIGHT, BOTTOM, LEFT (deterministic, reproducible).
// ════════════════════════════════════════════════════════════
class DFSAlgorithm : public SearchAlgorithm {
    struct DFSNode {
        Position from, pos;
        DFSNode(Position f, Position p) : from(f), pos(p) {}
    };
    std::stack<DFSNode>          stk;
    std::unordered_set<Position> visited;

public:
    std::string getName()      const override { return "DFS"; }
    bool isFinished()          const override { return finished_; }
    bool foundPath()           const override { return found_;    }
    std::vector<Position> getPath() const override { return path; }

    void initialize(Maze& maze, Position start, Position goal) override {
        pMaze    = &maze;
        goalPos  = goal;
        finished_= found_ = false;
        stats    = SearchStats{};
        path.clear(); parent.clear();
        stk      = std::stack<DFSNode>(); visited.clear();

        stk.push(DFSNode(Position::invalid(), start));
        maze.at(start).inFrontier = true;
        currentPos = start;
    }

    bool step() override {
        if (finished_) return true;

        while (!stk.empty()) {
            DFSNode node = stk.top(); stk.pop();
            Position from = node.from;
            Position cur  = node.pos;

            if (visited.count(cur)) {
                stats.backtracks++;
                continue;
            }
            visited.insert(cur);
            parent[cur] = from;

            pMaze->at(cur).inFrontier = false;
            pMaze->at(cur).explored   = true;
            currentPos = cur;
            stats.nodesExplored++;

            if (cur == goalPos) {
                reconstructPath(goalPos);
                finished_ = found_ = true;
                return true;
            }

            for (const Position& nb : pMaze->getNeighbors(cur)) {
                if (!visited.count(nb)) {
                    stk.push(DFSNode(cur, nb));
                    if (!pMaze->at(nb).explored)
                        pMaze->at(nb).inFrontier = true;
                }
            }
            stats.maxFrontier = std::max(stats.maxFrontier,(int)stk.size());
            return false;   // yield one expansion per step
        }

        finished_ = true; stats.foundPath = false;
        return true;
    }
};

// ════════════════════════════════════════════════════════════
// DijkstraAlgorithm  —  Uniform Cost Search (full weighted)
// Uses float traversal costs. Guaranteed minimum-cost path.
// ════════════════════════════════════════════════════════════
class DijkstraAlgorithm : public SearchAlgorithm {
    struct PQNode {
        float cost; Position pos;
        bool operator>(const PQNode& o) const { return cost > o.cost; }
    };
    std::priority_queue<PQNode,std::vector<PQNode>,std::greater<PQNode>> pq;
    std::unordered_map<Position,float>  dist;
    std::unordered_set<Position>        closed;

public:
    std::string getName()      const override { return "Dijkstra"; }
    bool isFinished()          const override { return finished_; }
    bool foundPath()           const override { return found_;    }
    std::vector<Position> getPath() const override { return path; }

    void initialize(Maze& maze, Position start, Position goal) override {
        pMaze    = &maze;
        goalPos  = goal;
        finished_= found_ = false;
        stats    = SearchStats{};
        path.clear(); parent.clear();
        pq   = decltype(pq)(); dist.clear(); closed.clear();

        dist[start] = 0.f;
        PQNode n0; n0.cost = 0.f; n0.pos = start;
        pq.push(n0);
        maze.at(start).inFrontier = true;
        currentPos = start;
    }

    bool step() override {
        if (finished_) return true;
        if (pq.empty()) {
            finished_ = true; stats.foundPath = false; return true;
        }

        PQNode top = pq.top(); pq.pop();
        float    cost = top.cost;
        Position cur  = top.pos;

        if (closed.count(cur)) return false;
        closed.insert(cur);

        pMaze->at(cur).inFrontier = false;
        pMaze->at(cur).explored   = true;
        currentPos = cur;
        stats.nodesExplored++;
        stats.currentG = cost;
        stats.currentH = h(cur);
        stats.currentF = cost + stats.currentH;

        if (cur == goalPos) {
            reconstructPath(goalPos);
            finished_ = found_ = true;
            return true;
        }

        for (const Position& nb : pMaze->getNeighbors(cur)) {
            if (closed.count(nb)) continue;
            float edge    = std::max(0.1f, pMaze->at(nb).traversalCost());
            float newCost = cost + edge;
            if (!dist.count(nb) || newCost < dist[nb]) {
                dist[nb]   = newCost;
                parent[nb] = cur;
                PQNode nn; nn.cost = newCost; nn.pos = nb;
                pq.push(nn);
                pMaze->at(nb).inFrontier = true;
            }
        }
        stats.maxFrontier = std::max(stats.maxFrontier,(int)pq.size());
        return false;
    }
};

// ════════════════════════════════════════════════════════════
// GreedyBFSAlgorithm  —  Greedy Best-First Search
// Uses h(n) only. Fast but NOT guaranteed optimal.
// ════════════════════════════════════════════════════════════
class GreedyBFSAlgorithm : public SearchAlgorithm {
    struct PQNode {
        float hval; Position pos;
        bool operator>(const PQNode& o) const { return hval > o.hval; }
    };
    std::priority_queue<PQNode,std::vector<PQNode>,std::greater<PQNode>> pq;
    std::unordered_set<Position> visited;

public:
    std::string getName()      const override { return "Greedy BFS"; }
    bool isFinished()          const override { return finished_; }
    bool foundPath()           const override { return found_;    }
    std::vector<Position> getPath() const override { return path; }

    void initialize(Maze& maze, Position start, Position goal) override {
        pMaze    = &maze;
        goalPos  = goal;
        finished_= found_ = false;
        stats    = SearchStats{};
        path.clear(); parent.clear();
        pq = decltype(pq)(); visited.clear();

        visited.insert(start);
        PQNode n0; n0.hval = h(start); n0.pos = start;
        pq.push(n0);
        maze.at(start).inFrontier = true;
        currentPos = start;
    }

    bool step() override {
        if (finished_) return true;
        if (pq.empty()) {
            finished_ = true; stats.foundPath = false; return true;
        }

        PQNode top = pq.top(); pq.pop();
        float    hval = top.hval;
        Position cur  = top.pos;

        pMaze->at(cur).inFrontier = false;
        pMaze->at(cur).explored   = true;
        currentPos = cur;
        stats.nodesExplored++;
        stats.currentH = hval;

        if (cur == goalPos) {
            reconstructPath(goalPos);
            finished_ = found_ = true;
            return true;
        }

        for (const Position& nb : pMaze->getNeighbors(cur)) {
            if (!visited.count(nb)) {
                visited.insert(nb);
                parent[nb] = cur;
                PQNode nn; nn.hval = h(nb); nn.pos = nb;
                pq.push(nn);
                pMaze->at(nb).inFrontier = true;
            }
        }
        stats.maxFrontier = std::max(stats.maxFrontier,(int)pq.size());
        return false;
    }
};

// ════════════════════════════════════════════════════════════
// AStarAlgorithm  —  A* Search  f(n) = g(n) + h(n)
// ════════════════════════════════════════════════════════════
// g(n): accumulated float traversal cost from start.
//       Reward roads reduce g (cost 0.5), penalty roads raise it.
// h(n): min_cost * Manhattan distance  (admissible lower bound).
//       min_cost = reward_cost = 0.5 ensures h never overestimates.
// Result: A* finds the minimum-cost delivery route.
//         It will prefer reward roads and avoid penalty roads.
// ════════════════════════════════════════════════════════════
class AStarAlgorithm : public SearchAlgorithm {
    struct PQNode {
        float f, g; Position pos;
        bool operator>(const PQNode& o) const {
            if (f != o.f) return f > o.f;
            return g < o.g;  // tie-break: prefer larger g
        }
    };
    std::priority_queue<PQNode,std::vector<PQNode>,std::greater<PQNode>> pq;
    std::unordered_map<Position,float>   gScore;
    std::unordered_set<Position>         closed;

public:
    std::string getName()      const override { return "A*"; }
    bool isFinished()          const override { return finished_; }
    bool foundPath()           const override { return found_;    }
    std::vector<Position> getPath() const override { return path; }

    void initialize(Maze& maze, Position start, Position goal) override {
        pMaze    = &maze;
        goalPos  = goal;
        finished_= found_ = false;
        stats    = SearchStats{};
        path.clear(); parent.clear();
        pq = decltype(pq)(); gScore.clear(); closed.clear();

        float hs = h(start);
        gScore[start] = 0.f;
        PQNode n0; n0.f = hs; n0.g = 0.f; n0.pos = start;
        pq.push(n0);
        maze.at(start).inFrontier = true;
        currentPos = start;
        stats.currentG = 0.f; stats.currentH = hs; stats.currentF = hs;
    }

    bool step() override {
        if (finished_) return true;
        if (pq.empty()) {
            finished_ = true; stats.foundPath = false; return true;
        }

        PQNode top = pq.top(); pq.pop();
        float    f   = top.f;
        float    g   = top.g;
        Position cur = top.pos;

        if (closed.count(cur)) return false;
        closed.insert(cur);

        pMaze->at(cur).inFrontier = false;
        pMaze->at(cur).explored   = true;
        currentPos = cur;
        stats.nodesExplored++;
        stats.currentG = g;
        stats.currentH = h(cur);
        stats.currentF = f;

        if (cur == goalPos) {
            reconstructPath(goalPos);
            finished_ = found_ = true;
            return true;
        }

        for (const Position& nb : pMaze->getNeighbors(cur)) {
            if (closed.count(nb)) continue;
            float edge = std::max(0.1f, pMaze->at(nb).traversalCost());
            float ng   = g + edge;
            if (!gScore.count(nb) || ng < gScore[nb]) {
                gScore[nb] = ng;
                parent[nb] = cur;
                float nf = ng + h(nb);
                PQNode nn; nn.f = nf; nn.g = ng; nn.pos = nb;
                pq.push(nn);
                pMaze->at(nb).inFrontier = true;
            }
        }
        stats.maxFrontier = std::max(stats.maxFrontier,(int)pq.size());
        return false;
    }
};
