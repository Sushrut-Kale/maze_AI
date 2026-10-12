#pragma once
// ============================================================
// Generators.h  —  MAZEAI Maze Generation Algorithms
//   1. DFSGenerator    — Randomized Depth-First Search
//   2. PrimGenerator   — Randomized Prim's Algorithm
//   3. KruskalGenerator— Randomized Kruskal's Algorithm
// ============================================================
#include "Maze.h"
#include <random>
#include <stack>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>

// ════════════════════════════════════════════════════════════
// MazeGenerator — Abstract base class
// ════════════════════════════════════════════════════════════
class MazeGenerator {
public:
    Position currentCell{0, 0};
    bool     finished = false;

    virtual void        initialize(Maze& maze, unsigned int seed) = 0;
    virtual bool        step()     = 0;   // true when done
    virtual std::string getName()  const = 0;
    virtual ~MazeGenerator()             = default;

    bool isFinished() const { return finished; }

    void runToCompletion() {
        while (!finished) step();
    }

protected:
    Maze*       pMaze = nullptr;
    std::mt19937 rng;
};

// ════════════════════════════════════════════════════════════
// DFSGenerator — Randomized Depth-First Search (iterative)
//   Produces "long winding" perfect mazes.
// ════════════════════════════════════════════════════════════
class DFSGenerator : public MazeGenerator {
    std::stack<Position> stack;

public:
    std::string getName() const override { return "Randomized DFS"; }

    void initialize(Maze& maze, unsigned int seed) override {
        pMaze    = &maze;
        finished = false;
        rng.seed(seed);

        // Clear stack
        stack = std::stack<Position>();

        // Begin at (0,0)
        currentCell = {0, 0};
        pMaze->at(currentCell).genVisited = true;
        stack.push(currentCell);
    }

    bool step() override {
        if (finished) return true;

        if (stack.empty()) {
            finished             = true;
            pMaze->generated     = true;
            return true;
        }

        currentCell = stack.top();

        auto unvisited = pMaze->getUnvisitedAdjacent(currentCell);

        if (!unvisited.empty()) {
            // Choose random unvisited neighbour
            std::uniform_int_distribution<int> dist(0, (int)unvisited.size()-1);
            Position next = unvisited[dist(rng)];

            pMaze->removeWall(currentCell, next);
            pMaze->at(next).genVisited = true;
            stack.push(next);
            currentCell = next;
        } else {
            // Backtrack
            stack.pop();
            if (!stack.empty())
                currentCell = stack.top();
        }
        return false;
    }
};

// ════════════════════════════════════════════════════════════
// PrimGenerator — Randomized Prim's Algorithm
//   Grows the maze outward from a random seed cell.
//   Produces mazes with many short dead-ends.
// ════════════════════════════════════════════════════════════
class PrimGenerator : public MazeGenerator {
    // Each entry: (in-tree cell, candidate cell)
    std::vector<std::pair<Position,Position>> frontier;

public:
    std::string getName() const override { return "Randomized Prim"; }

    void initialize(Maze& maze, unsigned int seed) override {
        pMaze    = &maze;
        finished = false;
        rng.seed(seed);
        frontier.clear();

        // Start from a random cell
        std::uniform_int_distribution<int> rDist(0, maze.rows-1);
        std::uniform_int_distribution<int> cDist(0, maze.cols-1);
        currentCell = {rDist(rng), cDist(rng)};
        pMaze->at(currentCell).genVisited = true;
        addFrontier(currentCell);
    }

    bool step() override {
        if (finished) return true;

        if (frontier.empty()) {
            finished         = true;
            pMaze->generated = true;
            return true;
        }

        // Pick random frontier edge
        std::uniform_int_distribution<int> dist(0, (int)frontier.size()-1);
        int idx = dist(rng);
        Position from = frontier[idx].first;
        Position to   = frontier[idx].second;
        frontier.erase(frontier.begin() + idx);

        if (!pMaze->at(to).genVisited) {
            pMaze->removeWall(from, to);
            pMaze->at(to).genVisited = true;
            currentCell = to;
            addFrontier(to);
        }
        return false;
    }

private:
    void addFrontier(Position p) {
        for (int d = 0; d < 4; d++) {
            int nr = p.row + dRow[d], nc = p.col + dCol[d];
            if (pMaze->inBounds(nr, nc) && !pMaze->at(nr, nc).genVisited)
                frontier.push_back({p, {nr, nc}});
        }
    }
};

// ════════════════════════════════════════════════════════════
// KruskalGenerator — Randomized Kruskal's Algorithm
//   Uses Union-Find over a randomly ordered wall list.
//   Produces uniformly random spanning trees.
// ════════════════════════════════════════════════════════════
class KruskalGenerator : public MazeGenerator {
    // Union-Find
    std::vector<int> ufParent, ufRank;

    int find(int x) {
        if (ufParent[x] != x) ufParent[x] = find(ufParent[x]);
        return ufParent[x];
    }
    bool unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return false;
        if (ufRank[a] < ufRank[b]) std::swap(a, b);
        ufParent[b] = a;
        if (ufRank[a] == ufRank[b]) ufRank[a]++;
        return true;
    }
    int cellId(Position p) const { return p.row * pMaze->cols + p.col; }

    // All edges (pairs of adjacent cells)
    std::vector<std::pair<Position,Position>> edges;
    int edgeIdx = 0;

public:
    std::string getName() const override { return "Randomized Kruskal"; }

    void initialize(Maze& maze, unsigned int seed) override {
        pMaze    = &maze;
        finished = false;
        edgeIdx  = 0;
        rng.seed(seed);
        edges.clear();

        int N = maze.rows * maze.cols;
        ufParent.resize(N); ufRank.assign(N, 0);
        std::iota(ufParent.begin(), ufParent.end(), 0);

        // Enumerate all internal edges
        for (int r = 0; r < maze.rows; r++) {
            for (int c = 0; c < maze.cols; c++) {
                if (c+1 < maze.cols) edges.push_back({{r,c},{r,c+1}});
                if (r+1 < maze.rows) edges.push_back({{r,c},{r+1,c}});
            }
        }
        std::shuffle(edges.begin(), edges.end(), rng);

        currentCell = {0, 0};
    }

    bool step() override {
        if (finished) return true;

        // Process edges until one gets accepted or list exhausted
        while (edgeIdx < (int)edges.size()) {
            Position a = edges[edgeIdx].first;
            Position b = edges[edgeIdx].second;
            edgeIdx++;
            if (unite(cellId(a), cellId(b))) {
                pMaze->removeWall(a, b);
                pMaze->at(a).genVisited = true;
                pMaze->at(b).genVisited = true;
                currentCell = b;
                return false;   // accepted one edge — yield
            }
        }

        // All edges processed
        // Mark all cells visited for display consistency
        for (int r = 0; r < pMaze->rows; r++)
            for (int c = 0; c < pMaze->cols; c++)
                pMaze->at(r,c).genVisited = true;

        finished         = true;
        pMaze->generated = true;
        return true;
    }
};
