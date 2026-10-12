#pragma once
// ============================================================
// Maze.h  —  SmartRoute AI Road Network Grid
// ============================================================
#include "Cell.h"
#include <vector>
#include <string>
#include <algorithm>

// ── Customer Destination Representation ──────────────────────
struct Customer {
    int id = 1;            // Unique, stable identifier (1, 2, 3...)
    Position pos{0, 0};    // Grid cell location
    bool active = true;    // Included in multi-stop delivery

    std::string label() const { return "C" + std::to_string(id); }
};

class Maze {
public:
    int rows = 20, cols = 20;
    std::vector<std::vector<Cell>> grid;
    Position start{0, 0};
    Position goal{19, 19};
    bool generated = false;
    unsigned int seed = 0;

    // Customer destinations (1 Store -> Multiple Customers)
    std::vector<Customer> customers;
    int nextCustomerId = 1;
    int selectedCustomerId = 1;

    // ── Construction ─────────────────────────────────────────
    Maze(int r=20, int c=20) { resize(r, c); }

    void resize(int r, int c) {
        rows = r;  cols = c;
        start = {0, 0};
        goal  = {r-1, c-1};
        grid.assign(r, std::vector<Cell>(c));
        generated = false;

        // Reset customers with default C1 at goal
        customers.clear();
        nextCustomerId = 1;
        selectedCustomerId = 1;
        customers.push_back({1, {r-1, c-1}, true});
        nextCustomerId = 2;
    }

    // ── Cell access ──────────────────────────────────────────
    Cell&       at(int r, int c)       { return grid[r][c]; }
    const Cell& at(int r, int c) const { return grid[r][c]; }
    Cell&       at(Position p)         { return grid[p.row][p.col]; }
    const Cell& at(Position p)   const { return grid[p.row][p.col]; }

    bool inBounds(int r, int c) const {
        return r>=0 && r<rows && c>=0 && c<cols;
    }
    bool inBounds(Position p) const { return inBounds(p.row, p.col); }

    // ── Customer Management ──────────────────────────────────
    const Customer* getCustomer(int id) const {
        for (const auto& c : customers) {
            if (c.id == id) return &c;
        }
        return nullptr;
    }

    Customer* getCustomer(int id) {
        for (auto& c : customers) {
            if (c.id == id) return &c;
        }
        return nullptr;
    }

    bool hasCustomerAt(Position p) const {
        for (const auto& c : customers) {
            if (c.pos == p) return true;
        }
        return false;
    }

    int getCustomerIdAt(Position p) const {
        for (const auto& c : customers) {
            if (c.pos == p) return c.id;
        }
        return -1;
    }

    int addCustomer(Position p) {
        if (!inBounds(p)) return -1;
        if (p == start) return -1;
        if (grid[p.row][p.col].isWall()) return -1;
        if (hasCustomerAt(p)) return -1;
        if (customers.size() >= 10) return -1;

        int newId = nextCustomerId++;
        customers.push_back({newId, p, true});
        selectedCustomerId = newId;
        goal = p;
        return newId;
    }

    bool removeCustomer(int id) {
        auto it = std::find_if(customers.begin(), customers.end(), [id](const Customer& c){
            return c.id == id;
        });
        if (it == customers.end()) return false;
        customers.erase(it);

        if (selectedCustomerId == id) {
            if (!customers.empty()) {
                selectedCustomerId = customers.front().id;
                goal = customers.front().pos;
            } else {
                selectedCustomerId = -1;
                goal = Position::invalid();
            }
        }
        return true;
    }

    bool moveCustomer(int id, Position newPos) {
        if (!inBounds(newPos)) return false;
        if (newPos == start) return false;
        if (grid[newPos.row][newPos.col].isWall()) return false;
        for (const auto& c : customers) {
            if (c.id != id && c.pos == newPos) return false;
        }
        Customer* cust = getCustomer(id);
        if (!cust) return false;
        cust->pos = newPos;
        if (selectedCustomerId == id) {
            goal = newPos;
        }
        return true;
    }

    // ── Wall operations ──────────────────────────────────────
    void removeWall(Position a, Position b) {
        if(!inBounds(a) || !inBounds(b)) return;
        int dr = b.row-a.row, dc = b.col-a.col;
        int d = -1;
        if(dr==-1) d=TOP; else if(dr==1) d=BOTTOM;
        else if(dc==-1) d=LEFT; else if(dc==1) d=RIGHT;
        if(d < 0) return;
        grid[a.row][a.col].walls[d]      = false;
        grid[b.row][b.col].walls[opp[d]] = false;
    }

    void addWallBetween(Position a, Position b) {
        if(!inBounds(a) || !inBounds(b)) return;
        int dr = b.row-a.row, dc = b.col-a.col;
        int d = -1;
        if(dr==-1) d=TOP; else if(dr==1) d=BOTTOM;
        else if(dc==-1) d=LEFT; else if(dc==1) d=RIGHT;
        if(d < 0) return;
        grid[a.row][a.col].walls[d]      = true;
        grid[b.row][b.col].walls[opp[d]] = true;
    }

    // Block a cell: set terrain=BLOCKED, seal all surrounding walls
    void blockCell(Position p) {
        if(!inBounds(p)) return;
        grid[p.row][p.col].terrain = TerrainType::BLOCKED;
        for(int d=0; d<4; d++) {
            grid[p.row][p.col].walls[d] = true;
            int nr=p.row+dRow[d], nc=p.col+dCol[d];
            if(inBounds(nr,nc))
                grid[nr][nc].walls[opp[d]] = true;
        }
    }

    // Unblock a cell: set terrain=NORMAL, open walls toward non-wall neighbours
    void unblockCell(Position p) {
        if(!inBounds(p)) return;
        grid[p.row][p.col].terrain = TerrainType::NORMAL;
        grid[p.row][p.col].special = SpecialType::NONE;
        for(int d=0; d<4; d++) {
            int nr=p.row+dRow[d], nc=p.col+dCol[d];
            if(inBounds(nr,nc) && !grid[nr][nc].isWall()) {
                grid[p.row][p.col].walls[d]      = false;
                grid[nr][nc].walls[opp[d]]        = false;
            }
        }
    }

    // ── Neighbour queries ────────────────────────────────────
    std::vector<Position> getNeighbors(Position p) const {
        std::vector<Position> res;
        if(!inBounds(p)) return res;
        const Cell& cell = grid[p.row][p.col];
        for(int d=0; d<4; d++) {
            if(!cell.walls[d]) {
                int nr=p.row+dRow[d], nc=p.col+dCol[d];
                if(inBounds(nr,nc) && !grid[nr][nc].isWall())
                    res.push_back({nr,nc});
            }
        }
        return res;
    }

    std::vector<Position> getAllAdjacent(Position p) const {
        std::vector<Position> res;
        for(int d=0; d<4; d++) {
            int nr=p.row+dRow[d], nc=p.col+dCol[d];
            if(inBounds(nr,nc)) res.push_back({nr,nc});
        }
        return res;
    }

    std::vector<Position> getUnvisitedAdjacent(Position p) const {
        std::vector<Position> res;
        for(int d=0; d<4; d++) {
            int nr=p.row+dRow[d], nc=p.col+dCol[d];
            if(inBounds(nr,nc) && !grid[nr][nc].genVisited)
                res.push_back({nr,nc});
        }
        return res;
    }

    // ── State management ─────────────────────────────────────
    void resetSearch() {
        for(auto& row : grid)
            for(auto& c : row)
                c.resetSearchState();
    }

    void resetAllFlags() {
        for(auto& row : grid)
            for(auto& c : row)
                c.resetAllState();
    }

    void hardReset() {
        for(auto& row : grid)
            for(auto& c : row)
                c.fullReset();
        start = {0,0};
        goal  = {rows-1, cols-1};
        generated = false;
        customers.clear();
        nextCustomerId = 1;
        selectedCustomerId = 1;
        customers.push_back({1, {rows-1, cols-1}, true});
        nextCustomerId = 2;
    }

    // ── Path marking & stats ─────────────────────────────────
    void markPath(const std::vector<Position>& path) {
        for(const auto& p : path)
            if(inBounds(p)) grid[p.row][p.col].inPath = true;
    }

    void clearPath() {
        for(auto& row : grid)
            for(auto& c : row)
                c.inPath = false;
    }

    float pathCostWeighted() const {
        float cost = 0.f;
        bool hasPath = false;
        for(const auto& row : grid) {
            for(const auto& c : row) {
                if(c.inPath) {
                    cost += c.traversalCost();
                    hasPath = true;
                }
            }
        }
        if(hasPath && inBounds(start) && grid[start.row][start.col].inPath) {
            cost -= grid[start.row][start.col].traversalCost();
        }
        return std::max(0.0f, cost);
    }

    float pathCostWeighted(const std::vector<Position>& p) const {
        if(p.size() <= 1) return 0.0f;
        float cost = 0.0f;
        for(size_t i = 1; i < p.size(); ++i) {
            if(inBounds(p[i])) {
                cost += grid[p[i].row][p[i].col].traversalCost();
            }
        }
        return cost;
    }

    int pathCost() const { return (int)pathCostWeighted(); }

    int pathLength() const {
        int len = 0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.inPath) len++;
        return len;
    }

    int routeMoves() const {
        int len = pathLength();
        return len > 0 ? len - 1 : 0;
    }

    int countRewardsOnPath() const {
        int n=0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.inPath && c.special==SpecialType::REWARD) n++;
        return n;
    }

    int countPenaltiesOnPath() const {
        int n=0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.inPath && c.special==SpecialType::PENALTY) n++;
        return n;
    }

    int countRewardPellets() const {
        int n=0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.special==SpecialType::REWARD) n++;
        return n;
    }

    int countPenaltyPellets() const {
        int n=0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.special==SpecialType::PENALTY) n++;
        return n;
    }

    int countCongestedCells() const {
        int n=0;
        for(const auto& row : grid)
            for(const auto& c : row)
                if(c.terrain==TerrainType::CONGESTED && c.special==SpecialType::NONE) n++;
        return n;
    }

    void clearAllSpecialPellets() {
        for(auto& row : grid) {
            for(auto& c : row) {
                c.special = SpecialType::NONE;
                if(!c.isWall() && (c.terrain == TerrainType::CONGESTED || c.terrain == TerrainType::DAMAGED)) {
                    c.terrain = TerrainType::NORMAL;
                }
            }
        }
    }
};
