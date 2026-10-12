#pragma once
// ============================================================
// Cell.h  —  SmartRoute AI Core Cell Data Structure
// ============================================================
// Terrain costs model road traversal effort (illustrative units).
// Reward roads: cost 0.5  (fast / shortcut road)
// Normal roads: cost 1.0
// Congested:    cost 2.0
// Damaged:      cost 4.0
// Blocked:      impassable
// ============================================================
#include <functional>
#include <cmath>
#include <string>

// ── Direction indices ────────────────────────────────────────
enum Dir : int { TOP=0, RIGHT=1, BOTTOM=2, LEFT=3 };
static constexpr int dRow[4] = {-1,  0,  1,  0};
static constexpr int dCol[4] = { 0,  1,  0, -1};
// Opposite of each direction
static constexpr int opp[4]  = {BOTTOM, LEFT, TOP, RIGHT};

// ── Road / Terrain types ─────────────────────────────────────
enum class TerrainType : int {
    NORMAL    = 0,  // Standard road            cost = 1.0
    CONGESTED = 1,  // Traffic congestion        cost = 2.0
    DAMAGED   = 2,  // Road damage / construction cost = 4.0
    BLOCKED   = 3,  // Impassable road (wall)
    WALL      = 4   // Physical wall/barrier
};

// ── Special road segment overlays ───────────────────────────
enum class SpecialType : int {
    NONE    = 0,
    REWARD  = 1,   // Beneficial shortcut/fast road  cost = 0.5
    PENALTY = 2    // Extra cost on top of terrain   (penalty cost = 4.0 total)
};

inline const char* terrainName(TerrainType t) {
    switch(t){
        case TerrainType::NORMAL:    return "Normal Road";
        case TerrainType::CONGESTED: return "Congested Road";
        case TerrainType::DAMAGED:   return "Damaged Road";
        case TerrainType::BLOCKED:   return "Blocked Road";
        case TerrainType::WALL:      return "Wall";
    }
    return "?";
}

inline const char* specialName(SpecialType s) {
    switch(s){
        case SpecialType::NONE:    return "None";
        case SpecialType::REWARD:  return "Shortcut Road";
        case SpecialType::PENALTY: return "Penalty Road";
    }
    return "?";
}

// ── Position ─────────────────────────────────────────────────
struct Position {
    int row = -1, col = -1;

    Position() = default;
    Position(int r, int c) : row(r), col(c) {}

    bool operator==(const Position& o) const { return row==o.row && col==o.col; }
    bool operator!=(const Position& o) const { return !(*this==o); }
    bool operator< (const Position& o) const {
        return row!=o.row ? row<o.row : col<o.col;
    }

    bool isValid()               const { return row>=0 && col>=0; }
    static Position invalid()          { return {-1,-1}; }

    int manhattan(const Position& o) const {
        return (int)(std::abs(row-o.row) + std::abs(col-o.col));
    }
};

namespace std {
    template<> struct hash<Position> {
        size_t operator()(const Position& p) const noexcept {
            size_t a = (size_t)(p.row >= 0 ? 2*p.row   : -2*p.row-1);
            size_t b = (size_t)(p.col >= 0 ? 2*p.col   : -2*p.col-1);
            return (a >= b) ? (a*a + a + b) : (a + b*b);
        }
    };
}

// ── Cost configuration (configurable at runtime) ─────────────
struct CostConfig {
    float normalCost    = 1.0f;
    float congestedCost = 2.0f;
    float damagedCost   = 4.0f;
    float rewardCost    = 0.5f;   // fast/shortcut road
    float penaltyCost   = 4.0f;   // total cost for penalty road

    static CostConfig& get() {
        static CostConfig inst;
        return inst;
    }
};

// ── Cell ─────────────────────────────────────────────────────
struct Cell {
    // Walls: TOP, RIGHT, BOTTOM, LEFT (true = wall present)
    bool walls[4] = {true, true, true, true};

    TerrainType terrain = TerrainType::NORMAL;
    SpecialType special = SpecialType::NONE;

    // ── Visualization / algorithm state flags ────────────────
    bool genVisited  = false; // visited during maze generation
    bool explored    = false; // expanded by search algorithm
    bool inFrontier  = false; // sitting in search frontier
    bool inPath      = false; // on the final solution path
    bool specialUsed = false; // reward/penalty collected in play mode

    // ── Traversal cost (float, for weighted algorithms) ──────
    // Returns the traversal cost for entering this cell.
    // Reward special overrides terrain → 0.5 (fast road).
    // Penalty special forces cost = penaltyCost.
    float traversalCost() const {
        const CostConfig& cfg = CostConfig::get();
        if (special == SpecialType::REWARD)  return cfg.rewardCost;
        if (special == SpecialType::PENALTY) return cfg.penaltyCost;
        switch(terrain){
            case TerrainType::NORMAL:    return cfg.normalCost;
            case TerrainType::CONGESTED: return cfg.congestedCost;
            case TerrainType::DAMAGED:   return cfg.damagedCost;
            case TerrainType::BLOCKED:
            case TerrainType::WALL:      return 99999.0f;
        }
        return cfg.normalCost;
    }

    // Legacy int cost (kept for compatibility)
    int terrainCost() const {
        return (int)traversalCost();
    }

    bool isWall() const {
        return terrain == TerrainType::WALL || terrain == TerrainType::BLOCKED;
    }

    // ── State resets ─────────────────────────────────────────
    void resetSearchState() {
        explored = inFrontier = inPath = false;
    }

    void resetAllState() {
        genVisited = explored = inFrontier = inPath = specialUsed = false;
    }

    void fullReset() {
        resetAllState();
        for(int i=0;i<4;i++) walls[i] = true;
        terrain = TerrainType::NORMAL;
        special = SpecialType::NONE;
    }
};
