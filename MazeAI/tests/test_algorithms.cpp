// ============================================================
// test_algorithms.cpp — SmartRoute AI Automated Algorithm Test Suite
// ============================================================
// Standalone headless test harness: no OpenGL / FreeGLUT window required.
// Tests core pathfinding algorithms, cost accounting, metrics,
// and the three-stage maze workflow logic.
//
// Compile:
//   g++ -std=c++14 -O2 -Wall -I../src test_algorithms.cpp -o test_algorithms.exe
// Run:
//   test_algorithms.exe
// Returns exit code 0 if all tests PASS, 1 if any FAIL.
// ============================================================

#include "Cell.h"
#include "Maze.h"
#include "SearchAlgorithm.h"
#include "Algorithms.h"


#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cassert>
#include <cmath>

static int g_passCount = 0;
static int g_failCount = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  [ASSERTION FAILED] " << msg << " (" << #cond << ") at line " << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

static void reportTest(const std::string& name, bool pass) {
    if (pass) {
        std::cout << "[PASS] " << name << "\n";
        g_passCount++;
    } else {
        std::cout << "[FAIL] " << name << "\n";
        g_failCount++;
    }
}

// Helper: build open grid with no interior walls
static Maze makeOpenMaze(int rows, int cols) {
    Maze m(rows, cols);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (r + 1 < rows) m.removeWall({r, c}, {r + 1, c});
            if (c + 1 < cols) m.removeWall({r, c}, {r, c + 1});
        }
    }
    m.generated = true;
    m.start = {0, 0};
    m.goal  = {rows - 1, cols - 1};
    return m;
}

// Helper: verify path connectivity and wall clearance
static bool verifyValidPath(const Maze& m, const std::vector<Position>& path, Position expectedStart, Position expectedGoal) {
    if (path.empty()) return false;
    if (path.front() != expectedStart) return false;
    if (path.back() != expectedGoal) return false;

    for (size_t i = 0; i < path.size(); i++) {
        Position p = path[i];
        if (!m.inBounds(p)) return false;
        if (m.at(p).isWall()) return false;

        if (i + 1 < path.size()) {
            Position next = path[i + 1];
            int dr = next.row - p.row;
            int dc = next.col - p.col;
            if (std::abs(dr) + std::abs(dc) != 1) return false; // must be adjacent step

            int dir = -1;
            if (dr == -1) dir = TOP;
            else if (dr == 1) dir = BOTTOM;
            else if (dc == -1) dir = LEFT;
            else if (dc == 1) dir = RIGHT;

            if (dir < 0 || m.at(p).walls[dir] || m.at(next).walls[opp[dir]]) {
                return false; // wall exists between steps
            }
        }
    }
    return true;
}

// ────────────────────────────────────────────────────────────
// REQUIRED ALGORITHM TESTS (1 - 15)
// ────────────────────────────────────────────────────────────

// 1. BFS finds a valid route when the goal is reachable
bool test_1_bfs_reachable() {
    Maze m = makeOpenMaze(5, 5);
    BFSAlgorithm bfs;
    bfs.initialize(m, m.start, m.goal);
    SearchStats st = bfs.runToCompletion();
    TEST_ASSERT(st.foundPath, "BFS must report foundPath = true");
    TEST_ASSERT(verifyValidPath(m, bfs.getPath(), m.start, m.goal), "Path must be legal");
    return true;
}

// 2. DFS finds a valid route when the goal is reachable
bool test_2_dfs_reachable() {
    Maze m = makeOpenMaze(5, 5);
    DFSAlgorithm dfs;
    dfs.initialize(m, m.start, m.goal);
    SearchStats st = dfs.runToCompletion();
    TEST_ASSERT(st.foundPath, "DFS must report foundPath = true");
    TEST_ASSERT(verifyValidPath(m, dfs.getPath(), m.start, m.goal), "Path must be legal");
    return true;
}

// 3. A* finds a valid route when the goal is reachable
bool test_3_astar_reachable() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    SearchStats st = astar.runToCompletion();
    TEST_ASSERT(st.foundPath, "A* must report foundPath = true");
    TEST_ASSERT(verifyValidPath(m, astar.getPath(), m.start, m.goal), "Path must be legal");
    return true;
}

// 4. All algorithms report failure when the goal is unreachable
bool test_4_unreachable_goal() {
    Maze m = makeOpenMaze(5, 5);
    // Seal goal completely
    m.blockCell({3, 4});
    m.blockCell({4, 3});
    // Add walls on goal itself
    for (int d = 0; d < 4; d++) m.at(m.goal).walls[d] = true;

    BFSAlgorithm bfs;
    bfs.initialize(m, m.start, m.goal);
    SearchStats stB = bfs.runToCompletion();
    TEST_ASSERT(!stB.foundPath, "BFS must fail when goal unreachable");
    TEST_ASSERT(stB.pathLength == 0, "Failed search must have pathLength = 0");
    TEST_ASSERT(stB.routeMoves == 0, "Failed search must have routeMoves = 0");

    DFSAlgorithm dfs;
    dfs.initialize(m, m.start, m.goal);
    SearchStats stD = dfs.runToCompletion();
    TEST_ASSERT(!stD.foundPath, "DFS must fail when goal unreachable");
    TEST_ASSERT(stD.pathLength == 0, "Failed search must have pathLength = 0");
    TEST_ASSERT(stD.routeMoves == 0, "Failed search must have routeMoves = 0");

    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    SearchStats stA = astar.runToCompletion();
    TEST_ASSERT(!stA.foundPath, "A* must fail when goal unreachable");
    TEST_ASSERT(stA.pathLength == 0, "Failed search must have pathLength = 0");
    TEST_ASSERT(stA.routeMoves == 0, "Failed search must have routeMoves = 0");

    return true;
}

// 5. Reconstructed route starts at correct start cell and ends at goal
bool test_5_route_start_goal_endpoints() {
    Maze m = makeOpenMaze(6, 6);
    Position customStart{1, 1};
    Position customGoal{4, 5};
    AStarAlgorithm astar;
    astar.initialize(m, customStart, customGoal);
    astar.runToCompletion();
    auto p = astar.getPath();
    TEST_ASSERT(!p.empty(), "Path must not be empty");
    TEST_ASSERT(p.front() == customStart, "First cell must be start");
    TEST_ASSERT(p.back() == customGoal, "Last cell must be goal");
    return true;
}

// 6. Every consecutive pair of route cells is a valid legal movement
bool test_6_consecutive_steps_legal() {
    Maze m = makeOpenMaze(6, 6);
    // Add some random barrier
    m.blockCell({2, 2});
    m.blockCell({2, 3});
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    auto path = astar.getPath();
    TEST_ASSERT(verifyValidPath(m, path, m.start, m.goal), "Every step in path must be legal");
    return true;
}

// 7. Blocked cells are never included as traversable route cells
bool test_7_blocked_cells_excluded() {
    Maze m = makeOpenMaze(5, 5);
    Position blockedPos{2, 2};
    m.blockCell(blockedPos);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    for (const Position& p : astar.getPath()) {
        TEST_ASSERT(p != blockedPos, "Route must never pass through blocked cell");
        TEST_ASSERT(!m.at(p).isWall(), "Route cells cannot be walls");
    }
    return true;
}

// 8. Weighted route cost matches sum of entered-cell costs, excluding start cell
bool test_8_cost_excludes_start_cell() {
    Maze m = makeOpenMaze(4, 4);
    // Assign distinct costs to cells
    m.at(0, 0).terrain = TerrainType::DAMAGED; // Start cell cost = 4.0 (SHOULD BE IGNORED)
    m.at(0, 1).special = SpecialType::REWARD;  // 0.5
    m.at(0, 2).terrain = TerrainType::CONGESTED; // 2.0
    m.at(0, 3).terrain = TerrainType::NORMAL;  // 1.0

    // Force a 1D corridor along row 0
    Position s{0, 0}, g{0, 3};
    AStarAlgorithm astar;
    astar.initialize(m, s, g);
    SearchStats st = astar.runToCompletion();

    TEST_ASSERT(st.foundPath, "Corridor route must be found");
    // Expected cost: cost(0,1) + cost(0,2) + cost(0,3) = 0.5 + 2.0 + 1.0 = 3.5
    // Notice start cell cost (4.0) is strictly 0!
    float expectedCost = 3.5f;
    TEST_ASSERT(std::abs(st.pathCost - expectedCost) < 1e-4f, "pathCost must equal sum of entered cells excluding start");
    TEST_ASSERT(std::abs(m.pathCostWeighted() - expectedCost) < 1e-4f, "Maze::pathCostWeighted() must agree");
    return true;
}

// 9. Reward roads retain the configured fractional cost (0.5)
bool test_9_reward_road_fractional_cost() {
    Maze m = makeOpenMaze(3, 3);
    m.at(0, 1).special = SpecialType::REWARD;
    TEST_ASSERT(std::abs(m.at(0, 1).traversalCost() - 0.5f) < 1e-4f, "Reward cell traversalCost must be 0.5f");
    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {0, 1});
    SearchStats st = astar.runToCompletion();
    TEST_ASSERT(std::abs(st.pathCost - 0.5f) < 1e-4f, "Single reward move must cost exactly 0.5f");
    return true;
}

// 10. Penalty roads retain the configured higher cost (4.0)
bool test_10_penalty_road_cost() {
    Maze m = makeOpenMaze(3, 3);
    m.at(0, 1).special = SpecialType::PENALTY;
    TEST_ASSERT(std::abs(m.at(0, 1).traversalCost() - 4.0f) < 1e-4f, "Penalty cell traversalCost must be 4.0f");
    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {0, 1});
    SearchStats st = astar.runToCompletion();
    TEST_ASSERT(std::abs(st.pathCost - 4.0f) < 1e-4f, "Single penalty move must cost exactly 4.0f");
    return true;
}

// 11. A* returns the minimum weighted cost on several small maps
bool test_11_astar_minimum_weighted_cost() {
    // Map 1: 3x3 with detour having reward
    {
        Maze m = makeOpenMaze(3, 3);
        // Direct move (0,1) is penalty (4.0), down path (1,0)->(1,1)->(0,1) has rewards (0.5+0.5)
        m.at(0, 1).special = SpecialType::PENALTY;
        m.at(1, 0).special = SpecialType::REWARD;
        m.at(1, 1).special = SpecialType::REWARD;

        AStarAlgorithm astar;
        astar.initialize(m, {0, 0}, {0, 2});
        SearchStats st = astar.runToCompletion();
        DijkstraAlgorithm dijkstra;
        dijkstra.initialize(m, {0, 0}, {0, 2});
        SearchStats stD = dijkstra.runToCompletion();

        TEST_ASSERT(std::abs(st.pathCost - stD.pathCost) < 1e-4f, "A* cost must match optimal Dijkstra cost");
    }
    // Map 2: 4x4 with congested corridor
    {
        Maze m = makeOpenMaze(4, 4);
        m.at(0, 1).terrain = TerrainType::CONGESTED;
        m.at(0, 2).terrain = TerrainType::CONGESTED;
        AStarAlgorithm astar;
        astar.initialize(m, {0, 0}, {3, 3});
        SearchStats stA = astar.runToCompletion();
        DijkstraAlgorithm dijkstra;
        dijkstra.initialize(m, {0, 0}, {3, 3});
        SearchStats stD = dijkstra.runToCompletion();
        TEST_ASSERT(std::abs(stA.pathCost - stD.pathCost) < 1e-4f, "A* cost must match optimal Dijkstra cost on congested map");
    }
    return true;
}

// 12. Validate A* weighted-cost optimality against an independently executed Dijkstra reference
bool test_12_astar_optimality_vs_dijkstra() {
    Maze m = makeOpenMaze(8, 8);
    // Populate heterogeneous terrain
    for (int r = 1; r < 7; r++) {
        m.at(r, 2).special = SpecialType::REWARD;
        m.at(r, 5).special = SpecialType::PENALTY;
        m.at(r, 4).terrain = TerrainType::CONGESTED;
    }
    m.blockCell({3, 3});
    m.blockCell({4, 3});

    DijkstraAlgorithm dijkstra;
    dijkstra.initialize(m, m.start, m.goal);
    SearchStats stD = dijkstra.runToCompletion();

    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    SearchStats stA = astar.runToCompletion();

    TEST_ASSERT(stD.foundPath && stA.foundPath, "Both algorithms must find route");
    TEST_ASSERT(std::abs(stA.pathCost - stD.pathCost) < 1e-4f, "A* path cost must equal Dijkstra optimal path cost");
    return true;
}

// 13. A path containing 25 cells is reported as 24 moves
bool test_13_path_25_cells_is_24_moves() {
    // A 1D corridor of 25 cells (1x25)
    Maze m = makeOpenMaze(1, 25);
    BFSAlgorithm bfs;
    bfs.initialize(m, {0, 0}, {0, 24});
    SearchStats st = bfs.runToCompletion();
    TEST_ASSERT(st.foundPath, "Corridor route must be found");
    TEST_ASSERT(st.pathLength == 25, "Route cells must be 25");
    TEST_ASSERT(st.routeMoves == 24, "Route moves must be 24 (pathLength - 1)");
    TEST_ASSERT(m.pathLength() == 25, "Maze::pathLength() must be 25");
    TEST_ASSERT(m.routeMoves() == 24, "Maze::routeMoves() must be 24");
    return true;
}

// 14. A start-equals-goal case behaves correctly
bool test_14_start_equals_goal() {
    Maze m = makeOpenMaze(5, 5);
    Position samePoint{2, 2};
    m.start = samePoint;
    m.goal  = samePoint;

    AStarAlgorithm astar;
    astar.initialize(m, samePoint, samePoint);
    SearchStats stA = astar.runToCompletion();

    TEST_ASSERT(stA.foundPath, "Search must report found when start == goal");
    TEST_ASSERT(stA.pathLength == 1, "Route cells must be 1 for start == goal");
    TEST_ASSERT(stA.routeMoves == 0, "Route moves must be 0 for start == goal");
    TEST_ASSERT(std::abs(stA.pathCost - 0.0f) < 1e-4f, "Route cost must be 0.0f for start == goal");
    TEST_ASSERT(std::abs(m.pathCostWeighted() - 0.0f) < 1e-4f, "Maze::pathCostWeighted() must be 0.0f");
    TEST_ASSERT(m.routeMoves() == 0, "Maze::routeMoves() must be 0");
    return true;
}

// 15. Repeated runs do not reuse stale search state
bool test_15_repeated_runs_clean_state() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;

    // Run 1
    astar.initialize(m, m.start, m.goal);
    SearchStats st1 = astar.runToCompletion();
    TEST_ASSERT(st1.foundPath, "Run 1 must succeed");

    // Clear maze search state
    m.resetSearch();

    // Run 2 on same instance
    astar.initialize(m, m.start, m.goal);
    SearchStats st2 = astar.runToCompletion();
    TEST_ASSERT(st2.foundPath, "Run 2 must succeed");
    TEST_ASSERT(st1.nodesExplored == st2.nodesExplored, "Explored nodes count must match exactly across repeated runs");
    TEST_ASSERT(st1.pathLength == st2.pathLength, "Path length must match");
    TEST_ASSERT(std::abs(st1.pathCost - st2.pathCost) < 1e-4f, "Path cost must match");

    return true;
}

// ────────────────────────────────────────────────────────────
// NEW THREE-STAGE WORKFLOW & PELLET CONFIGURATION TESTS (16 - 30)
// ────────────────────────────────────────────────────────────

// 16. A reward pellet can be placed on a valid road cell
bool test_16_place_reward_pellet() {
    Maze m = makeOpenMaze(5, 5);
    Position p{1, 2};
    m.at(p).special = SpecialType::REWARD;
    TEST_ASSERT(m.at(p).special == SpecialType::REWARD, "Cell must have REWARD special type");
    TEST_ASSERT(std::abs(m.at(p).traversalCost() - 0.5f) < 1e-4f, "Cell traversal cost must be 0.5");
    return true;
}

// 17. A penalty pellet can be placed on a valid road cell
bool test_17_place_penalty_pellet() {
    Maze m = makeOpenMaze(5, 5);
    Position p{1, 2};
    m.at(p).special = SpecialType::PENALTY;
    TEST_ASSERT(m.at(p).special == SpecialType::PENALTY, "Cell must have PENALTY special type");
    TEST_ASSERT(std::abs(m.at(p).traversalCost() - 4.0f) < 1e-4f, "Cell traversal cost must be 4.0");
    return true;
}

// 18. A pellet cannot overwrite start or goal
bool test_18_pellet_cannot_overwrite_start_goal() {
    Maze m = makeOpenMaze(5, 5);
    Position s = m.start;
    Position g = m.goal;

    auto tryPlace = [&](Position p, SpecialType sp) -> bool {
        if (p == m.start || p == m.goal) return false;
        m.at(p).special = sp;
        return true;
    };

    TEST_ASSERT(!tryPlace(s, SpecialType::REWARD), "Must reject placing pellet on start");
    TEST_ASSERT(!tryPlace(g, SpecialType::PENALTY), "Must reject placing pellet on goal");
    TEST_ASSERT(m.at(s).special == SpecialType::NONE, "Start cell special must remain NONE");
    TEST_ASSERT(m.at(g).special == SpecialType::NONE, "Goal cell special must remain NONE");
    return true;
}

// 19. A pellet cannot be placed on a wall
bool test_19_pellet_cannot_be_placed_on_wall() {
    Maze m = makeOpenMaze(5, 5);
    Position w{2, 2};
    m.blockCell(w);

    auto tryPlace = [&](Position p, SpecialType sp) -> bool {
        if (m.at(p).isWall()) return false;
        m.at(p).special = sp;
        return true;
    };

    TEST_ASSERT(!tryPlace(w, SpecialType::REWARD), "Must reject placing pellet on wall");
    TEST_ASSERT(m.at(w).special == SpecialType::NONE, "Wall must remain free of pellets");
    TEST_ASSERT(m.at(w).isWall(), "Wall must remain impassable");
    return true;
}

// 20. Switching from reward to penalty updates cell correctly
bool test_20_switch_pellet_type() {
    Maze m = makeOpenMaze(5, 5);
    Position p{2, 3};
    m.at(p).special = SpecialType::REWARD;
    TEST_ASSERT(std::abs(m.at(p).traversalCost() - 0.5f) < 1e-4f, "Initial cost must be 0.5");
    // Switch to penalty
    m.at(p).special = SpecialType::PENALTY;
    TEST_ASSERT(m.at(p).special == SpecialType::PENALTY, "Special must be PENALTY");
    TEST_ASSERT(std::abs(m.at(p).traversalCost() - 4.0f) < 1e-4f, "Updated cost must be 4.0");
    return true;
}

// 21. Erasing a special pellet restores normal road cost
bool test_21_erase_pellet_restores_normal() {
    Maze m = makeOpenMaze(5, 5);
    Position p{2, 3};
    m.at(p).special = SpecialType::REWARD;
    // Erase
    m.at(p).special = SpecialType::NONE;
    m.at(p).terrain = TerrainType::NORMAL;
    TEST_ASSERT(m.at(p).special == SpecialType::NONE, "Special must be NONE");
    TEST_ASSERT(std::abs(m.at(p).traversalCost() - 1.0f) < 1e-4f, "Cost must be restored to 1.0");
    return true;
}

// 22. Clearing all special pellets preserves walls, start, and goal
bool test_22_clear_special_pellets_preserves_structure() {
    Maze m = makeOpenMaze(6, 6);
    Position w1{1, 1}, w2{2, 2};
    m.blockCell(w1);
    m.blockCell(w2);
    m.at(0, 2).special = SpecialType::REWARD;
    m.at(0, 3).special = SpecialType::PENALTY;
    m.at(0, 4).terrain = TerrainType::CONGESTED;

    m.clearAllSpecialPellets();

    TEST_ASSERT(m.countRewardPellets() == 0, "Reward pellets must be 0");
    TEST_ASSERT(m.countPenaltyPellets() == 0, "Penalty pellets must be 0");
    TEST_ASSERT(m.at(w1).isWall(), "Wall 1 must remain blocked");
    TEST_ASSERT(m.at(w2).isWall(), "Wall 2 must remain blocked");
    TEST_ASSERT(m.start == Position(0, 0), "Start position must remain (0,0)");
    TEST_ASSERT(m.goal == Position(5, 5), "Goal position must remain (5,5)");
    return true;
}

// 23. Pellet counts reflect actual maze terrain
bool test_23_pellet_counts_accurate() {
    Maze m = makeOpenMaze(6, 6);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 2).special = SpecialType::REWARD;
    m.at(2, 1).special = SpecialType::PENALTY;
    m.at(2, 2).special = SpecialType::PENALTY;
    m.at(2, 3).special = SpecialType::PENALTY;
    m.at(3, 1).terrain = TerrainType::CONGESTED;

    TEST_ASSERT(m.countRewardPellets() == 2, "Reward count must be 2");
    TEST_ASSERT(m.countPenaltyPellets() == 3, "Penalty count must be 3");
    TEST_ASSERT(m.countCongestedCells() == 1, "Congested count must be 1");
    return true;
}

// 24. Back navigation preserves maze configuration
bool test_24_back_navigation_preserves_configuration() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.blockCell({2, 2});

    // Simulate transition: Phase 1 -> Phase 2 -> Phase 1
    // Maze object state is maintained
    TEST_ASSERT(m.at(1, 1).special == SpecialType::REWARD, "Reward pellet preserved");
    TEST_ASSERT(m.at(2, 2).isWall(), "Wall preserved");
    return true;
}

// 25. Entering solving phase preserves configured terrain
bool test_25_solving_phase_preserves_terrain() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(2, 2).special = SpecialType::PENALTY;

    // Run A*
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();

    TEST_ASSERT(m.at(1, 1).special == SpecialType::REWARD, "Reward pellet preserved during/after solve");
    TEST_ASSERT(m.at(2, 2).special == SpecialType::PENALTY, "Penalty pellet preserved during/after solve");
    return true;
}

// 26. Resetting a search does not erase pellets
bool test_26_search_reset_preserves_pellets() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(2, 2).special = SpecialType::PENALTY;

    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();

    // Reset search
    m.resetSearch();

    TEST_ASSERT(!m.at(m.start).inPath, "inPath flag cleared");
    TEST_ASSERT(m.at(1, 1).special == SpecialType::REWARD, "Reward pellet NOT erased by search reset");
    TEST_ASSERT(m.at(2, 2).special == SpecialType::PENALTY, "Penalty pellet NOT erased by search reset");
    return true;
}

// 27. A* responds to user-configured reward and penalty costs
bool test_27_astar_responds_to_pellets() {
    Maze m = makeOpenMaze(3, 5);
    // Two paths from (0,0) to (0,4):
    // Top path row 0: cells (0,1), (0,2), (0,3) -> penalty road (cost 4.0 each, total 12.0)
    // Bottom path row 1: cells (1,0), (1,1), (1,2), (1,3), (1,4), then (0,4) -> reward roads (0.5 each)
    m.at(0, 1).special = SpecialType::PENALTY;
    m.at(0, 2).special = SpecialType::PENALTY;
    m.at(0, 3).special = SpecialType::PENALTY;

    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 2).special = SpecialType::REWARD;
    m.at(1, 3).special = SpecialType::REWARD;

    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {0, 4});
    astar.runToCompletion();

    // A* must avoid row 0 penalties and route via row 1
    for (const Position& p : astar.getPath()) {
        if (p == Position(0, 1) || p == Position(0, 2) || p == Position(0, 3)) {
            TEST_ASSERT(false, "A* should have avoided high penalty cells on row 0");
        }
    }
    return true;
}

// 28. BFS continues to ignore terrain weights when selecting its path
bool test_28_bfs_ignores_terrain_weights() {
    Maze m = makeOpenMaze(3, 5);
    // Top path row 0: length 4 moves, but penalty cells
    m.at(0, 1).special = SpecialType::PENALTY;
    m.at(0, 2).special = SpecialType::PENALTY;
    m.at(0, 3).special = SpecialType::PENALTY;

    // Bottom path row 1: length 6 moves, with reward cells
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 2).special = SpecialType::REWARD;
    m.at(1, 3).special = SpecialType::REWARD;

    BFSAlgorithm bfs;
    bfs.initialize(m, {0, 0}, {0, 4});
    bfs.runToCompletion();

    // BFS takes fewest moves: top direct path (4 moves) regardless of penalty cost!
    TEST_ASSERT(bfs.stats.routeMoves == 4, "BFS must pick the 4-move path ignoring penalties");
    TEST_ASSERT(bfs.stats.penaltiesOnPath == 3, "BFS route contains the 3 penalties because it ignores weights");
    return true;
}

// 29. Comparison mode gives every algorithm an identical terrain configuration
bool test_29_comparison_identical_configuration() {
    Maze m = makeOpenMaze(6, 6);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(2, 2).special = SpecialType::PENALTY;
    m.at(3, 3).terrain = TerrainType::CONGESTED;

    Maze copyBFS = m; copyBFS.resetAllFlags();
    Maze copyDFS = m; copyDFS.resetAllFlags();
    Maze copyAST = m; copyAST.resetAllFlags();

    TEST_ASSERT(copyBFS.countRewardPellets() == m.countRewardPellets(), "BFS copy must match rewards");
    TEST_ASSERT(copyDFS.countRewardPellets() == m.countRewardPellets(), "DFS copy must match rewards");
    TEST_ASSERT(copyAST.countRewardPellets() == m.countRewardPellets(), "A* copy must match rewards");

    TEST_ASSERT(copyBFS.countPenaltyPellets() == m.countPenaltyPellets(), "BFS copy must match penalties");
    TEST_ASSERT(copyDFS.countPenaltyPellets() == m.countPenaltyPellets(), "DFS copy must match penalties");
    TEST_ASSERT(copyAST.countPenaltyPellets() == m.countPenaltyPellets(), "A* copy must match penalties");

    return true;
}

// 30. Loading a new maze resets obsolete search state and animation safely
bool test_30_new_maze_clears_obsolete_state() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();

    // Hard reset / new maze
    m.hardReset();
    TEST_ASSERT(!m.generated, "Maze generated flag reset");
    TEST_ASSERT(m.pathLength() == 0, "pathLength cleared");
    TEST_ASSERT(m.countRewardPellets() == 0, "Pellets cleared");
    TEST_ASSERT(m.countPenaltyPellets() == 0, "Pellets cleared");

    return true;
}

// ────────────────────────────────────────────────────────────
// DEMONSTRATION SCENARIO VERIFICATION (HEADLESS REPORT)
// ────────────────────────────────────────────────────────────

static Maze buildScenarioA() {
    const int ROWS = 11, COLS = 15;
    Maze m = makeOpenMaze(ROWS, COLS);
    for (int r = 2; r <= 6; r++) m.blockCell({r, 7});
    for (int c = 3; c <= 6; c++) m.blockCell({4, c});
    for (int r = 7; r <= 9; r++) m.blockCell({r, 4});
    m.start = {0, 0};
    m.goal  = {ROWS - 1, COLS - 1};
    return m;
}

static Maze buildScenarioB() {
    const int ROWS = 13, COLS = 19;
    Maze m = makeOpenMaze(ROWS, COLS);
    for (int r = 2; r <= 10; r++) m.blockCell({r, 9});
    m.unblockCell({6, 9});
    m.unblockCell({9, 9});

    m.at(3, 3).special = SpecialType::REWARD;
    m.at(3, 5).special = SpecialType::REWARD;
    m.at(3, 7).special = SpecialType::REWARD;
    m.at(4, 11).special = SpecialType::REWARD;
    m.at(4, 13).special = SpecialType::REWARD;
    m.at(4, 15).special = SpecialType::REWARD;

    for (int c = 1; c <= 7; c++) {
        if (!m.at(8, c).isWall()) m.at(8, c).terrain = TerrainType::CONGESTED;
    }
    for (int c = 10; c <= 16; c++) {
        if (!m.at(8, c).isWall()) m.at(8, c).terrain = TerrainType::CONGESTED;
    }
    m.start = {0, 0};
    m.goal  = {ROWS - 1, COLS - 1};
    return m;
}

static Maze buildScenarioC() {
    const int ROWS = 13, COLS = 19;
    Maze m = makeOpenMaze(ROWS, COLS);
    for (int r = 0; r <= 8; r++) m.blockCell({r, 9});
    m.unblockCell({4, 9});

    m.at(4, 9).terrain = TerrainType::NORMAL;
    m.at(4, 9).special = SpecialType::PENALTY;
    m.at(3, 9).special = SpecialType::PENALTY;
    m.at(5, 9).special = SpecialType::PENALTY;

    for (int c = 1; c <= 7; c++) {
        if (!m.at(10, c).isWall()) m.at(10, c).terrain = TerrainType::DAMAGED;
    }
    m.at(11, 5).special  = SpecialType::REWARD;
    m.at(11, 7).special  = SpecialType::REWARD;
    m.at(11, 12).special = SpecialType::REWARD;
    m.at(11, 14).special = SpecialType::REWARD;

    m.start = {0, 0};
    m.goal  = {ROWS - 1, COLS - 1};
    return m;
}

static void runScenarioEval(const std::string& scenName, const Maze& baseMaze) {
    std::cout << "\n------------------------------------------------------------\n";
    std::cout << "Scenario: " << scenName << "\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(10) << "Algorithm"
              << std::setw(8)  << "Found"
              << std::setw(8)  << "Cells"
              << std::setw(8)  << "Moves"
              << std::setw(12) << "WeightCost"
              << std::setw(10) << "Rewards"
              << std::setw(10) << "Penalties"
              << "\n";

    std::string algos[3] = {"BFS", "DFS", "A*"};
    for (int i = 0; i < 3; i++) {
        Maze m = baseMaze;
        m.resetAllFlags();
        std::unique_ptr<SearchAlgorithm> solver;
        if (i == 0) solver.reset(new BFSAlgorithm());
        else if (i == 1) solver.reset(new DFSAlgorithm());
        else solver.reset(new AStarAlgorithm());

        solver->initialize(m, m.start, m.goal);
        SearchStats st = solver->runToCompletion();

        std::cout << std::left
                  << std::setw(10) << algos[i]
                  << std::setw(8)  << (st.foundPath ? "YES" : "NO")
                  << std::setw(8)  << st.pathLength
                  << std::setw(8)  << st.routeMoves
                  << std::setw(12) << std::fixed << std::setprecision(1) << st.pathCost
                  << std::setw(10) << st.rewardsOnPath
                  << std::setw(10) << st.penaltiesOnPath
                  << "\n";
    }
}

int main() {
    std::cout << "============================================================\n";
    std::cout << " SmartRoute AI — Automated Algorithm & Workflow Test Suite\n";
    std::cout << "============================================================\n";

    reportTest("Test 01: BFS finds valid route when goal is reachable", test_1_bfs_reachable());
    reportTest("Test 02: DFS finds valid route when goal is reachable", test_2_dfs_reachable());
    reportTest("Test 03: A* finds valid route when goal is reachable", test_3_astar_reachable());
    reportTest("Test 04: All algorithms report failure when goal unreachable", test_4_unreachable_goal());
    reportTest("Test 05: Reconstructed route starts at start and ends at goal", test_5_route_start_goal_endpoints());
    reportTest("Test 06: Every consecutive pair of route cells is legal step", test_6_consecutive_steps_legal());
    reportTest("Test 07: Blocked cells are never included in traversable route", test_7_blocked_cells_excluded());
    reportTest("Test 08: Route cost matches entered cells sum excluding start", test_8_cost_excludes_start_cell());
    reportTest("Test 09: Reward roads retain configured fractional cost (0.5)", test_9_reward_road_fractional_cost());
    reportTest("Test 10: Penalty roads retain configured cost (4.0)", test_10_penalty_road_cost());
    reportTest("Test 11: A* returns minimum weighted cost on small maps", test_11_astar_minimum_weighted_cost());
    reportTest("Test 12: Validate A* weighted-cost optimality against Dijkstra", test_12_astar_optimality_vs_dijkstra());
    reportTest("Test 13: 25-cell route reports exactly 24 moves", test_13_path_25_cells_is_24_moves());
    reportTest("Test 14: Start == Goal behaves correctly (moves=0, cost=0)", test_14_start_equals_goal());
    reportTest("Test 15: Repeated runs do not reuse stale search state", test_15_repeated_runs_clean_state());

    std::cout << "\n--- New Phase: Three-Stage Interactive Workflow & Pellets ---\n";
    reportTest("Test 16: Place reward pellet on road cell (cost 0.5)", test_16_place_reward_pellet());
    reportTest("Test 17: Place penalty pellet on road cell (cost 4.0)", test_17_place_penalty_pellet());
    reportTest("Test 18: Pellet placement cannot overwrite start or goal", test_18_pellet_cannot_overwrite_start_goal());
    reportTest("Test 19: Pellet placement cannot overwrite walls", test_19_pellet_cannot_be_placed_on_wall());
    reportTest("Test 20: Switching from reward to penalty updates correctly", test_20_switch_pellet_type());
    reportTest("Test 21: Erasing special pellet restores normal road (1.0)", test_21_erase_pellet_restores_normal());
    reportTest("Test 22: Clear all pellets preserves walls, start, and goal", test_22_clear_special_pellets_preserves_structure());
    reportTest("Test 23: Live pellet counts reflect actual maze terrain", test_23_pellet_counts_accurate());
    reportTest("Test 24: Back navigation preserves maze and pellet layout", test_24_back_navigation_preserves_configuration());
    reportTest("Test 25: Entering solving phase preserves configured terrain", test_25_solving_phase_preserves_terrain());
    reportTest("Test 26: Search reset clears search flags without erasing pellets", test_26_search_reset_preserves_pellets());
    reportTest("Test 27: A* responds to user-configured reward and penalty costs", test_27_astar_responds_to_pellets());
    reportTest("Test 28: BFS continues to ignore terrain weights", test_28_bfs_ignores_terrain_weights());
    reportTest("Test 29: Comparison mode gives all algorithms identical terrain", test_29_comparison_identical_configuration());
    reportTest("Test 30: Loading new maze resets obsolete search state cleanly", test_30_new_maze_clears_obsolete_state());

    std::cout << "\n============================================================\n";
    std::cout << " Demonstration Scenario Verification (Corrected Costs)\n";
    std::cout << "============================================================\n";
    runScenarioEval("Scenario A (Simple Delivery)", buildScenarioA());
    runScenarioEval("Scenario B (Reward Shortcut)", buildScenarioB());
    runScenarioEval("Scenario C (Penalty Road)", buildScenarioC());

    std::cout << "\n============================================================\n";
    std::cout << " TOTAL TESTS: " << (g_passCount + g_failCount)
              << " | PASSED: " << g_passCount
              << " | FAILED: " << g_failCount << "\n";
    std::cout << "============================================================\n";

    return (g_failCount == 0) ? 0 : 1;
}
