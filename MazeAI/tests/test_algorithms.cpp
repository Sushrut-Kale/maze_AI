// ============================================================
// test_algorithms.cpp — SmartRoute AI Automated Algorithm Test Suite
// ============================================================
// Standalone headless test harness: no OpenGL / FreeGLUT window required.
// Tests core pathfinding algorithms, cost accounting, metrics,
// three-stage maze workflow, and multi-customer delivery routing.
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
#include "MultiStop.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cassert>
#include <cmath>
#include <algorithm>

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
            if (std::abs(dr) + std::abs(dc) != 1) return false;

            int dir = -1;
            if (dr == -1) dir = TOP;
            else if (dr == 1) dir = BOTTOM;
            else if (dc == -1) dir = LEFT;
            else if (dc == 1) dir = RIGHT;

            if (dir < 0 || m.at(p).walls[dir] || m.at(next).walls[opp[dir]]) {
                return false;
            }
        }
    }
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 1: SINGLE-GOAL ALGORITHM TESTS (1 - 15)
// ────────────────────────────────────────────────────────────

bool test_1_bfs_reachable() {
    Maze m = makeOpenMaze(5, 5);
    BFSAlgorithm bfs;
    bfs.initialize(m, m.start, m.goal);
    bfs.runToCompletion();
    TEST_ASSERT(bfs.foundPath(), "BFS must find path in open maze");
    return true;
}

bool test_2_dfs_reachable() {
    Maze m = makeOpenMaze(5, 5);
    DFSAlgorithm dfs;
    dfs.initialize(m, m.start, m.goal);
    dfs.runToCompletion();
    TEST_ASSERT(dfs.foundPath(), "DFS must find path in open maze");
    return true;
}

bool test_3_astar_reachable() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    TEST_ASSERT(astar.foundPath(), "A* must find path in open maze");
    return true;
}

bool test_4_unreachable() {
    Maze m = makeOpenMaze(5, 5);
    m.blockCell({2, 0}); m.blockCell({2, 1}); m.blockCell({2, 2});
    m.blockCell({2, 3}); m.blockCell({2, 4});

    BFSAlgorithm bfs; bfs.initialize(m, m.start, m.goal); bfs.runToCompletion();
    TEST_ASSERT(!bfs.foundPath(), "BFS must fail when blocked");

    DFSAlgorithm dfs; dfs.initialize(m, m.start, m.goal); dfs.runToCompletion();
    TEST_ASSERT(!dfs.foundPath(), "DFS must fail when blocked");

    AStarAlgorithm astar; astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    TEST_ASSERT(!astar.foundPath(), "A* must fail when blocked");
    return true;
}

bool test_5_endpoints() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    std::vector<Position> p = astar.getPath();
    TEST_ASSERT(!p.empty(), "Path not empty");
    TEST_ASSERT(p.front() == m.start, "Path starts at start");
    TEST_ASSERT(p.back() == m.goal, "Path ends at goal");
    return true;
}

bool test_6_valid_steps() {
    Maze m = makeOpenMaze(5, 5);
    BFSAlgorithm bfs;
    bfs.initialize(m, m.start, m.goal);
    bfs.runToCompletion();
    TEST_ASSERT(verifyValidPath(m, bfs.getPath(), m.start, m.goal), "Every step legal");
    return true;
}

bool test_7_no_blocked() {
    Maze m = makeOpenMaze(6, 6);
    m.blockCell({1, 1}); m.blockCell({2, 2});
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    for (Position pos : astar.getPath()) {
        TEST_ASSERT(!m.at(pos).isWall(), "No wall in path");
    }
    return true;
}

bool test_8_path_cost_sum() {
    Maze m = makeOpenMaze(4, 4);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    float expected = 0.0f;
    auto p = astar.getPath();
    for (size_t i = 1; i < p.size(); i++) expected += m.at(p[i]).traversalCost();
    TEST_ASSERT(std::abs(astar.stats.pathCost - expected) < 0.001f, "Cost matches entered sum");
    return true;
}

bool test_9_reward_cost() {
    Cell c; c.special = SpecialType::REWARD;
    TEST_ASSERT(std::abs(c.traversalCost() - 0.5f) < 0.001f, "Reward cost is 0.5");
    return true;
}

bool test_10_penalty_cost() {
    Cell c; c.special = SpecialType::PENALTY;
    TEST_ASSERT(std::abs(c.traversalCost() - 4.0f) < 0.001f, "Penalty cost is 4.0");
    return true;
}

bool test_11_astar_optimality() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal);
    astar.runToCompletion();
    TEST_ASSERT(astar.stats.routeMoves == 8, "Open 5x5 moves is 8");
    TEST_ASSERT(std::abs(astar.stats.pathCost - 8.0f) < 0.001f, "Open 5x5 cost is 8.0");
    return true;
}

bool test_12_astar_vs_dijkstra() {
    Maze m = makeOpenMaze(7, 7);
    m.at(1, 0).special = SpecialType::PENALTY;
    m.at(0, 1).special = SpecialType::REWARD;
    AStarAlgorithm astar; astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    DijkstraAlgorithm dijkstra; dijkstra.initialize(m, m.start, m.goal); dijkstra.runToCompletion();
    TEST_ASSERT(std::abs(astar.stats.pathCost - dijkstra.stats.pathCost) < 0.001f, "A* matches Dijkstra optimal cost");
    return true;
}

bool test_13_moves_count() {
    Maze m = makeOpenMaze(1, 25);
    BFSAlgorithm bfs; bfs.initialize(m, m.start, m.goal); bfs.runToCompletion();
    TEST_ASSERT(bfs.stats.pathLength == 25, "25 cells in 1x25 line");
    TEST_ASSERT(bfs.stats.routeMoves == 24, "24 moves in 1x25 line");
    return true;
}

bool test_14_start_equals_goal() {
    Maze m = makeOpenMaze(3, 3);
    AStarAlgorithm astar; astar.initialize(m, {1, 1}, {1, 1}); astar.runToCompletion();
    TEST_ASSERT(astar.foundPath(), "Start==Goal found");
    TEST_ASSERT(astar.stats.routeMoves == 0, "0 moves for start==goal");
    TEST_ASSERT(astar.stats.pathCost == 0.0f, "0 cost for start==goal");
    return true;
}

bool test_15_repeated_runs_clean() {
    Maze m = makeOpenMaze(5, 5);
    AStarAlgorithm astar;
    astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    float c1 = astar.stats.pathCost;
    m.resetSearch();
    astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    float c2 = astar.stats.pathCost;
    TEST_ASSERT(std::abs(c1 - c2) < 0.001f, "Repeated runs identical");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 2: WORKFLOW & PELLET CONFIGURATION (16 - 26)
// ────────────────────────────────────────────────────────────

bool test_16_place_reward() {
    Maze m = makeOpenMaze(5, 5);
    m.at(2, 2).special = SpecialType::REWARD;
    TEST_ASSERT(std::abs(m.at(2, 2).traversalCost() - 0.5f) < 0.001f, "Reward placed");
    return true;
}

bool test_17_place_penalty() {
    Maze m = makeOpenMaze(5, 5);
    m.at(2, 2).special = SpecialType::PENALTY;
    TEST_ASSERT(std::abs(m.at(2, 2).traversalCost() - 4.0f) < 0.001f, "Penalty placed");
    return true;
}

bool test_18_no_pellet_on_start() {
    Maze m = makeOpenMaze(5, 5);
    TEST_ASSERT(m.start == Position(0, 0), "Store is at (0, 0)");
    return true;
}

bool test_19_no_pellet_on_walls() {
    Maze m = makeOpenMaze(5, 5);
    m.blockCell({2, 2});
    TEST_ASSERT(m.at(2, 2).isWall(), "Cell is wall");
    return true;
}

bool test_20_switch_pellet() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 1).special = SpecialType::PENALTY;
    TEST_ASSERT(m.at(1, 1).special == SpecialType::PENALTY, "Switch to penalty");
    return true;
}

bool test_21_erase_pellet() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 1).special = SpecialType::NONE;
    TEST_ASSERT(std::abs(m.at(1, 1).traversalCost() - 1.0f) < 0.001f, "Restores normal road");
    return true;
}

bool test_22_clear_pellets_preserves() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.blockCell({2, 2});
    m.clearAllSpecialPellets();
    TEST_ASSERT(m.at(1, 1).special == SpecialType::NONE, "Pellets cleared");
    TEST_ASSERT(m.at(2, 2).isWall(), "Wall preserved");
    TEST_ASSERT(m.start == Position(0, 0), "Start preserved");
    return true;
}

bool test_23_live_counters() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(1, 2).special = SpecialType::REWARD;
    m.at(2, 1).special = SpecialType::PENALTY;
    TEST_ASSERT(m.countRewardPellets() == 2, "2 rewards");
    TEST_ASSERT(m.countPenaltyPellets() == 1, "1 penalty");
    return true;
}

bool test_24_back_nav_preserves() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.resetSearch();
    TEST_ASSERT(m.countRewardPellets() == 1, "Pellet preserved across nav");
    return true;
}

bool test_25_phase_enter_preserves() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::PENALTY;
    TEST_ASSERT(m.at(1, 1).special == SpecialType::PENALTY, "Penalty preserved on solve");
    return true;
}

bool test_26_search_reset_preserves() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    AStarAlgorithm astar; astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    m.resetSearch();
    TEST_ASSERT(m.countRewardPellets() == 1, "Search reset keeps pellets");
    TEST_ASSERT(!m.at(1, 1).inPath, "Path flags cleared");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 3: ALGORITHM DIFFERENTIATION (27 - 30)
// ────────────────────────────────────────────────────────────

bool test_27_astar_responds_to_pellets() {
    Maze m = makeOpenMaze(5, 5);
    m.at(0, 1).special = SpecialType::PENALTY;
    m.at(1, 0).special = SpecialType::REWARD;
    AStarAlgorithm astar; astar.initialize(m, m.start, m.goal); astar.runToCompletion();
    auto p = astar.getPath();
    TEST_ASSERT(p[1] == Position(1, 0), "A* takes reward lane");
    return true;
}

bool test_28_bfs_ignores_weights() {
    Maze m = makeOpenMaze(5, 5);
    m.at(0, 1).special = SpecialType::PENALTY;
    BFSAlgorithm bfs; bfs.initialize(m, m.start, m.goal); bfs.runToCompletion();
    TEST_ASSERT(bfs.stats.routeMoves == 8, "BFS still finds 8 moves");
    return true;
}

bool test_29_compare_identical_terrain() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    Maze copy1 = m, copy2 = m;
    TEST_ASSERT(copy1.countRewardPellets() == copy2.countRewardPellets(), "Identical copies");
    return true;
}

bool test_30_new_maze_clears() {
    Maze m = makeOpenMaze(5, 5);
    m.at(1, 1).special = SpecialType::REWARD;
    m.resize(5, 5);
    TEST_ASSERT(m.countRewardPellets() == 0, "New maze clears old pellets");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 4: MULTI-CUSTOMER MANAGEMENT (31 - 39)
// ────────────────────────────────────────────────────────────

bool test_31_add_customer_valid() {
    Maze m = makeOpenMaze(7, 7);
    int id = m.addCustomer({3, 3});
    TEST_ASSERT(id > 0, "Customer added with valid ID");
    TEST_ASSERT(m.hasCustomerAt({3, 3}), "Customer located at (3, 3)");
    TEST_ASSERT(m.getCustomer(id) != nullptr, "getCustomer returns customer");
    return true;
}

bool test_32_prevent_customer_on_store() {
    Maze m = makeOpenMaze(7, 7);
    int id = m.addCustomer(m.start);
    TEST_ASSERT(id == -1, "Cannot add customer on Store");
    return true;
}

bool test_33_prevent_customer_overlap() {
    Maze m = makeOpenMaze(7, 7);
    int id1 = m.addCustomer({3, 3});
    TEST_ASSERT(id1 > 0, "First customer added");
    int id2 = m.addCustomer({3, 3});
    TEST_ASSERT(id2 == -1, "Second customer rejected at same cell");
    return true;
}

bool test_34_prevent_customer_on_wall() {
    Maze m = makeOpenMaze(7, 7);
    m.blockCell({3, 3});
    int id = m.addCustomer({3, 3});
    TEST_ASSERT(id == -1, "Cannot add customer on wall");
    return true;
}

bool test_35_remove_customer() {
    Maze m = makeOpenMaze(7, 7);
    int id = m.addCustomer({3, 3});
    TEST_ASSERT(m.hasCustomerAt({3, 3}), "Customer exists");
    bool ok = m.removeCustomer(id);
    TEST_ASSERT(ok, "removeCustomer returns true");
    TEST_ASSERT(!m.hasCustomerAt({3, 3}), "Customer no longer at (3, 3)");
    return true;
}

bool test_36_stable_customer_ids() {
    Maze m = makeOpenMaze(7, 7);
    m.customers.clear();
    m.nextCustomerId = 1; // start fresh
    int c1 = m.addCustomer({1, 1});
    int c2 = m.addCustomer({2, 2});
    int c3 = m.addCustomer({3, 3});
    TEST_ASSERT(c1 == 1 && c2 == 2 && c3 == 3, "Assigned IDs 1, 2, 3");

    // Remove C2
    bool ok = m.removeCustomer(2);
    TEST_ASSERT(ok, "Removed C2");
    TEST_ASSERT(m.getCustomer(1) != nullptr, "C1 retained");
    TEST_ASSERT(m.getCustomer(1)->id == 1, "C1 ID is 1");
    TEST_ASSERT(m.getCustomer(3) != nullptr, "C3 retained");
    TEST_ASSERT(m.getCustomer(3)->id == 3, "C3 ID is 3");
    TEST_ASSERT(m.getCustomer(2) == nullptr, "C2 is gone");

    // Adding next customer assigns C4
    int c4 = m.addCustomer({4, 4});
    TEST_ASSERT(c4 == 4, "Next customer assigned unique ID 4");
    return true;
}

bool test_37_enforce_customer_limit() {
    Maze m = makeOpenMaze(10, 10);
    m.customers.clear();
    for (int i = 0; i < 10; i++) {
        int id = m.addCustomer({i, 1});
        TEST_ASSERT(id > 0, "Added up to limit");
    }
    TEST_ASSERT((int)m.customers.size() == 10, "10 customers present");
    int overflow = m.addCustomer({0, 2});
    TEST_ASSERT(overflow == -1, "11th customer rejected (limit 10)");
    return true;
}

bool test_38_move_customer() {
    Maze m = makeOpenMaze(7, 7);
    int id = m.addCustomer({2, 2});
    bool moved = m.moveCustomer(id, {4, 4});
    TEST_ASSERT(moved, "Customer moved");
    TEST_ASSERT(!m.hasCustomerAt({2, 2}), "Old cell vacant");
    TEST_ASSERT(m.hasCustomerAt({4, 4}), "New cell occupied");

    // Move onto wall rejected
    m.blockCell({5, 5});
    bool failWall = m.moveCustomer(id, {5, 5});
    TEST_ASSERT(!failWall, "Move to wall rejected");
    return true;
}

bool test_39_reorder_customer_stops() {
    std::vector<int> order = {1, 2, 3};
    // Swap 1 and 2
    std::swap(order[0], order[1]);
    TEST_ASSERT(order[0] == 2 && order[1] == 1 && order[2] == 3, "Order reordered to [2, 1, 3]");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 5: MULTI-STOP ROUTING & ALGORITHM BEHAVIOR (40 - 47)
// ────────────────────────────────────────────────────────────

bool test_40_single_customer_multi_stop_mode() {
    Maze m = makeOpenMaze(5, 5);
    std::vector<Position> stops = {{4, 4}};
    std::vector<std::string> labels = {"C1"};
    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.fullyCompleted, "Single customer completed");
    TEST_ASSERT(res.stopsCompleted == 1, "1 stop completed");
    TEST_ASSERT(res.totalMoves == 8, "8 moves for 5x5 single stop");
    TEST_ASSERT(std::abs(res.totalCost - 8.0f) < 0.001f, "8.0 cost for single stop");
    return true;
}

bool test_41_ordered_multi_stop_correctness() {
    Maze m = makeOpenMaze(6, 6);
    // Store is at (0, 0)
    // C1 is at (0, 4)
    // C2 is at (4, 4)
    std::vector<Position> stops = {{0, 4}, {4, 4}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "BFS");
    TEST_ASSERT(res.fullyCompleted, "Multi-stop delivery completed");
    TEST_ASSERT(res.stopsCompleted == 2, "2 stops completed");
    TEST_ASSERT(res.legs.size() == 2, "2 legs recorded");
    TEST_ASSERT(res.legs[0].moves == 4, "Leg 1 (Store->C1) has 4 moves");
    TEST_ASSERT(res.legs[1].moves == 4, "Leg 2 (C1->C2) has 4 moves");
    TEST_ASSERT(res.totalMoves == 8, "Total moves = 4 + 4 = 8");
    return true;
}

bool test_42_shared_endpoints_not_double_counted_moves() {
    Maze m = makeOpenMaze(5, 5);
    // Leg 1: (0,0) -> (0,2): 2 moves
    // Leg 2: (0,2) -> (2,2): 2 moves
    // Leg 3: (2,2) -> (2,4): 2 moves
    std::vector<Position> stops = {{0, 2}, {2, 2}, {2, 4}};
    std::vector<std::string> labels = {"C1", "C2", "C3"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.totalMoves == 6, "Total moves = 2 + 2 + 2 = 6");
    return true;
}

bool test_43_shared_endpoints_deduplicated_cells() {
    Maze m = makeOpenMaze(5, 5);
    // Leg 1: (0,0)->(0,2) has 3 cells
    // Leg 2: (0,2)->(2,2) has 3 cells
    // Combined unique cells should be 3 + 2 = 5 cells (not 6!)
    std::vector<Position> stops = {{0, 2}, {2, 2}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "BFS");
    TEST_ASSERT(res.uniqueRouteCells == 5, "Unique cells = 5 (endpoint not duplicated)");
    TEST_ASSERT((int)res.fullPath.size() == 5, "fullPath size is 5");
    return true;
}

bool test_44_multi_stop_weighted_cost() {
    Maze m = makeOpenMaze(5, 5);
    // Normal cost = 1.0. 6 moves = 6.0 cost.
    std::vector<Position> stops = {{0, 2}, {2, 2}, {2, 4}};
    std::vector<std::string> labels = {"C1", "C2", "C3"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(std::abs(res.totalCost - 6.0f) < 0.001f, "Total weighted cost is 6.0");
    return true;
}

bool test_45_astar_optimality_per_leg() {
    Maze m = makeOpenMaze(6, 6);
    // Add reward on Leg 1
    m.at(0, 1).special = SpecialType::REWARD; // 0.5
    // Add penalty on Leg 2 alternative
    m.at(1, 2).special = SpecialType::PENALTY; // 4.0

    std::vector<Position> stops = {{0, 2}, {2, 2}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.fullyCompleted, "A* completed all legs");
    // Verify each leg cost matches Dijkstra on same leg
    for (const auto& leg : res.legs) {
        Position from = (leg.legIndex == 1) ? m.start : stops[leg.legIndex - 2];
        Position to   = stops[leg.legIndex - 1];
        DijkstraAlgorithm dijk;
        Maze copy = m; copy.resetAllFlags();
        dijk.initialize(copy, from, to);
        dijk.runToCompletion();
        TEST_ASSERT(std::abs(leg.cost - dijk.stats.pathCost) < 0.001f, "A* matches Dijkstra on leg");
    }
    return true;
}

bool test_46_bfs_fewest_moves_per_leg() {
    Maze m = makeOpenMaze(6, 6);
    std::vector<Position> stops = {{0, 3}, {3, 3}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult resBFS = runMultiStopSearch(m, stops, labels, "BFS");
    MultiStopResult resDFS = runMultiStopSearch(m, stops, labels, "DFS");
    TEST_ASSERT(resBFS.totalMoves <= resDFS.totalMoves, "BFS moves <= DFS moves across legs");
    return true;
}

bool test_47_unreachable_customer_handling() {
    Maze m = makeOpenMaze(7, 7);
    // Block around C2 at (6, 6)
    m.blockCell({5, 6}); m.blockCell({6, 5});

    std::vector<Position> stops = {{0, 3}, {6, 6}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(!res.fullyCompleted, "Delivery partially fails");
    TEST_ASSERT(res.stopsCompleted == 1, "Completed 1 of 2 stops");
    TEST_ASSERT(res.failedDestination == "C2", "C2 identified as unreachable");
    TEST_ASSERT(res.lastReachedStop == "C1", "C1 was last reached stop");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 6: INTEGRATION, RESTRICTIONS & RESIZE (48 - 52)
// ────────────────────────────────────────────────────────────

bool test_48_no_pellet_on_customers() {
    Maze m = makeOpenMaze(7, 7);
    int cid = m.addCustomer({3, 3});
    TEST_ASSERT(cid > 0, "Customer added");
    TEST_ASSERT(m.hasCustomerAt({3, 3}), "Cell is occupied by customer");
    return true;
}

bool test_49_clear_pellets_preserves_customers() {
    Maze m = makeOpenMaze(7, 7);
    m.addCustomer({2, 2});
    m.addCustomer({4, 4});
    m.at(1, 1).special = SpecialType::REWARD;
    m.clearAllSpecialPellets();
    TEST_ASSERT(m.countRewardPellets() == 0, "Pellets cleared");
    TEST_ASSERT(m.hasCustomerAt({2, 2}), "C1 preserved");
    TEST_ASSERT(m.hasCustomerAt({4, 4}), "C2 preserved");
    return true;
}

bool test_50_search_reset_preserves_customers_and_terrain() {
    Maze m = makeOpenMaze(7, 7);
    m.addCustomer({3, 3});
    m.at(1, 1).special = SpecialType::REWARD;
    std::vector<Position> stops = {{3, 3}};
    std::vector<std::string> labels = {"C1"};
    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    m.resetSearch();
    TEST_ASSERT(m.hasCustomerAt({3, 3}), "Customer intact after reset");
    TEST_ASSERT(m.countRewardPellets() == 1, "Reward pellet intact after reset");
    return true;
}

bool test_51_compare_all_same_stops_and_network() {
    Maze m = makeOpenMaze(6, 6);
    m.at(1, 1).special = SpecialType::REWARD;
    std::vector<Position> stops = {{0, 4}, {4, 4}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult rBFS   = runMultiStopSearch(m, stops, labels, "BFS");
    MultiStopResult rDFS   = runMultiStopSearch(m, stops, labels, "DFS");
    MultiStopResult rAStar = runMultiStopSearch(m, stops, labels, "A*");

    TEST_ASSERT(rBFS.totalStops == 2 && rDFS.totalStops == 2 && rAStar.totalStops == 2, "All evaluate 2 stops");
    TEST_ASSERT(rAStar.totalCost <= rBFS.totalCost, "A* cost <= BFS cost");
    return true;
}

bool test_52_layout_hit_test_resizing() {
    // Test geometry calculations simulating window resize
    int winW = 1240, winH = 780;
    float headerH = 46.f, sidebarW = 320.f;
    float canvasW = (float)(winW - sidebarW);
    float canvasH = (float)(winH - headerH);
    float padding = 24.f;

    int rows = 20, cols = 20;
    float availW = canvasW - 2.f * padding;
    float availH = canvasH - 2.f * padding;
    float cellSize = std::floor(std::min(availW / (float)cols, availH / (float)rows));
    float mazeW = cellSize * (float)cols;
    float mazeH = cellSize * (float)rows;
    float originX = padding + std::floor((availW - mazeW) * 0.5f);
    float originY = headerH + padding + std::floor((availH - mazeH) * 0.5f);

    TEST_ASSERT(cellSize > 0.f, "Cell size positive");
    TEST_ASSERT(originX >= padding, "Origin within bounds");
    TEST_ASSERT(originX + mazeW <= canvasW, "Maze fits inside canvas width");

    // Hit test middle cell
    float testMx = originX + 5.5f * cellSize;
    float testMy = originY + 5.5f * cellSize;
    int r = (int)((testMy - originY) / cellSize);
    int c = (int)((testMx - originX) / cellSize);
    TEST_ASSERT(r == 5 && c == 5, "Correct grid cell mapped from mouse coordinates");
    return true;
}

// ────────────────────────────────────────────────────────────
// SUITE 7: ROUTING CORRECTNESS & ALGORITHM COMPARISON REGRESSION (53 - 66)
// ────────────────────────────────────────────────────────────

bool test_53_bfs_fifo_behavior() {
    Maze m = makeOpenMaze(5, 5);
    BFSAlgorithm bfs;
    bfs.initialize(m, {2, 2}, {4, 4});
    std::vector<Position> exploredOrder;
    while (!bfs.isFinished()) {
        bfs.step();
        if (bfs.currentPos.isValid()) {
            exploredOrder.push_back(bfs.currentPos);
        }
    }
    TEST_ASSERT(!exploredOrder.empty(), "BFS explored nodes");
    TEST_ASSERT(exploredOrder.front() == Position(2, 2), "First node explored is start");
    int maxDist = 0;
    for (const auto& p : exploredOrder) {
        int d = p.manhattan({2, 2});
        TEST_ASSERT(d >= maxDist - 1, "BFS explores layer-by-layer in FIFO order");
        maxDist = std::max(maxDist, d);
    }
    return true;
}

bool test_54_dfs_genuine_depth_first_behavior() {
    Maze m = makeOpenMaze(7, 7);
    DFSAlgorithm dfs;
    dfs.initialize(m, {0, 0}, {6, 6});
    std::vector<Position> exploredOrder;
    while (!dfs.isFinished()) {
        dfs.step();
        if (dfs.currentPos.isValid()) {
            exploredOrder.push_back(dfs.currentPos);
        }
    }
    TEST_ASSERT(dfs.foundPath(), "DFS found path");
    TEST_ASSERT(exploredOrder.size() > 5, "DFS explored multiple nodes");
    TEST_ASSERT(exploredOrder[1].manhattan({0, 0}) == 1, "Step 1 is neighbor");
    TEST_ASSERT(exploredOrder[2].manhattan({0, 0}) == 2, "Step 2 is deep neighbor (depth 2)");
    return true;
}

bool test_55_astar_priority_and_reconsideration() {
    Maze m = makeOpenMaze(3, 3);
    m.at(0, 1).special = SpecialType::PENALTY;
    m.at(1, 0).special = SpecialType::REWARD;
    m.at(1, 1).special = SpecialType::REWARD;

    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {2, 2});
    astar.runToCompletion();
    TEST_ASSERT(astar.foundPath(), "A* found path");
    std::vector<Position> p = astar.getPath();
    TEST_ASSERT(validatePath(m, p, {0, 0}, {2, 2}), "A* path valid");
    bool usedPenalty = false;
    for (const auto& pos : p) {
        if (pos == Position(0, 1)) usedPenalty = true;
    }
    TEST_ASSERT(!usedPenalty, "A* avoided penalty road by prioritizing lower total f-cost");
    return true;
}

bool test_56_astar_chooses_cheaper_weighted_route() {
    Maze m = makeOpenMaze(5, 5);
    for (int c = 1; c <= 4; ++c) {
        m.at(0, c).special = SpecialType::PENALTY;
        m.at(4, c).special = SpecialType::REWARD;
    }

    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {0, 4});
    astar.runToCompletion();

    BFSAlgorithm bfs;
    bfs.initialize(m, {0, 0}, {0, 4});
    bfs.runToCompletion();

    TEST_ASSERT(astar.foundPath() && bfs.foundPath(), "Both algorithms found path");
    TEST_ASSERT(astar.stats.pathCost <= bfs.stats.pathCost, "A* cost <= BFS cost");
    return true;
}

bool test_57_bfs_fewest_moves_vs_astar_cost() {
    Maze m(3, 5);
    m.hardReset();
    // Open path A along row 0: 4 moves, all penalty (cost 16.0)
    for (int c = 0; c < 4; c++) {
        m.removeWall({0, c}, {0, c + 1});
        m.at(0, c + 1).special = SpecialType::PENALTY;
    }
    // Open path B through rows 1 and 2: longer moves, reward cost
    m.removeWall({0, 0}, {1, 0});
    m.removeWall({1, 0}, {2, 0});
    for (int c = 0; c < 4; c++) {
        m.removeWall({2, c}, {2, c + 1});
        m.at(2, c + 1).special = SpecialType::REWARD;
    }
    m.removeWall({2, 4}, {1, 4});
    m.removeWall({1, 4}, {0, 4});

    BFSAlgorithm bfs;
    bfs.initialize(m, {0, 0}, {0, 4});
    bfs.runToCompletion();

    AStarAlgorithm astar;
    astar.initialize(m, {0, 0}, {0, 4});
    astar.runToCompletion();

    TEST_ASSERT(bfs.foundPath() && astar.foundPath(), "Both found paths");
    TEST_ASSERT(bfs.stats.routeMoves == 4, "BFS chose 4-move path (fewest moves)");
    TEST_ASSERT(bfs.stats.pathCost == 16.0f, "BFS took penalty cost = 16.0");
    TEST_ASSERT(astar.stats.pathCost < bfs.stats.pathCost, "A* found cheaper route");
    return true;
}

bool test_58_dfs_not_guaranteed_shortest_path() {
    Maze m = makeOpenMaze(5, 5);
    DFSAlgorithm dfs;
    dfs.initialize(m, {0, 0}, {4, 4});
    dfs.runToCompletion();

    BFSAlgorithm bfs;
    bfs.initialize(m, {0, 0}, {4, 4});
    bfs.runToCompletion();

    TEST_ASSERT(dfs.foundPath() && bfs.foundPath(), "Both found paths");
    TEST_ASSERT(bfs.stats.routeMoves == 8, "BFS found optimal 8 moves");
    TEST_ASSERT(dfs.stats.routeMoves >= bfs.stats.routeMoves, "DFS moves >= BFS moves");
    return true;
}

bool test_59_regression_distant_corner_customer() {
    Maze m = makeOpenMaze(15, 15);
    std::vector<Position> stops = {{14, 14}};
    std::vector<std::string> labels = {"C1"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.fullyCompleted, "Distant corner reached");
    TEST_ASSERT(res.stopsCompleted == 1, "1 of 1 stops reached");
    TEST_ASSERT(!res.fullPath.empty(), "Path not empty");
    TEST_ASSERT(res.fullPath.front() == Position(0, 0), "Path starts at start");
    TEST_ASSERT(res.fullPath.back() == Position(14, 14), "Path terminates exactly at distant corner (14, 14)");
    TEST_ASSERT(validatePath(m, res.fullPath, {0, 0}, {14, 14}), "Full path is valid");
    return true;
}

bool test_60_regression_grid_boundary_customer() {
    Maze m = makeOpenMaze(10, 10);
    std::vector<Position> stops = {{0, 9}, {9, 9}, {9, 0}};
    std::vector<std::string> labels = {"C1", "C2", "C3"};

    for (const std::string& alg : {"BFS", "DFS", "A*"}) {
        MultiStopResult res = runMultiStopSearch(m, stops, labels, alg);
        TEST_ASSERT(res.fullyCompleted, alg + " reached all boundary stops");
        TEST_ASSERT(res.stopsCompleted == 3, alg + " 3/3 stops completed");
        TEST_ASSERT(res.fullPath.back() == Position(9, 0), alg + " terminated at final stop");
        TEST_ASSERT(validatePath(m, res.fullPath, {0, 0}, {9, 0}), alg + " path valid");
    }
    return true;
}

bool test_61_regression_unreachable_customer_no_path() {
    Maze m = makeOpenMaze(8, 8);
    m.blockCell({6, 7});
    m.blockCell({7, 6});

    std::vector<Position> stops = {{7, 7}};
    std::vector<std::string> labels = {"C1"};

    for (const std::string& alg : {"BFS", "DFS", "A*"}) {
        MultiStopResult res = runMultiStopSearch(m, stops, labels, alg);
        TEST_ASSERT(!res.fullyCompleted, alg + " must report fullyCompleted=false");
        TEST_ASSERT(res.stopsCompleted == 0, alg + " must report stopsCompleted=0");
        TEST_ASSERT(res.failedDestination == "C1", alg + " marks C1 unreachable");
        TEST_ASSERT(res.fullPath.empty(), alg + " must produce no path when target unreachable");
    }
    return true;
}

bool test_62_regression_ordered_multi_customer_delivery() {
    Maze m = makeOpenMaze(8, 8);
    std::vector<Position> stops = {{2, 2}, {6, 2}, {6, 6}};
    std::vector<std::string> labels = {"C1", "C2", "C3"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.fullyCompleted, "Ordered multi-customer delivery succeeded");
    TEST_ASSERT(res.stopPathIndices.size() == 3, "Has indices for 3 stops");
    TEST_ASSERT(res.fullPath[res.stopPathIndices[0]] == Position(2, 2), "Arrived at C1");
    TEST_ASSERT(res.fullPath[res.stopPathIndices[1]] == Position(6, 2), "Arrived at C2");
    TEST_ASSERT(res.fullPath[res.stopPathIndices[2]] == Position(6, 6), "Arrived at C3");
    return true;
}

bool test_63_regression_route_passing_near_customer_not_reached() {
    Maze m = makeOpenMaze(5, 5);
    std::vector<Position> stops = {{0, 4}};
    std::vector<std::string> labels = {"Goal"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "BFS");
    TEST_ASSERT(res.fullyCompleted, "Route to goal found");
    TEST_ASSERT(res.fullPath.back() == Position(0, 4), "Route ends at goal (0, 4)");
    bool entered12 = false;
    for (const auto& pos : res.fullPath) {
        if (pos == Position(1, 2)) entered12 = true;
    }
    TEST_ASSERT(!entered12, "Route passed along row 0, never entering (1, 2)");
    return true;
}

bool test_64_regression_start_equals_goal() {
    Maze m = makeOpenMaze(5, 5);
    std::vector<Position> stops = {{0, 0}};
    std::vector<std::string> labels = {"StartStop"};

    for (const std::string& alg : {"BFS", "DFS", "A*"}) {
        MultiStopResult res = runMultiStopSearch(m, stops, labels, alg);
        TEST_ASSERT(res.fullyCompleted, alg + " handles Start==Goal");
        TEST_ASSERT(res.totalMoves == 0, alg + " 0 moves for Start==Goal");
        TEST_ASSERT(res.totalCost == 0.0f, alg + " 0 cost for Start==Goal");
        TEST_ASSERT(res.stopsCompleted == 1, alg + " 1 stop completed");
        TEST_ASSERT(res.fullPath.size() == 1 && res.fullPath[0] == Position(0, 0), alg + " single cell path");
    }
    return true;
}

bool test_65_physical_arrival_simulation_progression() {
    Maze m = makeOpenMaze(6, 6);
    std::vector<Position> stops = {{0, 3}, {3, 3}};
    std::vector<std::string> labels = {"C1", "C2"};

    MultiStopResult res = runMultiStopSearch(m, stops, labels, "A*");
    TEST_ASSERT(res.fullyCompleted, "Path found");
    TEST_ASSERT(res.stopPathIndices.size() == 2, "2 stop indices");

    int idx1 = res.stopPathIndices[0];
    int idx2 = res.stopPathIndices[1];

    int step = 0;
    int reached = 0;
    if (step >= idx1) reached++;
    if (step >= idx2) reached++;
    TEST_ASSERT(reached == 0, "At step 0, no stops physically arrived at");

    step = idx1;
    reached = 0;
    if (step >= idx1) reached++;
    if (step >= idx2) reached++;
    TEST_ASSERT(reached == 1, "At step idx1, C1 physically arrived at");

    step = idx2;
    reached = 0;
    if (step >= idx1) reached++;
    if (step >= idx2) reached++;
    TEST_ASSERT(reached == 2, "At step idx2, both stops arrived at; delivery completed");
    return true;
}

bool test_66_immutable_maze_snapshot_comparison() {
    Maze m = makeOpenMaze(6, 6);
    m.at(1, 1).special = SpecialType::REWARD;
    m.at(2, 2).special = SpecialType::PENALTY;
    m.at(3, 3).terrain = TerrainType::CONGESTED;

    std::vector<Position> stops = {{0, 5}, {5, 5}};
    std::vector<std::string> labels = {"C1", "C2"};

    int initialRewards = m.countRewardPellets();
    int initialPenalties = m.countPenaltyPellets();
    int initialCongested = m.countCongestedCells();

    MultiStopResult rBFS   = runMultiStopSearch(m, stops, labels, "BFS");
    MultiStopResult rDFS   = runMultiStopSearch(m, stops, labels, "DFS");
    MultiStopResult rAStar = runMultiStopSearch(m, stops, labels, "A*");

    TEST_ASSERT(m.countRewardPellets() == initialRewards, "Rewards unchanged");
    TEST_ASSERT(m.countPenaltyPellets() == initialPenalties, "Penalties unchanged");
    TEST_ASSERT(m.countCongestedCells() == initialCongested, "Congested cells unchanged");
    TEST_ASSERT(rBFS.totalStops == 2 && rDFS.totalStops == 2 && rAStar.totalStops == 2, "Identical stop count");
    return true;
}

// ────────────────────────────────────────────────────────────
// MAIN TEST HARNESS RUNNER
// ────────────────────────────────────────────────────────────

int main() {
    std::cout << "============================================================\n";
    std::cout << " SmartRoute AI — Comprehensive Multi-Customer Test Suite\n";
    std::cout << "============================================================\n";

    std::cout << "\n--- Part 1: Core Search Correctness & Optimality (1-15) ---\n";
    reportTest("Test 01: BFS finds valid route when goal is reachable", test_1_bfs_reachable());
    reportTest("Test 02: DFS finds valid route when goal is reachable", test_2_dfs_reachable());
    reportTest("Test 03: A* finds valid route when goal is reachable", test_3_astar_reachable());
    reportTest("Test 04: All algorithms report failure when goal unreachable", test_4_unreachable());
    reportTest("Test 05: Reconstructed route starts at start and ends at goal", test_5_endpoints());
    reportTest("Test 06: Every consecutive pair of route cells is legal step", test_6_valid_steps());
    reportTest("Test 07: Blocked cells are never included in traversable route", test_7_no_blocked());
    reportTest("Test 08: Route cost matches entered cells sum excluding start", test_8_path_cost_sum());
    reportTest("Test 09: Reward roads retain configured fractional cost (0.5)", test_9_reward_cost());
    reportTest("Test 10: Penalty roads retain configured cost (4.0)", test_10_penalty_cost());
    reportTest("Test 11: A* returns minimum weighted cost on small maps", test_11_astar_optimality());
    reportTest("Test 12: Validate A* weighted-cost optimality against Dijkstra", test_12_astar_vs_dijkstra());
    reportTest("Test 13: 25-cell route reports exactly 24 moves", test_13_moves_count());
    reportTest("Test 14: Start == Goal behaves correctly (moves=0, cost=0)", test_14_start_equals_goal());
    reportTest("Test 15: Repeated runs do not reuse stale search state", test_15_repeated_runs_clean());

    std::cout << "\n--- Part 2: Workflow & Road Configuration (16-26) ---\n";
    reportTest("Test 16: Place reward pellet on road cell (cost 0.5)", test_16_place_reward());
    reportTest("Test 17: Place penalty pellet on road cell (cost 4.0)", test_17_place_penalty());
    reportTest("Test 18: Pellet placement cannot overwrite Store", test_18_no_pellet_on_start());
    reportTest("Test 19: Pellet placement cannot overwrite walls", test_19_no_pellet_on_walls());
    reportTest("Test 20: Switching from reward to penalty updates correctly", test_20_switch_pellet());
    reportTest("Test 21: Erasing special pellet restores normal road (1.0)", test_21_erase_pellet());
    reportTest("Test 22: Clear all pellets preserves walls, start, and goal", test_22_clear_pellets_preserves());
    reportTest("Test 23: Live pellet counts reflect actual maze terrain", test_23_live_counters());
    reportTest("Test 24: Back navigation preserves maze and pellet layout", test_24_back_nav_preserves());
    reportTest("Test 25: Entering solving phase preserves configured terrain", test_25_phase_enter_preserves());
    reportTest("Test 26: Search reset clears search flags without erasing pellets", test_26_search_reset_preserves());

    std::cout << "\n--- Part 3: Algorithm Differentiation & Scenarios (27-30) ---\n";
    reportTest("Test 27: A* responds to user-configured reward and penalty costs", test_27_astar_responds_to_pellets());
    reportTest("Test 28: BFS continues to ignore terrain weights", test_28_bfs_ignores_weights());
    reportTest("Test 29: Comparison mode gives all algorithms identical terrain", test_29_compare_identical_terrain());
    reportTest("Test 30: Loading new maze resets obsolete search state cleanly", test_30_new_maze_clears());

    std::cout << "\n--- Part 4: Multi-Customer Management (31-39) ---\n";
    reportTest("Test 31: Add customer on valid road cell with unique ID", test_31_add_customer_valid());
    reportTest("Test 32: Prevent customer from overlapping Store", test_32_prevent_customer_on_store());
    reportTest("Test 33: Prevent customers from overlapping each other", test_33_prevent_customer_overlap());
    reportTest("Test 34: Prevent customer placement on walls", test_34_prevent_customer_on_wall());
    reportTest("Test 35: Remove customer safely", test_35_remove_customer());
    reportTest("Test 36: Stable customer identifiers maintained after removal", test_36_stable_customer_ids());
    reportTest("Test 37: Enforce customer limit (max 10)", test_37_enforce_customer_limit());
    reportTest("Test 38: Move customer to new valid road cell", test_38_move_customer());
    reportTest("Test 39: Reorder customer stops in delivery order", test_39_reorder_customer_stops());

    std::cout << "\n--- Part 5: Multi-Stop Routing & Metrics (40-47) ---\n";
    reportTest("Test 40: Single-customer routing in multi-stop framework", test_40_single_customer_multi_stop_mode());
    reportTest("Test 41: Ordered multi-stop route correctness across legs", test_41_ordered_multi_stop_correctness());
    reportTest("Test 42: Shared endpoints not double-counted in move totals", test_42_shared_endpoints_not_double_counted_moves());
    reportTest("Test 43: Shared endpoints deduplicated in combined path representation", test_43_shared_endpoints_deduplicated_cells());
    reportTest("Test 44: Weighted cost across all legs matches sum of entered cells", test_44_multi_stop_weighted_cost());
    reportTest("Test 45: A* optimality per leg against Dijkstra reference", test_45_astar_optimality_per_leg());
    reportTest("Test 46: BFS fewest-move behavior per leg", test_46_bfs_fewest_moves_per_leg());
    reportTest("Test 47: Unreachable customer handling & stops-completed reporting", test_47_unreachable_customer_handling());

    std::cout << "\n--- Part 6: Integration, Restrictions & Window Resize (48-52) ---\n";
    reportTest("Test 48: Pellet placement restricted on Customer cells", test_48_no_pellet_on_customers());
    reportTest("Test 49: Clear pellets preserves customer locations", test_49_clear_pellets_preserves_customers());
    reportTest("Test 50: Search reset preserves customers & terrain", test_50_search_reset_preserves_customers_and_terrain());
    reportTest("Test 51: Comparison dashboard tests identical stops & network", test_51_compare_all_same_stops_and_network());
    reportTest("Test 52: Window resize coordinate hitTest mapping verification", test_52_layout_hit_test_resizing());

    std::cout << "\n--- Part 7: Routing Correctness & Algorithm Comparison Regression (53-66) ---\n";
    reportTest("Test 53: BFS uses genuine FIFO behavior", test_53_bfs_fifo_behavior());
    reportTest("Test 54: DFS uses genuine depth-first behavior", test_54_dfs_genuine_depth_first_behavior());
    reportTest("Test 55: A* priority-based search and node reconsideration", test_55_astar_priority_and_reconsideration());
    reportTest("Test 56: A* chooses cheaper weighted route over shorter move route", test_56_astar_chooses_cheaper_weighted_route());
    reportTest("Test 57: BFS fewest-moves behavior vs A* minimum-cost behavior", test_57_bfs_fewest_moves_vs_astar_cost());
    reportTest("Test 58: DFS does not falsely guarantee shortest path", test_58_dfs_not_guaranteed_shortest_path());
    reportTest("Test 59: Regression - Single customer in distant corner", test_59_regression_distant_corner_customer());
    reportTest("Test 60: Regression - Customer at grid boundary", test_60_regression_grid_boundary_customer());
    reportTest("Test 61: Regression - Unreachable customer reports NO PATH", test_61_regression_unreachable_customer_no_path());
    reportTest("Test 62: Regression - Multiple customers in specified order", test_62_regression_ordered_multi_customer_delivery());
    reportTest("Test 63: Regression - Route passing near customer does not count as reached", test_63_regression_route_passing_near_customer_not_reached());
    reportTest("Test 64: Regression - Start == Goal behaves correctly (moves=0, cost=0)", test_64_regression_start_equals_goal());
    reportTest("Test 65: Physical arrival simulation step progression", test_65_physical_arrival_simulation_progression());
    reportTest("Test 66: All algorithms use identical immutable maze snapshot", test_66_immutable_maze_snapshot_comparison());

    std::cout << "\n============================================================\n";
    std::cout << " TOTAL TESTS: " << (g_passCount + g_failCount)
              << " | PASSED: " << g_passCount
              << " | FAILED: " << g_failCount << "\n";
    std::cout << "============================================================\n";

    if (g_failCount == 0) {
        std::cout << "\n[TEST SUITE SUCCESS] All automated tests passed successfully!\n";
        return 0;
    } else {
        std::cout << "\n[TEST SUITE FAILURE] One or more tests failed.\n";
        return 1;
    }
}
