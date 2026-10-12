// ============================================================
// main.cpp  —  SmartRoute AI
//   Intelligent Last-Mile Delivery Route Optimization Simulator
// ============================================================
// Professional Desktop UI/UX Redesign for College AI MSE Presentation
//
// THREE-PHASE WORKFLOW:
//   Phase 1: Create Road Network (Maze / Grid)
//   Phase 2: Configure Reward (0.5), Congested (2.0) & Penalty (4.0) Roads
//   Phase 3: Solve & Compare (BFS, DFS, A*)
//
// Layout Architecture:
//   A. Top Header across window (Title, Subtitle, Phase Indicator, Status)
//   B. Main Maze Canvas (Centered, square cells, clear contrast, padding)
//   C. Right Sidebar (Phase-specific, clean cards, aligned typography)
// ============================================================

#include <GL/glut.h>
#include "Cell.h"
#include "Maze.h"
#include "Generators.h"
#include "SearchAlgorithm.h"
#include "Algorithms.h"
#include "Button.h"
#include "Recorder.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <random>
#include <cstring>
#include <iostream>

// ════════════════════════════════════════════════════════════
// ── CONSTANTS & LAYOUT GEOMETRY ─────────────────────────────
// ════════════════════════════════════════════════════════════
static int WIN_W           = 1240;
static int WIN_H           = 780;
static const float HEADER_H    = 46.f;
static const float SIDEBAR_W   = 320.f;
static const float TIMER_MS    = 32.f; // ~31 FPS

// ════════════════════════════════════════════════════════════
// ── APPLICATION STATE MACHINE ───────────────────────────────
// ════════════════════════════════════════════════════════════
enum class AppPhase {
    PHASE_1_CREATE = 1,
    PHASE_2_CONFIG = 2,
    PHASE_3_SOLVE  = 3
};
static AppPhase currentPhase = AppPhase::PHASE_1_CREATE;

enum class AppState {
    MAZE_WORKFLOW,
    GENERATING,
    SOLVING,
    SOLVED,
    COMPARING,
    HELP,
    LEGEND
};
static AppState appState = AppState::MAZE_WORKFLOW;

// Tools & Selections
enum class GenType    { DFS_GRID, PRIM_BRANCH };
enum class SearchType { BFS, DFS_S, ASTAR };
enum class PelletTool { REWARD_05, PENALTY_40, CONGESTED_20, NORMAL_10 };
enum class EditTool   { WALL, ERASE };
enum class MazeSizeE  { SMALL, MEDIUM, LARGE };
enum class ScenarioE  { NONE, A_SIMPLE, B_REWARD, C_PENALTY };

static GenType    selGen         = GenType::DFS_GRID;
static SearchType selSearch      = SearchType::ASTAR;
static PelletTool pelletTool     = PelletTool::REWARD_05;
static EditTool   editWallTool   = EditTool::WALL;
static bool       wallEditActive = false;
static MazeSizeE  selSize        = MazeSizeE::SMALL;
static ScenarioE  activeScenario = ScenarioE::NONE;

// Grid sizes: Small (15x15), Medium (20x20), Large (25x25)
static const int MSIZES[3][2] = {{15, 15}, {20, 20}, {25, 25}};

// Pellet Undo Stack
struct PelletHistory {
    Position    pos;
    TerrainType oldTerrain;
    SpecialType oldSpecial;
};
static std::vector<PelletHistory> pelletUndoStack;

// Simulation Controls
static bool paused       = false;
static int  stepsPerTick = 1;
static bool mouseDown    = false;

// Status & Messaging
static std::string statusMsg = "Phase 1: Create road network or select a scenario.";
static std::string errorMsg  = "";
static float       errTimer  = 0.f;

// Seed & RNG
static unsigned int currentSeed = 42;
static std::mt19937 globalRng;

// ════════════════════════════════════════════════════════════
// ── COLOR PALETTE (Professional Dark Presentation Theme) ────
// ════════════════════════════════════════════════════════════
namespace Col {
    static const float BG_DARK      [] = {0.04f, 0.06f, 0.09f}; // Deep navy canvas background
    static const float HEADER_BG    [] = {0.07f, 0.09f, 0.14f}; // Header bar
    static const float SIDEBAR_BG   [] = {0.08f, 0.10f, 0.15f}; // Sidebar panel
    static const float CARD_BG      [] = {0.11f, 0.14f, 0.20f}; // Section card box
    static const float BORDER       [] = {0.18f, 0.22f, 0.30f}; // UI panel borders

    static const float ROAD_NORM    [] = {0.09f, 0.11f, 0.16f}; // Traversable road cell
    static const float ROAD_LINE    [] = {0.14f, 0.17f, 0.23f}; // Road grid border
    static const float ROAD_CONGEST [] = {0.24f, 0.16f, 0.08f}; // Congested road tile
    static const float WALL_BLOCK   [] = {0.18f, 0.21f, 0.28f}; // Impassable wall cell
    static const float WALL_LINE    [] = {0.34f, 0.40f, 0.50f}; // Wall barrier line

    static const float START        [] = {0.10f, 0.75f, 0.38f}; // Store (S - Emerald)
    static const float GOAL         [] = {0.20f, 0.52f, 0.95f}; // Customer (G - Cobalt)
    static const float GREEN_REWARD [] = {0.12f, 0.68f, 0.38f}; // Reward shortcut (Emerald 0.5)
    static const float RED_PENALTY  [] = {0.82f, 0.22f, 0.22f}; // Penalty road (Red 4.0)
    static const float ORANGE_CONGEST[] = {0.85f, 0.52f, 0.12f}; // Congested road (Amber 2.0)
    static const float RIDER        [] = {0.00f, 0.88f, 0.95f}; // Delivery van

    static const float EXPLORED     [] = {0.12f, 0.24f, 0.42f}; // Explored search tint
    static const float FRONTIER     [] = {0.00f, 0.60f, 0.80f}; // Frontier tint
    static const float PATH         [] = {0.96f, 0.62f, 0.12f}; // Final route highlight

    static const float TEXT_WHITE   [] = {0.94f, 0.96f, 0.98f};
    static const float TEXT_MUTED   [] = {0.58f, 0.63f, 0.72f};
    static const float TITLE_CYAN   [] = {0.25f, 0.75f, 0.98f};
    static const float PRIMARY_BLUE [] = {0.18f, 0.45f, 0.85f};
    static const float ACCENT       [] = {0.35f, 0.65f, 0.95f};
}

// ════════════════════════════════════════════════════════════
// ── MAZE CANVAS GEOMETRY & COORDINATE MAPPING ───────────────
// ════════════════════════════════════════════════════════════
struct MazeLayout {
    float canvasX = 0.f;
    float canvasY = HEADER_H;
    float canvasW = 920.f;
    float canvasH = 734.f;
    float padding = 24.f;
    float cellSize = 36.f;
    float mazeW    = 540.f;
    float mazeH    = 540.f;
    float originX  = 190.f;
    float originY  = 120.f;

    void update(int winW, int winH, int rows, int cols) {
        canvasX = 0.f;
        canvasY = HEADER_H;
        canvasW = (float)(winW - SIDEBAR_W);
        canvasH = (float)(winH - HEADER_H);
        padding = 24.f;

        float availW = canvasW - 2.f * padding;
        float availH = canvasH - 2.f * padding;
        if (availW < 120.f) availW = 120.f;
        if (availH < 120.f) availH = 120.f;

        cellSize = std::floor(std::min(availW / (float)cols, availH / (float)rows));
        if (cellSize < 8.f) cellSize = 8.f;

        mazeW = cellSize * (float)cols;
        mazeH = cellSize * (float)rows;

        originX = canvasX + padding + std::floor((availW - mazeW) * 0.5f);
        originY = canvasY + padding + std::floor((availH - mazeH) * 0.5f);
    }

    bool hitTest(float mx, float my, int& outRow, int& outCol, int rows, int cols) const {
        if (mx < originX || mx >= originX + mazeW ||
            my < originY || my >= originY + mazeH) {
            return false;
        }
        outCol = (int)((mx - originX) / cellSize);
        outRow = (int)((my - originY) / cellSize);
        return (outRow >= 0 && outRow < rows && outCol >= 0 && outCol < cols);
    }

    float cellCenterX(int col) const { return originX + ((float)col + 0.5f) * cellSize; }
    float cellCenterY(int row) const { return originY + ((float)row + 0.5f) * cellSize; }
};
static MazeLayout mazeLayout;

// ════════════════════════════════════════════════════════════
// ── CORE OBJECTS ────────────────────────────────────────────
// ════════════════════════════════════════════════════════════
static Maze             maze(15, 15);
static DFSGenerator     dfsGen;
static PrimGenerator    primGen;
static MazeGenerator*   activeGen = nullptr;
static std::unique_ptr<SearchAlgorithm> solver;
static FrameRecorder    recorder("frames");

// Delivery Vehicle Path Animation
static std::vector<Position> riderPath;
static int    riderStep     = 0;
static float  riderProgress = 0.f;
static bool   riderActive   = false;
static float  riderSpeed    = 0.08f;

// Algorithm Comparison
struct CompResult {
    std::string name;
    SearchStats stats;
    bool        found = false;
};
static std::vector<CompResult> compResults;
static const int NUM_COMPARE_ALGOS = 3; // BFS, DFS, A*

// ════════════════════════════════════════════════════════════
// ── UI BUTTONS ──────────────────────────────────────────────
// ════════════════════════════════════════════════════════════
// Top Header
static Button btnTopHelp, btnTopLegend;

// Phase 1: Create Maze
static Button btnSzSm, btnSzMd, btnSzLg;
static Button btnGenDFS, btnGenPrim;
static Button btnGenerate, btnNewSeed;
static Button btnScenA, btnScenB, btnScenC;
static Button btnToggleWallEdit, btnClearWalls;
static Button btnPhase1Continue;

// Phase 2: Configure Roads (Equal 2-Column Grid)
static Button btnToolReward, btnToolPenalty, btnToolCongested, btnToolNormal;
static Button btnUndoPellet, btnClearPellets;
static Button btnPhase2Continue, btnPhase2Back;

// Phase 3: Solve & Compare
static Button btnSelBFS, btnSelDFS, btnSelAStar;
static Button btnRun, btnResetSearch, btnPause, btnStep;
static Button btnAnimateRoute, btnStopAnim;
static Button btnCompareAll;
static Button btnPhase3BackToConfig, btnPhase3NewMaze;

// Modal / Screen Return
static Button btnHelpBack, btnLegendBack, btnBackToWorkflow;

static std::vector<Button*> allButtons;

// Forward declarations
static void doGenerate();
static void doHardReset();
static void doResetSearch();
static void doStartSolving();
static void doRunComparison();
static void doStep();
static void loadScenarioA();
static void loadScenarioB();
static void loadScenarioC();
static void showError(const std::string& msg, float dur = 3.f);
static void startRiderAnimation();
static void stopRiderAnimation();

// ════════════════════════════════════════════════════════════
// ── BUTTON LAYOUT & ALIGNMENT ───────────────────────────────
// ════════════════════════════════════════════════════════════
static void initButtonLayout() {
    allButtons.clear();

    const float pad    = 16.f;
    const float sideX  = (float)(WIN_W - SIDEBAR_W) + pad;
    const float fullW  = SIDEBAR_W - 2.f * pad;               // 288px
    const float gap    = 8.f;
    const float halfW  = (fullW - gap) * 0.5f;                // 140px
    const float col2X  = sideX + halfW + gap;                 // sideX + 148px
    const float thirdW = (fullW - 2.f * gap) / 3.f;           // ~90.66px
    const float th2X   = sideX + thirdW + gap;
    const float th3X   = sideX + 2.f * (thirdW + gap);

    // Top Header Buttons
    btnTopLegend = Button((float)WIN_W - 150.f, 10.f, 65.f, 26.f, "Legend");
    btnTopHelp   = Button((float)WIN_W - 75.f,  10.f, 65.f, 26.f, "Help");
    allButtons.push_back(&btnTopLegend);
    allButtons.push_back(&btnTopHelp);

    // ────────────────────────────────────────────────────────
    // Phase 1 Setup: Create Road Network
    // ────────────────────────────────────────────────────────
    float y1 = HEADER_H + 16.f;
    y1 += 58.f; // Space for Phase 1 header card

    // MAZE SETUP — Grid Size
    y1 += 14.f;
    btnSzSm = Button(sideX, y1, thirdW, 28.f, "15x15");
    btnSzMd = Button(th2X,  y1, thirdW, 28.f, "20x20");
    btnSzLg = Button(th3X,  y1, thirdW, 28.f, "25x25");
    y1 += 34.f;

    // Generator method
    btnGenDFS  = Button(sideX, y1, halfW, 28.f, "DFS Grid");
    btnGenPrim = Button(col2X, y1, halfW, 28.f, "Prim Branch");
    y1 += 34.f;

    // Actions
    btnGenerate = Button(sideX, y1, halfW, 30.f, "Generate Maze"); btnGenerate.setGreen();
    btnNewSeed  = Button(col2X, y1, halfW, 30.f, "New / Reset");   btnNewSeed.setNeutral();
    y1 += 38.f;

    // Demo Scenarios (compact 3-button row)
    y1 += 14.f;
    btnScenA = Button(sideX, y1, thirdW, 26.f, "Scenario A");
    btnScenB = Button(th2X,  y1, thirdW, 26.f, "Scenario B");
    btnScenC = Button(th3X,  y1, thirdW, 26.f, "Scenario C");
    y1 += 32.f;

    // Wall editing tools
    btnToggleWallEdit = Button(sideX, y1, halfW, 26.f, "Edit Walls");
    btnClearWalls     = Button(col2X, y1, halfW, 26.f, "Open Grid");
    y1 += 38.f;

    // Grid Legend occupies y1 to y1 + 88px
    y1 += 98.f;

    // Navigation
    y1 += 14.f;
    btnPhase1Continue = Button(sideX, y1, fullW, 38.f, "CONTINUE TO ROAD CONFIG ->");
    btnPhase1Continue.setGreen();

    allButtons.push_back(&btnSzSm); allButtons.push_back(&btnSzMd); allButtons.push_back(&btnSzLg);
    allButtons.push_back(&btnGenDFS); allButtons.push_back(&btnGenPrim);
    allButtons.push_back(&btnGenerate); allButtons.push_back(&btnNewSeed);
    allButtons.push_back(&btnScenA); allButtons.push_back(&btnScenB); allButtons.push_back(&btnScenC);
    allButtons.push_back(&btnToggleWallEdit); allButtons.push_back(&btnClearWalls);
    allButtons.push_back(&btnPhase1Continue);

    // ────────────────────────────────────────────────────────
    // Phase 2 Setup: Configure Roads
    // ────────────────────────────────────────────────────────
    float y2 = HEADER_H + 16.f;
    y2 += 58.f; // Space for Phase 2 header card

    // Road Tools (Clean 2-Column Grid with Equal-Sized Buttons)
    y2 += 14.f;
    btnToolReward    = Button(sideX, y2, halfW, 32.f, "Reward · 0.5");  btnToolReward.setGreen();
    btnToolPenalty   = Button(col2X, y2, halfW, 32.f, "Penalty · 4.0"); btnToolPenalty.setRed();
    y2 += 38.f;
    btnToolCongested = Button(sideX, y2, halfW, 32.f, "Congested · 2.0"); btnToolCongested.setAmber();
    btnToolNormal    = Button(col2X, y2, halfW, 32.f, "Normal / Erase · 1.0"); btnToolNormal.setNeutral();
    y2 += 44.f;

    // Pellet Summary Card occupies y2 to y2 + 74px
    y2 += 88.f;

    // Configuration Actions
    y2 += 14.f;
    btnUndoPellet   = Button(sideX, y2, halfW, 30.f, "Undo Last");     btnUndoPellet.setNeutral();
    btnClearPellets = Button(col2X, y2, halfW, 30.f, "Clear Pellets"); btnClearPellets.setRed();
    y2 += 42.f;

    // Workflow Navigation
    y2 += 14.f;
    btnPhase2Continue = Button(sideX, y2, fullW, 38.f, "CONTINUE TO SOLVING ->"); btnPhase2Continue.setPrimary();
    y2 += 46.f;
    btnPhase2Back     = Button(sideX, y2, fullW, 30.f, "<- BACK TO MAZE CREATION"); btnPhase2Back.setNeutral();

    allButtons.push_back(&btnToolReward); allButtons.push_back(&btnToolPenalty);
    allButtons.push_back(&btnToolCongested); allButtons.push_back(&btnToolNormal);
    allButtons.push_back(&btnUndoPellet); allButtons.push_back(&btnClearPellets);
    allButtons.push_back(&btnPhase2Continue); allButtons.push_back(&btnPhase2Back);

    // ────────────────────────────────────────────────────────
    // Phase 3 Setup: Solve and Compare
    // ────────────────────────────────────────────────────────
    float y3 = HEADER_H + 16.f;
    y3 += 58.f; // Space for Phase 3 header card

    // Algorithm selection
    y3 += 14.f;
    btnSelBFS   = Button(sideX, y3, fullW, 28.f, "BFS -- Fewest Moves");        y3 += 32.f;
    btnSelDFS   = Button(sideX, y3, fullW, 28.f, "DFS -- Depth-First");         y3 += 32.f;
    btnSelAStar = Button(sideX, y3, fullW, 28.f, "A* -- Lowest Weighted Cost"); btnSelAStar.setBlue(); y3 += 36.f;

    // Execution Controls
    y3 += 14.f;
    btnRun         = Button(sideX, y3, halfW, 30.f, "Run Search");   btnRun.setGreen();
    btnResetSearch = Button(col2X, y3, halfW, 30.f, "Reset Search"); btnResetSearch.setNeutral();
    y3 += 36.f;

    btnStep  = Button(sideX, y3, halfW, 28.f, "Step");
    btnPause = Button(col2X, y3, halfW, 28.f, "Pause / Resume");
    y3 += 34.f;

    btnAnimateRoute = Button(sideX, y3, halfW, 28.f, "Animate Van"); btnAnimateRoute.setAmber();
    btnStopAnim     = Button(col2X, y3, halfW, 28.f, "Stop Anim");
    y3 += 38.f;

    // Results Card occupies y3 to y3 + 124px
    y3 += 136.f;

    // Navigation
    y3 += 14.f;
    btnCompareAll = Button(sideX, y3, fullW, 36.f, "COMPARE ALL (BFS, DFS, A*)"); btnCompareAll.setPrimary();
    y3 += 44.f;
    btnPhase3BackToConfig = Button(sideX, y3, halfW, 28.f, "<- Road Config");
    btnPhase3NewMaze      = Button(col2X, y3, halfW, 28.f, "New Network");

    allButtons.push_back(&btnSelBFS); allButtons.push_back(&btnSelDFS); allButtons.push_back(&btnSelAStar);
    allButtons.push_back(&btnRun); allButtons.push_back(&btnResetSearch);
    allButtons.push_back(&btnStep); allButtons.push_back(&btnPause);
    allButtons.push_back(&btnAnimateRoute); allButtons.push_back(&btnStopAnim);
    allButtons.push_back(&btnCompareAll);
    allButtons.push_back(&btnPhase3BackToConfig); allButtons.push_back(&btnPhase3NewMaze);

    // Modal / Screen Return Buttons
    btnHelpBack   = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 52.f, 160.f, 32.f, "RETURN");
    btnLegendBack = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 52.f, 160.f, 32.f, "RETURN");
    btnBackToWorkflow = Button((float)WIN_W * 0.5f - 110.f, (float)WIN_H - 52.f, 220.f, 34.f, "RETURN TO SOLVING");
    btnBackToWorkflow.setGreen();

    allButtons.push_back(&btnHelpBack);
    allButtons.push_back(&btnLegendBack);
    allButtons.push_back(&btnBackToWorkflow);
}

// ════════════════════════════════════════════════════════════
// ── FACTORY HELPERS ─────────────────────────────────────────
// ════════════════════════════════════════════════════════════
static std::unique_ptr<SearchAlgorithm> makeAlgo(SearchType t) {
    switch (t) {
        case SearchType::BFS:   return std::unique_ptr<SearchAlgorithm>(new BFSAlgorithm());
        case SearchType::DFS_S: return std::unique_ptr<SearchAlgorithm>(new DFSAlgorithm());
        case SearchType::ASTAR: return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
    }
    return std::unique_ptr<SearchAlgorithm>(new AStarAlgorithm());
}

static MazeGenerator* getGen() {
    switch (selGen) {
        case GenType::DFS_GRID:    return &dfsGen;
        case GenType::PRIM_BRANCH: return &primGen;
    }
    return &dfsGen;
}

static void showError(const std::string& msg, float dur) {
    errorMsg = msg;
    errTimer = dur;
}

// ════════════════════════════════════════════════════════════
// ── DELIVERY VAN ANIMATION ──────────────────────────────────
// ════════════════════════════════════════════════════════════
static void startRiderAnimation() {
    if (!solver || !solver->foundPath()) {
        showError("No route found. Run search first.");
        return;
    }
    riderPath     = solver->getPath();
    riderStep     = 0;
    riderProgress = 0.f;
    riderActive   = true;
    statusMsg     = "Delivery van en route to customer...";
}

static void stopRiderAnimation() {
    riderActive = false;
    riderStep   = 0;
    statusMsg   = "Delivery animation stopped.";
}

static void updateRider() {
    if (!riderActive) return;
    if (riderPath.empty() || riderStep >= (int)riderPath.size() - 1) {
        riderActive = false;
        statusMsg   = "Package delivered! Reached destination customer.";
        return;
    }
    riderProgress += riderSpeed;
    if (riderProgress >= 1.f) {
        riderProgress = 0.f;
        riderStep++;
        if (riderStep >= (int)riderPath.size() - 1) {
            riderActive = false;
            statusMsg   = "Package delivered! Reached destination customer.";
        }
    }
}

// ════════════════════════════════════════════════════════════
// ── APPLICATION LOGIC & ACTIONS ─────────────────────────────
// ════════════════════════════════════════════════════════════
static void doGenerate() {
    if (appState == AppState::SOLVING || appState == AppState::GENERATING) {
        showError("Stop current operation first.");
        return;
    }
    currentSeed = (unsigned int)std::time(nullptr) + (unsigned int)rand();
    globalRng.seed(currentSeed);

    maze.hardReset();
    activeGen = getGen();
    activeGen->initialize(maze, currentSeed);
    appState     = AppState::GENERATING;
    currentPhase = AppPhase::PHASE_1_CREATE;
    pelletUndoStack.clear();
    paused    = false;
    statusMsg = "Generating road network (Phase 1)...";
    solver.reset();
    compResults.clear();
    riderActive    = false;
    activeScenario = ScenarioE::NONE;
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

static void doHardReset() {
    maze.hardReset();
    solver.reset();
    activeGen = nullptr;
    compResults.clear();
    currentPhase    = AppPhase::PHASE_1_CREATE;
    pelletUndoStack.clear();
    appState        = AppState::MAZE_WORKFLOW;
    statusMsg       = "Network reset. Phase 1: Ready to generate new map.";
    paused          = false;
    errorMsg        = "";
    riderActive     = false;
    activeScenario  = ScenarioE::NONE;
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

static void doResetSearch() {
    maze.resetSearch();
    solver.reset();
    riderActive = false;
    compResults.clear();
    paused    = false;
    errorMsg  = "";
    statusMsg = "Route reset. Configured pellets and terrain preserved.";
}

static void doEnterRoadConfig() {
    if (!maze.generated) {
        showError("Create or generate a road network first.");
        return;
    }
    maze.resetSearch();
    solver.reset();
    riderActive  = false;
    currentPhase = AppPhase::PHASE_2_CONFIG;
    appState     = AppState::MAZE_WORKFLOW;
    statusMsg    = "Phase 2: Configure Road Costs. Click cells to place pellets.";
}

static void doStartSolving() {
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    if (maze.start == maze.goal) {
        showError("Store == Customer -- Invalid setup!");
        return;
    }

    maze.resetSearch();
    riderActive  = false;
    currentPhase = AppPhase::PHASE_3_SOLVE;
    solver = makeAlgo(selSearch);
    solver->initialize(maze, maze.start, maze.goal);
    appState = AppState::SOLVING;
    paused   = false;

    // Instant pure execution timing measurement (excluding animation)
    Maze mazeCopy = maze;
    auto testAlgo = makeAlgo(selSearch);
    testAlgo->initialize(mazeCopy, maze.start, maze.goal);
    SearchStats pureStats = testAlgo->runToCompletion();
    solver->stats.timeMs = pureStats.timeMs;

    std::string algName;
    switch (selSearch) {
        case SearchType::BFS:   algName = "BFS (Fewest Moves)"; break;
        case SearchType::DFS_S: algName = "DFS (Depth-First)"; break;
        case SearchType::ASTAR: algName = "A* (Lowest Weighted Cost)"; break;
    }
    statusMsg = "Solving with " + algName + "...";
}

static void doRunComparison() {
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    compResults.clear();

    static const SearchType types[NUM_COMPARE_ALGOS] = {
        SearchType::BFS, SearchType::DFS_S, SearchType::ASTAR
    };

    for (int i = 0; i < NUM_COMPARE_ALGOS; i++) {
        Maze copy = maze;
        copy.resetAllFlags();

        std::unique_ptr<SearchAlgorithm> algo = makeAlgo(types[i]);
        algo->initialize(copy, copy.start, copy.goal);
        SearchStats st = algo->runToCompletion();

        CompResult cr;
        cr.name  = algo->getName();
        cr.stats = st;
        cr.found = algo->foundPath();
        compResults.push_back(cr);
    }

    appState  = AppState::COMPARING;
    statusMsg = "Comparison complete: Same network & pellets across all 3 algorithms.";
}

static void doStep() {
    if (appState == AppState::SOLVING && solver && !solver->isFinished()) {
        solver->step();
        if (solver->isFinished()) {
            appState  = AppState::SOLVED;
            statusMsg = solver->foundPath()
                ? "Route found! Press ANIMATE VAN to view delivery path."
                : "No route exists -- customer unreachable.";
        }
    } else if (appState == AppState::GENERATING && activeGen && !activeGen->isFinished()) {
        activeGen->step();
        if (activeGen->isFinished()) {
            appState  = AppState::MAZE_WORKFLOW;
            statusMsg = "Road network ready. Click CONTINUE TO ROAD CONFIG.";
        }
    }
}

// ════════════════════════════════════════════════════════════
// ── PREDEFINED PRESENTATION SCENARIOS ───────────────────────
// ════════════════════════════════════════════════════════════
static void buildOpenGrid(int rows, int cols) {
    maze.resize(rows, cols);
    maze.hardReset();
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (r + 1 < rows) maze.removeWall({r, c}, {r + 1, c});
            if (c + 1 < cols) maze.removeWall({r, c}, {r, c + 1});
        }
    }
    maze.generated = true;
    maze.start = {0, 0};
    maze.goal  = {rows - 1, cols - 1};
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

static void loadScenarioA() {
    const int ROWS = 11, COLS = 15;
    buildOpenGrid(ROWS, COLS);

    auto blk = [&](int r, int c) { maze.blockCell({r, c}); };
    for (int r = 2; r <= 6; r++) blk(r, 7);
    for (int c = 3; c <= 6; c++) blk(4, c);
    for (int r = 7; r <= 9; r++) blk(r, 4);

    maze.start = {0, 0};
    maze.goal  = {ROWS - 1, COLS - 1};
    selSize    = MazeSizeE::SMALL;
    activeScenario = ScenarioE::A_SIMPLE;
    solver.reset();
    compResults.clear();
    riderActive     = false;
    currentPhase    = AppPhase::PHASE_1_CREATE;
    pelletUndoStack.clear();
    appState  = AppState::MAZE_WORKFLOW;
    statusMsg = "Scenario A: Simple Delivery loaded. Click Continue to configure.";
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

static void loadScenarioB() {
    const int ROWS = 13, COLS = 19;
    buildOpenGrid(ROWS, COLS);

    auto blk = [&](int r, int c) { maze.blockCell({r, c}); };
    for (int r = 2; r <= 10; r++) blk(r, 9);
    maze.unblockCell({6, 9});
    maze.unblockCell({9, 9});

    // Reward shortcut cells on upper route
    maze.at(3, 3).special  = SpecialType::REWARD;
    maze.at(3, 5).special  = SpecialType::REWARD;
    maze.at(3, 7).special  = SpecialType::REWARD;
    maze.at(4, 11).special = SpecialType::REWARD;
    maze.at(4, 13).special = SpecialType::REWARD;
    maze.at(4, 15).special = SpecialType::REWARD;

    maze.start = {0, 0};
    maze.goal  = {ROWS - 1, COLS - 1};
    selSize    = MazeSizeE::MEDIUM;
    activeScenario = ScenarioE::B_REWARD;
    solver.reset();
    compResults.clear();
    riderActive     = false;
    currentPhase    = AppPhase::PHASE_1_CREATE;
    pelletUndoStack.clear();
    appState  = AppState::MAZE_WORKFLOW;
    statusMsg = "Scenario B: Reward Shortcut loaded. Click Continue to configure.";
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

static void loadScenarioC() {
    const int ROWS = 13, COLS = 19;
    buildOpenGrid(ROWS, COLS);

    auto blk = [&](int r, int c) { maze.blockCell({r, c}); };
    for (int r = 0; r <= 8; r++) blk(r, 9);
    maze.unblockCell({4, 9});

    // High penalty corridor (cost 4.0)
    maze.at(4, 9).special = SpecialType::PENALTY;
    maze.at(3, 9).special = SpecialType::PENALTY;
    maze.at(5, 9).special = SpecialType::PENALTY;

    // Detour reward shortcut
    maze.at(11, 5).special  = SpecialType::REWARD;
    maze.at(11, 7).special  = SpecialType::REWARD;
    maze.at(11, 12).special = SpecialType::REWARD;
    maze.at(11, 14).special = SpecialType::REWARD;

    maze.start = {0, 0};
    maze.goal  = {ROWS - 1, COLS - 1};
    selSize    = MazeSizeE::MEDIUM;
    activeScenario = ScenarioE::C_PENALTY;
    solver.reset();
    compResults.clear();
    riderActive     = false;
    currentPhase    = AppPhase::PHASE_1_CREATE;
    pelletUndoStack.clear();
    appState  = AppState::MAZE_WORKFLOW;
    statusMsg = "Scenario C: Penalty Corridor loaded. Click Continue to configure.";
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
}

// ════════════════════════════════════════════════════════════
// ── INTERACTIVE MOUSE TERRAIN EDITING ───────────────────────
// ════════════════════════════════════════════════════════════
static void applyWallEditTool(int row, int col) {
    if (!maze.inBounds(row, col)) return;
    Position p{row, col};
    if (p == maze.start || p == maze.goal) return;

    if (editWallTool == EditTool::WALL) {
        maze.blockCell(p);
        maze.at(p).special = SpecialType::NONE;
    } else {
        maze.unblockCell(p);
        maze.at(p).special = SpecialType::NONE;
    }
    maze.resetSearch();
    solver.reset();
    riderActive = false;
    compResults.clear();
}

static void applyRoadConfigTool(int row, int col) {
    if (!maze.inBounds(row, col)) return;
    Position p{row, col};

    // Placement restrictions
    if (maze.at(p).isWall()) {
        showError("Cannot place pellet on wall / barrier.");
        return;
    }
    if (p == maze.start) {
        showError("Cannot overwrite Store (Start).");
        return;
    }
    if (p == maze.goal) {
        showError("Cannot overwrite Customer (Goal).");
        return;
    }

    PelletHistory hist{p, maze.at(p).terrain, maze.at(p).special};
    bool changed = false;

    switch (pelletTool) {
        case PelletTool::REWARD_05:
            if (maze.at(p).special != SpecialType::REWARD || maze.at(p).terrain != TerrainType::NORMAL) {
                maze.at(p).special = SpecialType::REWARD;
                maze.at(p).terrain = TerrainType::NORMAL;
                changed   = true;
                statusMsg = "Placed Reward Pellet (Cost 0.5) at (" + std::to_string(row) + "," + std::to_string(col) + ")";
            }
            break;
        case PelletTool::PENALTY_40:
            if (maze.at(p).special != SpecialType::PENALTY || maze.at(p).terrain != TerrainType::NORMAL) {
                maze.at(p).special = SpecialType::PENALTY;
                maze.at(p).terrain = TerrainType::NORMAL;
                changed   = true;
                statusMsg = "Placed Penalty Pellet (Cost 4.0) at (" + std::to_string(row) + "," + std::to_string(col) + ")";
            }
            break;
        case PelletTool::CONGESTED_20:
            if (maze.at(p).terrain != TerrainType::CONGESTED || maze.at(p).special != SpecialType::NONE) {
                maze.at(p).terrain = TerrainType::CONGESTED;
                maze.at(p).special = SpecialType::NONE;
                changed   = true;
                statusMsg = "Configured Congested Road (Cost 2.0) at (" + std::to_string(row) + "," + std::to_string(col) + ")";
            }
            break;
        case PelletTool::NORMAL_10:
            if (maze.at(p).special != SpecialType::NONE || maze.at(p).terrain != TerrainType::NORMAL) {
                maze.at(p).special = SpecialType::NONE;
                maze.at(p).terrain = TerrainType::NORMAL;
                changed   = true;
                statusMsg = "Restored Normal Road (Cost 1.0) at (" + std::to_string(row) + "," + std::to_string(col) + ")";
            }
            break;
    }

    if (changed) {
        pelletUndoStack.push_back(hist);
        maze.resetSearch();
        solver.reset();
        riderActive = false;
        compResults.clear();
    }
}

// ════════════════════════════════════════════════════════════
// ── TOP HEADER BAR RENDERING ────────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderTopHeader() {
    // Background bar
    drawPanel(0.f, 0.f, (float)WIN_W, HEADER_H, Col::HEADER_BG, Col::BORDER, 1.0f);

    // Left Title Branding & Subtitle
    glColor3fv(Col::TITLE_CYAN);
    drawBitmapText(18.f, 26.f, "SmartRoute AI", GLUT_BITMAP_HELVETICA_18);

    glColor3fv(Col::TEXT_MUTED);
    drawBitmapText(158.f, 24.f, "|  AI Search Strategies for Last-Mile Delivery", GLUT_BITMAP_HELVETICA_10);

    // Center Current Phase Indicator Badge
    float centerX = (float)(WIN_W - SIDEBAR_W) * 0.5f - 110.f;
    float bw = 220.f, bh = 28.f, by = 9.f;
    float bgPhase[3] = {0.12f, 0.28f, 0.55f};
    float bdPhase[3] = {0.30f, 0.60f, 0.95f};
    drawPanel(centerX, by, bw, bh, bgPhase, bdPhase, 1.5f);

    const char* phaseStr = "1 / 3 -- Create Maze";
    if (currentPhase == AppPhase::PHASE_2_CONFIG) phaseStr = "2 / 3 -- Configure Roads";
    else if (currentPhase == AppPhase::PHASE_3_SOLVE) phaseStr = "3 / 3 -- Solve and Compare";

    glColor3fv(Col::TEXT_WHITE);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)phaseStr);
    drawBitmapText(centerX + (bw - (float)tw) * 0.5f, by + 18.f, phaseStr, GLUT_BITMAP_HELVETICA_12);

    // Right Header Utility Buttons
    btnTopLegend.draw();
    btnTopHelp.draw();
}

// ════════════════════════════════════════════════════════════
// ── MAZE CANVAS RENDERING ───────────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderCell(int row, int col, Position solverCur) {
    float cw = mazeLayout.cellSize;
    float x  = mazeLayout.originX + col * cw;
    float y  = mazeLayout.originY + row * cw;
    const Cell& cell = maze.at(row, col);
    Position curPos{row, col};

    // 1. Base Cell Fill
    const float* fill = Col::ROAD_NORM;
    if (cell.isWall()) {
        fill = Col::WALL_BLOCK;
    } else {
        if (cell.terrain == TerrainType::CONGESTED && cell.special == SpecialType::NONE) {
            fill = Col::ROAD_CONGEST;
        }
        if (cell.explored)   fill = Col::EXPLORED;
        if (cell.inFrontier) fill = Col::FRONTIER;
        if (curPos == maze.start) fill = Col::START;
        else if (curPos == maze.goal) fill = Col::GOAL;
        if (solverCur.isValid() && curPos == solverCur) fill = Col::TEXT_WHITE;
    }

    glColor3fv(fill);
    glBegin(GL_QUADS);
    glVertex2f(x, y);           glVertex2f(x + cw, y);
    glVertex2f(x + cw, y + cw); glVertex2f(x, y + cw);
    glEnd();

    // Subtle Road Grid Line
    if (!cell.isWall()) {
        glColor3fv(Col::ROAD_LINE);
        glLineWidth(1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(x, y);           glVertex2f(x + cw, y);
        glVertex2f(x + cw, y + cw); glVertex2f(x, y + cw);
        glEnd();
    }

    // Congested Road Marker (cost 2.0 indicator lines)
    if (!cell.isWall() && cell.terrain == TerrainType::CONGESTED && cell.special == SpecialType::NONE) {
        float cx = x + cw * 0.5f, cy = y + cw * 0.5f;
        float r  = cw * 0.28f;
        glColor3fv(Col::ORANGE_CONGEST);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex2f(cx - r, cy - 2.f); glVertex2f(cx + r, cy - 2.f);
        glVertex2f(cx - r, cy + 3.f); glVertex2f(cx + r, cy + 3.f);
        glEnd();
        glLineWidth(1.0f);
    }

    // 2. Route Path Highlight (tinted ribbon if on final path)
    if (cell.inPath && curPos != maze.start && curPos != maze.goal) {
        glColor3f(Col::PATH[0] * 0.35f, Col::PATH[1] * 0.35f, Col::PATH[2] * 0.35f);
        glBegin(GL_QUADS);
        glVertex2f(x + 2.f, y + 2.f);
        glVertex2f(x + cw - 2.f, y + 2.f);
        glVertex2f(x + cw - 2.f, y + cw - 2.f);
        glVertex2f(x + 2.f, y + cw - 2.f);
        glEnd();
    }

    // 3. Pellets Overlay (Rendered with clear contrast)
    if (!cell.isWall() && cell.special != SpecialType::NONE) {
        float cx = x + cw * 0.5f, cy = y + cw * 0.5f;
        float r  = cw * 0.32f;

        if (cell.special == SpecialType::REWARD) {
            // Emerald Green Diamond ◆
            glColor3fv(Col::GREEN_REWARD);
            drawDiamond(cx, cy, r);
            // Core sparkle highlight
            glColor3f(0.55f, 1.0f, 0.75f);
            drawDiamond(cx, cy, r * 0.45f);
        } else if (cell.special == SpecialType::PENALTY) {
            // Crimson Red Cross ✕
            glColor3fv(Col::RED_PENALTY);
            float w2 = r * 0.30f;
            glBegin(GL_QUADS);
            glVertex2f(cx - r, cy - w2); glVertex2f(cx + r, cy - w2);
            glVertex2f(cx + r, cy + w2); glVertex2f(cx - r, cy + w2);
            glEnd();
            glBegin(GL_QUADS);
            glVertex2f(cx - w2, cy - r); glVertex2f(cx + w2, cy - r);
            glVertex2f(cx + w2, cy + r); glVertex2f(cx - w2, cy + r);
            glEnd();
        }
    }

    // 4. Wall Segments (Crisp, aligned borders)
    if (!cell.isWall()) {
        glColor3fv(Col::WALL_LINE);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        if (cell.walls[TOP])    { glVertex2f(x, y);           glVertex2f(x + cw, y); }
        if (cell.walls[RIGHT])  { glVertex2f(x + cw, y);      glVertex2f(x + cw, y + cw); }
        if (cell.walls[BOTTOM]) { glVertex2f(x, y + cw);      glVertex2f(x + cw, y + cw); }
        if (cell.walls[LEFT])   { glVertex2f(x, y);           glVertex2f(x, y + cw); }
        glEnd();
        glLineWidth(1.0f);
    }

    // 5. Start (S) & Goal (G) Icons (Centered with consistent dimensions)
    if (curPos == maze.start) {
        float inset = cw * 0.14f;
        float mw = cw - 2.f * inset;
        glColor3f(0.06f, 0.48f, 0.22f);
        glBegin(GL_QUADS);
        glVertex2f(x + inset, y + inset);           glVertex2f(x + inset + mw, y + inset);
        glVertex2f(x + inset + mw, y + inset + mw); glVertex2f(x + inset, y + inset + mw);
        glEnd();
        glColor3fv(Col::TEXT_WHITE);
        int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)"S");
        drawBitmapText(x + (cw - (float)tw) * 0.5f, y + cw * 0.5f + 4.f, "S", GLUT_BITMAP_HELVETICA_12);
    }
    if (curPos == maze.goal) {
        float inset = cw * 0.14f;
        float mw = cw - 2.f * inset;
        glColor3f(0.12f, 0.32f, 0.70f);
        glBegin(GL_QUADS);
        glVertex2f(x + inset, y + inset);           glVertex2f(x + inset + mw, y + inset);
        glVertex2f(x + inset + mw, y + inset + mw); glVertex2f(x + inset, y + inset + mw);
        glEnd();
        glColor3fv(Col::TEXT_WHITE);
        int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)"G");
        drawBitmapText(x + (cw - (float)tw) * 0.5f, y + cw * 0.5f + 4.f, "G", GLUT_BITMAP_HELVETICA_12);
    }
}

// Draw continuous delivery path line
static void renderRouteRibbon() {
    if (!solver || !solver->foundPath()) return;
    const auto& path = solver->getPath();
    if (path.size() < 2) return;

    glColor3fv(Col::PATH);
    glLineWidth(3.5f);
    glBegin(GL_LINE_STRIP);
    for (const Position& p : path) {
        glVertex2f(mazeLayout.cellCenterX(p.col), mazeLayout.cellCenterY(p.row));
    }
    glEnd();
    glLineWidth(1.0f);
}

static void renderDeliveryVan() {
    if (!riderActive || riderPath.empty()) return;

    Position a = riderPath[riderStep];
    Position b = (riderStep + 1 < (int)riderPath.size()) ? riderPath[riderStep + 1] : a;
    float ax = mazeLayout.cellCenterX(a.col), ay = mazeLayout.cellCenterY(a.row);
    float bx = mazeLayout.cellCenterX(b.col), by = mazeLayout.cellCenterY(b.row);

    float px = (1.f - riderProgress) * ax + riderProgress * bx;
    float py = (1.f - riderProgress) * ay + riderProgress * by;
    float pr = mazeLayout.cellSize * 0.36f;

    // Outer glow
    glColor3f(0.0f, 0.25f, 0.35f);
    drawCircle(px, py, pr + 3.f);

    // Van Body
    glColor3fv(Col::RIDER);
    drawCircle(px, py, pr);

    // Border
    glColor3f(0.0f, 0.45f, 0.55f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i <= 24; i++) {
        float ang = 6.2831853f * (float)i / 24.f;
        glVertex2f(px + pr * cosf(ang), py + pr * sinf(ang));
    }
    glEnd();
    glLineWidth(1.0f);

    // Center indicator
    glColor3f(0.04f, 0.12f, 0.24f);
    drawCircle(px, py, pr * 0.45f);
}

static void renderMazeCanvas() {
    // Canvas Background
    drawPanel(0.f, HEADER_H, mazeLayout.canvasW, mazeLayout.canvasH, Col::BG_DARK);

    // Outer frame framing the maze grid cleanly
    drawPanel(mazeLayout.originX - 2.f, mazeLayout.originY - 2.f,
              mazeLayout.mazeW + 4.f, mazeLayout.mazeH + 4.f,
              nullptr, Col::BORDER, 2.0f);

    Position solverCur = Position::invalid();
    if (solver && !solver->isFinished())
        solverCur = solver->currentPos;
    else if (appState == AppState::GENERATING && activeGen)
        solverCur = activeGen->currentCell;

    for (int r = 0; r < maze.rows; r++) {
        for (int c = 0; c < maze.cols; c++) {
            renderCell(r, c, solverCur);
        }
    }

    renderRouteRibbon();
    renderDeliveryVan();

    // Destination Unreachable Notice
    if (appState == AppState::SOLVED && solver && !solver->foundPath()) {
        float cw = mazeLayout.canvasW, ch = mazeLayout.canvasH;
        float bx = cw * 0.5f - 160.f, by = HEADER_H + ch * 0.5f - 30.f;
        float bgErr[3] = {0.20f, 0.08f, 0.08f};
        drawPanel(bx, by, 320.f, 60.f, bgErr, Col::RED_PENALTY, 2.0f);
        glColor3fv(Col::RED_PENALTY);
        drawBitmapText(bx + 18.f, by + 26.f, "NO DELIVERY ROUTE FOUND", GLUT_BITMAP_HELVETICA_12);
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(bx + 18.f, by + 46.f, "Customer location is unreachable from Store.", GLUT_BITMAP_HELVETICA_10);
    }
}

// ════════════════════════════════════════════════════════════
// ── RIGHT SIDEBAR RENDERING (PHASE SPECIFIC) ────────────────
// ════════════════════════════════════════════════════════════
static void renderSidebar() {
    float sideX = (float)(WIN_W - SIDEBAR_W);
    drawPanel(sideX, HEADER_H, SIDEBAR_W, (float)(WIN_H - HEADER_H), Col::SIDEBAR_BG, Col::BORDER, 1.0f);

    const float pad   = 16.f;
    const float x     = sideX + pad;
    const float fullW = SIDEBAR_W - 2.f * pad;

    // ────────────────────────────────────────────────────────
    // PHASE 1 — CREATE ROAD NETWORK
    // ────────────────────────────────────────────────────────
    if (currentPhase == AppPhase::PHASE_1_CREATE) {
        float y = HEADER_H + 16.f;
        drawPhaseHeader(x, y, fullW, "PHASE 1", "CREATE MAZE",
                        "Generate street grid or configure road layout.");
        y += 68.f;

        drawSectionHeader(x, y, "MAZE SETUP"); y += 12.f;
        btnSzSm.selected = (selSize == MazeSizeE::SMALL);
        btnSzMd.selected = (selSize == MazeSizeE::MEDIUM);
        btnSzLg.selected = (selSize == MazeSizeE::LARGE);
        btnSzSm.draw(); btnSzMd.draw(); btnSzLg.draw();
        y += 34.f;

        btnGenDFS.selected  = (selGen == GenType::DFS_GRID);
        btnGenPrim.selected = (selGen == GenType::PRIM_BRANCH);
        btnGenDFS.draw(); btnGenPrim.draw();
        y += 34.f;

        btnGenerate.draw(); btnNewSeed.draw();
        y += 38.f;

        drawSectionHeader(x, y, "DEMO SCENARIOS"); y += 12.f;
        btnScenA.selected = (activeScenario == ScenarioE::A_SIMPLE); btnScenA.draw();
        btnScenB.selected = (activeScenario == ScenarioE::B_REWARD); btnScenB.draw();
        btnScenC.selected = (activeScenario == ScenarioE::C_PENALTY); btnScenC.draw();
        y += 34.f;

        btnToggleWallEdit.selected = wallEditActive;
        btnToggleWallEdit.draw(); btnClearWalls.draw();
        y += 40.f;

        drawGridLegend(x, y, fullW);
        y += 100.f;

        drawSectionHeader(x, y, "WORKFLOW NAVIGATION"); y += 12.f;
        btnPhase1Continue.draw();
    }

    // ────────────────────────────────────────────────────────
    // PHASE 2 — CONFIGURE REWARD & PENALTY ROADS
    // ────────────────────────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
        float y = HEADER_H + 16.f;
        drawPhaseHeader(x, y, fullW, "PHASE 2", "CONFIGURE ROADS",
                        "Select a tool, then click a road cell",
                        "to assign its travel cost.");
        y += 74.f;

        // 1. Road Tools (Clean 2-Column Grid with Equal-Sized Buttons)
        drawSectionHeader(x, y, "ROAD TOOLS"); y += 14.f;
        btnToolReward.selected    = (pelletTool == PelletTool::REWARD_05);
        btnToolPenalty.selected   = (pelletTool == PelletTool::PENALTY_40);
        btnToolCongested.selected = (pelletTool == PelletTool::CONGESTED_20);
        btnToolNormal.selected    = (pelletTool == PelletTool::NORMAL_10);

        btnToolReward.draw(); btnToolPenalty.draw();
        y += 38.f;
        btnToolCongested.draw(); btnToolNormal.draw();
        y += 44.f;

        // 2. Pellet Summary Card
        drawSectionHeader(x, y, "PELLET SUMMARY"); y += 14.f;
        drawPanel(x, y, fullW, 74.f, Col::CARD_BG, Col::BORDER, 1.0f);
        drawMetricRow(x + 12.f, y + 20.f, fullW - 24.f, "Reward Pellets (0.5):",
                      std::to_string(maze.countRewardPellets()).c_str(),
                      Col::TEXT_MUTED, Col::GREEN_REWARD);
        drawMetricRow(x + 12.f, y + 40.f, fullW - 24.f, "Penalty Pellets (4.0):",
                      std::to_string(maze.countPenaltyPellets()).c_str(),
                      Col::TEXT_MUTED, Col::RED_PENALTY);
        drawMetricRow(x + 12.f, y + 60.f, fullW - 24.f, "Congested Roads (2.0):",
                      std::to_string(maze.countCongestedCells()).c_str(),
                      Col::TEXT_MUTED, Col::ORANGE_CONGEST);
        y += 88.f;

        // 3. Configuration Actions
        drawSectionHeader(x, y, "CONFIGURATION ACTIONS"); y += 14.f;
        btnUndoPellet.draw(); btnClearPellets.draw();
        y += 44.f;

        // 4. Workflow Navigation
        drawSectionHeader(x, y, "WORKFLOW NAVIGATION"); y += 14.f;
        btnPhase2Continue.draw(); y += 46.f;
        btnPhase2Back.draw();

        // 5. Terrain Cost Legend (cleanly anchored near bottom)
        drawTerrainLegend(x, (float)WIN_H - 96.f, fullW, true);
    }

    // ────────────────────────────────────────────────────────
    // PHASE 3 — SOLVE & COMPARE (BFS, DFS, A*)
    // ────────────────────────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
        float y = HEADER_H + 16.f;
        drawPhaseHeader(x, y, fullW, "PHASE 3", "SOLVE AND COMPARE",
                        "Select an AI search algorithm to find",
                        "and compare delivery routes.");
        y += 74.f;

        // Algorithm Selection
        drawSectionHeader(x, y, "ALGORITHM"); y += 12.f;
        btnSelBFS.selected   = (selSearch == SearchType::BFS);   btnSelBFS.draw();   y += 32.f;
        btnSelDFS.selected   = (selSearch == SearchType::DFS_S); btnSelDFS.draw();   y += 32.f;
        btnSelAStar.selected = (selSearch == SearchType::ASTAR); btnSelAStar.draw(); y += 36.f;

        // Execution Controls
        drawSectionHeader(x, y, "SEARCH CONTROLS"); y += 12.f;
        btnRun.disabled = (appState == AppState::SOLVING);
        btnRun.draw(); btnResetSearch.draw(); y += 36.f;
        btnPause.selected = paused;
        btnStep.draw(); btnPause.draw(); y += 34.f;

        btnAnimateRoute.disabled = !(solver && solver->foundPath());
        btnAnimateRoute.draw(); btnStopAnim.draw(); y += 38.f;

        // Live Route Metrics Card
        drawSectionHeader(x, y, "RESULTS"); y += 12.f;
        drawPanel(x, y, fullW, 124.f, Col::CARD_BG, Col::BORDER, 1.0f);

        if (solver && (appState == AppState::SOLVED || appState == AppState::SOLVING)) {
            const auto& st = solver->stats;
            drawMetricRow(x + 12.f, y + 20.f, fullW - 24.f, "Route Found:",
                          st.foundPath ? "YES" : "NO", Col::TEXT_MUTED, st.foundPath ? Col::GREEN_REWARD : Col::RED_PENALTY);
            drawMetricRow(x + 12.f, y + 38.f, fullW - 24.f, "Route Moves:",
                          std::to_string(st.routeMoves).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE);
            drawMetricRow(x + 12.f, y + 56.f, fullW - 24.f, "Weighted Cost:",
                          (std::to_string((int)st.pathCost) + (st.pathCost == (int)st.pathCost ? ".0" : ".5")).c_str(),
                          Col::TEXT_MUTED, Col::PRIMARY_BLUE);
            drawMetricRow(x + 12.f, y + 74.f, fullW - 24.f, "Expanded Nodes:",
                          std::to_string(st.nodesExplored).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE);

            std::ostringstream ts; ts << std::fixed << std::setprecision(2) << st.timeMs << " ms";
            drawMetricRow(x + 12.f, y + 92.f, fullW - 24.f, "Search Time:",
                          ts.str().c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE);

            std::string rwPn = std::to_string(st.rewardsOnPath) + " / " + std::to_string(st.penaltiesOnPath);
            drawMetricRow(x + 12.f, y + 110.f, fullW - 24.f, "Shortcuts / Penalties:",
                          rwPn.c_str(), Col::TEXT_MUTED, Col::GREEN_REWARD);
        } else {
            glColor3fv(Col::TEXT_MUTED);
            drawBitmapText(x + 14.f, y + 42.f, "No search executed yet.", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(x + 14.f, y + 66.f, "Select an algorithm and press Run.", GLUT_BITMAP_HELVETICA_10);
        }
        y += 138.f;

        // Workflow Navigation
        drawSectionHeader(x, y, "WORKFLOW NAVIGATION"); y += 12.f;
        btnCompareAll.draw(); y += 44.f;
        btnPhase3BackToConfig.draw(); btnPhase3NewMaze.draw();
    }
}

// ════════════════════════════════════════════════════════════
// ── FULL-SCREEN COMPARISON DASHBOARD ────────────────────────
// ════════════════════════════════════════════════════════════
static void renderComparisonDashboard() {
    drawPanel(0.f, 0.f, (float)WIN_W, (float)WIN_H, Col::BG_DARK);

    float cx = (float)WIN_W * 0.5f, y = 36.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"ALGORITHM COMPARISON DASHBOARD");
    drawBitmapText(cx - (float)tw * 0.5f, y, "ALGORITHM COMPARISON DASHBOARD", GLUT_BITMAP_HELVETICA_18);
    y += 24.f;

    glColor3fv(Col::TEXT_MUTED);
    const char* sub = "Benchmarking BFS, DFS, and A* on the Exact Same Road Network & Pellets";
    int sw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)sub);
    drawBitmapText(cx - (float)sw * 0.5f, y, sub, GLUT_BITMAP_HELVETICA_12);
    y += 26.f;

    float tX = 70.f, tW = (float)WIN_W - 140.f;
    drawSep(tX, y, tW); y += 18.f;

    // Table Column Offsets
    float colX[] = {
        tX,
        tX + 140.f,
        tX + 260.f,
        tX + 370.f,
        tX + 490.f,
        tX + 620.f,
        tX + 760.f,
        tX + 870.f,
        tX + 970.f
    };

    const char* hdrs[] = {
        "Algorithm", "Route Found", "Route Moves", "Route Cells",
        "Weighted Cost", "Expanded Nodes", "Search Time (ms)", "Shortcuts", "Penalties"
    };

    glColor3fv(Col::ACCENT);
    for (int i = 0; i < 9; i++) {
        drawBitmapText(colX[i], y, hdrs[i], GLUT_BITMAP_HELVETICA_12);
    }
    y += 18.f;
    drawSep(tX, y, tW); y += 16.f;

    // Optimal values
    int   bestMoves = 999999;
    float bestCost  = 999999.f;
    int   bestNodes = 999999;
    for (const auto& cr : compResults) {
        if (cr.found) {
            bestMoves = std::min(bestMoves, cr.stats.routeMoves);
            bestCost  = std::min(bestCost,  cr.stats.pathCost);
            bestNodes = std::min(bestNodes, cr.stats.nodesExplored);
        }
    }

    const float* algCols[] = {Col::TITLE_CYAN, Col::RED_PENALTY, Col::GREEN_REWARD};
    int ci = 0;

    for (const auto& cr : compResults) {
        const auto& st = cr.stats;
        glColor3fv(algCols[ci < 3 ? ci : 2]); ci++;
        drawBitmapText(colX[0], y, cr.name.c_str(), GLUT_BITMAP_HELVETICA_12);

        if (!cr.found) {
            glColor3fv(Col::RED_PENALTY);
            drawBitmapText(colX[1], y, "NO", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(colX[2], y, "--", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(colX[3], y, "--", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(colX[4], y, "--", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(colX[5], y, std::to_string(st.nodesExplored).c_str(), GLUT_BITMAP_HELVETICA_12);
            std::ostringstream ts; ts << std::fixed << std::setprecision(2) << st.timeMs;
            drawBitmapText(colX[6], y, ts.str().c_str(), GLUT_BITMAP_HELVETICA_12);
            y += 32.f;
            continue;
        }

        glColor3fv(Col::GREEN_REWARD);
        drawBitmapText(colX[1], y, "YES", GLUT_BITMAP_HELVETICA_12);

        // Moves
        if (st.routeMoves == bestMoves) {
            glColor3fv(Col::GREEN_REWARD);
            drawBitmapText(colX[2], y, (std::to_string(st.routeMoves) + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[2], y, std::to_string(st.routeMoves).c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Cells
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(colX[3], y, std::to_string(st.pathLength).c_str(), GLUT_BITMAP_HELVETICA_12);

        // Cost
        std::string costStr = std::to_string((int)st.pathCost) + (st.pathCost == (int)st.pathCost ? ".0" : ".5");
        if (st.pathCost == bestCost) {
            glColor3fv(Col::GREEN_REWARD);
            drawBitmapText(colX[4], y, (costStr + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[4], y, costStr.c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Nodes
        if (st.nodesExplored == bestNodes) {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[5], y, (std::to_string(st.nodesExplored) + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[5], y, std::to_string(st.nodesExplored).c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Time
        std::ostringstream ts; ts << std::fixed << std::setprecision(3) << st.timeMs;
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(colX[6], y, ts.str().c_str(), GLUT_BITMAP_HELVETICA_12);

        // Shortcuts & Penalties
        glColor3fv(Col::GREEN_REWARD);
        drawBitmapText(colX[7], y, std::to_string(st.rewardsOnPath).c_str(), GLUT_BITMAP_HELVETICA_12);

        glColor3fv(Col::RED_PENALTY);
        drawBitmapText(colX[8], y, std::to_string(st.penaltiesOnPath).c_str(), GLUT_BITMAP_HELVETICA_12);

        y += 32.f;
    }

    drawSep(tX, y, tW); y += 22.f;

    // Takeaways Card
    drawSectionHeader(tX, y, "KEY PRESENTATION TAKEAWAYS"); y += 18.f;
    const char* takeaways[] = {
        "- BFS minimizes route moves (edges) on unweighted graphs, but ignores terrain costs.",
        "- DFS explores deeply without cost or distance guarantees; prone to long, non-optimal paths.",
        "- A* achieves minimum weighted delivery cost by balancing cost-so-far g(n) with admissible heuristic h(n).",
        "- When reward pellets (cost 0.5) exist, A* routes through them to minimize overall travel cost.",
        "- When penalty pellets (cost 4.0) exist, A* detours around them, whereas BFS may traverse expensive roads.",
        "- Note: Asterisk (*) indicates optimal performance for that specific metric across the three algorithms.",
        nullptr
    };

    for (int i = 0; takeaways[i]; i++) {
        if (i == 2) glColor3fv(Col::GREEN_REWARD);
        else glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(tX, y, takeaways[i], GLUT_BITMAP_HELVETICA_12);
        y += 20.f;
    }

    btnBackToWorkflow.draw();
}

// ════════════════════════════════════════════════════════════
// ── HELP & LEGEND MODAL SCREENS ─────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderHelpScreen() {
    drawPanel(0.f, 0.f, (float)WIN_W, (float)WIN_H, Col::BG_DARK);

    float cx = (float)WIN_W * 0.5f, y = 36.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"HOW SMARTROUTE AI WORKS");
    drawBitmapText(cx - (float)tw * 0.5f, y, "HOW SMARTROUTE AI WORKS", GLUT_BITMAP_HELVETICA_18);
    y += 48.f;

    float lx = 80.f, rx = cx + 30.f;
    float colY = y;

    auto sec = [&](float sx, float& sy, const char* hdr, const float* col, const char* b1, const char* b2, const char* b3) {
        glColor3fv(col);
        drawBitmapText(sx, sy, hdr, GLUT_BITMAP_HELVETICA_18); sy += 22.f;
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(sx + 10.f, sy, b1, GLUT_BITMAP_HELVETICA_12); sy += 18.f;
        drawBitmapText(sx + 10.f, sy, b2, GLUT_BITMAP_HELVETICA_12); sy += 18.f;
        drawBitmapText(sx + 10.f, sy, b3, GLUT_BITMAP_HELVETICA_12); sy += 26.f;
    };

    sec(lx, colY, "1. Breadth-First Search (BFS)", Col::TITLE_CYAN,
        "FIFO Queue exploration level-by-level.",
        "Guarantees fewest moves (unweighted shortest path).",
        "Ignores road travel costs (blind search).");

    sec(lx, colY, "2. Depth-First Search (DFS)", Col::RED_PENALTY,
        "LIFO Stack traversal; explores deeply before backtracking.",
        "Does not guarantee shortest path or lowest cost.",
        "Demonstrates non-optimal depth search.");

    sec(lx, colY, "3. A* Search (Heuristic)", Col::GREEN_REWARD,
        "f(n) = g(n) + h(n) evaluated via Priority Queue.",
        "g(n) = accumulated weighted traversal cost from Store.",
        "h(n) = 0.5 * (|dr| + |dc|) (Admissible Manhattan distance).");

    float ry = y;
    glColor3fv(Col::ACCENT);
    drawBitmapText(rx, ry, "AI PROBLEM FORMULATION", GLUT_BITMAP_HELVETICA_18); ry += 24.f;

    const char* formNotes[] = {
        "- Initial State: Dispatch Store / Warehouse (S) at (r0, c0).",
        "- Goal State: Customer delivery location (G) at (r_goal, c_goal).",
        "- Actions: 4-Directional grid movement (Up, Down, Left, Right).",
        "- Transition Model: Reaches adjacent non-blocked cell.",
        "- Path Cost Model: Sum of step costs along entered road cells:",
        "    * Normal Road: 1.0 unit cost",
        "    * Reward Pellet Road: 0.5 unit cost (shortcut / priority lane)",
        "    * Congested Road: 2.0 unit cost (traffic congestion)",
        "    * Penalty Pellet Road: 4.0 unit cost (construction / delay)",
        "- Admissibility: Because minimum edge cost = 0.5, heuristic",
        "  h(n) = 0.5 * Manhattan never overestimates true remaining cost.",
        nullptr
    };

    glColor3fv(Col::TEXT_WHITE);
    for (int i = 0; formNotes[i]; i++) {
        drawBitmapText(rx, ry, formNotes[i], GLUT_BITMAP_HELVETICA_12);
        ry += 18.f;
    }

    ry += 20.f;
    drawSep(lx, ry, (float)WIN_W - 160.f); ry += 16.f;
    glColor3fv(Col::TEXT_MUTED);
    const char* disc = "SmartRoute AI is an educational grid-based route-planning simulation. Its terrain costs and maze layouts are illustrative.";
    drawBitmapText(lx, ry, disc, GLUT_BITMAP_HELVETICA_10); ry += 14.f;
    const char* disc2 = "It does not use live traffic, GPS, or proprietary routing algorithms from delivery companies.";
    drawBitmapText(lx, ry, disc2, GLUT_BITMAP_HELVETICA_10);

    btnHelpBack.draw();
}

static void renderLegendScreen() {
    drawPanel(0.f, 0.f, (float)WIN_W, (float)WIN_H, Col::BG_DARK);

    float cx = (float)WIN_W * 0.5f, y = 36.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"TERRAIN & VISUALIZATION LEGEND");
    drawBitmapText(cx - (float)tw * 0.5f, y, "TERRAIN & VISUALIZATION LEGEND", GLUT_BITMAP_HELVETICA_18);
    y += 54.f;

    struct LegendItem {
        const float* fill;
        const char* label;
        const char* cost;
        const char* desc;
    };

    LegendItem items[] = {
        {Col::START,          "Store / Start (S)",       "Cost 0.0 (entry)", "Dispatch warehouse / rider start point"},
        {Col::GOAL,           "Customer / Goal (G)",     "Cost 1.0 (entry)", "Package delivery destination"},
        {Col::ROAD_NORM,      "Normal Road",             "Cost 1.0",         "Standard road cell"},
        {Col::GREEN_REWARD,   "Reward Road (Diamond)",   "Cost 0.5",         "Priority shortcut pellet; lowers route travel cost"},
        {Col::ORANGE_CONGEST, "Congested Road",          "Cost 2.0",         "Traffic congestion; increases route cost"},
        {Col::RED_PENALTY,    "Penalty Road (Cross X)",  "Cost 4.0",         "Road obstruction / delay pellet; raises route cost"},
        {Col::WALL_BLOCK,     "Blocked Road / Wall",     "Impassable",       "Building barrier or closed road segment"},
        {Col::EXPLORED,       "Explored Search Cell",    "--",               "Road cell evaluated by AI algorithm"},
        {Col::FRONTIER,       "Frontier Cell",           "--",               "Discovered road cell pending evaluation"},
        {Col::PATH,           "Final Delivery Route",    "--",               "Optimal reconstructed solution path"},
        {Col::RIDER,          "Delivery Vehicle",        "--",               "Animated vehicle traversing computed solution"},
        {nullptr, nullptr, nullptr, nullptr}
    };

    float bx = 110.f, by = y;
    float bw = 32.f, bh = 22.f;

    for (int i = 0; items[i].fill; i++) {
        glColor3fv(items[i].fill);
        glBegin(GL_QUADS);
        glVertex2f(bx, by);           glVertex2f(bx + bw, by);
        glVertex2f(bx + bw, by + bh); glVertex2f(bx, by + bh);
        glEnd();
        glColor3fv(Col::BORDER);
        glLineWidth(1.f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(bx, by); glVertex2f(bx + bw, by);
        glVertex2f(bx + bw, by + bh); glVertex2f(bx, by + bh);
        glEnd();

        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(bx + 48.f, by + 16.f, items[i].label, GLUT_BITMAP_HELVETICA_12);

        glColor3fv(Col::ACCENT);
        drawBitmapText(bx + 260.f, by + 16.f, items[i].cost, GLUT_BITMAP_HELVETICA_12);

        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(bx + 420.f, by + 16.f, items[i].desc, GLUT_BITMAP_HELVETICA_12);

        by += bh + 14.f;
    }

    btnLegendBack.draw();
}

// ════════════════════════════════════════════════════════════
// ── GLUT EVENT CALLBACKS ────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void display() {
    glClearColor(0.04f, 0.06f, 0.09f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    if (appState == AppState::COMPARING) {
        renderComparisonDashboard();
    } else if (appState == AppState::HELP) {
        renderHelpScreen();
    } else if (appState == AppState::LEGEND) {
        renderLegendScreen();
    } else {
        renderTopHeader();
        renderMazeCanvas();
        renderSidebar();
    }

    glutSwapBuffers();
    recorder.captureFrame(WIN_W, WIN_H);
}

static void handleSidebarClick(float sx, float sy) {
    // ── Phase 1 Buttons ──────────────────────────────────────
    if (currentPhase == AppPhase::PHASE_1_CREATE) {
        if (btnSzSm.isClicked(sx, sy)) {
            selSize = MazeSizeE::SMALL;
            maze.resize(MSIZES[0][0], MSIZES[0][1]);
            maze.hardReset();
            statusMsg = "Size: Small (15x15). Press Generate.";
            activeScenario = ScenarioE::NONE;
            mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
        } else if (btnSzMd.isClicked(sx, sy)) {
            selSize = MazeSizeE::MEDIUM;
            maze.resize(MSIZES[1][0], MSIZES[1][1]);
            maze.hardReset();
            statusMsg = "Size: Medium (20x20). Press Generate.";
            activeScenario = ScenarioE::NONE;
            mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
        } else if (btnSzLg.isClicked(sx, sy)) {
            selSize = MazeSizeE::LARGE;
            maze.resize(MSIZES[2][0], MSIZES[2][1]);
            maze.hardReset();
            statusMsg = "Size: Large (25x25). Press Generate.";
            activeScenario = ScenarioE::NONE;
            mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
        } else if (btnGenDFS.isClicked(sx, sy)) {
            selGen = GenType::DFS_GRID;
            statusMsg = "Generator: DFS Grid (winding street network).";
        } else if (btnGenPrim.isClicked(sx, sy)) {
            selGen = GenType::PRIM_BRANCH;
            statusMsg = "Generator: Prim Branching.";
        } else if (btnGenerate.isClicked(sx, sy)) {
            doGenerate();
        } else if (btnNewSeed.isClicked(sx, sy)) {
            doHardReset();
        } else if (btnScenA.isClicked(sx, sy)) {
            loadScenarioA();
        } else if (btnScenB.isClicked(sx, sy)) {
            loadScenarioB();
        } else if (btnScenC.isClicked(sx, sy)) {
            loadScenarioC();
        } else if (btnToggleWallEdit.isClicked(sx, sy)) {
            wallEditActive = !wallEditActive;
            statusMsg = wallEditActive ? "Wall Edit: ON. Click maze cells to toggle barriers." : "Wall Edit: OFF.";
        } else if (btnClearWalls.isClicked(sx, sy)) {
            buildOpenGrid(maze.rows, maze.cols);
            statusMsg = "Opened all roads (open street grid).";
        } else if (btnPhase1Continue.isClicked(sx, sy)) {
            doEnterRoadConfig();
        }
    }

    // ── Phase 2 Buttons ──────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
        if (btnToolReward.isClicked(sx, sy)) {
            pelletTool = PelletTool::REWARD_05;
            statusMsg = "Tool: Reward Pellet (Cost 0.5). Click road cells to place.";
        } else if (btnToolPenalty.isClicked(sx, sy)) {
            pelletTool = PelletTool::PENALTY_40;
            statusMsg = "Tool: Penalty Pellet (Cost 4.0). Click road cells to place.";
        } else if (btnToolCongested.isClicked(sx, sy)) {
            pelletTool = PelletTool::CONGESTED_20;
            statusMsg = "Tool: Congested Road (Cost 2.0). Click road cells to assign.";
        } else if (btnToolNormal.isClicked(sx, sy)) {
            pelletTool = PelletTool::NORMAL_10;
            statusMsg = "Tool: Normal / Erase (Cost 1.0). Click cells to restore.";
        } else if (btnUndoPellet.isClicked(sx, sy)) {
            if (!pelletUndoStack.empty()) {
                PelletHistory u = pelletUndoStack.back();
                pelletUndoStack.pop_back();
                maze.at(u.pos).terrain = u.oldTerrain;
                maze.at(u.pos).special = u.oldSpecial;
                statusMsg = "Undid modification at (" + std::to_string(u.pos.row) + "," + std::to_string(u.pos.col) + ").";
            } else {
                statusMsg = "Undo stack is empty.";
            }
        } else if (btnClearPellets.isClicked(sx, sy)) {
            for (int r = 0; r < maze.rows; r++) {
                for (int c = 0; c < maze.cols; c++) {
                    if (maze.at(r, c).terrain == TerrainType::CONGESTED) {
                        maze.at(r, c).terrain = TerrainType::NORMAL;
                    }
                }
            }
            maze.clearAllSpecialPellets();
            pelletUndoStack.clear();
            statusMsg = "Cleared all pellets. All road cells restored to normal 1.0.";
        } else if (btnPhase2Continue.isClicked(sx, sy)) {
            currentPhase = AppPhase::PHASE_3_SOLVE;
            statusMsg = "Phase 3: Ready to solve. Select algorithm and press Run.";
        } else if (btnPhase2Back.isClicked(sx, sy)) {
            currentPhase = AppPhase::PHASE_1_CREATE;
            statusMsg = "Phase 1: Create Maze. Maze configuration preserved.";
        }
    }

    // ── Phase 3 Buttons ──────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
        if (btnSelBFS.isClicked(sx, sy)) {
            selSearch = SearchType::BFS;
            statusMsg = "Strategy: BFS -- Fewest moves (unweighted).";
        } else if (btnSelDFS.isClicked(sx, sy)) {
            selSearch = SearchType::DFS_S;
            statusMsg = "Strategy: DFS -- Depth-first (not optimal).";
        } else if (btnSelAStar.isClicked(sx, sy)) {
            selSearch = SearchType::ASTAR;
            statusMsg = "Strategy: A* -- Lowest weighted cost f(n)=g(n)+h(n).";
        } else if (btnRun.isClicked(sx, sy)) {
            doStartSolving();
        } else if (btnResetSearch.isClicked(sx, sy)) {
            doResetSearch();
        } else if (btnPause.isClicked(sx, sy)) {
            paused = !paused;
        } else if (btnStep.isClicked(sx, sy)) {
            paused = true;
            doStep();
        } else if (btnCompareAll.isClicked(sx, sy)) {
            doRunComparison();
        } else if (btnAnimateRoute.isClicked(sx, sy)) {
            startRiderAnimation();
        } else if (btnStopAnim.isClicked(sx, sy)) {
            stopRiderAnimation();
        } else if (btnPhase3BackToConfig.isClicked(sx, sy)) {
            doEnterRoadConfig();
        } else if (btnPhase3NewMaze.isClicked(sx, sy)) {
            currentPhase = AppPhase::PHASE_1_CREATE;
            statusMsg = "Phase 1: Maze Creation. Generate or edit road network.";
        }
    }
}

static void mouse(int button, int state, int x, int y) {
    bool press = (state == GLUT_DOWN);
    mouseDown  = press && (button == GLUT_LEFT_BUTTON);

    if (!press) return;

    if (appState == AppState::COMPARING) {
        if (btnBackToWorkflow.isClicked((float)x, (float)y)) {
            appState = AppState::MAZE_WORKFLOW;
        }
        glutPostRedisplay();
        return;
    }
    if (appState == AppState::HELP) {
        if (btnHelpBack.isClicked((float)x, (float)y)) {
            appState = AppState::MAZE_WORKFLOW;
        }
        glutPostRedisplay();
        return;
    }
    if (appState == AppState::LEGEND) {
        if (btnLegendBack.isClicked((float)x, (float)y)) {
            appState = AppState::MAZE_WORKFLOW;
        }
        glutPostRedisplay();
        return;
    }

    // Top Header Clicks
    if ((float)y < HEADER_H) {
        if (btnTopLegend.isClicked((float)x, (float)y)) {
            appState = AppState::LEGEND;
        } else if (btnTopHelp.isClicked((float)x, (float)y)) {
            appState = AppState::HELP;
        }
        glutPostRedisplay();
        return;
    }

    // Right Sidebar Clicks
    if ((float)x >= (float)(WIN_W - SIDEBAR_W)) {
        handleSidebarClick((float)x, (float)y);
        glutPostRedisplay();
        return;
    }

    // Maze Canvas Clicks (strictly bounded to actual maze grid)
    int row, col;
    if (mazeLayout.hitTest((float)x, (float)y, row, col, maze.rows, maze.cols)) {
        if (currentPhase == AppPhase::PHASE_1_CREATE && wallEditActive) {
            applyWallEditTool(row, col);
        } else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
            applyRoadConfigTool(row, col);
        }
    }

    glutPostRedisplay();
}

static void mouseMotion(int x, int y) {
    if (appState == AppState::COMPARING) {
        btnBackToWorkflow.updateHover((float)x, (float)y);
        glutPostRedisplay();
        return;
    }
    if (appState == AppState::HELP) {
        btnHelpBack.updateHover((float)x, (float)y);
        glutPostRedisplay();
        return;
    }
    if (appState == AppState::LEGEND) {
        btnLegendBack.updateHover((float)x, (float)y);
        glutPostRedisplay();
        return;
    }

    for (Button* b : allButtons) {
        b->updateHover((float)x, (float)y);
    }

    // Drag placement for wall editing or pellet placement
    if (mouseDown && (float)y >= HEADER_H && (float)x < (float)(WIN_W - SIDEBAR_W)) {
        int row, col;
        if (mazeLayout.hitTest((float)x, (float)y, row, col, maze.rows, maze.cols)) {
            if (currentPhase == AppPhase::PHASE_1_CREATE && wallEditActive) {
                applyWallEditTool(row, col);
            } else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
                applyRoadConfigTool(row, col);
            }
        }
    }

    glutPostRedisplay();
}

static void passiveMotion(int x, int y) {
    mouseMotion(x, y);
}

static void keyboard(unsigned char key, int /*x*/, int /*y*/) {
    switch (key) {
        case 27: // ESC
            if (appState == AppState::COMPARING || appState == AppState::HELP || appState == AppState::LEGEND) {
                appState = AppState::MAZE_WORKFLOW;
            } else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
                doEnterRoadConfig();
            } else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
                currentPhase = AppPhase::PHASE_1_CREATE;
            }
            break;
        case 'g': case 'G': doGenerate(); break;
        case '1':
            selSearch = SearchType::BFS;
            statusMsg = "Strategy: BFS -- Fewest Moves.";
            break;
        case '2':
            selSearch = SearchType::DFS_S;
            statusMsg = "Strategy: DFS -- Depth-First Search.";
            break;
        case '3':
            selSearch = SearchType::ASTAR;
            statusMsg = "Strategy: A* -- Lowest Weighted Cost.";
            break;
        case ' ':
            if (appState == AppState::SOLVING || appState == AppState::GENERATING)
                paused = !paused;
            break;
        case 'n': case 'N':
            paused = true; doStep();
            break;
        case 'c': case 'C':
            doRunComparison();
            break;
        case 'a': case 'A':
            startRiderAnimation();
            break;
        case 'h': case 'H':
            appState = AppState::HELP;
            break;
        case 'l': case 'L':
            appState = AppState::LEGEND;
            break;
        case 'r': case 'R':
            if (currentPhase == AppPhase::PHASE_3_SOLVE) doResetSearch();
            else doHardReset();
            break;
        case 'f': case 'F':
            recorder.toggle();
            break;
    }
    glutPostRedisplay();
}

static void specialKey(int key, int /*x*/, int /*y*/) {
    if (key == GLUT_KEY_F9) recorder.toggle();
    glutPostRedisplay();
}

static void timerCallback(int /*val*/) {
    if (errTimer > 0.f) {
        errTimer -= TIMER_MS / 1000.f;
        if (errTimer <= 0.f) { errTimer = 0.f; errorMsg = ""; }
    }

    if (!paused) {
        if (appState == AppState::GENERATING && activeGen) {
            for (int s = 0; s < stepsPerTick; s++) {
                if (activeGen->isFinished()) break;
                activeGen->step();
            }
            if (activeGen->isFinished()) {
                appState  = AppState::MAZE_WORKFLOW;
                statusMsg = "Road network ready. Click CONTINUE TO ROAD CONFIG.";
            }
        } else if (appState == AppState::SOLVING && solver) {
            for (int s = 0; s < stepsPerTick; s++) {
                if (solver->isFinished()) break;
                solver->step();
            }
            if (solver->isFinished()) {
                appState  = AppState::SOLVED;
                statusMsg = solver->foundPath()
                    ? "Route found! Press ANIMATE VAN to follow route."
                    : "No route exists -- customer unreachable.";
            }
        }
    }

    updateRider();
    glutPostRedisplay();
    glutTimerFunc((unsigned int)TIMER_MS, timerCallback, 0);
}

static void reshape(int w, int h) {
    WIN_W = (w < 800 ? 800 : w);
    WIN_H = (h < 600 ? 600 : h);
    glViewport(0, 0, WIN_W, WIN_H);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0, WIN_W, WIN_H, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();

    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
    initButtonLayout();
    glutPostRedisplay();
}

// ════════════════════════════════════════════════════════════
// ── MAIN ENTRY POINT ────────────────────────────────────────
// ════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    currentSeed = 42;
    globalRng.seed(currentSeed);
    srand(currentSeed);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("SmartRoute AI -- Intelligent Last-Mile Delivery Route Optimizer");

    glClearColor(0.04f, 0.06f, 0.09f, 1.f);

    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0, WIN_W, WIN_H, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    glutPassiveMotionFunc(passiveMotion);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKey);
    glutReshapeFunc(reshape);
    glutTimerFunc((unsigned int)TIMER_MS, timerCallback, 0);

    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
    initButtonLayout();

    // Start in Phase 1 with Scenario A preloaded
    loadScenarioA();
    currentPhase = AppPhase::PHASE_1_CREATE;
    statusMsg    = "Phase 1: Road network ready. Click CONTINUE TO ROAD CONFIG ->";

    std::cout << "\n========================================================\n";
    std::cout << "  SmartRoute AI -- Last-Mile Delivery Optimizer\n";
    std::cout << "  Professional UI/UX Redesign\n";
    std::cout << "========================================================\n\n";

    glutMainLoop();
    return 0;
}
