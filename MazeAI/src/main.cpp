// ============================================================
// main.cpp  —  SmartRoute AI
//   Intelligent Last-Mile Delivery Route Optimization Simulator
// ============================================================
// Professional Desktop UI/UX for College AI MSE Presentation
//
// THREE-PHASE WORKFLOW:
//   Phase 1: Create Road Network & Multi-Customer Destinations (C1, C2, C3...)
//   Phase 2: Configure Reward (0.5), Congested (2.0) & Penalty (4.0) Roads
//   Phase 3: Solve & Compare (BFS, DFS, A*) with Single or Multi-Stop Routing
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
#include "MultiStop.h"
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
static const float HEADER_H    = 48.f;
static const float SIDEBAR_W   = 340.f;
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

// Customer Delivery Modes
enum class DeliveryMode { SINGLE_TARGET, MULTI_STOP };
static DeliveryMode deliveryMode = DeliveryMode::SINGLE_TARGET;

enum class CustPlacementTool { NONE, ADD_CUST, MOVE_CUST };
static CustPlacementTool custPlacementTool = CustPlacementTool::NONE;

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

// Ordered customer stop IDs for multi-stop routing
static std::vector<int> deliveryOrder = {1};

// Simulation Controls
static bool paused       = false;
static int  stepsPerTick = 1;
static bool mouseDown    = false;

// Status & Messaging
static std::string statusMsg = "Phase 1: Configure road network & customer destinations.";
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
    static const float GOAL         [] = {0.15f, 0.45f, 0.88f}; // Customer (C - Sapphire)
    static const float GOAL_SEL     [] = {0.95f, 0.75f, 0.10f}; // Selected Customer halo (Gold)
    static const float GREEN_REWARD [] = {0.12f, 0.68f, 0.38f}; // Reward shortcut (Emerald 0.5)
    static const float RED_PENALTY  [] = {0.82f, 0.22f, 0.22f}; // Penalty road (Red 4.0)
    static const float ORANGE_CONGEST[] = {0.85f, 0.52f, 0.12f}; // Congested road (Amber 2.0)
    static const float RIDER        [] = {0.00f, 0.88f, 0.95f}; // Delivery van (Cyan)
    static const float CYAN         [] = {0.00f, 0.88f, 0.95f}; // BFS route line
    static const float CORAL        [] = {1.00f, 0.45f, 0.25f}; // DFS route line

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
    float canvasW = 900.f;
    float canvasH = 732.f;
    float padding = 20.f;
    float cellSize = 34.f;
    float mazeW    = 510.f;
    float mazeH    = 510.f;
    float originX  = 180.f;
    float originY  = 110.f;

    void update(int winW, int winH, int rows, int cols) {
        canvasX = 0.f;
        canvasY = HEADER_H;
        canvasW = (float)(winW - SIDEBAR_W);
        canvasH = (float)(winH - HEADER_H);
        padding = 20.f;

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
static FrameRecorder    recorder("frames");

// Algorithm Independent Run & Animation State (BFS, DFS, A*)
struct AlgorithmState {
    std::string           name;
    MultiStopResult       result;
    bool                  hasRun = false;
    bool                  foundPath = false;
    std::vector<Position> animPath;
    int                   animStep = 0;
    float                 animProgress = 0.f;
    bool                  animActive = false;
    int                   stopsReached = 0;
    bool                  deliveryCompleted = false;
    std::string           statusText = "Not executed yet";
};

static AlgorithmState algoStates[3] = {
    {"BFS", MultiStopResult{}, false, false, {}, 0, 0.f, false, 0, false, "Not executed yet"},
    {"DFS", MultiStopResult{}, false, false, {}, 0, 0.f, false, 0, false, "Not executed yet"},
    {"A*",  MultiStopResult{}, false, false, {}, 0, 0.f, false, 0, false, "Not executed yet"}
};

static int  selectedAlgoIdx   = 2; // 0=BFS, 1=DFS, 2=A*
static bool overlayComparison = false;
static float riderSpeed       = 0.08f;

// Algorithm Comparison Results Snapshot
struct CompResult {
    std::string     name;
    MultiStopResult multiRes;
};
static std::vector<CompResult> compResults;

// ════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════
// ── SIDEBAR COORDINATE & LAYOUT SYSTEM ──────────────────────
// ════════════════════════════════════════════════════════════
struct SidebarLayout {
    float scrollY     = 0.f;
    float maxScrollY  = 0.f;

    float sideX       = 0.f;
    float sideW       = SIDEBAR_W;
    float pad         = 14.f;
    float x           = 0.f;
    float fullW       = 0.f;
    float halfW       = 0.f;
    float thirdW      = 0.f;
    float col2X       = 0.f;
    float th2X        = 0.f;
    float th3X        = 0.f;

    // Phase 1 section anchors
    float p1HeaderY   = 0.f;
    float p1GenSecY   = 0.f;
    float p1CustSecY  = 0.f;
    float p1LegendY   = 0.f;
    float p1NavY      = 0.f;

    // Phase 2 section anchors
    float p2HeaderY      = 0.f;
    float p2ToolsSecY    = 0.f;
    float p2SummarySecY  = 0.f;
    float p2SummaryCardY = 0.f;
    float p2ActionsSecY  = 0.f;
    float p2LegendY      = 0.f;
    float p2NavSecY      = 0.f;

    // Phase 3 section anchors
    float p3HeaderY      = 0.f;
    float p3TargetSecY   = 0.f;
    float p3TargetCardY  = 0.f;
    float p3AlgSecY      = 0.f;
    float p3ExecSecY     = 0.f;
    float p3ResultsSecY  = 0.f;
    float p3ResultsCardY = 0.f;
    float p3CompareY     = 0.f;
    float p3NavY         = 0.f;
};
static SidebarLayout sideLayout;

// ════════════════════════════════════════════════════════════
// ── UI BUTTONS & LAYOUT SYSTEM ──────────────────────────────
// ════════════════════════════════════════════════════════════
// Top Header
static Button btnTopHelp, btnTopLegend;

// Phase 1: Setup & Customers
static Button btnSzSm, btnSzMd, btnSzLg;
static Button btnGenDFS, btnGenPrim;
static Button btnGenerate, btnNewSeed;
static Button btnScenA, btnScenB, btnScenC;
static Button btnToggleWallEdit, btnClearWalls;

// Customer Management Panel
static Button btnModeSingle, btnModeMulti;
static Button btnCustAdd, btnCustMove, btnCustRemove;
static Button btnCustUp, btnCustDown;
static std::vector<Button> btnCustChips;

// Workflow Navigation
static Button btnPhase1Continue;

// Phase 2: Configure Roads (2-Column Grid)
static Button btnToolReward, btnToolPenalty, btnToolCongested, btnToolNormal;
static Button btnUndoPellet, btnClearPellets;
static Button btnPhase2Continue, btnPhase2Back;

// Phase 3: Solve & Compare
static Button btnSelBFS, btnSelDFS, btnSelAStar;
static Button btnRun, btnRunAll, btnResetSearch, btnPause, btnStep;
static Button btnAnimateRoute, btnStopAnim;
static Button btnCompareAll, btnToggleOverlay;
static Button btnPhase3BackToConfig, btnPhase3NewMaze;

// Modal / Screen Return
static Button btnHelpBack, btnLegendBack, btnBackToWorkflow, btnDashOverlay, btnDashRerun;

static std::vector<Button*> allButtons;

// ════════════════════════════════════════════════════════════
// ── UPDATE UI LAYOUT & BUTTON HITBOXES ──────────────────────
// ════════════════════════════════════════════════════════════
static void updateUILayout() {
    allButtons.clear();
    btnCustChips.clear();

    // Geometric constants
    sideLayout.sideX   = (float)(WIN_W - SIDEBAR_W);
    sideLayout.sideW   = SIDEBAR_W;
    sideLayout.pad     = 14.f;
    sideLayout.x       = sideLayout.sideX + sideLayout.pad;
    sideLayout.fullW   = SIDEBAR_W - (sideLayout.pad * 2.f);
    sideLayout.halfW   = (sideLayout.fullW - 8.f) * 0.5f;
    sideLayout.thirdW  = (sideLayout.fullW - 12.f) / 3.f;
    sideLayout.col2X   = sideLayout.x + sideLayout.halfW + 8.f;
    sideLayout.th2X    = sideLayout.x + sideLayout.thirdW + 6.f;
    sideLayout.th3X    = sideLayout.x + 2.f * (sideLayout.thirdW + 6.f);

    float x      = sideLayout.x;
    float fullW  = sideLayout.fullW;
    float halfW  = sideLayout.halfW;
    float thirdW = sideLayout.thirdW;
    float col2X  = sideLayout.col2X;
    float th2X   = sideLayout.th2X;
    float th3X   = sideLayout.th3X;

    // Top Header Buttons (Right Anchored)
    btnTopLegend = Button((float)WIN_W - 156.f, 11.f, 72.f, 26.f, "Legend");
    btnTopHelp   = Button((float)WIN_W - 76.f,  11.f, 64.f, 26.f, "Help");
    allButtons.push_back(&btnTopLegend);
    allButtons.push_back(&btnTopHelp);

    // ────────────────────────────────────────────────────────
    // Phase 1 Setup: Create Maze & Manage Customers
    // ────────────────────────────────────────────────────────
    float y1 = HEADER_H + 10.f;
    sideLayout.p1HeaderY = y1;
    y1 += 50.f; // Phase 1 header card (h=44) + gap

    sideLayout.p1GenSecY = y1; // "MAZE & GENERATOR"
    y1 += 12.f;

    btnSzSm = Button(x,    y1, thirdW, 24.f, "15x15");
    btnSzMd = Button(th2X, y1, thirdW, 24.f, "20x20");
    btnSzLg = Button(th3X, y1, thirdW, 24.f, "25x25");
    y1 += 28.f;

    btnGenDFS  = Button(x,     y1, halfW, 24.f, "DFS Grid");
    btnGenPrim = Button(col2X, y1, halfW, 24.f, "Prim Branch");
    y1 += 28.f;

    btnGenerate = Button(x,     y1, halfW, 26.f, "Generate Maze"); btnGenerate.setGreen();
    btnNewSeed  = Button(col2X, y1, halfW, 26.f, "New / Reset");   btnNewSeed.setNeutral();
    y1 += 30.f;

    btnScenA = Button(x,    y1, thirdW, 22.f, "Scen A");
    btnScenB = Button(th2X, y1, thirdW, 22.f, "Scen B");
    btnScenC = Button(th3X, y1, thirdW, 22.f, "Scen C");
    y1 += 26.f;

    btnToggleWallEdit = Button(x,     y1, halfW, 24.f, "Edit Walls");
    btnClearWalls     = Button(col2X, y1, halfW, 24.f, "Open Grid");
    y1 += 32.f;

    // Customer Management Section
    sideLayout.p1CustSecY = y1; // "CUSTOMERS (N/10)"
    y1 += 12.f;

    btnModeSingle = Button(x,     y1, halfW, 24.f, "Single Target");
    btnModeMulti  = Button(col2X, y1, halfW, 24.f, "Multi-Stop");
    y1 += 28.f;

    // Customer Chips (up to 10 customers, 5 per row)
    float chipW = (fullW - 16.f) / 5.f;
    for (size_t i = 0; i < maze.customers.size() && i < 10; i++) {
        int rIdx = (int)i / 5;
        int cIdx = (int)i % 5;
        float cx = x + cIdx * (chipW + 4.f);
        float cy = y1 + rIdx * 25.f;
        Button chip(cx, cy, chipW, 22.f, maze.customers[i].label().c_str());
        if (maze.customers[i].id == maze.selectedCustomerId) chip.setBlue();
        btnCustChips.push_back(chip);
    }
    int chipRows = ((int)maze.customers.size() + 4) / 5;
    if (chipRows < 1) chipRows = 1;
    y1 += chipRows * 25.f + 4.f;

    // Customer Actions: + Add, Move, Delete
    btnCustAdd    = Button(x,    y1, thirdW, 24.f, "+ Add");   btnCustAdd.setGreen();
    btnCustMove   = Button(th2X, y1, thirdW, 24.f, "Move");   btnCustMove.setNeutral();
    btnCustRemove = Button(th3X, y1, thirdW, 24.f, "Delete"); btnCustRemove.setRed();
    y1 += 28.f;

    // Reordering / Sequence space
    if (deliveryMode == DeliveryMode::MULTI_STOP) {
        btnCustUp   = Button(x,     y1, halfW, 22.f, "Move Up");
        btnCustDown = Button(col2X, y1, halfW, 22.f, "Move Down");
        y1 += 26.f;
        y1 += 26.f; // Space for sequence label + text
    } else {
        btnCustUp   = Button(x,     y1, halfW, 22.f, "Move Up");
        btnCustDown = Button(col2X, y1, halfW, 22.f, "Move Down");
        btnCustUp.visible = false;
        btnCustDown.visible = false;
        y1 += 18.f; // Space for single target label
    }

    y1 += 6.f;
    sideLayout.p1LegendY = y1; // Grid Legend (h=66)
    y1 += 76.f;

    sideLayout.p1NavY = y1; // Workflow Navigation
    btnPhase1Continue = Button(x, y1, fullW, 34.f, "CONTINUE TO ROAD CONFIG ->");
    btnPhase1Continue.setGreen();
    y1 += 42.f;

    allButtons.push_back(&btnSzSm); allButtons.push_back(&btnSzMd); allButtons.push_back(&btnSzLg);
    allButtons.push_back(&btnGenDFS); allButtons.push_back(&btnGenPrim);
    allButtons.push_back(&btnGenerate); allButtons.push_back(&btnNewSeed);
    allButtons.push_back(&btnScenA); allButtons.push_back(&btnScenB); allButtons.push_back(&btnScenC);
    allButtons.push_back(&btnToggleWallEdit); allButtons.push_back(&btnClearWalls);
    allButtons.push_back(&btnModeSingle); allButtons.push_back(&btnModeMulti);
    allButtons.push_back(&btnCustAdd); allButtons.push_back(&btnCustMove); allButtons.push_back(&btnCustRemove);
    allButtons.push_back(&btnCustUp); allButtons.push_back(&btnCustDown);
    for (size_t i = 0; i < btnCustChips.size(); i++) {
        allButtons.push_back(&btnCustChips[i]);
    }
    allButtons.push_back(&btnPhase1Continue);

    // ────────────────────────────────────────────────────────
    // Phase 2 Setup: Configure Roads
    // ────────────────────────────────────────────────────────
    float y2 = HEADER_H + 10.f;
    sideLayout.p2HeaderY = y2;
    y2 += 52.f; // Phase 2 header (h=44) + gap

    sideLayout.p2ToolsSecY = y2; // "ROAD TOOLS"
    y2 += 12.f;
    btnToolReward    = Button(x,     y2, halfW, 28.f, "Reward · 0.5");  btnToolReward.setGreen();
    btnToolPenalty   = Button(col2X, y2, halfW, 28.f, "Penalty · 4.0"); btnToolPenalty.setRed();
    y2 += 32.f;
    btnToolCongested = Button(x,     y2, halfW, 28.f, "Congested · 2.0"); btnToolCongested.setAmber();
    btnToolNormal    = Button(col2X, y2, halfW, 28.f, "Normal / Erase"); btnToolNormal.setNeutral();
    y2 += 34.f;

    sideLayout.p2SummarySecY = y2; // "PELLET SUMMARY"
    y2 += 12.f;
    sideLayout.p2SummaryCardY = y2; // Summary panel (h=64)
    y2 += 72.f;

    sideLayout.p2ActionsSecY = y2; // "ACTIONS"
    y2 += 12.f;
    btnUndoPellet   = Button(x,     y2, halfW, 26.f, "Undo Last");     btnUndoPellet.setNeutral();
    btnClearPellets = Button(col2X, y2, halfW, 26.f, "Clear Pellets"); btnClearPellets.setRed();
    y2 += 32.f;

    sideLayout.p2LegendY = y2; // Terrain Cost Legend (h=66)
    y2 += 76.f;

    sideLayout.p2NavSecY = y2; // "WORKFLOW NAVIGATION"
    y2 += 12.f;
    btnPhase2Continue = Button(x, y2, fullW, 34.f, "CONTINUE TO SOLVING ->"); btnPhase2Continue.setPrimary();
    y2 += 38.f;
    btnPhase2Back     = Button(x, y2, fullW, 26.f, "<- BACK TO MAZE CREATION"); btnPhase2Back.setNeutral();
    y2 += 34.f;

    allButtons.push_back(&btnToolReward); allButtons.push_back(&btnToolPenalty);
    allButtons.push_back(&btnToolCongested); allButtons.push_back(&btnToolNormal);
    allButtons.push_back(&btnUndoPellet); allButtons.push_back(&btnClearPellets);
    allButtons.push_back(&btnPhase2Continue); allButtons.push_back(&btnPhase2Back);

    // ────────────────────────────────────────────────────────
    // Phase 3 Setup: Solve and Compare
    // ────────────────────────────────────────────────────────
    float y3 = HEADER_H + 10.f;
    sideLayout.p3HeaderY = y3;
    y3 += 52.f; // Phase 3 header (h=44) + gap

    sideLayout.p3TargetSecY = y3; // "DELIVERY TARGET"
    y3 += 12.f;
    sideLayout.p3TargetCardY = y3; // Target panel (h=40)
    y3 += 46.f;

    sideLayout.p3AlgSecY = y3; // "ALGORITHM"
    y3 += 12.f;
    btnSelBFS   = Button(x, y3, fullW, 24.f, "BFS -- Fewest Moves");        y3 += 28.f;
    btnSelDFS   = Button(x, y3, fullW, 24.f, "DFS -- Depth-First");         y3 += 28.f;
    btnSelAStar = Button(x, y3, fullW, 24.f, "A* -- Lowest Weighted Cost"); btnSelAStar.setBlue(); y3 += 30.f;

    sideLayout.p3ExecSecY = y3; // "EXECUTION CONTROLS"
    y3 += 12.f;
    btnRun         = Button(x,     y3, halfW, 26.f, "Run Selected"); btnRun.setGreen();
    btnRunAll      = Button(col2X, y3, halfW, 26.f, "Run All 3");    btnRunAll.setPrimary();
    y3 += 30.f;

    btnResetSearch   = Button(x,     y3, halfW, 24.f, "Reset Algo");   btnResetSearch.setNeutral();
    btnToggleOverlay = Button(col2X, y3, halfW, 24.f, overlayComparison ? "Overlay: ON" : "Overlay: OFF");
    if (overlayComparison) btnToggleOverlay.setAmber(); else btnToggleOverlay.setNeutral();
    y3 += 28.f;

    btnAnimateRoute = Button(x,     y3, halfW, 24.f, "Animate Van"); btnAnimateRoute.setAmber();
    btnStopAnim     = Button(col2X, y3, halfW, 24.f, "Pause Anim");  btnStopAnim.setNeutral();
    y3 += 32.f;

    sideLayout.p3ResultsSecY = y3; // "RESULTS"
    y3 += 12.f;
    sideLayout.p3ResultsCardY = y3; // Results card (h=122)
    y3 += 130.f;

    sideLayout.p3CompareY = y3; // Compare All
    btnCompareAll = Button(x, y3, fullW, 32.f, "COMPARE DASHBOARD (TABLE)"); btnCompareAll.setPrimary();
    y3 += 38.f;

    sideLayout.p3NavY = y3; // Navigation
    btnPhase3BackToConfig = Button(x,     y3, halfW, 26.f, "<- Road Config");
    btnPhase3NewMaze      = Button(col2X, y3, halfW, 26.f, "New Network");
    y3 += 34.f;

    allButtons.push_back(&btnSelBFS); allButtons.push_back(&btnSelDFS); allButtons.push_back(&btnSelAStar);
    allButtons.push_back(&btnRun); allButtons.push_back(&btnRunAll);
    allButtons.push_back(&btnResetSearch); allButtons.push_back(&btnToggleOverlay);
    allButtons.push_back(&btnAnimateRoute); allButtons.push_back(&btnStopAnim);
    allButtons.push_back(&btnCompareAll);
    allButtons.push_back(&btnPhase3BackToConfig); allButtons.push_back(&btnPhase3NewMaze);

    // Modal / Screen Return Buttons
    btnHelpBack       = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 46.f, 160.f, 30.f, "RETURN");
    btnLegendBack     = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 46.f, 160.f, 30.f, "RETURN");
    btnBackToWorkflow = Button((float)WIN_W * 0.5f - 275.f, (float)WIN_H - 46.f, 175.f, 32.f, "RETURN TO SOLVING");
    btnBackToWorkflow.setGreen();
    btnDashOverlay    = Button((float)WIN_W * 0.5f - 90.f,  (float)WIN_H - 46.f, 175.f, 32.f, overlayComparison ? "OVERLAY: ON" : "OVERLAY: OFF");
    btnDashOverlay.setPrimary();
    btnDashRerun      = Button((float)WIN_W * 0.5f + 95.f,  (float)WIN_H - 46.f, 175.f, 32.f, "RERUN BENCHMARK");
    btnDashRerun.setAmber();

    allButtons.push_back(&btnHelpBack);
    allButtons.push_back(&btnLegendBack);
    allButtons.push_back(&btnBackToWorkflow);
    allButtons.push_back(&btnDashOverlay);
    allButtons.push_back(&btnDashRerun);

    // Calculate maximum scroll range for current phase
    float totalH = 0.f;
    if (currentPhase == AppPhase::PHASE_1_CREATE)      totalH = y1 - HEADER_H;
    else if (currentPhase == AppPhase::PHASE_2_CONFIG) totalH = y2 - HEADER_H;
    else if (currentPhase == AppPhase::PHASE_3_SOLVE)  totalH = y3 - HEADER_H;

    float availH = (float)(WIN_H - HEADER_H);
    if (totalH > availH) {
        sideLayout.maxScrollY = totalH - availH + 16.f;
    } else {
        sideLayout.maxScrollY = 0.f;
        sideLayout.scrollY    = 0.f;
    }
    if (sideLayout.scrollY > sideLayout.maxScrollY) sideLayout.scrollY = sideLayout.maxScrollY;
    if (sideLayout.scrollY < 0.f)                  sideLayout.scrollY = 0.f;
}

static void initButtonLayout() {
    updateUILayout();
}

// ── FACTORY HELPERS ─────────────────────────────────────────
// ════════════════════════════════════════════════════════════
static MazeGenerator* getGen() {
    switch (selGen) {
        case GenType::DFS_GRID:    return &dfsGen;
        case GenType::PRIM_BRANCH: return &primGen;
    }
    return &dfsGen;
}

static void showError(const std::string& msg, float dur = 3.5f) {
    errorMsg = msg;
    errTimer = dur;
}

// Ensure deliveryOrder contains valid customer IDs
static void syncDeliveryOrder() {
    std::vector<int> valid;
    for (int id : deliveryOrder) {
        if (maze.getCustomer(id) != nullptr) valid.push_back(id);
    }
    for (const auto& c : maze.customers) {
        if (std::find(valid.begin(), valid.end(), c.id) == valid.end()) {
            valid.push_back(c.id);
        }
    }
    deliveryOrder = valid;
}

// ════════════════════════════════════════════════════════════
// ── SEARCH CACHE INVALIDATION & DATA RETRIEVAL ──────────────
// ════════════════════════════════════════════════════════════
static void invalidateSearchResults() {
    for (int i = 0; i < 3; i++) {
        algoStates[i].hasRun            = false;
        algoStates[i].foundPath         = false;
        algoStates[i].result            = MultiStopResult{};
        algoStates[i].animPath.clear();
        algoStates[i].animStep          = 0;
        algoStates[i].animProgress      = 0.f;
        algoStates[i].animActive        = false;
        algoStates[i].stopsReached      = 0;
        algoStates[i].deliveryCompleted = false;
        algoStates[i].statusText        = "Not executed yet";
    }
    compResults.clear();
    maze.resetSearch();
}

static bool getActiveStops(std::vector<Position>& outStops, std::vector<std::string>& outLabels) {
    outStops.clear();
    outLabels.clear();
    syncDeliveryOrder();
    if (deliveryMode == DeliveryMode::SINGLE_TARGET) {
        const Customer* cust = maze.getCustomer(maze.selectedCustomerId);
        if (!cust && !maze.customers.empty()) {
            cust = &maze.customers.front();
            maze.selectedCustomerId = cust->id;
        }
        if (cust) {
            outStops.push_back(cust->pos);
            outLabels.push_back(cust->label());
        }
    } else {
        for (int cid : deliveryOrder) {
            const Customer* cust = maze.getCustomer(cid);
            if (cust && cust->active) {
                outStops.push_back(cust->pos);
                outLabels.push_back(cust->label());
            }
        }
    }
    return !outStops.empty();
}

// ════════════════════════════════════════════════════════════
// ── INDEPENDENT SEARCH EXECUTION & FAIR COMPARISON ──────────
// ════════════════════════════════════════════════════════════
static void runAlgorithm(int algIdx) {
    if (algIdx < 0 || algIdx >= 3) return;
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    if (maze.customers.empty()) {
        showError("Add at least one customer destination.");
        return;
    }
    std::vector<Position> stops;
    std::vector<std::string> labels;
    if (!getActiveStops(stops, labels)) {
        showError("No active customer stops found.");
        return;
    }

    const char* algNames[3] = {"BFS", "DFS", "A*"};
    AlgorithmState& as = algoStates[algIdx];
    as.animActive        = false;
    as.animStep          = 0;
    as.animProgress      = 0.f;
    as.stopsReached      = 0;
    as.deliveryCompleted = false;

    // Run independent search run on identical maze snapshot
    as.result    = runMultiStopSearch(maze, stops, labels, algNames[algIdx]);
    as.hasRun    = true;
    as.foundPath = as.result.fullyCompleted;
    as.animPath  = as.result.fullPath;

    if (as.result.fullyCompleted) {
        std::ostringstream ss;
        ss << as.name << " route found: " << as.result.totalMoves << " moves | Cost: "
           << std::fixed << std::setprecision(1) << as.result.totalCost;
        as.statusText = ss.str();
        statusMsg = as.statusText;
    } else {
        as.statusText = "NO PATH: Leg to " + as.result.failedDestination + " unreachable!";
        statusMsg = as.name + " -- " + as.statusText;
    }

    if (algIdx == selectedAlgoIdx) {
        maze.resetSearch();
        if (as.foundPath) {
            maze.markPath(as.animPath);
        }
    }
}

static void runAllAlgorithms() {
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    if (maze.customers.empty()) {
        showError("Add at least one customer destination.");
        return;
    }
    std::vector<Position> stops;
    std::vector<std::string> labels;
    if (!getActiveStops(stops, labels)) {
        showError("No active customer stops found.");
        return;
    }

    compResults.clear();
    const char* algNames[3] = {"BFS", "DFS", "A*"};
    for (int i = 0; i < 3; i++) {
        runAlgorithm(i);
        CompResult cr;
        cr.name     = algNames[i];
        cr.multiRes = algoStates[i].result;
        compResults.push_back(cr);
    }
    statusMsg = "All 3 algorithms independently evaluated on identical network snapshot.";
}

static void startVanAnimation() {
    AlgorithmState& as = algoStates[selectedAlgoIdx];
    if (!as.hasRun || as.animPath.empty()) {
        showError("No valid route found to animate for " + as.name + ".");
        return;
    }
    as.animStep          = 0;
    as.animProgress      = 0.f;
    as.stopsReached      = 0;
    as.deliveryCompleted = false;
    as.animActive        = true;
    statusMsg = "Simulating delivery van along " + as.name + " route...";
}

static void stopVanAnimation() {
    algoStates[selectedAlgoIdx].animActive = false;
    statusMsg = "Delivery van simulation paused.";
}

static void updateRider() {
    AlgorithmState& as = algoStates[selectedAlgoIdx];
    if (!as.animActive || as.animPath.size() < 2) return;

    as.animProgress += riderSpeed;
    if (as.animProgress >= 1.f) {
        as.animProgress = 0.f;
        as.animStep++;

        // Update physically arrived stops
        int reached = 0;
        for (size_t i = 0; i < as.result.stopPathIndices.size(); i++) {
            if (as.animStep >= as.result.stopPathIndices[i]) {
                reached++;
            }
        }
        as.stopsReached = reached;

        if (as.animStep >= (int)as.animPath.size() - 1) {
            as.animStep   = (int)as.animPath.size() - 1;
            as.animActive = false;
            if (as.result.fullyCompleted) {
                as.deliveryCompleted = true;
                as.stopsReached      = as.result.totalStops;
                statusMsg = "Delivery completed! Van arrived at final destination via " + as.name + ".";
            } else {
                as.deliveryCompleted = false;
                statusMsg = "Van halted at last reachable stop. Final destination is unreachable.";
            }
        }
    } else {
        int reached = 0;
        for (size_t i = 0; i < as.result.stopPathIndices.size(); i++) {
            if (as.animStep >= as.result.stopPathIndices[i]) {
                reached++;
            }
        }
        as.stopsReached = reached;
    }
}

static void doStep() {
    if (appState == AppState::GENERATING && activeGen && !activeGen->isFinished()) {
        activeGen->step();
        if (activeGen->isFinished()) {
            appState  = AppState::MAZE_WORKFLOW;
            statusMsg = "Road network ready. Click CONTINUE TO ROAD CONFIG.";
        }
    } else if (appState == AppState::SOLVED) {
        startVanAnimation();
    }
}

// ════════════════════════════════════════════════════════════
// ── PREDEFINED SCENARIOS ────────────────────────────────────
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
    maze.customers.clear();
    maze.nextCustomerId = 1;
    maze.addCustomer({rows - 1, cols - 1});
    mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols);
    syncDeliveryOrder();
    initButtonLayout();
}

static void loadScenarioA() {
    const int ROWS = 11, COLS = 15;
    buildOpenGrid(ROWS, COLS);
    activeScenario = ScenarioE::A_SIMPLE;
    statusMsg = "Scenario A loaded: Open grid delivery from Store to C1.";
}

static void loadScenarioB() {
    const int ROWS = 11, COLS = 15;
    buildOpenGrid(ROWS, COLS);
    for (int c = 2; c <= 7; c++) {
        maze.at(0, c).special = SpecialType::REWARD;
    }
    activeScenario = ScenarioE::B_REWARD;
    statusMsg = "Scenario B loaded: Express shortcut (Cost 0.5) along top avenue.";
}

static void loadScenarioC() {
    const int ROWS = 11, COLS = 15;
    buildOpenGrid(ROWS, COLS);
    for (int r = 1; r <= 8; r++) {
        maze.at(r, 7).special = SpecialType::PENALTY;
    }
    for (int c = 2; c <= 6; c++) {
        maze.at(10, c).special = SpecialType::REWARD;
    }
    activeScenario = ScenarioE::C_PENALTY;
    statusMsg = "Scenario C loaded: Roadblocks (Cost 4.0) vs bottom bypass (Cost 0.5).";
}

// ════════════════════════════════════════════════════════════
// ── MOUSE INTERACTION HANDLERS ──────────────────────────────
// ════════════════════════════════════════════════════════════
static void applyWallEditTool(int row, int col) {
    Position p{row, col};
    if (p == maze.start) {
        showError("Cannot edit walls on Store (Start).");
        return;
    }
    if (maze.hasCustomerAt(p)) {
        showError("Cannot edit walls on Customer destination.");
        return;
    }

    if (editWallTool == EditTool::WALL) {
        maze.blockCell(p);
    } else {
        maze.unblockCell(p);
    }
    invalidateSearchResults();
}

static void applyRoadConfigTool(int row, int col) {
    Position p{row, col};
    if (p == maze.start) {
        showError("Cannot place pellet on Store (Start).");
        return;
    }
    if (maze.hasCustomerAt(p)) {
        showError("Cannot place pellet on Customer destination.");
        return;
    }
    if (maze.at(p).isWall()) {
        showError("Cannot place pellet on impassable wall.");
        return;
    }

    Cell& cell = maze.at(p);
    pelletUndoStack.push_back({p, cell.terrain, cell.special});

    switch (pelletTool) {
        case PelletTool::REWARD_05:
            cell.special = SpecialType::REWARD;
            cell.terrain = TerrainType::NORMAL;
            break;
        case PelletTool::PENALTY_40:
            cell.special = SpecialType::PENALTY;
            cell.terrain = TerrainType::NORMAL;
            break;
        case PelletTool::CONGESTED_20:
            cell.special = SpecialType::NONE;
            cell.terrain = TerrainType::CONGESTED;
            break;
        case PelletTool::NORMAL_10:
            cell.special = SpecialType::NONE;
            cell.terrain = TerrainType::NORMAL;
            break;
    }
    invalidateSearchResults();
}

static void handleSidebarClick(float mx, float my) {
    // ── Phase 1 Buttons ──────────────────────────────────────
    if (currentPhase == AppPhase::PHASE_1_CREATE) {
        if (btnSzSm.isClicked(mx, my)) { selSize = MazeSizeE::SMALL;  maze.resize(MSIZES[0][0], MSIZES[0][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); invalidateSearchResults(); }
        else if (btnSzMd.isClicked(mx, my)) { selSize = MazeSizeE::MEDIUM; maze.resize(MSIZES[1][0], MSIZES[1][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); invalidateSearchResults(); }
        else if (btnSzLg.isClicked(mx, my)) { selSize = MazeSizeE::LARGE;  maze.resize(MSIZES[2][0], MSIZES[2][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); invalidateSearchResults(); }
        else if (btnGenDFS.isClicked(mx, my))  { selGen = GenType::DFS_GRID; }
        else if (btnGenPrim.isClicked(mx, my)) { selGen = GenType::PRIM_BRANCH; }
        else if (btnGenerate.isClicked(mx, my)) {
            maze.resize(MSIZES[(int)selSize][0], MSIZES[(int)selSize][1]);
            activeGen = getGen();
            activeGen->initialize(maze, currentSeed);
            activeGen->runToCompletion();
            maze.generated = true;
            syncDeliveryOrder();
            initButtonLayout();
            invalidateSearchResults();
            statusMsg = "Road network generated. Add customers or continue to road config.";
        }
        else if (btnNewSeed.isClicked(mx, my)) {
            currentSeed++;
            globalRng.seed(currentSeed);
            maze.hardReset();
            syncDeliveryOrder();
            initButtonLayout();
            invalidateSearchResults();
            statusMsg = "Reset network to blank grid. New Seed: " + std::to_string(currentSeed);
        }
        else if (btnScenA.isClicked(mx, my)) { loadScenarioA(); invalidateSearchResults(); }
        else if (btnScenB.isClicked(mx, my)) { loadScenarioB(); invalidateSearchResults(); }
        else if (btnScenC.isClicked(mx, my)) { loadScenarioC(); invalidateSearchResults(); }
        else if (btnToggleWallEdit.isClicked(mx, my)) {
            wallEditActive = !wallEditActive;
            custPlacementTool = CustPlacementTool::NONE;
            statusMsg = wallEditActive ? "Wall Edit: Click/drag road cells to toggle barrier walls." : "Wall Edit deactivated.";
        }
        else if (btnClearWalls.isClicked(mx, my)) {
            buildOpenGrid(maze.rows, maze.cols);
            invalidateSearchResults();
            statusMsg = "All interior walls removed (Open Grid layout).";
        }
        else if (btnModeSingle.isClicked(mx, my)) {
            deliveryMode = DeliveryMode::SINGLE_TARGET;
            statusMsg = "Delivery Mode: Single Customer target.";
            invalidateSearchResults();
            updateUILayout();
        }
        else if (btnModeMulti.isClicked(mx, my)) {
            deliveryMode = DeliveryMode::MULTI_STOP;
            statusMsg = "Delivery Mode: Multi-Stop route following configured stop order.";
            invalidateSearchResults();
            updateUILayout();
        }
        else if (btnCustAdd.isClicked(mx, my)) {
            if ((int)maze.customers.size() >= 10) {
                showError("Customer limit reached (max 10).");
            } else {
                custPlacementTool = CustPlacementTool::ADD_CUST;
                wallEditActive = false;
                statusMsg = "Click any open road cell to place new customer.";
            }
        }
        else if (btnCustMove.isClicked(mx, my)) {
            if (maze.customers.empty()) {
                showError("No customer selected to move.");
            } else {
                custPlacementTool = CustPlacementTool::MOVE_CUST;
                wallEditActive = false;
                statusMsg = "Click a road cell to move C" + std::to_string(maze.selectedCustomerId) + ".";
            }
        }
        else if (btnCustRemove.isClicked(mx, my)) {
            if (maze.customers.size() <= 1) {
                showError("Must maintain at least 1 customer destination.");
            } else {
                int remId = maze.selectedCustomerId;
                maze.removeCustomer(remId);
                deliveryOrder.erase(std::remove(deliveryOrder.begin(), deliveryOrder.end(), remId), deliveryOrder.end());
                syncDeliveryOrder();
                initButtonLayout();
                invalidateSearchResults();
                statusMsg = "Removed customer C" + std::to_string(remId) + ". Remaining IDs preserved.";
            }
        }
        else if (btnCustUp.isClicked(mx, my)) {
            auto it = std::find(deliveryOrder.begin(), deliveryOrder.end(), maze.selectedCustomerId);
            if (it != deliveryOrder.end() && it != deliveryOrder.begin()) {
                std::iter_swap(it, it - 1);
                invalidateSearchResults();
                statusMsg = "Moved C" + std::to_string(maze.selectedCustomerId) + " earlier in stop order.";
                updateUILayout();
            }
        }
        else if (btnCustDown.isClicked(mx, my)) {
            auto it = std::find(deliveryOrder.begin(), deliveryOrder.end(), maze.selectedCustomerId);
            if (it != deliveryOrder.end() && it + 1 != deliveryOrder.end()) {
                std::iter_swap(it, it + 1);
                invalidateSearchResults();
                statusMsg = "Moved C" + std::to_string(maze.selectedCustomerId) + " later in stop order.";
                updateUILayout();
            }
        }
        else {
            // Customer chips click check
            for (size_t i = 0; i < btnCustChips.size(); i++) {
                if (btnCustChips[i].isClicked(mx, my)) {
                    if (i < maze.customers.size()) {
                        maze.selectedCustomerId = maze.customers[i].id;
                        maze.goal = maze.customers[i].pos;
                        statusMsg = "Selected customer " + maze.customers[i].label() + ".";
                        invalidateSearchResults();
                        initButtonLayout();
                    }
                    break;
                }
            }
        }

        if (btnPhase1Continue.isClicked(mx, my)) {
            if (!maze.generated) {
                showError("Generate or load a road network first.");
                return;
            }
            if (maze.customers.empty()) {
                showError("Add at least one customer destination.");
                return;
            }
            wallEditActive    = false;
            custPlacementTool = CustPlacementTool::NONE;
            currentPhase      = AppPhase::PHASE_2_CONFIG;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg         = "Phase 2: Configure Reward (0.5), Congested (2.0) or Penalty (4.0) roads.";
        }
    }

    // ── Phase 2 Buttons ──────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
        if (btnToolReward.isClicked(mx, my))         { pelletTool = PelletTool::REWARD_05; }
        else if (btnToolPenalty.isClicked(mx, my))   { pelletTool = PelletTool::PENALTY_40; }
        else if (btnToolCongested.isClicked(mx, my)) { pelletTool = PelletTool::CONGESTED_20; }
        else if (btnToolNormal.isClicked(mx, my))    { pelletTool = PelletTool::NORMAL_10; }
        else if (btnUndoPellet.isClicked(mx, my)) {
            if (!pelletUndoStack.empty()) {
                PelletHistory h = pelletUndoStack.back();
                pelletUndoStack.pop_back();
                if (maze.inBounds(h.pos)) {
                    maze.at(h.pos).terrain = h.oldTerrain;
                    maze.at(h.pos).special = h.oldSpecial;
                    invalidateSearchResults();
                    statusMsg = "Undid last terrain edit.";
                }
            } else {
                showError("Undo history is empty.");
            }
        }
        else if (btnClearPellets.isClicked(mx, my)) {
            maze.clearAllSpecialPellets();
            pelletUndoStack.clear();
            invalidateSearchResults();
            statusMsg = "All reward, penalty & congested pellets cleared. Walls & endpoints intact.";
        }
        else if (btnPhase2Continue.isClicked(mx, my)) {
            currentPhase = AppPhase::PHASE_3_SOLVE;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg = "Phase 3: Select an algorithm and click Run Search.";
        }
        else if (btnPhase2Back.isClicked(mx, my)) {
            currentPhase = AppPhase::PHASE_1_CREATE;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg = "Returned to Phase 1. Road network and customers preserved.";
        }
    }

    // ── Phase 3 Buttons ──────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
        if (btnSelBFS.isClicked(mx, my)) {
            selectedAlgoIdx = 0; selSearch = SearchType::BFS;
            maze.resetSearch();
            if (algoStates[0].hasRun && algoStates[0].foundPath) maze.markPath(algoStates[0].animPath);
            statusMsg = "Selected BFS (Fewest Moves).";
        }
        else if (btnSelDFS.isClicked(mx, my)) {
            selectedAlgoIdx = 1; selSearch = SearchType::DFS_S;
            maze.resetSearch();
            if (algoStates[1].hasRun && algoStates[1].foundPath) maze.markPath(algoStates[1].animPath);
            statusMsg = "Selected DFS (Depth-First).";
        }
        else if (btnSelAStar.isClicked(mx, my)) {
            selectedAlgoIdx = 2; selSearch = SearchType::ASTAR;
            maze.resetSearch();
            if (algoStates[2].hasRun && algoStates[2].foundPath) maze.markPath(algoStates[2].animPath);
            statusMsg = "Selected A* (Lowest Weighted Cost).";
        }
        else if (btnRun.isClicked(mx, my))    { runAlgorithm(selectedAlgoIdx); }
        else if (btnRunAll.isClicked(mx, my)) { runAllAlgorithms(); }
        else if (btnToggleOverlay.isClicked(mx, my)) {
            overlayComparison = !overlayComparison;
            updateUILayout();
            statusMsg = overlayComparison ? "Overlay comparison mode active: all computed routes displayed." : "Single path display mode.";
        }
        else if (btnResetSearch.isClicked(mx, my)) {
            algoStates[selectedAlgoIdx].hasRun            = false;
            algoStates[selectedAlgoIdx].foundPath         = false;
            algoStates[selectedAlgoIdx].result            = MultiStopResult{};
            algoStates[selectedAlgoIdx].animPath.clear();
            algoStates[selectedAlgoIdx].animStep          = 0;
            algoStates[selectedAlgoIdx].animProgress      = 0.f;
            algoStates[selectedAlgoIdx].animActive        = false;
            algoStates[selectedAlgoIdx].stopsReached      = 0;
            algoStates[selectedAlgoIdx].deliveryCompleted = false;
            algoStates[selectedAlgoIdx].statusText        = "Not executed yet";
            maze.resetSearch();
            statusMsg = "Reset search for " + algoStates[selectedAlgoIdx].name + ".";
        }
        else if (btnAnimateRoute.isClicked(mx, my)) { startVanAnimation(); }
        else if (btnStopAnim.isClicked(mx, my))     { stopVanAnimation(); }
        else if (btnCompareAll.isClicked(mx, my))   { runAllAlgorithms(); appState = AppState::COMPARING; }
        else if (btnPhase3BackToConfig.isClicked(mx, my)) {
            maze.resetSearch();
            algoStates[selectedAlgoIdx].animActive = false;
            currentPhase = AppPhase::PHASE_2_CONFIG;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg    = "Returned to Phase 2: Road & terrain configuration.";
        }
        else if (btnPhase3NewMaze.isClicked(mx, my)) {
            maze.resetSearch();
            algoStates[selectedAlgoIdx].animActive = false;
            currentPhase = AppPhase::PHASE_1_CREATE;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg    = "Returned to Phase 1: Create a new road network.";
        }
    }
}

// ════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════
// ── TOP HEADER RENDERING ────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderHeader() {
    drawPanel(0.f, 0.f, (float)WIN_W, HEADER_H, Col::HEADER_BG, Col::BORDER, 1.0f);

    float tx = 16.f, ty = 28.f;
    glColor3fv(Col::TITLE_CYAN);
    drawBitmapText(tx, ty, "SmartRoute AI", GLUT_BITMAP_HELVETICA_18);
    int titleW = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"SmartRoute AI");

    float subX = tx + (float)titleW + 12.f;
    if (WIN_W >= 1150) {
        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(subX, ty, "|  Intelligent Delivery Optimizer", GLUT_BITMAP_HELVETICA_12);
        subX += (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)"|  Intelligent Delivery Optimizer") + 16.f;
    } else if (WIN_W >= 980) {
        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(subX, ty, "|  Delivery Optimizer", GLUT_BITMAP_HELVETICA_12);
        subX += (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)"|  Delivery Optimizer") + 16.f;
    }

    // Phase Indicator Chip
    float pBadgeX = subX;
    float pBadgeW = 186.f;
    const char* pBadgeText = "PHASE 1: CREATE MAZE";
    const float* pBadgeCol = Col::PRIMARY_BLUE;
    if (currentPhase == AppPhase::PHASE_2_CONFIG) {
        pBadgeText = "PHASE 2: CONFIGURE ROADS";
        pBadgeCol  = Col::GREEN_REWARD;
    } else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
        pBadgeText = "PHASE 3: SOLVE & COMPARE";
        pBadgeCol  = Col::TITLE_CYAN;
    }

    // Header buttons boundary (Legend starts at WIN_W - 156.f)
    float rightEdge = (float)WIN_W - 166.f;

    if (pBadgeX + pBadgeW + 16.f < rightEdge) {
        drawPanel(pBadgeX, 11.f, pBadgeW, 26.f, Col::CARD_BG, pBadgeCol, 1.5f);
        glColor3fv(pBadgeCol);
        drawBitmapText(pBadgeX + 10.f, 28.f, pBadgeText, GLUT_BITMAP_HELVETICA_10);

        float statusX = pBadgeX + pBadgeW + 14.f;
        float availStatusW = rightEdge - statusX - 10.f;
        if (availStatusW > 60.f) {
            if (!errorMsg.empty() && errTimer > 0.f) {
                drawTextTruncated(statusX, ty, errorMsg, availStatusW, GLUT_BITMAP_HELVETICA_12, Col::RED_PENALTY);
            } else {
                drawTextTruncated(statusX, ty, statusMsg, availStatusW, GLUT_BITMAP_HELVETICA_12, Col::TEXT_WHITE);
            }
        }
    } else {
        float availStatusW = rightEdge - subX - 10.f;
        if (availStatusW > 60.f) {
            if (!errorMsg.empty() && errTimer > 0.f) {
                drawTextTruncated(subX, ty, errorMsg, availStatusW, GLUT_BITMAP_HELVETICA_12, Col::RED_PENALTY);
            } else {
                drawTextTruncated(subX, ty, statusMsg, availStatusW, GLUT_BITMAP_HELVETICA_12, Col::TEXT_WHITE);
            }
        }
    }

    btnTopLegend.draw();
    btnTopHelp.draw();
}

// ── MAZE CANVAS RENDERING ───────────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderCell(int row, int col) {
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

    // Congested Road Marker
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

    // 2. Route Path Highlight
    if (cell.inPath && curPos != maze.start && !maze.hasCustomerAt(curPos)) {
        glColor3f(Col::PATH[0] * 0.35f, Col::PATH[1] * 0.35f, Col::PATH[2] * 0.35f);
        glBegin(GL_QUADS);
        glVertex2f(x + 2.f, y + 2.f);
        glVertex2f(x + cw - 2.f, y + 2.f);
        glVertex2f(x + cw - 2.f, y + cw - 2.f);
        glVertex2f(x + 2.f, y + cw - 2.f);
        glEnd();
    }

    // 3. Pellets Overlay
    if (!cell.isWall() && cell.special != SpecialType::NONE) {
        float cx = x + cw * 0.5f, cy = y + cw * 0.5f;
        float r  = cw * 0.32f;

        if (cell.special == SpecialType::REWARD) {
            glColor3fv(Col::GREEN_REWARD);
            drawDiamond(cx, cy, r);
            glColor3f(0.55f, 1.0f, 0.75f);
            drawDiamond(cx, cy, r * 0.45f);
        } else if (cell.special == SpecialType::PENALTY) {
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

    // 4. Wall Segments
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

    // 5. Store (S) Marker
    if (curPos == maze.start) {
        float inset = cw * 0.12f;
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

    // 6. Customer (C1, C2...) Marker
    if (maze.hasCustomerAt(curPos)) {
        int cid = maze.getCustomerIdAt(curPos);
        std::string lbl = "C" + std::to_string(cid);

        float inset = cw * 0.10f;
        float mw = cw - 2.f * inset;

        // Base box
        glColor3fv(Col::GOAL);
        glBegin(GL_QUADS);
        glVertex2f(x + inset, y + inset);           glVertex2f(x + inset + mw, y + inset);
        glVertex2f(x + inset + mw, y + inset + mw); glVertex2f(x + inset, y + inset + mw);
        glEnd();

        // Selected customer halo ring
        if (cid == maze.selectedCustomerId) {
            glColor3fv(Col::GOAL_SEL);
            glLineWidth(2.5f);
            glBegin(GL_LINE_LOOP);
            glVertex2f(x + inset - 1.f, y + inset - 1.f);
            glVertex2f(x + inset + mw + 1.f, y + inset - 1.f);
            glVertex2f(x + inset + mw + 1.f, y + inset + mw + 1.f);
            glVertex2f(x + inset - 1.f, y + inset + mw + 1.f);
            glEnd();
            glLineWidth(1.0f);
        }

        // Draw Customer label
        glColor3fv(Col::TEXT_WHITE);
        int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char*)lbl.c_str());
        drawBitmapText(x + (cw - (float)tw) * 0.5f, y + cw * 0.5f + 3.f, lbl.c_str(), GLUT_BITMAP_HELVETICA_10);
    }
}

// Draw continuous delivery path line across all stops
static void renderRouteRibbon() {
    if (overlayComparison) {
        const float* algCols[3] = {Col::CYAN, Col::CORAL, Col::START}; // BFS Cyan, DFS Coral, A* Emerald
        const float offsets[3][2] = {{-2.2f, -2.2f}, {2.2f, 2.2f}, {0.0f, 0.0f}};
        for (int i = 0; i < 3; i++) {
            if (algoStates[i].hasRun && algoStates[i].foundPath && algoStates[i].animPath.size() >= 2) {
                glColor3fv(algCols[i]);
                glLineWidth(i == selectedAlgoIdx ? 3.5f : 2.5f);
                glBegin(GL_LINE_STRIP);
                for (const Position& p : algoStates[i].animPath) {
                    glVertex2f(mazeLayout.cellCenterX(p.col) + offsets[i][0],
                               mazeLayout.cellCenterY(p.row) + offsets[i][1]);
                }
                glEnd();
            }
        }
        glLineWidth(1.0f);
    } else {
        const AlgorithmState& as = algoStates[selectedAlgoIdx];
        if (as.hasRun && as.foundPath && as.animPath.size() >= 2) {
            const float* col = (selectedAlgoIdx == 0) ? Col::CYAN :
                               (selectedAlgoIdx == 1) ? Col::CORAL : Col::START;
            glColor3fv(col);
            glLineWidth(3.5f);
            glBegin(GL_LINE_STRIP);
            for (const Position& p : as.animPath) {
                glVertex2f(mazeLayout.cellCenterX(p.col), mazeLayout.cellCenterY(p.row));
            }
            glEnd();
            glLineWidth(1.0f);
        }
    }
}

static void renderDeliveryVan() {
    const AlgorithmState& as = algoStates[selectedAlgoIdx];
    if (!as.animActive || as.animPath.empty()) return;

    Position a = as.animPath[as.animStep];
    Position b = (as.animStep + 1 < (int)as.animPath.size()) ? as.animPath[as.animStep + 1] : a;
    float ax = mazeLayout.cellCenterX(a.col), ay = mazeLayout.cellCenterY(a.row);
    float bx = mazeLayout.cellCenterX(b.col), by = mazeLayout.cellCenterY(b.row);

    float px = (1.f - as.animProgress) * ax + as.animProgress * bx;
    float py = (1.f - as.animProgress) * ay + as.animProgress * by;
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
}

static void renderCanvasHUD() {
    if (currentPhase != AppPhase::PHASE_3_SOLVE) return;

    float hudX = mazeLayout.canvasX + 16.f;
    float hudY = mazeLayout.canvasY + 14.f;
    float hudW = 270.f;
    float hudH = overlayComparison ? 96.f : 78.f;

    // Semi-transparent background panel
    drawPanel(hudX, hudY, hudW, hudH, Col::CARD_BG, Col::BORDER, 1.0f);

    if (overlayComparison) {
        glColor3fv(Col::TITLE_CYAN);
        drawBitmapText(hudX + 10.f, hudY + 16.f, "OVERLAY COMPARISON MODE", GLUT_BITMAP_HELVETICA_10);

        auto drawLegLine = [&](float lx, float ly, const float* col, const char* name, const AlgorithmState& st) {
            glColor3fv(col);
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            glVertex2f(lx, ly); glVertex2f(lx + 16.f, ly);
            glEnd();
            glLineWidth(1.0f);

            drawBitmapText(lx + 22.f, ly + 4.f, name, GLUT_BITMAP_HELVETICA_10);
            if (st.hasRun && st.foundPath) {
                std::string s = std::to_string(st.result.totalMoves) + " m / " +
                                std::to_string((int)st.result.totalCost) + (st.result.totalCost == (int)st.result.totalCost ? ".0" : ".5");
                glColor3fv(Col::TEXT_WHITE);
                drawBitmapText(lx + 136.f, ly + 4.f, s.c_str(), GLUT_BITMAP_HELVETICA_10);
            } else if (st.hasRun) {
                glColor3fv(Col::RED_PENALTY);
                drawBitmapText(lx + 136.f, ly + 4.f, "NO PATH", GLUT_BITMAP_HELVETICA_10);
            } else {
                glColor3fv(Col::TEXT_MUTED);
                drawBitmapText(lx + 136.f, ly + 4.f, "Not executed", GLUT_BITMAP_HELVETICA_10);
            }
        };

        drawLegLine(hudX + 10.f, hudY + 36.f, Col::CYAN,  "BFS (Fewest Moves)", algoStates[0]);
        drawLegLine(hudX + 10.f, hudY + 54.f, Col::CORAL, "DFS (Depth-First)",  algoStates[1]);
        drawLegLine(hudX + 10.f, hudY + 72.f, Col::START, "A*  (Lowest Cost)",  algoStates[2]);
    } else {
        const AlgorithmState& cur = algoStates[selectedAlgoIdx];
        const float* col = (selectedAlgoIdx == 0) ? Col::CYAN :
                           (selectedAlgoIdx == 1) ? Col::CORAL : Col::START;

        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(hudX + 10.f, hudY + 16.f, "ACTIVE ROUTE DISPLAY", GLUT_BITMAP_HELVETICA_10);

        glColor3fv(col);
        glLineWidth(3.5f);
        glBegin(GL_LINES);
        glVertex2f(hudX + 10.f, hudY + 32.f); glVertex2f(hudX + 28.f, hudY + 32.f);
        glEnd();
        glLineWidth(1.0f);

        std::string title = cur.name + " (" + (selectedAlgoIdx == 0 ? "Fewest Moves" : selectedAlgoIdx == 1 ? "Depth-First" : "Lowest Cost") + ")";
        drawBitmapText(hudX + 34.f, hudY + 36.f, title.c_str(), GLUT_BITMAP_HELVETICA_12);

        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(hudX + 10.f, hudY + 58.f, "Toggle Overlay: Press 'O' or click button", GLUT_BITMAP_HELVETICA_10);
    }
}

// ════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════
// ── SIDEBAR RENDERING ───────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void renderSidebar() {
    float sideX = sideLayout.sideX;
    float x     = sideLayout.x;
    float fullW = sideLayout.fullW;

    // Sidebar Background Base
    drawPanel(sideX, HEADER_H, SIDEBAR_W, (float)(WIN_H - HEADER_H), Col::SIDEBAR_BG, Col::BORDER, 1.0f);

    // Enable scissor test so scrolling content never leaks into header or outside sidebar
    glEnable(GL_SCISSOR_TEST);
    glScissor((int)sideX, 0, (int)SIDEBAR_W, (int)(WIN_H - HEADER_H));

    glPushMatrix();
    glTranslatef(0.f, -sideLayout.scrollY, 0.f);

    // ────────────────────────────────────────────────────────
    // PHASE 1 — CREATE ROAD NETWORK & DESTINATIONS
    // ────────────────────────────────────────────────────────
    if (currentPhase == AppPhase::PHASE_1_CREATE) {
        drawPhaseHeader(x, sideLayout.p1HeaderY, fullW, "PHASE 1", "CREATE ROAD NETWORK",
                        "Generate street grid, set Store & Customers.");

        // 1. Grid & Generator
        drawSectionHeader(x, sideLayout.p1GenSecY, "MAZE & GENERATOR");
        btnSzSm.selected = (selSize == MazeSizeE::SMALL);
        btnSzMd.selected = (selSize == MazeSizeE::MEDIUM);
        btnSzLg.selected = (selSize == MazeSizeE::LARGE);
        btnSzSm.draw(); btnSzMd.draw(); btnSzLg.draw();

        btnGenDFS.selected  = (selGen == GenType::DFS_GRID);
        btnGenPrim.selected = (selGen == GenType::PRIM_BRANCH);
        btnGenDFS.draw(); btnGenPrim.draw();

        btnGenerate.draw(); btnNewSeed.draw();

        btnScenA.selected = (activeScenario == ScenarioE::A_SIMPLE); btnScenA.draw();
        btnScenB.selected = (activeScenario == ScenarioE::B_REWARD); btnScenB.draw();
        btnScenC.selected = (activeScenario == ScenarioE::C_PENALTY); btnScenC.draw();

        btnToggleWallEdit.selected = wallEditActive;
        btnToggleWallEdit.draw(); btnClearWalls.draw();

        // 2. Customer Management Panel
        std::string custHdr = "CUSTOMERS (" + std::to_string(maze.customers.size()) + "/10)";
        drawSectionHeader(x, sideLayout.p1CustSecY, custHdr.c_str());

        btnModeSingle.selected = (deliveryMode == DeliveryMode::SINGLE_TARGET);
        btnModeMulti.selected  = (deliveryMode == DeliveryMode::MULTI_STOP);
        btnModeSingle.draw(); btnModeMulti.draw();

        for (size_t i = 0; i < btnCustChips.size(); i++) {
            btnCustChips[i].draw();
        }

        btnCustAdd.selected  = (custPlacementTool == CustPlacementTool::ADD_CUST);
        btnCustMove.selected = (custPlacementTool == CustPlacementTool::MOVE_CUST);
        btnCustAdd.draw(); btnCustMove.draw(); btnCustRemove.draw();

        if (deliveryMode == DeliveryMode::MULTI_STOP) {
            btnCustUp.draw(); btnCustDown.draw();
            float seqY = btnCustUp.y + 26.f;
            glColor3fv(Col::TEXT_MUTED);
            drawBitmapText(x, seqY, "Delivery Route Sequence:", GLUT_BITMAP_HELVETICA_10);
            std::string seq = "Store";
            for (int cid : deliveryOrder) {
                const Customer* c = maze.getCustomer(cid);
                if (c && c->active) seq += " -> " + c->label();
            }
            drawTextTruncated(x + 4.f, seqY + 14.f, seq, fullW - 8.f, GLUT_BITMAP_HELVETICA_10, Col::ACCENT);
        } else {
            float tgtY = btnCustAdd.y + 28.f + 14.f;
            std::string selStr = "Selected Target: Customer C" + std::to_string(maze.selectedCustomerId);
            drawTextTruncated(x, tgtY, selStr, fullW, GLUT_BITMAP_HELVETICA_10, Col::TEXT_MUTED);
        }

        // 3. Grid Legend
        drawGridLegend(x, sideLayout.p1LegendY, fullW);

        // 4. Navigation
        btnPhase1Continue.draw();
    }

    // ────────────────────────────────────────────────────────
    // PHASE 2 — CONFIGURE REWARD & PENALTY ROADS
    // ────────────────────────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
        drawPhaseHeader(x, sideLayout.p2HeaderY, fullW, "PHASE 2", "CONFIGURE ROADS",
                        "Select a tool, then click road cells",
                        "to assign delivery travel costs.");

        // 1. Road Tools (2-Column Grid)
        drawSectionHeader(x, sideLayout.p2ToolsSecY, "ROAD TOOLS");
        btnToolReward.selected    = (pelletTool == PelletTool::REWARD_05);
        btnToolPenalty.selected   = (pelletTool == PelletTool::PENALTY_40);
        btnToolCongested.selected = (pelletTool == PelletTool::CONGESTED_20);
        btnToolNormal.selected    = (pelletTool == PelletTool::NORMAL_10);

        btnToolReward.draw(); btnToolPenalty.draw();
        btnToolCongested.draw(); btnToolNormal.draw();

        // 2. Pellet Summary Card
        drawSectionHeader(x, sideLayout.p2SummarySecY, "PELLET SUMMARY");
        drawPanel(x, sideLayout.p2SummaryCardY, fullW, 64.f, Col::CARD_BG, Col::BORDER, 1.0f);
        drawMetricRow(x + 10.f, sideLayout.p2SummaryCardY + 18.f, fullW - 20.f, "Reward Pellets (0.5):",
                      std::to_string(maze.countRewardPellets()).c_str(),
                      Col::TEXT_MUTED, Col::GREEN_REWARD, GLUT_BITMAP_HELVETICA_10);
        drawMetricRow(x + 10.f, sideLayout.p2SummaryCardY + 36.f, fullW - 20.f, "Penalty Pellets (4.0):",
                      std::to_string(maze.countPenaltyPellets()).c_str(),
                      Col::TEXT_MUTED, Col::RED_PENALTY, GLUT_BITMAP_HELVETICA_10);
        drawMetricRow(x + 10.f, sideLayout.p2SummaryCardY + 54.f, fullW - 20.f, "Congested Roads (2.0):",
                      std::to_string(maze.countCongestedCells()).c_str(),
                      Col::TEXT_MUTED, Col::ORANGE_CONGEST, GLUT_BITMAP_HELVETICA_10);

        // 3. Configuration Actions
        drawSectionHeader(x, sideLayout.p2ActionsSecY, "ACTIONS");
        btnUndoPellet.draw(); btnClearPellets.draw();

        // 4. Terrain Cost Legend
        drawTerrainLegend(x, sideLayout.p2LegendY, fullW, true);

        // 5. Workflow Navigation
        drawSectionHeader(x, sideLayout.p2NavSecY, "WORKFLOW NAVIGATION");
        btnPhase2Continue.draw();
        btnPhase2Back.draw();
    }

    // ────────────────────────────────────────────────────────
    // PHASE 3 — SOLVE & COMPARE (BFS, DFS, A*)
    // ────────────────────────────────────────────────────────
    else if (currentPhase == AppPhase::PHASE_3_SOLVE) {
        drawPhaseHeader(x, sideLayout.p3HeaderY, fullW, "PHASE 3", "SOLVE AND COMPARE",
                        "Evaluate BFS, DFS, and A* routing",
                        "across single or multi-stop deliveries.");

        // 1. Delivery Target
        drawSectionHeader(x, sideLayout.p3TargetSecY, "DELIVERY TARGET");
        drawPanel(x, sideLayout.p3TargetCardY, fullW, 40.f, Col::CARD_BG, Col::BORDER, 1.0f);
        if (deliveryMode == DeliveryMode::SINGLE_TARGET) {
            std::string tStr = "Single Target: Customer C" + std::to_string(maze.selectedCustomerId);
            drawTextTruncated(x + 10.f, sideLayout.p3TargetCardY + 16.f, tStr, fullW - 20.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_WHITE);
            drawTextTruncated(x + 10.f, sideLayout.p3TargetCardY + 30.f, "Finding path from Store to selected stop", fullW - 20.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_MUTED);
        } else {
            drawTextTruncated(x + 10.f, sideLayout.p3TargetCardY + 16.f, "Multi-Stop Route:", fullW - 20.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_WHITE);
            std::string seq = "Store";
            for (int cid : deliveryOrder) {
                const Customer* c = maze.getCustomer(cid);
                if (c && c->active) seq += " -> " + c->label();
            }
            drawTextTruncated(x + 10.f, sideLayout.p3TargetCardY + 30.f, seq, fullW - 20.f, GLUT_BITMAP_HELVETICA_10, Col::ACCENT);
        }

        // 2. Algorithm Selection
        drawSectionHeader(x, sideLayout.p3AlgSecY, "ALGORITHM");
        btnSelBFS.selected   = (selectedAlgoIdx == 0); btnSelBFS.draw();
        btnSelDFS.selected   = (selectedAlgoIdx == 1); btnSelDFS.draw();
        btnSelAStar.selected = (selectedAlgoIdx == 2); btnSelAStar.draw();

        // 3. Execution Controls
        drawSectionHeader(x, sideLayout.p3ExecSecY, "EXECUTION CONTROLS");
        btnRun.draw(); btnRunAll.draw();
        btnResetSearch.draw(); btnToggleOverlay.draw();
        btnAnimateRoute.disabled = !algoStates[selectedAlgoIdx].hasRun || algoStates[selectedAlgoIdx].animPath.empty();
        btnAnimateRoute.draw(); btnStopAnim.draw();

        // 4. Results Card
        drawSectionHeader(x, sideLayout.p3ResultsSecY, "RESULTS");
        drawPanel(x, sideLayout.p3ResultsCardY, fullW, 122.f, Col::CARD_BG, Col::BORDER, 1.0f);

        const AlgorithmState& curAs = algoStates[selectedAlgoIdx];
        if (curAs.hasRun) {
            const char* compStatusStr = curAs.deliveryCompleted ? "COMPLETED" : (curAs.foundPath ? "IN PROGRESS" : "NO PATH / UNREACHABLE");
            const float* compStatusCol = curAs.deliveryCompleted ? Col::GREEN_REWARD : (curAs.foundPath ? Col::ORANGE_CONGEST : Col::RED_PENALTY);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 17.f, fullW - 20.f, "Delivery Status:",
                          compStatusStr, Col::TEXT_MUTED, compStatusCol, GLUT_BITMAP_HELVETICA_10);

            std::string stpStr = std::to_string(curAs.stopsReached) + " / " + std::to_string(curAs.result.totalStops);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 33.f, fullW - 20.f, "Stops Reached:",
                          stpStr.c_str(), Col::TEXT_MUTED, curAs.deliveryCompleted ? Col::GREEN_REWARD : Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

            if (curAs.foundPath) {
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 49.f, fullW - 20.f, "Route Moves:",
                              std::to_string(curAs.result.totalMoves).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

                std::ostringstream cst; cst << std::fixed << std::setprecision(1) << curAs.result.totalCost;
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 65.f, fullW - 20.f, "Weighted Cost:",
                              cst.str().c_str(), Col::TEXT_MUTED, Col::PRIMARY_BLUE, GLUT_BITMAP_HELVETICA_10);

                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 81.f, fullW - 20.f, "Expanded Nodes:",
                              std::to_string(curAs.result.totalNodesExplored).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

                std::ostringstream ts; ts << std::fixed << std::setprecision(2) << curAs.result.totalTimeMs << " ms";
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 97.f, fullW - 20.f, "Search Time:",
                              ts.str().c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

                std::string rwPn = std::to_string(curAs.result.rewardsTraversed) + " / " + std::to_string(curAs.result.penaltiesTraversed);
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 113.f, fullW - 20.f, "Rewards / Penalties:",
                              rwPn.c_str(), Col::TEXT_MUTED, Col::GREEN_REWARD, GLUT_BITMAP_HELVETICA_10);
            } else {
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 49.f, fullW - 20.f, "Route Moves:",
                              "NO PATH", Col::TEXT_MUTED, Col::RED_PENALTY, GLUT_BITMAP_HELVETICA_10);
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 65.f, fullW - 20.f, "Weighted Cost:",
                              "N/A", Col::TEXT_MUTED, Col::RED_PENALTY, GLUT_BITMAP_HELVETICA_10);
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 81.f, fullW - 20.f, "Expanded Nodes:",
                              std::to_string(curAs.result.totalNodesExplored).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);
                std::ostringstream ts; ts << std::fixed << std::setprecision(2) << curAs.result.totalTimeMs << " ms";
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 97.f, fullW - 20.f, "Search Time:",
                              ts.str().c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);
                drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 113.f, fullW - 20.f, "Unreachable Leg:",
                              curAs.result.failedDestination.c_str(), Col::TEXT_MUTED, Col::RED_PENALTY, GLUT_BITMAP_HELVETICA_10);
            }
        } else {
            glColor3fv(Col::TEXT_MUTED);
            drawBitmapText(x + 12.f, sideLayout.p3ResultsCardY + 38.f, "No search executed yet.", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(x + 12.f, sideLayout.p3ResultsCardY + 60.f, "Click 'Run Selected' or 'Run All 3'.", GLUT_BITMAP_HELVETICA_10);
            if (deliveryMode == DeliveryMode::MULTI_STOP) {
                drawBitmapText(x + 12.f, sideLayout.p3ResultsCardY + 80.f, "Multi-stop follows user visit order.", GLUT_BITMAP_HELVETICA_10);
            }
        }

        // 5. Compare All Button (Below Results Card!)
        btnCompareAll.draw();

        // 6. Navigation (Below Compare All!)
        btnPhase3BackToConfig.draw();
        btnPhase3NewMaze.draw();
    }

    glPopMatrix();
    glDisable(GL_SCISSOR_TEST);

    // Subtle scrollbar thumb if content is scrollable
    if (sideLayout.maxScrollY > 0.f) {
        float trackH = (float)(WIN_H - HEADER_H);
        float thumbH = std::max(20.f, trackH * (trackH / (trackH + sideLayout.maxScrollY)));
        float thumbY = HEADER_H + (sideLayout.scrollY / sideLayout.maxScrollY) * (trackH - thumbH);
        drawPanel((float)WIN_W - 5.f, thumbY, 3.f, thumbH, Col::ACCENT);
    }
}

// ── FULL-SCREEN COMPARISON DASHBOARD ────────────────────────
// ════════════════════════════════════════════════════════════
static void renderComparisonDashboard() {
    drawPanel(0.f, 0.f, (float)WIN_W, (float)WIN_H, Col::BG_DARK);

    // If comparison hasn't been run yet, run all algorithms automatically
    if (!algoStates[0].hasRun && !algoStates[1].hasRun && !algoStates[2].hasRun && maze.generated && !maze.customers.empty()) {
        runAllAlgorithms();
    }

    // Build evaluation data from current algorithm states
    std::vector<AlgoEvalData> evals;
    for (int i = 0; i < 3; i++) {
        AlgoEvalData d;
        d.name = algoStates[i].name;
        d.hasRun = algoStates[i].hasRun;
        d.foundPath = algoStates[i].foundPath;
        d.totalMoves = algoStates[i].result.totalMoves;
        d.totalCost = algoStates[i].result.totalCost;
        d.totalNodesExplored = algoStates[i].result.totalNodesExplored;
        d.medianTimeMs = algoStates[i].result.benchmark.medianMs;
        d.singleRunTimeMs = algoStates[i].result.totalTimeMs;
        d.stopsCompleted = algoStates[i].result.stopsCompleted;
        d.totalStops = algoStates[i].result.totalStops;
        evals.push_back(d);
    }
    WinnerResult wMoves = computeRouteMovesWinner(evals);
    WinnerResult wCost  = computeWeightedCostWinner(evals);
    WinnerResult wNodes = computeFewestNodesWinner(evals);
    WinnerResult wTime  = computeExecutionTimeWinner(evals);
    bool allEqual       = checkAllEqualRouteQuality(evals);

    float cx = (float)WIN_W * 0.5f, y = 26.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"ALGORITHM COMPARISON DASHBOARD");
    drawBitmapText(cx - (float)tw * 0.5f, y, "ALGORITHM COMPARISON DASHBOARD", GLUT_BITMAP_HELVETICA_18);
    y += 20.f;

    glColor3fv(Col::TEXT_MUTED);
    std::string sub = (deliveryMode == DeliveryMode::SINGLE_TARGET)
        ? "Single Customer C" + std::to_string(maze.selectedCustomerId) + " | Monotonic Steady Clock Benchmark (25 runs)"
        : "Ordered Route: " + (compResults.empty() ? "" : compResults[0].multiRes.stopSequenceStr) + " | Monotonic Benchmark (25 runs)";
    int sw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)sub.c_str());
    drawBitmapText(cx - (float)sw * 0.5f, y, sub.c_str(), GLUT_BITMAP_HELVETICA_12);
    y += 22.f;

    float tX = 64.f, tW = (float)WIN_W - 128.f;
    drawSep(tX, y, tW); y += 14.f;

    // Table Column Coordinates: Metric | BFS | DFS | A*
    float cMetric = tX;
    float cBFS    = tX + 270.f;
    float cDFS    = tX + 540.f;
    float cAStar  = tX + 810.f;

    // Table Header Row
    glColor3fv(Col::ACCENT);
    drawBitmapText(cMetric, y, "METRIC", GLUT_BITMAP_HELVETICA_12);
    glColor3fv(Col::CYAN);
    drawBitmapText(cBFS,    y, "BFS (FIFO Queue)", GLUT_BITMAP_HELVETICA_12);
    glColor3fv(Col::CORAL);
    drawBitmapText(cDFS,    y, "DFS (LIFO Stack)", GLUT_BITMAP_HELVETICA_12);
    glColor3fv(Col::START);
    drawBitmapText(cAStar,  y, "A* (f = g + h)", GLUT_BITMAP_HELVETICA_12);
    y += 14.f;
    drawSep(tX, y, tW); y += 14.f;

    // Optimal values across completed runs
    int   bestMoves = 999999;
    float bestCost  = 999999.f;
    int   bestNodes = 999999;
    double bestTime = 1e9;
    for (int i = 0; i < 3; i++) {
        const auto& mr = algoStates[i].result;
        if (algoStates[i].hasRun && mr.fullyCompleted) {
            bestMoves = std::min(bestMoves, mr.totalMoves);
            bestCost  = std::min(bestCost,  mr.totalCost);
        }
        if (algoStates[i].hasRun) {
            bestNodes = std::min(bestNodes, mr.totalNodesExplored);
            if (mr.benchmark.medianMs > 0.0005) {
                bestTime = std::min(bestTime, mr.benchmark.medianMs);
            }
        }
    }

    auto renderTableRow = [&](const char* metricName, const std::string& vBfs, const std::string& vDfs, const std::string& vAStar,
                              const float* colBfs = Col::TEXT_WHITE, const float* colDfs = Col::TEXT_WHITE, const float* colAStar = Col::TEXT_WHITE) {
        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(cMetric, y, metricName, GLUT_BITMAP_HELVETICA_12);

        glColor3fv(colBfs);
        drawBitmapText(cBFS, y, vBfs.c_str(), GLUT_BITMAP_HELVETICA_12);

        glColor3fv(colDfs);
        drawBitmapText(cDFS, y, vDfs.c_str(), GLUT_BITMAP_HELVETICA_12);

        glColor3fv(colAStar);
        drawBitmapText(cAStar, y, vAStar.c_str(), GLUT_BITMAP_HELVETICA_12);

        y += 22.f;
    };

    // 1. Result status
    std::string statBfs   = !algoStates[0].hasRun ? "Not executed" : (algoStates[0].foundPath ? "Path Found (Complete)" : "DESTINATION UNREACHABLE");
    std::string statDfs   = !algoStates[1].hasRun ? "Not executed" : (algoStates[1].foundPath ? "Path Found (Complete)" : "DESTINATION UNREACHABLE");
    std::string statAStar = !algoStates[2].hasRun ? "Not executed" : (algoStates[2].foundPath ? "Path Found (Complete)" : "DESTINATION UNREACHABLE");
    renderTableRow("Result status", statBfs, statDfs, statAStar,
                   algoStates[0].foundPath ? Col::GREEN_REWARD : Col::RED_PENALTY,
                   algoStates[1].foundPath ? Col::GREEN_REWARD : Col::RED_PENALTY,
                   algoStates[2].foundPath ? Col::GREEN_REWARD : Col::RED_PENALTY);

    // 2. Route moves
    auto formatMoves = [&](int idx) -> std::pair<std::string, const float*> {
        if (!algoStates[idx].hasRun) return {"-", Col::TEXT_MUTED};
        if (!algoStates[idx].foundPath) return {"NO PATH", Col::RED_PENALTY};
        int m = algoStates[idx].result.totalMoves;
        std::string s = std::to_string(m) + " moves";
        if (m == bestMoves) {
            std::string tag = wMoves.isTie ? " (Shortest * - tie)" : " (Shortest *)";
            return {s + tag, Col::GREEN_REWARD};
        }
        return {s + " (+" + std::to_string(m - bestMoves) + ")", Col::TEXT_WHITE};
    };
    auto pBfsMoves = formatMoves(0), pDfsMoves = formatMoves(1), pAStarMoves = formatMoves(2);
    renderTableRow("Route moves", pBfsMoves.first, pDfsMoves.first, pAStarMoves.first, pBfsMoves.second, pDfsMoves.second, pAStarMoves.second);

    // 3. Weighted route cost
    auto formatCost = [&](int idx) -> std::pair<std::string, const float*> {
        if (!algoStates[idx].hasRun) return {"-", Col::TEXT_MUTED};
        if (!algoStates[idx].foundPath) return {"N/A", Col::RED_PENALTY};
        float c = algoStates[idx].result.totalCost;
        std::ostringstream ss; ss << std::fixed << std::setprecision(1) << c;
        if (c <= bestCost + 0.001f) {
            std::string tag = wCost.isTie ? " (Lowest * - tie)" : " (Lowest *)";
            return {ss.str() + tag, Col::GREEN_REWARD};
        }
        std::ostringstream diff; diff << " (+" << std::fixed << std::setprecision(1) << (c - bestCost) << ")";
        return {ss.str() + diff.str(), Col::TEXT_WHITE};
    };
    auto pBfsCost = formatCost(0), pDfsCost = formatCost(1), pAStarCost = formatCost(2);
    renderTableRow("Weighted route cost", pBfsCost.first, pDfsCost.first, pAStarCost.first, pBfsCost.second, pDfsCost.second, pAStarCost.second);

    // 4. Expanded nodes
    auto formatNodes = [&](int idx) -> std::pair<std::string, const float*> {
        if (!algoStates[idx].hasRun) return {"-", Col::TEXT_MUTED};
        int n = algoStates[idx].result.totalNodesExplored;
        std::string s = std::to_string(n) + " nodes";
        if (n == bestNodes) {
            std::string tag = wNodes.isTie ? " (Fewest * - tie)" : " (Fewest *)";
            return {s + tag, Col::START};
        }
        float pct = bestNodes > 0 ? (float)(n - bestNodes) / (float)bestNodes * 100.f : 0.f;
        std::ostringstream diff; diff << " (+" << (int)pct << "%)";
        return {s + diff.str(), Col::TEXT_WHITE};
    };
    auto pBfsNodes = formatNodes(0), pDfsNodes = formatNodes(1), pAStarNodes = formatNodes(2);
    renderTableRow("Expanded nodes", pBfsNodes.first, pDfsNodes.first, pAStarNodes.first, pBfsNodes.second, pDfsNodes.second, pAStarNodes.second);

    // 5. Median search time (25 runs)
    auto formatMedianTime = [&](int idx) -> std::pair<std::string, const float*> {
        if (!algoStates[idx].hasRun) return {"-", Col::TEXT_MUTED};
        double t = algoStates[idx].result.benchmark.medianMs;
        std::string durStr = formatSearchDuration(t, true);
        bool isFastest = false;
        for (const auto& w : wTime.winners) {
            if (w == algoStates[idx].name) { isFastest = true; break; }
        }
        if (isFastest && wTime.isAvailable) {
            std::string tag = wTime.isTie ? " (Fastest * - tie)" : " (Fastest *)";
            return {durStr + tag, Col::GREEN_REWARD};
        }
        return {durStr, Col::TEXT_WHITE};
    };
    auto pBfsTime = formatMedianTime(0), pDfsTime = formatMedianTime(1), pAStarTime = formatMedianTime(2);
    renderTableRow("Median search time", pBfsTime.first, pDfsTime.first, pAStarTime.first, pBfsTime.second, pDfsTime.second, pAStarTime.second);

    // 6. Single-run time
    auto formatSingleTime = [&](int idx) -> std::string {
        if (!algoStates[idx].hasRun) return "-";
        return formatSearchDuration(algoStates[idx].result.totalTimeMs, false);
    };
    renderTableRow("Single-run time", formatSingleTime(0), formatSingleTime(1), formatSingleTime(2));

    // 7. Stops reached
    auto formatStops = [&](int idx) -> std::string {
        if (!algoStates[idx].hasRun) return "-";
        return std::to_string(algoStates[idx].result.stopsCompleted) + " / " + std::to_string(algoStates[idx].result.totalStops);
    };
    renderTableRow("Stops reached", formatStops(0), formatStops(1), formatStops(2));

    // 8. Shortcuts / Penalties
    auto formatPellets = [&](int idx) -> std::string {
        if (!algoStates[idx].hasRun) return "-";
        return std::to_string(algoStates[idx].result.rewardsTraversed) + " rew / " + std::to_string(algoStates[idx].result.penaltiesTraversed) + " pen";
    };
    renderTableRow("Shortcuts / Penalties", formatPellets(0), formatPellets(1), formatPellets(2));

    y += 2.f;
    drawSep(tX, y, tW); y += 14.f;

    // ── RELATIVE COMPARISON & WINNER BADGES ──────────────────────
    float cardW = (tW - 20.f) * 0.5f;
    float cardH = 132.f;
    float c1X   = tX;
    float c2X   = tX + cardW + 20.f;

    // Card 1: Relative Comparisons
    drawPanel(c1X, y, cardW, cardH, Col::CARD_BG, Col::BORDER, 1.0f);
    drawSectionHeader(c1X + 12.f, y + 8.f, "RELATIVE EFFICIENCY & METRIC DELTAS");
    float cy1 = y + 30.f;

    // Moves difference
    std::string moveDiffStr = "Move Delta vs Shortest (" + (bestMoves < 999999 ? std::to_string(bestMoves) : "-") + "): ";
    for (int i = 0; i < 3; i++) {
        if (algoStates[i].hasRun && algoStates[i].foundPath) {
            int d = algoStates[i].result.totalMoves - bestMoves;
            moveDiffStr += algoStates[i].name + (d == 0 ? " [0]" : (" [+" + std::to_string(d) + "]")) + "  ";
        }
    }
    drawTextTruncated(c1X + 12.f, cy1, moveDiffStr, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_WHITE); cy1 += 20.f;

    // Cost difference
    std::ostringstream costDiffSS;
    costDiffSS << "Cost Delta vs Cheapest (" << (bestCost < 999999.f ? std::to_string((int)bestCost) + ((bestCost == (int)bestCost) ? ".0" : ".5") : "-") << "): ";
    for (int i = 0; i < 3; i++) {
        if (algoStates[i].hasRun && algoStates[i].foundPath) {
            float cd = algoStates[i].result.totalCost - bestCost;
            costDiffSS << algoStates[i].name << (cd <= 0.001f ? " [0.0]" : " [+" + std::to_string((int)cd) + (cd == (int)cd ? ".0" : ".5") + "]") << "  ";
        }
    }
    drawTextTruncated(c1X + 12.f, cy1, costDiffSS.str(), cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::PRIMARY_BLUE); cy1 += 20.f;

    // Nodes Explored Relative
    std::string nodeDiffStr = "Node Delta vs Fewest (" + (bestNodes < 999999 ? std::to_string(bestNodes) : "-") + "): ";
    for (int i = 0; i < 3; i++) {
        if (algoStates[i].hasRun) {
            int nd = algoStates[i].result.totalNodesExplored - bestNodes;
            nodeDiffStr += algoStates[i].name + (nd == 0 ? " [0]" : (" [+" + std::to_string(nd) + "]")) + "  ";
        }
    }
    drawTextTruncated(c1X + 12.f, cy1, nodeDiffStr, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_MUTED); cy1 += 20.f;

    // Median Time Delta vs Fastest
    std::string timeDiffStr = "Median Time vs Fastest: ";
    if (bestTime < 1e8) {
        for (int i = 0; i < 3; i++) {
            if (algoStates[i].hasRun && algoStates[i].result.benchmark.medianMs > 0.0005) {
                double td = algoStates[i].result.benchmark.medianMs - bestTime;
                if (td <= 0.002) {
                    timeDiffStr += algoStates[i].name + " [0 \xc2\xb5s]  ";
                } else {
                    double diffUs = td * 1000.0;
                    timeDiffStr += algoStates[i].name + " [+" + std::to_string((int)std::round(diffUs)) + " \xc2\xb5s]  ";
                }
            }
        }
    } else {
        timeDiffStr += "N/A";
    }
    drawTextTruncated(c1X + 12.f, cy1, timeDiffStr, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TITLE_CYAN); cy1 += 20.f;

    drawTextTruncated(c1X + 12.f, cy1, "A* balances moves & terrain cost; BFS optimizes step count only.", cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::GREEN_REWARD);

    // Card 2: Winners & Honest Takeaways
    drawPanel(c2X, y, cardW, cardH, Col::CARD_BG, Col::BORDER, 1.0f);
    drawSectionHeader(c2X + 12.f, y + 8.f, "RUN WINNERS & ALGORITHMIC GUARANTEES");
    float cy2 = y + 30.f;

    std::string moveWin = "Fewest Moves -- " + wMoves.summary;
    drawTextTruncated(c2X + 12.f, cy2, moveWin, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::CYAN); cy2 += 20.f;

    std::string costWin = "Lowest Weighted Cost -- " + wCost.summary;
    drawTextTruncated(c2X + 12.f, cy2, costWin, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::START); cy2 += 20.f;

    std::string nodeWin = "Fewest Expanded Nodes -- " + wNodes.summary;
    drawTextTruncated(c2X + 12.f, cy2, nodeWin, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_WHITE); cy2 += 20.f;

    std::string timeWin = "Fastest Median Search -- " + wTime.summary;
    drawTextTruncated(c2X + 12.f, cy2, timeWin, cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::GREEN_REWARD); cy2 += 20.f;

    if (allEqual) {
        drawTextTruncated(c2X + 12.f, cy2, "Equal quality: Different search strategies found identical paths.", cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::ACCENT);
    } else {
        drawTextTruncated(c2X + 12.f, cy2, "Measured winners reflect this specific maze & customer order.", cardW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_MUTED);
    }

    // Card 3: Algorithm Characteristics & Guarantees
    float c3Y = y + cardH + 12.f;
    float c3H = 92.f;
    drawPanel(tX, c3Y, tW, c3H, Col::CARD_BG, Col::BORDER, 1.0f);
    drawSectionHeader(tX + 12.f, c3Y + 8.f, "ALGORITHM SEARCH PROPERTIES & GUARANTEES");
    float cy3 = c3Y + 28.f;
    drawTextTruncated(tX + 12.f, cy3, "BFS:  FIFO queue. Guarantees minimum moves (unweighted shortest path). Ignores road terrain weights.", tW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::CYAN); cy3 += 18.f;
    drawTextTruncated(tX + 12.f, cy3, "DFS:  LIFO stack. No move or cost optimality guarantees; behavior depends entirely on traversal order.", tW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::CORAL); cy3 += 18.f;
    drawTextTruncated(tX + 12.f, cy3, "A*:   Priority queue on f(n)=g(n)+h(n). Guarantees optimal weighted cost when heuristic h(n) is admissible.", tW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::START); cy3 += 18.f;
    if (allEqual) {
        drawTextTruncated(tX + 12.f, cy3, "Note: On simple corridors, BFS, DFS, and A* can find equal routes while exploring different node counts.", tW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_WHITE);
    } else {
        drawTextTruncated(tX + 12.f, cy3, "Note: Concatenated multi-stop legs preserve start-to-goal continuity without double-counting shared stop points.", tW - 24.f, GLUT_BITMAP_HELVETICA_10, Col::TEXT_MUTED);
    }

    btnBackToWorkflow.draw();
    btnDashOverlay.draw();
    btnDashRerun.draw();
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

    sec(lx, colY, "4. Multi-Customer Routing", Col::ACCENT,
        "Supports single customer destination or user-ordered stops.",
        "Sequential leg routing: Store -> C1 -> C2 -> ...",
        "Concatenates legs without double-counting shared endpoints.");

    float ry = y;
    glColor3fv(Col::ACCENT);
    drawBitmapText(rx, ry, "AI PROBLEM FORMULATION", GLUT_BITMAP_HELVETICA_18); ry += 24.f;

    const char* formNotes[] = {
        "- Initial State: Dispatch Store / Warehouse (S) at (r0, c0).",
        "- Goal State: Customer delivery location(s) (C1, C2, C3...).",
        "- Actions: 4-Directional grid movement (Up, Down, Left, Right).",
        "- Path Cost: Sum of entered cell costs (Reward=0.5, Normal=1.0, Congested=2.0, Penalty=4.0).",
        "- Heuristic Function: h(n) = 0.5 * (|dr| + |dc|) to target customer.",
        "- Admissibility: h(n) <= true cost because 0.5 is the minimum possible step cost.",
        "- Multi-Stop Sequence: Store -> C1 -> C2 ... follows user-specified order."
    };
    for (const char* fn : formNotes) {
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(rx + 10.f, ry, fn, GLUT_BITMAP_HELVETICA_12);
        ry += 20.f;
    }

    btnHelpBack.draw();
}

static void renderLegendScreen() {
    drawPanel(0.f, 0.f, (float)WIN_W, (float)WIN_H, Col::BG_DARK);

    float cx = (float)WIN_W * 0.5f, y = 36.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"VISUAL INTERFACE LEGEND");
    drawBitmapText(cx - (float)tw * 0.5f, y, "VISUAL INTERFACE LEGEND", GLUT_BITMAP_HELVETICA_18);
    y += 48.f;

    float lx = 90.f, rx = cx + 40.f;

    // Left Column: Map Elements
    glColor3fv(Col::ACCENT);
    drawBitmapText(lx, y, "MAP GRID ELEMENTS", GLUT_BITMAP_HELVETICA_18); float ly = y + 28.f;

    auto legRow = [&](float sx, float& sy, const float* col, const char* name, const char* desc, bool diamond = false, bool cross = false) {
        if (diamond) {
            glColor3fv(col);
            drawDiamond(sx + 10.f, sy - 4.f, 7.f);
        } else if (cross) {
            glColor3fv(col);
            drawBitmapText(sx + 4.f, sy, "✕", GLUT_BITMAP_HELVETICA_18);
        } else {
            drawRect(sx, sy - 12.f, 16.f, 16.f, col[0], col[1], col[2]);
        }
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(sx + 28.f, sy, name, GLUT_BITMAP_HELVETICA_12);
        glColor3fv(Col::TEXT_MUTED);
        drawBitmapText(sx + 180.f, sy, desc, GLUT_BITMAP_HELVETICA_12);
        sy += 26.f;
    };

    legRow(lx, ly, Col::START, "Store / Warehouse", "(S) Fixed delivery dispatch origin");
    legRow(lx, ly, Col::GOAL, "Customer Stop", "(C1..C10) Configurable delivery destinations");
    legRow(lx, ly, Col::GOAL_SEL, "Selected Customer", "Highlighted with golden halo ring");
    legRow(lx, ly, Col::ROAD_NORM, "Normal Road", "Traversable grid road, Base cost = 1.0");
    legRow(lx, ly, Col::GREEN_REWARD, "Reward Pellet", "Fast shortcut road, Traversal cost = 0.5", true);
    legRow(lx, ly, Col::RED_PENALTY, "Penalty Road", "Delay / construction roadblock, Cost = 4.0", false, true);
    legRow(lx, ly, Col::ORANGE_CONGEST, "Congested Road", "Heavy traffic road segment, Cost = 2.0");
    legRow(lx, ly, Col::WALL_BLOCK, "Impassable Wall", "Blocked building / barrier obstacle");

    // Right Column: Search & Algorithm Overlays
    glColor3fv(Col::ACCENT);
    drawBitmapText(rx, y, "ALGORITHM VISUALIZATION", GLUT_BITMAP_HELVETICA_18); float ry = y + 28.f;

    legRow(rx, ry, Col::EXPLORED, "Explored Nodes", "States expanded by the search algorithm");
    legRow(rx, ry, Col::FRONTIER, "Frontier Nodes", "Active boundary states in queue / priority queue");
    legRow(rx, ry, Col::PATH, "Computed Route", "Final route path connecting Store to customer stops");
    legRow(rx, ry, Col::RIDER, "Delivery Van", "Vehicle traversing computed delivery journey");

    btnLegendBack.draw();
}

// ════════════════════════════════════════════════════════════
// ── DISPLAY CALLBACK ────────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void display() {
    glClearColor(Col::BG_DARK[0], Col::BG_DARK[1], Col::BG_DARK[2], 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    if (appState == AppState::HELP) {
        renderHelpScreen();
        glutSwapBuffers();
        return;
    }
    if (appState == AppState::LEGEND) {
        renderLegendScreen();
        glutSwapBuffers();
        return;
    }
    if (appState == AppState::COMPARING) {
        renderComparisonDashboard();
        glutSwapBuffers();
        return;
    }

    // Main 3-Phase Application View
    renderHeader();

    // Render Canvas Cells
    for (int r = 0; r < maze.rows; r++) {
        for (int c = 0; c < maze.cols; c++) {
            renderCell(r, c);
        }
    }

    renderRouteRibbon();
    renderDeliveryVan();
    renderCanvasHUD();
    renderSidebar();

    glutSwapBuffers();
}

// ════════════════════════════════════════════════════════════
// ── GLUT EVENT HANDLERS ─────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void timerCallback(int val) {
    if (errTimer > 0.f) {
        errTimer -= (TIMER_MS / 1000.f);
        if (errTimer <= 0.f) errorMsg = "";
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
    updateUILayout();
    glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y) {
    // Standard GLUT Mouse Wheel (Button 3 = Wheel Up, Button 4 = Wheel Down)
    if (button == 3 || button == 4) {
        if (state == GLUT_DOWN && (float)x >= (float)(WIN_W - SIDEBAR_W)) {
            if (sideLayout.maxScrollY > 0.f) {
                if (button == 3) sideLayout.scrollY -= 30.f;
                else             sideLayout.scrollY += 30.f;
                if (sideLayout.scrollY < 0.f)                  sideLayout.scrollY = 0.f;
                if (sideLayout.scrollY > sideLayout.maxScrollY) sideLayout.scrollY = sideLayout.maxScrollY;
                glutPostRedisplay();
            }
        }
        return;
    }

    bool press = (state == GLUT_DOWN);
    mouseDown  = press && (button == GLUT_LEFT_BUTTON);

    if (!press) return;

    if (appState == AppState::COMPARING) {
        if (btnBackToWorkflow.isClicked((float)x, (float)y)) {
            appState = AppState::MAZE_WORKFLOW;
        } else if (btnDashOverlay.isClicked((float)x, (float)y)) {
            overlayComparison = !overlayComparison;
            updateUILayout();
        } else if (btnDashRerun.isClicked((float)x, (float)y)) {
            runAllAlgorithms();
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

    // Right Sidebar Clicks (with scroll offset applied!)
    if ((float)x >= (float)(WIN_W - SIDEBAR_W)) {
        float clickY = (float)y + sideLayout.scrollY;
        handleSidebarClick((float)x, clickY);
        glutPostRedisplay();
        return;
    }

    // Maze Canvas Clicks
    int row, col;
    if (mazeLayout.hitTest((float)x, (float)y, row, col, maze.rows, maze.cols)) {
        Position clickPos{row, col};
        if (currentPhase == AppPhase::PHASE_1_CREATE) {
            if (custPlacementTool == CustPlacementTool::ADD_CUST) {
                if (clickPos == maze.start) {
                    showError("Cannot place Customer on Store (Start).");
                } else if (maze.hasCustomerAt(clickPos)) {
                    showError("Cell already has a Customer destination.");
                } else if (maze.at(clickPos).isWall()) {
                    showError("Cannot place Customer on wall / obstacle.");
                } else if ((int)maze.customers.size() >= 10) {
                    showError("Customer limit reached (max 10).");
                } else {
                    int newId = maze.addCustomer(clickPos);
                    if (newId > 0) {
                        deliveryOrder.push_back(newId);
                        syncDeliveryOrder();
                        updateUILayout();
                        invalidateSearchResults();
                        statusMsg = "Added customer C" + std::to_string(newId) + " at (" + std::to_string(row) + ", " + std::to_string(col) + ").";
                        custPlacementTool = CustPlacementTool::NONE;
                    }
                }
            } else if (custPlacementTool == CustPlacementTool::MOVE_CUST) {
                if (clickPos == maze.start) {
                    showError("Cannot move Customer onto Store.");
                } else if (maze.hasCustomerAt(clickPos)) {
                    showError("Another Customer already occupies this cell.");
                } else if (maze.at(clickPos).isWall()) {
                    showError("Cannot move Customer onto wall.");
                } else {
                    if (maze.moveCustomer(maze.selectedCustomerId, clickPos)) {
                        invalidateSearchResults();
                        statusMsg = "Moved C" + std::to_string(maze.selectedCustomerId) + " to (" + std::to_string(row) + ", " + std::to_string(col) + ").";
                        custPlacementTool = CustPlacementTool::NONE;
                    } else {
                        showError("Failed to move customer.");
                    }
                }
            } else if (maze.hasCustomerAt(clickPos)) {
                // Clicked directly on an existing customer: select it!
                int cid = maze.getCustomerIdAt(clickPos);
                maze.selectedCustomerId = cid;
                maze.goal = clickPos;
                statusMsg = "Selected customer C" + std::to_string(cid) + ".";
                invalidateSearchResults();
                updateUILayout();
            } else if (wallEditActive) {
                applyWallEditTool(row, col);
            }
        } else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
            applyRoadConfigTool(row, col);
        }
    }

    glutPostRedisplay();
}

static void mouseMotion(int x, int y) {
    if (appState == AppState::COMPARING) {
        btnBackToWorkflow.updateHover((float)x, (float)y);
        btnDashOverlay.updateHover((float)x, (float)y);
        btnDashRerun.updateHover((float)x, (float)y);
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

    float hoverY = (float)y;
    if ((float)x >= (float)(WIN_W - SIDEBAR_W) && (float)y >= HEADER_H) {
        hoverY += sideLayout.scrollY;
    }

    for (Button* b : allButtons) {
        b->updateHover((float)x, hoverY);
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

static void keyboard(unsigned char key, int x, int y) {
    if (appState == AppState::COMPARING || appState == AppState::HELP || appState == AppState::LEGEND) {
        if (key == 27 || key == 'c' || key == 'C' || key == 'h' || key == 'H' || key == 'l' || key == 'L') {
            appState = AppState::MAZE_WORKFLOW;
            glutPostRedisplay();
        }
        return;
    }

    switch (key) {
        case 'g': case 'G':
            maze.resize(MSIZES[(int)selSize][0], MSIZES[(int)selSize][1]);
            activeGen = getGen();
            activeGen->initialize(maze, currentSeed);
            activeGen->runToCompletion();
            maze.generated = true;
            syncDeliveryOrder();
            initButtonLayout();
            invalidateSearchResults();
            break;
        case '1':
            selectedAlgoIdx = 0; selSearch = SearchType::BFS;
            maze.resetSearch();
            if (algoStates[0].hasRun && algoStates[0].foundPath) maze.markPath(algoStates[0].animPath);
            statusMsg = "Selected: BFS (Fewest Moves)";
            break;
        case '2':
            selectedAlgoIdx = 1; selSearch = SearchType::DFS_S;
            maze.resetSearch();
            if (algoStates[1].hasRun && algoStates[1].foundPath) maze.markPath(algoStates[1].animPath);
            statusMsg = "Selected: DFS (Depth-First)";
            break;
        case '3':
            selectedAlgoIdx = 2; selSearch = SearchType::ASTAR;
            maze.resetSearch();
            if (algoStates[2].hasRun && algoStates[2].foundPath) maze.markPath(algoStates[2].animPath);
            statusMsg = "Selected: A* (Lowest Cost)";
            break;
        case 'r': case 'R':
            if (appState == AppState::COMPARING) {
                runAllAlgorithms();
            } else {
                runAlgorithm(selectedAlgoIdx);
            }
            break;
        case 'a': case 'A':
            runAllAlgorithms();
            break;
        case 'o': case 'O':
            overlayComparison = !overlayComparison;
            updateUILayout();
            statusMsg = overlayComparison ? "Overlay comparison mode: ON" : "Overlay comparison mode: OFF";
            break;
        case 'v': case 'V':
            startVanAnimation();
            break;
        case 'c': case 'C':
            runAllAlgorithms();
            appState = AppState::COMPARING;
            break;
        case ' ':
            paused = !paused;
            statusMsg = paused ? "Paused." : "Resumed.";
            break;
        case 'n': case 'N': doStep(); break;
        case 'l': case 'L': appState = AppState::LEGEND; break;
        case 'h': case 'H': appState = AppState::HELP; break;
        case 27: // ESC
            if (currentPhase == AppPhase::PHASE_3_SOLVE) {
                currentPhase = AppPhase::PHASE_2_CONFIG;
                sideLayout.scrollY = 0.f;
                updateUILayout();
            } else if (currentPhase == AppPhase::PHASE_2_CONFIG) {
                currentPhase = AppPhase::PHASE_1_CREATE;
                sideLayout.scrollY = 0.f;
                updateUILayout();
            }
            break;
    }
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

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    reshape(WIN_W, WIN_H);

    // Initial Maze generation (clean 15x15 grid with default C1)
    maze.resize(15, 15);
    activeGen = getGen();
    activeGen->initialize(maze, currentSeed);
    activeGen->runToCompletion();
    maze.generated = true;
    syncDeliveryOrder();
    initButtonLayout();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    glutPassiveMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutTimerFunc((unsigned int)TIMER_MS, timerCallback, 0);

    glutMainLoop();
    return 0;
}
