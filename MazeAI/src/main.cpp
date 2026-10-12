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
static MultiStopResult  lastMultiStopRes;
static FrameRecorder    recorder("frames");

// Delivery Vehicle Path Animation
static std::vector<Position> riderPath;
static int    riderStep     = 0;
static float  riderProgress = 0.f;
static bool   riderActive   = false;
static float  riderSpeed    = 0.08f;

// Algorithm Comparison Results
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
static Button btnRun, btnResetSearch, btnPause, btnStep;
static Button btnAnimateRoute, btnStopAnim;
static Button btnCompareAll;
static Button btnPhase3BackToConfig, btnPhase3NewMaze;

// Modal / Screen Return
static Button btnHelpBack, btnLegendBack, btnBackToWorkflow;

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
    btnRun         = Button(x,     y3, halfW, 26.f, "Run Search");   btnRun.setGreen();
    btnResetSearch = Button(col2X, y3, halfW, 26.f, "Reset Search"); btnResetSearch.setNeutral();
    y3 += 30.f;

    btnStep  = Button(x,     y3, halfW, 24.f, "Step");
    btnPause = Button(col2X, y3, halfW, 24.f, "Pause / Resume");
    y3 += 28.f;

    btnAnimateRoute = Button(x,     y3, halfW, 24.f, "Animate Van"); btnAnimateRoute.setAmber();
    btnStopAnim     = Button(col2X, y3, halfW, 24.f, "Stop Anim");
    y3 += 32.f;

    sideLayout.p3ResultsSecY = y3; // "RESULTS"
    y3 += 12.f;
    sideLayout.p3ResultsCardY = y3; // Results card (h=122)
    y3 += 130.f;

    sideLayout.p3CompareY = y3; // Compare All
    btnCompareAll = Button(x, y3, fullW, 32.f, "COMPARE ALL (BFS, DFS, A*)"); btnCompareAll.setPrimary();
    y3 += 38.f;

    sideLayout.p3NavY = y3; // Navigation
    btnPhase3BackToConfig = Button(x,     y3, halfW, 26.f, "<- Road Config");
    btnPhase3NewMaze      = Button(col2X, y3, halfW, 26.f, "New Network");
    y3 += 34.f;

    allButtons.push_back(&btnSelBFS); allButtons.push_back(&btnSelDFS); allButtons.push_back(&btnSelAStar);
    allButtons.push_back(&btnRun); allButtons.push_back(&btnResetSearch);
    allButtons.push_back(&btnStep); allButtons.push_back(&btnPause);
    allButtons.push_back(&btnAnimateRoute); allButtons.push_back(&btnStopAnim);
    allButtons.push_back(&btnCompareAll);
    allButtons.push_back(&btnPhase3BackToConfig); allButtons.push_back(&btnPhase3NewMaze);

    // Modal / Screen Return Buttons
    btnHelpBack       = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 46.f, 160.f, 30.f, "RETURN");
    btnLegendBack     = Button((float)WIN_W * 0.5f - 80.f, (float)WIN_H - 46.f, 160.f, 30.f, "RETURN");
    btnBackToWorkflow = Button((float)WIN_W * 0.5f - 110.f, (float)WIN_H - 46.f, 220.f, 32.f, "RETURN TO SOLVING");
    btnBackToWorkflow.setGreen();

    allButtons.push_back(&btnHelpBack);
    allButtons.push_back(&btnLegendBack);
    allButtons.push_back(&btnBackToWorkflow);

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
// ── SOLVER EXECUTION ────────────────────────────────────────
// ════════════════════════════════════════════════════════════
static void startVanAnimation() {
    if (lastMultiStopRes.fullPath.empty()) {
        showError("No delivery route available to animate.");
        return;
    }
    riderPath     = lastMultiStopRes.fullPath;
    riderStep     = 0;
    riderProgress = 0.f;
    riderActive   = true;
    statusMsg     = "Delivery van en route across customer stops...";
}

static void stopVanAnimation() {
    riderActive = false;
    statusMsg   = "Delivery van animation stopped.";
}

static void updateRider() {
    if (!riderActive || riderPath.empty()) return;
    riderProgress += riderSpeed;
    if (riderProgress >= 1.f) {
        riderProgress = 0.f;
        riderStep++;
        if (riderStep >= (int)riderPath.size() - 1) {
            riderActive = false;
            statusMsg   = "Delivery van successfully completed all customer deliveries!";
        }
    }
}

static void doStartSolving() {
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    if (maze.customers.empty()) {
        showError("Add at least one customer destination.");
        return;
    }

    syncDeliveryOrder();
    std::vector<Position> stops;
    std::vector<std::string> labels;

    if (deliveryMode == DeliveryMode::SINGLE_TARGET) {
        const Customer* cust = maze.getCustomer(maze.selectedCustomerId);
        if (!cust) {
            cust = &maze.customers.front();
            maze.selectedCustomerId = cust->id;
        }
        stops.push_back(cust->pos);
        labels.push_back(cust->label());
    } else {
        for (int cid : deliveryOrder) {
            const Customer* cust = maze.getCustomer(cid);
            if (cust && cust->active) {
                stops.push_back(cust->pos);
                labels.push_back(cust->label());
            }
        }
    }

    if (stops.empty()) {
        showError("No active customer stops selected.");
        return;
    }

    maze.resetSearch();
    riderActive  = false;
    currentPhase = AppPhase::PHASE_3_SOLVE;

    std::string algName;
    switch (selSearch) {
        case SearchType::BFS:   algName = "BFS"; break;
        case SearchType::DFS_S: algName = "DFS"; break;
        case SearchType::ASTAR: algName = "A*"; break;
    }

    lastMultiStopRes = runMultiStopSearch(maze, stops, labels, algName);
    maze.markPath(lastMultiStopRes.fullPath);
    appState = AppState::SOLVED;

    if (lastMultiStopRes.fullyCompleted) {
        std::ostringstream ss;
        ss << "Route found! Stops: " << lastMultiStopRes.stopsCompleted << "/" << lastMultiStopRes.totalStops
           << " | Moves: " << lastMultiStopRes.totalMoves
           << " | Cost: " << std::fixed << std::setprecision(1) << lastMultiStopRes.totalCost;
        statusMsg = ss.str();
    } else {
        statusMsg = "Delivery leg failed! Unreachable: " + lastMultiStopRes.failedDestination +
                    " (Reached " + std::to_string(lastMultiStopRes.stopsCompleted) + " stops).";
    }
}

static void doRunComparison() {
    if (!maze.generated) {
        showError("Generate a road network first.");
        return;
    }
    if (maze.customers.empty()) {
        showError("Add at least one customer destination.");
        return;
    }

    syncDeliveryOrder();
    std::vector<Position> stops;
    std::vector<std::string> labels;

    if (deliveryMode == DeliveryMode::SINGLE_TARGET) {
        const Customer* cust = maze.getCustomer(maze.selectedCustomerId);
        if (!cust && !maze.customers.empty()) cust = &maze.customers.front();
        if (cust) {
            stops.push_back(cust->pos);
            labels.push_back(cust->label());
        }
    } else {
        for (int cid : deliveryOrder) {
            const Customer* cust = maze.getCustomer(cid);
            if (cust && cust->active) {
                stops.push_back(cust->pos);
                labels.push_back(cust->label());
            }
        }
    }

    compResults.clear();
    const char* algs[3] = {"BFS", "DFS", "A*"};
    for (int i = 0; i < 3; i++) {
        CompResult cr;
        cr.name     = algs[i];
        cr.multiRes = runMultiStopSearch(maze, stops, labels, algs[i]);
        compResults.push_back(cr);
    }

    appState  = AppState::COMPARING;
    statusMsg = "Comparison complete: Same network & stops evaluated across BFS, DFS, and A*.";
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
}

static void handleSidebarClick(float mx, float my) {
    // ── Phase 1 Buttons ──────────────────────────────────────
    if (currentPhase == AppPhase::PHASE_1_CREATE) {
        if (btnSzSm.isClicked(mx, my)) { selSize = MazeSizeE::SMALL;  maze.resize(MSIZES[0][0], MSIZES[0][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); }
        else if (btnSzMd.isClicked(mx, my)) { selSize = MazeSizeE::MEDIUM; maze.resize(MSIZES[1][0], MSIZES[1][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); }
        else if (btnSzLg.isClicked(mx, my)) { selSize = MazeSizeE::LARGE;  maze.resize(MSIZES[2][0], MSIZES[2][1]); mazeLayout.update(WIN_W, WIN_H, maze.rows, maze.cols); syncDeliveryOrder(); initButtonLayout(); }
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
            statusMsg = "Road network generated. Add customers or continue to road config.";
        }
        else if (btnNewSeed.isClicked(mx, my)) {
            currentSeed++;
            globalRng.seed(currentSeed);
            maze.hardReset();
            syncDeliveryOrder();
            initButtonLayout();
            statusMsg = "Reset network to blank grid. New Seed: " + std::to_string(currentSeed);
        }
        else if (btnScenA.isClicked(mx, my)) loadScenarioA();
        else if (btnScenB.isClicked(mx, my)) loadScenarioB();
        else if (btnScenC.isClicked(mx, my)) loadScenarioC();
        else if (btnToggleWallEdit.isClicked(mx, my)) {
            wallEditActive = !wallEditActive;
            custPlacementTool = CustPlacementTool::NONE;
            statusMsg = wallEditActive ? "Wall Edit: Click/drag road cells to toggle barrier walls." : "Wall Edit deactivated.";
        }
        else if (btnClearWalls.isClicked(mx, my)) {
            buildOpenGrid(maze.rows, maze.cols);
            statusMsg = "All interior walls removed (Open Grid layout).";
        }
        else if (btnModeSingle.isClicked(mx, my)) {
            deliveryMode = DeliveryMode::SINGLE_TARGET;
            statusMsg = "Delivery Mode: Single Customer target.";
            updateUILayout();
        }
        else if (btnModeMulti.isClicked(mx, my)) {
            deliveryMode = DeliveryMode::MULTI_STOP;
            statusMsg = "Delivery Mode: Multi-Stop route following configured stop order.";
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
                statusMsg = "Removed customer C" + std::to_string(remId) + ". Remaining IDs preserved.";
            }
        }
        else if (btnCustUp.isClicked(mx, my)) {
            auto it = std::find(deliveryOrder.begin(), deliveryOrder.end(), maze.selectedCustomerId);
            if (it != deliveryOrder.end() && it != deliveryOrder.begin()) {
                std::iter_swap(it, it - 1);
                statusMsg = "Moved C" + std::to_string(maze.selectedCustomerId) + " earlier in stop order.";
                updateUILayout();
            }
        }
        else if (btnCustDown.isClicked(mx, my)) {
            auto it = std::find(deliveryOrder.begin(), deliveryOrder.end(), maze.selectedCustomerId);
            if (it != deliveryOrder.end() && it + 1 != deliveryOrder.end()) {
                std::iter_swap(it, it + 1);
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
        if (btnToolReward.isClicked(mx, my))    { pelletTool = PelletTool::REWARD_05; }
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
                    statusMsg = "Undid last terrain edit.";
                }
            } else {
                showError("Undo history is empty.");
            }
        }
        else if (btnClearPellets.isClicked(mx, my)) {
            maze.clearAllSpecialPellets();
            pelletUndoStack.clear();
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
        if (btnSelBFS.isClicked(mx, my))        { selSearch = SearchType::BFS; }
        else if (btnSelDFS.isClicked(mx, my))   { selSearch = SearchType::DFS_S; }
        else if (btnSelAStar.isClicked(mx, my)) { selSearch = SearchType::ASTAR; }
        else if (btnRun.isClicked(mx, my))      { doStartSolving(); }
        else if (btnResetSearch.isClicked(mx, my)) {
            maze.resetSearch();
            riderActive = false;
            lastMultiStopRes = MultiStopResult{};
            appState    = AppState::MAZE_WORKFLOW;
            statusMsg   = "Search reset. Road network, pellets, and customers preserved.";
        }
        else if (btnStep.isClicked(mx, my))         { doStep(); }
        else if (btnPause.isClicked(mx, my))        { paused = !paused; }
        else if (btnAnimateRoute.isClicked(mx, my)) { startVanAnimation(); }
        else if (btnStopAnim.isClicked(mx, my))     { stopVanAnimation(); }
        else if (btnCompareAll.isClicked(mx, my))   { doRunComparison(); }
        else if (btnPhase3BackToConfig.isClicked(mx, my)) {
            maze.resetSearch();
            riderActive  = false;
            currentPhase = AppPhase::PHASE_2_CONFIG;
            sideLayout.scrollY = 0.f;
            updateUILayout();
            statusMsg    = "Returned to Phase 2. Pellets and customer stops preserved.";
        }
        else if (btnPhase3NewMaze.isClicked(mx, my)) {
            maze.resetSearch();
            riderActive  = false;
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
    if (lastMultiStopRes.fullPath.size() < 2) return;

    glColor3fv(Col::PATH);
    glLineWidth(3.5f);
    glBegin(GL_LINE_STRIP);
    for (const Position& p : lastMultiStopRes.fullPath) {
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
            std::string seq = lastMultiStopRes.stopSequenceStr.empty() ? "Store -> Customer Stops" : lastMultiStopRes.stopSequenceStr;
            drawTextTruncated(x + 10.f, sideLayout.p3TargetCardY + 30.f, seq, fullW - 20.f, GLUT_BITMAP_HELVETICA_10, Col::ACCENT);
        }

        // 2. Algorithm Selection
        drawSectionHeader(x, sideLayout.p3AlgSecY, "ALGORITHM");
        btnSelBFS.selected   = (selSearch == SearchType::BFS);   btnSelBFS.draw();
        btnSelDFS.selected   = (selSearch == SearchType::DFS_S); btnSelDFS.draw();
        btnSelAStar.selected = (selSearch == SearchType::ASTAR); btnSelAStar.draw();

        // 3. Execution Controls
        drawSectionHeader(x, sideLayout.p3ExecSecY, "EXECUTION CONTROLS");
        btnRun.draw(); btnResetSearch.draw();
        btnPause.selected = paused;
        btnStep.draw(); btnPause.draw();
        btnAnimateRoute.disabled = lastMultiStopRes.fullPath.empty();
        btnAnimateRoute.draw(); btnStopAnim.draw();

        // 4. Results Card
        drawSectionHeader(x, sideLayout.p3ResultsSecY, "RESULTS");
        drawPanel(x, sideLayout.p3ResultsCardY, fullW, 122.f, Col::CARD_BG, Col::BORDER, 1.0f);

        if (!lastMultiStopRes.fullPath.empty()) {
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 17.f, fullW - 20.f, "Delivery Completed:",
                          lastMultiStopRes.fullyCompleted ? "YES" : "PARTIAL",
                          Col::TEXT_MUTED, lastMultiStopRes.fullyCompleted ? Col::GREEN_REWARD : Col::RED_PENALTY, GLUT_BITMAP_HELVETICA_10);
            std::string stpStr = std::to_string(lastMultiStopRes.stopsCompleted) + " / " + std::to_string(lastMultiStopRes.totalStops);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 33.f, fullW - 20.f, "Stops Reached:",
                          stpStr.c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 49.f, fullW - 20.f, "Total Route Moves:",
                          std::to_string(lastMultiStopRes.totalMoves).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);
            std::string costStr = std::to_string((int)lastMultiStopRes.totalCost) + (lastMultiStopRes.totalCost == (int)lastMultiStopRes.totalCost ? ".0" : ".5");
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 65.f, fullW - 20.f, "Total Weighted Cost:",
                          costStr.c_str(), Col::TEXT_MUTED, Col::PRIMARY_BLUE, GLUT_BITMAP_HELVETICA_10);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 81.f, fullW - 20.f, "Total Expanded Nodes:",
                          std::to_string(lastMultiStopRes.totalNodesExplored).c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

            std::ostringstream ts; ts << std::fixed << std::setprecision(2) << lastMultiStopRes.totalTimeMs << " ms";
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 97.f, fullW - 20.f, "Search Time:",
                          ts.str().c_str(), Col::TEXT_MUTED, Col::TEXT_WHITE, GLUT_BITMAP_HELVETICA_10);

            std::string rwPn = std::to_string(lastMultiStopRes.rewardsTraversed) + " / " + std::to_string(lastMultiStopRes.penaltiesTraversed);
            drawMetricRow(x + 10.f, sideLayout.p3ResultsCardY + 113.f, fullW - 20.f, "Shortcuts / Penalties:",
                          rwPn.c_str(), Col::TEXT_MUTED, Col::GREEN_REWARD, GLUT_BITMAP_HELVETICA_10);
        } else {
            glColor3fv(Col::TEXT_MUTED);
            drawBitmapText(x + 12.f, sideLayout.p3ResultsCardY + 38.f, "No search executed yet.", GLUT_BITMAP_HELVETICA_12);
            drawBitmapText(x + 12.f, sideLayout.p3ResultsCardY + 60.f, "Select algorithm & click Run Search.", GLUT_BITMAP_HELVETICA_10);
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

    float cx = (float)WIN_W * 0.5f, y = 36.f;
    glColor3fv(Col::TITLE_CYAN);
    int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)"ALGORITHM COMPARISON DASHBOARD");
    drawBitmapText(cx - (float)tw * 0.5f, y, "ALGORITHM COMPARISON DASHBOARD", GLUT_BITMAP_HELVETICA_18);
    y += 24.f;

    glColor3fv(Col::TEXT_MUTED);
    std::string sub = (deliveryMode == DeliveryMode::SINGLE_TARGET)
        ? "Comparing BFS, DFS, and A* for Single Customer C" + std::to_string(maze.selectedCustomerId)
        : "Comparing BFS, DFS, and A* on Multi-Stop Route: " + (compResults.empty() ? "" : compResults[0].multiRes.stopSequenceStr);
    int sw = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)sub.c_str());
    drawBitmapText(cx - (float)sw * 0.5f, y, sub.c_str(), GLUT_BITMAP_HELVETICA_12);
    y += 26.f;

    float tX = 60.f, tW = (float)WIN_W - 120.f;
    drawSep(tX, y, tW); y += 18.f;

    // Table Column Offsets
    float colX[] = {
        tX,
        tX + 110.f,
        tX + 220.f,
        tX + 320.f,
        tX + 440.f,
        tX + 570.f,
        tX + 710.f,
        tX + 830.f,
        tX + 930.f
    };

    const char* hdrs[] = {
        "Algorithm", "Completed", "Stops", "Moves",
        "Weighted Cost", "Expanded Nodes", "Search Time (ms)", "Shortcuts", "Penalties"
    };

    glColor3fv(Col::ACCENT);
    for (int i = 0; i < 9; i++) {
        drawBitmapText(colX[i], y, hdrs[i], GLUT_BITMAP_HELVETICA_12);
    }
    y += 18.f;
    drawSep(tX, y, tW); y += 16.f;

    // Optimal values across completed runs
    int   bestMoves = 999999;
    float bestCost  = 999999.f;
    int   bestNodes = 999999;
    for (const auto& cr : compResults) {
        if (cr.multiRes.fullyCompleted) {
            bestMoves = std::min(bestMoves, cr.multiRes.totalMoves);
            bestCost  = std::min(bestCost,  cr.multiRes.totalCost);
            bestNodes = std::min(bestNodes, cr.multiRes.totalNodesExplored);
        }
    }

    const float* algCols[] = {Col::TITLE_CYAN, Col::RED_PENALTY, Col::GREEN_REWARD};
    int ci = 0;

    for (const auto& cr : compResults) {
        const auto& mr = cr.multiRes;
        glColor3fv(algCols[ci < 3 ? ci : 2]); ci++;
        drawBitmapText(colX[0], y, cr.name.c_str(), GLUT_BITMAP_HELVETICA_12);

        // Completed
        if (mr.fullyCompleted) {
            glColor3fv(Col::GREEN_REWARD);
            drawBitmapText(colX[1], y, "YES", GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::RED_PENALTY);
            drawBitmapText(colX[1], y, "PARTIAL", GLUT_BITMAP_HELVETICA_12);
        }

        // Stops
        std::string stpStr = std::to_string(mr.stopsCompleted) + "/" + std::to_string(mr.totalStops);
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(colX[2], y, stpStr.c_str(), GLUT_BITMAP_HELVETICA_12);

        // Moves
        if (mr.totalMoves == bestMoves && mr.fullyCompleted) {
            glColor3fv(Col::GREEN_REWARD);
            drawBitmapText(colX[3], y, (std::to_string(mr.totalMoves) + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[3], y, std::to_string(mr.totalMoves).c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Cost
        std::string costStr = std::to_string((int)mr.totalCost) + (mr.totalCost == (int)mr.totalCost ? ".0" : ".5");
        if (mr.totalCost == bestCost && mr.fullyCompleted) {
            glColor3fv(Col::GREEN_REWARD);
            drawBitmapText(colX[4], y, (costStr + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[4], y, costStr.c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Nodes
        if (mr.totalNodesExplored == bestNodes && mr.fullyCompleted) {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[5], y, (std::to_string(mr.totalNodesExplored) + " *").c_str(), GLUT_BITMAP_HELVETICA_12);
        } else {
            glColor3fv(Col::TEXT_WHITE);
            drawBitmapText(colX[5], y, std::to_string(mr.totalNodesExplored).c_str(), GLUT_BITMAP_HELVETICA_12);
        }

        // Time
        std::ostringstream ts; ts << std::fixed << std::setprecision(2) << mr.totalTimeMs;
        glColor3fv(Col::TEXT_WHITE);
        drawBitmapText(colX[6], y, ts.str().c_str(), GLUT_BITMAP_HELVETICA_12);

        // Shortcuts & Penalties
        glColor3fv(Col::GREEN_REWARD);
        drawBitmapText(colX[7], y, std::to_string(mr.rewardsTraversed).c_str(), GLUT_BITMAP_HELVETICA_12);

        glColor3fv(Col::RED_PENALTY);
        drawBitmapText(colX[8], y, std::to_string(mr.penaltiesTraversed).c_str(), GLUT_BITMAP_HELVETICA_12);

        y += 32.f;
    }

    drawSep(tX, y, tW); y += 22.f;

    // Takeaways Card
    drawSectionHeader(tX, y, "KEY PRESENTATION TAKEAWAYS"); y += 18.f;
    const char* takeaways[] = {
        "- BFS minimizes movement steps (edges) for each delivery leg, but ignores terrain traversal costs.",
        "- DFS explores deeply without distance or cost optimality guarantees; prone to long, erratic routes.",
        "- A* achieves minimum weighted delivery cost by balancing cost-so-far g(n) with admissible heuristic h(n).",
        "- When reward pellets (cost 0.5) exist, A* routes through them to minimize total trip delivery cost.",
        "- When penalty pellets (cost 4.0) exist, A* routes around them, whereas BFS may traverse expensive roads.",
        "- Multi-stop mode follows your configured customer order; it does not solve the global TSP/VRP.",
        "- Note: Asterisk (*) indicates optimal performance for that specific metric across the three algorithms.",
        nullptr
    };

    for (int i = 0; takeaways[i]; i++) {
        if (i == 2) glColor3fv(Col::GREEN_REWARD);
        else if (i == 5) glColor3fv(Col::ACCENT);
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
            break;
        case '1': selSearch = SearchType::BFS;   statusMsg = "Selected: BFS (Fewest Moves)"; break;
        case '2': selSearch = SearchType::DFS_S; statusMsg = "Selected: DFS (Depth-First)"; break;
        case '3': selSearch = SearchType::ASTAR; statusMsg = "Selected: A* (Lowest Cost)"; break;
        case ' ':
            paused = !paused;
            statusMsg = paused ? "Paused." : "Resumed.";
            break;
        case 'n': case 'N': doStep(); break;
        case 'a': case 'A': startVanAnimation(); break;
        case 'c': case 'C': doRunComparison(); break;
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
