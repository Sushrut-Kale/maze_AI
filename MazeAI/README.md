# SmartRoute AI
### AI Search Strategies for Intelligent Last-Mile Delivery
**Interactive Road Network Simulator with Multi-Customer Routing & Comparative Evaluation**

**Course Project:** College AI MSE Presentation (20 Marks)  
**Topics Covered:** Last-Mile Delivery Problem, State-Space Search Formulation, Breadth-First Search (BFS), Depth-First Search (DFS), A* Search, Admissible Heuristics, Multi-Stop Routing, Route Cost Modeling, and Comparative Evaluation.

---

> **Disclaimer:**  
> SmartRoute AI is an educational grid-based route-planning simulation. Its terrain costs and road network layouts are illustrative models designed for academic exploration. It does not use live traffic, GPS APIs, or proprietary routing algorithms from delivery services.

---

## 1. Project Motivation & Last-Mile Delivery Context

Last-mile delivery—transporting shipments from local fulfillment centers or dark stores to customer doorsteps—accounts for up to 53% of total logistics costs. Delivery couriers face dynamic urban obstacles, speed corridors (expressways/shortcuts), and delay zones (heavy congestion, construction, and roadblocks).

**SmartRoute AI** is an interactive, desktop OpenGL/FreeGLUT simulation demonstrating how classical and heuristic Artificial Intelligence search algorithms plan delivery routes across configurable road networks:
- **Unweighted Search (BFS):** Explores uniformly level-by-level using a FIFO queue. Guarantees the fewest movement steps (edges), but completely ignores terrain traversal costs.
- **Uninformed Depth Search (DFS):** Explores deep branches using a LIFO stack. Does not guarantee shortest paths or lowest costs; prone to erratic, long detours.
- **Informed Heuristic Search (A\*):** Balances cost-so-far $g(n)$ with an admissible Manhattan heuristic $h(n) = 0.5 \times (|dr| + |dc|)$. Mathematically guarantees the minimum weighted delivery cost route, prioritizing speed corridors and bypassing high-penalty delays.

---

## 2. Three-Phase Workflow Architecture

The application is structured around a focused, three-phase presentation workflow:

```
[ Phase 1: Create Road Network & Customers ] ──▶ [ Phase 2: Configure Reward/Penalty Roads ] ──▶ [ Phase 3: Solve & Compare (BFS, DFS, A*) ]
                     ▲                                                    │                                                    │
                     └────────────────── Back to Phase 1 ─────────────────┴─────────────────── Back to Phase 2 ───────────────┘
```

### Phase 1 — Create Road Network & Manage Customers
- **Network Generation:** Generate maze corridors using Randomized Depth-First Search (long corridors) or Prim's Algorithm (branching street networks).
- **Grid Dimensions:** Small (15×15), Medium (20×20), and Large (25×25).
- **Interactive Wall Editing:** Click or drag to add barrier walls or open all interior walls for an open grid.
- **Multi-Customer Destination Management (NEW):**
  - **Single Store (S):** Fixed green dispatch origin at `(0, 0)`.
  - **Multiple Customers (C1..C10):** Add, select, move, and delete up to 10 distinct customer delivery stops.
  - **Stable Identifiers:** Removing a customer preserves the exact IDs of all remaining customers.
  - **Placement Restrictions:** Customers cannot overlap the Store, another customer, or barrier walls.
  - **Two Delivery Modes:**
    - **Single Customer Target:** Find the route from Store to one selected customer destination (e.g., `Target: C2`).
    - **Multi-Stop Delivery:** User-ordered multi-stop route visiting a sequence of customers (e.g., `Store -> C2 -> C1 -> C3`).
    - **Reordering Controls:** Reorder stops in the planned sequence using **`Move Up`** and **`Move Down`**.

### Phase 2 — Configure Reward & Penalty Roads
Before solving, manually configure road traversal conditions using the clean 2-column tool palette:
- **Reward / Shortcut Road (Cost 0.5):** Emerald Diamond (`◆`). Priority lanes or expressways that reduce delivery traversal costs.
- **Penalty / Roadblock Road (Cost 4.0):** Crimson Cross (`✕`). Road construction, hazards, or severe delays.
- **Congested Road (Cost 2.0):** Amber road marker (`═`). Moderate traffic slowdown zones.
- **Normal Road / Erase (Cost 1.0):** Standard neutral urban road.
- **Safety Protections:** Pellets cannot overwrite Store, any Customer, or barrier walls.
- **Live Pellet Counters:** Real-time counters showing active reward, penalty, and congested roads.
- **Utilities:** Step-by-step **Undo Last** and **Clear Pellets** (clears pellets without altering walls, Store, or Customers).

### Phase 3 — Solve & Compare (BFS, DFS, A\*)
- **Algorithm Selection:** Choose between **BFS (Fewest Moves)**, **DFS (Depth-First)**, and **A\* (Lowest Weighted Cost)**.
- **Sequential Multi-Stop Routing:**
  - In Multi-Stop mode, the algorithm routes sequentially: `Store -> Stop 1 -> Stop 2 -> ...`
  - Concatenates valid route segments without double-counting shared segment endpoints in move totals or route metrics!
  - Reports partial completions and unreachable legs clearly.
- **Execution Controls:** **Run Search**, **Pause/Resume**, **Step** (single node expansion), and **Reset Search**.
- **Delivery Vehicle Animation:** Click **`Animate Van`** to watch the vehicle smoothly navigate the entire multi-stop journey.
- **Real-Time Metrics Display:** Displays completed status, stops reached, total moves, total weighted cost, expanded nodes, search time (ms), and shortcuts/penalties traversed.
- **Full-Screen Comparison Dashboard:** Benchmarks BFS, DFS, and A\* side-by-side on the identical configured map and customer stop sequence.

---

## 3. Road Cost & Heuristic Model

| Road Type | Traversal Cost | Visual Marker | Routing Implications |
|---|---|---|---|
| **Store (Origin)** | `0.0` (Initial) | Emerald `S` | Delivery dispatch warehouse |
| **Reward / Shortcut** | `0.5` | Emerald Diamond `◆` | A* prioritizes these fast transit lanes |
| **Normal Road** | `1.0` | Dark Grid Tile | Neutral urban road |
| **Congested Road** | `2.0` | Amber Road Marker `═` | Moderate traffic slowdown |
| **Penalty Road** | `4.0` | Crimson Cross `✕` | A* detours around to minimize cost |
| **Customer Stop** | Destination | Sapphire `C1..C10` | Delivery target destination |
| **Wall / Obstacle** | `Impassable` | Dark Slate Barrier | Impassable building / roadblock |

### Heuristic Admissibility & Consistency
A\* utilizes the scaled Manhattan distance heuristic:
$$h(n) = 0.5 \times (|r_n - r_{goal}| + |c_n - c_{goal}|)$$
- Since the minimum step cost on any road is $0.5$ (Reward Road), $h(n)$ is a guaranteed lower bound on true remaining distance.
- $h(n)$ is **strictly admissible** ($h(n) \le h^*(n)$) and **consistent**, ensuring that A\* always finds the minimum weighted delivery cost route.

---

## 4. Building and Running

### Prerequisites
- **Operating System:** Windows 10/11
- **Compiler:** MinGW GCC 6.3+ on PATH (`g++ --version`)
- **Graphics:** OpenGL and FreeGLUT runtime (`freeglut.dll` included)

### Build Commands
```cmd
# 1. Compile the main application
compile.bat

# 2. Run the application
SmartRouteAI.exe
# Or double-click build_and_run.bat to build and launch in one step
```

### Keyboard Shortcuts
| Key | Action |
|---|---|
| `G` | Generate new road network |
| `1` | Select BFS (Fewest Moves) |
| `2` | Select DFS (Depth-First) |
| `3` | Select A* (Lowest Weighted Cost) |
| `R` | Run selected algorithm |
| `A` | Run all 3 algorithms independently |
| `O` | Toggle visual path overlay comparison mode |
| `V` | Animate delivery van route |
| `C` | Open full-screen Comparison Dashboard table |
| `SPACE` | Pause / Resume search or animation |
| `N` | Step one search node or advance |
| `L` | Open Map Entity Legend |
| `H` | Open How It Works Help screen |
| `ESC` | Back to previous phase / Return to workflow |

---

## 5. Automated Test Suite (66 Tests)

A complete standalone headless test suite tests core algorithms, metrics, multi-customer features, and regression suites without requiring a GUI window.

Run the test suite:
```cmd
run_tests.bat
```

### Test Coverage Summary
- **Part 1: Core Search & Optimality (Tests 01–15):** BFS, DFS, A\* reachability, wall avoidance, start==goal edge cases, Dijkstra equivalence, and move counts.
- **Part 2: Three-Phase Workflow & Pellets (Tests 16–26):** Reward/penalty placement, Store immutability, wall protections, live counters, undo, clear, and navigation preservation.
- **Part 3: Algorithm Differentiation (Tests 27–30):** A\* cost optimization, BFS unweighted hop selection, and state reset verification.
- **Part 4: Multi-Customer Management (Tests 31–39):** Customer additions, unique ID generation, Store overlap rejection, customer overlap rejection, wall rejection, safe removal, stable customer IDs after deletion, 10-customer limit, movement validation, and stop reordering.
- **Part 5: Multi-Stop Routing & Metrics (Tests 40–47):** Single customer and multi-stop sequential routing, shared endpoint deduplication, move summing across legs, total cost accumulation, leg-level A\* optimality vs Dijkstra, BFS fewest-move behavior, and unreachable customer reporting.
- **Part 6: Integration, Restrictions & Window Resize (Tests 48–52):** Pellet restrictions on customers, clear preservation, comparison dashboard consistency, and window-resize coordinate mapping.
- **Part 7: Routing Correctness & Algorithm Comparison Regression (Tests 53–66):**
  - **Test 53:** BFS genuine FIFO behavior
  - **Test 54:** DFS genuine depth-first stack behavior with deterministic neighbor ordering
  - **Test 55:** A\* priority queue ordering and node reconsideration when cheaper path found
  - **Test 56:** A\* chooses cheaper weighted route over fewer moves
  - **Test 57:** BFS chooses fewer moves despite higher weighted cost
  - **Test 58:** DFS does not falsely guarantee shortest path
  - **Test 59:** Distant corner customer reachability
  - **Test 60:** Grid boundary customer reachability
  - **Test 61:** Unreachable customer reporting `NO PATH` without false completion
  - **Test 62:** Ordered multi-customer delivery preserving sequence
  - **Test 63:** Route passing near customer does not count as reached until entering cell
  - **Test 64:** Start == Goal zero-step handling
  - **Test 65:** Physical arrival simulation step progression
  - **Test 66:** Immutable maze snapshot fair comparison across all 3 algorithms

**Results:** `TOTAL TESTS: 66 | PASSED: 66 | FAILED: 0 (100% PASS)`

---

## 6. Standalone Packaging

To generate the authoritative, portable release package:
```cmd
package.bat
```
Produces `SmartRouteAI.zip` containing all C++ sources, test harnesses, build scripts, headers, libraries, and the standalone executable ready for distribution and presentation.

---

## 7. Universal Troubleshooting Guide

For comprehensive troubleshooting of common Windows, MinGW, PowerShell, and Smart App Control issues, see:
- [`SMARTROUTE_AI_UNIVERSAL_TROUBLESHOOTING.txt`](SMARTROUTE_AI_UNIVERSAL_TROUBLESHOOTING.txt)

Key topics covered:
- **Error 1:** Why batch commands (`@echo off`, `setlocal`, `call`) must NOT be pasted into PowerShell.
- **PowerShell vs Command Prompt:** Proper execution (`.\compile.bat` vs `compile.bat`).
- **Environment Diagnostics:** Copyable commands to verify g++, PATH, and project files.
- **MinGW & Compiler Setup:** Resolving missing g++ or FreeGLUT header/library issues.
- **Security Guidance:** Safe, responsible handling of Windows 11 Smart App Control and SmartScreen.
- **Student Checklist:** Clean 7-step first-run verification for classmates and presentation partners.

