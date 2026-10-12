# SmartRoute AI
### AI Search Strategies for Intelligent Last-Mile Delivery

**Course Project:** College AI MSE Group Presentation (20 Marks)  
**Topics Covered:** Last-Mile Delivery Problem, State-Space Search Formulation, Breadth-First Search (BFS), Depth-First Search (DFS), A* Search, Admissible Heuristics, Route Cost Modeling, and Comparative Evaluation.

---

> **Disclaimer:**  
> SmartRoute AI is an educational grid-based route-planning simulation. Its terrain costs and maze layouts are illustrative. It does not use live traffic, GPS, or proprietary routing algorithms from UPS, Blinkit, Zepto, Swiggy Instamart, or other delivery companies.

---

## 1. Project Purpose & Last-Mile Delivery Motivation

Last-mile delivery—the final leg of a supply chain transporting goods from a local fulfillment center or dark store to the customer's doorstep—accounts for up to 53% of total delivery costs. Delivery couriers face dynamic urban obstacles, speed corridors (expressways/shortcuts), and delay zones (construction, heavy traffic, and physical blockages).

**SmartRoute AI** is an interactive, presentation-ready simulation demonstrating how classical and heuristic Artificial Intelligence search algorithms plan last-mile delivery routes across configurable road networks. The system illustrates:
- Why **unweighted search (BFS)** finds paths with the fewest turns/moves but often incurs high delivery costs.
- Why **uninformed depth search (DFS)** can take excessively long, erratic detours.
- How **informed heuristic search (A\*)** leverages an admissible heuristic function to find the route with the mathematically minimum weighted delivery cost.

---

## 2. The Three-Phase Workflow

The application is structured around an intuitive, linear three-phase workflow designed specifically for classroom presentation:

```
[ Phase 1: Create Road Network ] ──▶ [ Phase 2: Configure Reward/Penalty Pellets ] ──▶ [ Phase 3: Solve & Compare (BFS, DFS, A*) ]
           ▲                                             │                                                      │
           └────────────────── Back to Phase 1 ──────────┴─────────────────── Back to Phase 2 ─────────────────┘
```

### Phase 1 — Create Road Network (Maze / Grid)
- **Generate Network:** Generate maze corridors using Randomized Depth-First Search (long corridors) or Prim's Algorithm (branching street networks).
- **Grid Sizes:** Small (15×15), Medium (20×20), and Large (25×25).
- **Edit Walls:** Interactively add or remove barrier walls, or open all interior walls to test open grid networks.
- **Predefined Scenarios:** One-click loading of demonstration scenarios (Simple Delivery, Reward Shortcuts, and Penalty Road).
- **Fixed Dispatch & Delivery Points:** Clearly designated Store/Start cell `(S)` and Customer/Goal cell `(G)`.
- **Transition:** Click **`CONTINUE TO ROAD CONFIG ->`** to lock in the road layout and enter Phase 2.

### Phase 2 — Configure Reward & Penalty Pellets
Before running any search algorithms, the user manually configures road traversal costs by selecting a tool from the clean 2-column grid and clicking road cells:
- **Reward / Shortcut Road (Cost 0.5):** Represented by an emerald Diamond (`◆`). Designates priority lanes or fast shortcuts that reduce delivery time.
- **Penalty / Delay Road (Cost 4.0):** Represented by a crimson Cross (`✕`). Designates roadblocks, construction zones, or high-cost routes.
- **Congested Road (Cost 2.0):** Represented by an amber road marker (`═`). Designates heavy traffic delay zones.
- **Normal Road / Erase (Cost 1.0):** Restores cells to standard neutral roads.
- **Placement Restrictions:** Pellets cannot overwrite the Store (Start), Customer (Goal), or impassable walls.
- **Live Summary Counters:** Real-time counters displaying active `Reward Pellets (0.5)`, `Penalty Pellets (4.0)`, and `Congested Roads (2.0)`.
- **Utilities:** Step-by-step **Undo Last** and **Clear Pellets** (clears special pellets and restores congested cells without altering walls or endpoints).
- **Transition:** Click **`CONTINUE TO SOLVING ->`** to advance to Phase 3.

### Phase 3 — Solve and Compare (BFS, DFS, A\*)
- **Algorithm Selection:** Choose between **BFS (Fewest Moves)**, **DFS (Depth-First)**, and **A\* (Lowest Weighted Cost)**.
- **Execution Controls:** **Run Search**, **Pause/Resume**, **Step** (single node expansion), and **Reset Search** (resets search visualization while strictly preserving configured pellets).
- **Delivery Vehicle Animation:** Click **`Animate Van`** to visualize the delivery vehicle smoothly traversing the computed route from Store to Customer.
- **Real-Time Metrics Display:** Displays nodes evaluated, route moves, weighted traversal cost, execution time (ms), peak frontier, and reward/penalty counts.
- **Compare All Dashboard:** Benchmarks BFS, DFS, and A\* simultaneously on identical copies of the configured map, generating a side-by-side comparative evaluation table.

---

## 3. AI Problem Formulation (State-Space Model)

In AI search theory, a problem is formally defined by five components:

1. **State Space ($S$):** The set of all valid grid positions $(r, c)$ where $0 \le r < \text{Rows}$ and $0 \le c < \text{Cols}$.
2. **Initial State ($s_0$):** The coordinates of the dispatch warehouse / dark store, fixed at $(0, 0)$ or custom coordinates.
3. **Goal State ($G$):** The coordinates of the destination customer delivery location, fixed at $(R-1, C-1)$.
4. **Action Space ($A(s)$):** Four orthogonal directional moves: $\text{ACTIONS}(s) \in \{\text{North}, \text{East}, \text{South}, \text{West}\}$. Diagonal movement is disallowed.
5. **Transition Model ($\text{RESULT}(s, a)$):** Moves the courier from $(r, c)$ to the adjacent cell in direction $a$, provided no barrier wall exists.
6. **Path Cost Function ($c(s, a, s')$):** The cost of entering cell $s'$, defined by the road terrain model:
   - **Normal Road:** $1.0$
   - **Reward / Shortcut Road:** $0.5$
   - **Penalty / Delay Road:** $4.0$
   - **Blocked Road / Wall:** $\infty$ (impassable)
   - *Note:* The starting cell has an entry cost of $0.0$.

---

## 4. Search Algorithm Behavior & Theoretical Analysis

### Breadth-First Search (BFS)
- **Data Structure:** First-In, First-Out (FIFO) queue.
- **Search Strategy:** Explores nodes strictly in order of their graph depth (level-by-level).
- **Optimality:** Guarantees the path with the **fewest moves / road segments** on unweighted graphs.
- **Educational Takeaway:** **BFS ignores terrain weights.** It treats a 4.0 penalty road identically to a 0.5 shortcut road. Consequently, BFS frequently chooses paths with higher total weighted delivery cost.
- *Explanation in UI:* *"BFS minimizes the number of moves, not the weighted travel cost."*

### Depth-First Search (DFS)
- **Data Structure:** Last-In, First-Out (LIFO) stack (iterative exploration).
- **Search Strategy:** Explores as deeply as possible along each branch before backtracking.
- **Optimality:** Neither optimal in route moves nor in weighted travel cost.
- **Educational Takeaway:** Demonstrates the peril of unguided depth exploration: it frequently produces long, winding, inefficient delivery routes.
- *Explanation in UI:* *"DFS explores deeply before backtracking. It does not guarantee the shortest or lowest-cost route."*

### A* Search
- **Data Structure:** Min-Priority Queue ordered by evaluation function $f(n)$.
- **Evaluation Function:**
  $$f(n) = g(n) + h(n)$$
  where:
  - $g(n)$ is the exact accumulated weighted traversal cost from the Start cell to node $n$.
  - $h(n)$ is the estimated remaining travel cost from node $n$ to the Customer Goal cell.
- **Admissible & Consistent Heuristic:**
  Since the minimum possible single-step traversal cost anywhere on the grid is $c_{\min} = 0.5$ (a reward shortcut road), Manhattan distance multiplied by $0.5$ provides an admissible lower bound:
  $$h(n) = 0.5 \times (|r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|)$$
  Because $h(n) \le h^*(n)$ for all nodes, A* is mathematically guaranteed to return the **optimal minimum-cost delivery route**.
- *Explanation in UI:* *"A* combines the actual cost so far with an estimate of the remaining cost to find a minimum-cost route under the configured cost model."*

---

## 5. Comparison Dashboard & Metric Definitions

Clicking **`COMPARE ALL`** runs BFS, DFS, and A* sequentially on identical, independent copies of the configured maze.

| Metric | Definition & Meaning |
|---|---|
| **Route Found** | Whether a continuous traversable path from Store to Customer was located (`YES` or `NO`). |
| **Route Moves** | Total transitions / edges between cells, strictly calculated as $\max(0, \text{Route Cells} - 1)$. A 25-cell path equals exactly 24 moves. |
| **Route Cells** | Total number of grid cells constituting the reconstructed route (including start and goal). |
| **Weighted Cost** | Exact sum of traversal costs of all entered cells after leaving the dispatch warehouse. Start cell entry cost is excluded ($0.0$). |
| **Expanded Nodes** | Number of grid locations popped from the frontier and evaluated. Reflects search effort and time complexity. |
| **Search Time (ms)** | Pure algorithmic CPU execution time in milliseconds, excluding graphics rendering and frame animation. |
| **Shortcuts on Route** | Number of Reward pellet cells ($0.5$ cost) utilized on the final route. |
| **Penalties on Route** | Number of Penalty pellet cells ($4.0$ cost) traversed on the final route. |

---

## 6. Predefined Scenarios for Presentation

Three pre-configured delivery scenarios are built into the interface for live demonstration:

1. **Scenario A — Simple Delivery (Baseline):**
   - Moderate open network with barrier obstacles.
   - All roads have unit cost $1.0$.
   - **Result:** BFS, DFS, and A* all locate valid 24-move routes with $24.0$ weighted cost.

2. **Scenario B — Reward Shortcut (Incentivized Route):**
   - Upper corridor contains 0.5 reward pellets; lower corridor contains standard road.
   - **Result:** BFS selects the direct path ($30$ moves, Cost $30.0$, $0$ rewards). A* directs the vehicle through the shortcut corridor ($30$ moves, Cost $28.5$, $3$ rewards), saving travel cost.

3. **Scenario C — Penalty Corridor (Avoidance Strategy):**
   - The direct geometric path is obstructed by high-cost penalty road ($4.0$ cost), while an alternative detour features reward shortcuts.
   - **Result:** BFS blindly takes the direct route across the penalty ($30$ moves, Cost $33.0$, $1$ penalty). A* identifies the detour ($30$ moves, Cost $28.0$, $0$ penalties, $4$ shortcuts), successfully avoiding costly roads.

---

## 7. Controls Reference

### Mouse Interactions
- **Phase 1:** Click maze cells when `Edit Walls` is active to toggle obstacles.
- **Phase 2:** Click or drag across road cells to assign the selected road tool (Reward $0.5$, Congested $2.0$, Penalty $4.0$, or Normal $1.0$).
- **Sidebar Buttons:** Routed exclusively to active phase tools. Buttons never trigger unwanted edits to maze cells.

### Keyboard Shortcuts
| Key | Context | Action |
|---|---|---|
| **G** | Phase 1 | Generate road network |
| **1** | Phase 3 | Select BFS (Fewest Moves) |
| **2** | Phase 3 | Select DFS (Depth-First) |
| **3** | Phase 3 | Select A* (Lowest Weighted Cost) |
| **C** | Phase 3 | Open Compare All Dashboard |
| **A** | Phase 3 | Animate delivery van along route |
| **SPACE** | Phase 3 | Pause / Resume search animation |
| **N** | Phase 3 | Step single node expansion |
| **R** | Phase 3 / Phase 1 | Reset search route (preserves pellets) / Reset network |
| **L** | Any | Open Terrain Legend screen |
| **H** | Any | Open Help & Formulation screen |
| **ESC** | Any | Return to previous phase / Close dialog |

---

## 8. Build, Run, and Testing Instructions

### Prerequisites
- Windows operating system.
- MinGW GCC 6.3+ on system PATH (`g++ --version`).
- FreeGLUT libraries and headers (already included in `GL/`, `lib/`, and `freeglut.dll`).

### Compiling and Launching the Application
Double-click `build_and_run.bat` or run via command line:
```cmd
compile.bat
SmartRouteAI.exe
```

### Running Automated Test Suite
To verify algorithm correctness, heuristic admissibility, and workflow invariants without launching an OpenGL window:
```cmd
run_tests.bat
```

The test runner compiles `tests/test_algorithms.cpp` and validates all 30 automated test cases:
- Tests 01–04: Reachable and unreachable goal detection for BFS, DFS, and A*.
- Tests 05–07: Legal orthogonal steps, start/goal endpoint validation, and barrier exclusion.
- Tests 08–10: Cost accounting, fractional cost preservation ($0.5$), and penalty costs ($4.0$).
- Tests 11–12: A* weighted-cost optimality against independent Dijkstra search.
- Tests 13–15: Invariant checks ($25$-cell route = $24$ moves, start==goal, state reset cleanliness).
- Tests 16–30: Interactive pellet placement, undo, clearing, wall protection, start/goal protection, and cross-phase persistence.

---

## 9. Known Limitations

1. **Grid-Based Abstraction:** Road networks are modeled as 2D orthogonal grids rather than continuous vector road graphs (like OpenStreetMap).
2. **Turn Penalties:** Traversal costs are modeled per grid cell rather than calculating deceleration or intersection turning penalties.
3. **Single Vehicle:** The simulation optimizes a single delivery courier and does not solve multi-vehicle routing problems (VRP).

---

## 10. Live Presentation Script (5-Minute Walkthrough)

1. **Slide 1 / Intro (1 min):** State the last-mile delivery challenge and introduce SmartRoute AI.
2. **Phase 1 Demonstration (1 min):** Show Phase 1, click `Generate` or load `Scenario B`. Explain grid formulation (Store $S$ and Customer $G$). Click `Continue to Road Config`.
3. **Phase 2 Demonstration (1 min):** Select `Reward Pellet (0.5)` and click several cells. Select `Penalty Pellet (4.0)` and block a corridor. Point out the live pellet counter. Click `Continue to Solving`.
4. **Phase 3 Demonstration (1.5 min):** 
   - Select **BFS**, click `Run`, and note route moves and weighted cost.
   - Select **A\***, click `Run`, and show how A* routes through the gold pellets to achieve a lower weighted cost.
   - Click `Animate Van` to show the visual route execution.
5. **Comparison Dashboard (0.5 min):** Click `COMPARE ALL` to open the comparison table. Summarize the trade-off: BFS guarantees fewest moves, while A* guarantees minimum delivery cost.
