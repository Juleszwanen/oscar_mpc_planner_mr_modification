# Guidance Constraints Visual Flow Diagrams

This document provides visual representations of the data flow and execution sequence in the guidance constraints module.

## 1. High-Level System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    jules_ros1_jackalplanner.cpp                      │
│                         (Robot Controller)                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Timer Callback (20Hz)                                              │
│    └─> loopDirectTrajectory()                                       │
│        ├─> prepareObstacleData()                                    │
│        ├─> generatePlanningCommand()                                │
│        │   └─> _planner->solveMPC(_state, _data) ◄─────┐           │
│        └─> publishCmdAndVisualize()                     │           │
│                                                          │           │
└──────────────────────────────────────────────────────────┼───────────┘
                                                           │
                                                           │
┌──────────────────────────────────────────────────────────┼───────────┐
│                         planner.cpp                      │           │
│                       (MPC Orchestrator)                 │           │
├──────────────────────────────────────────────────────────┼───────────┤
│                                                          │           │
│  PlannerOutput solveMPC(State, RealTimeData) ◄──────────┘           │
│    │                                                                 │
│    ├─> Initialize warmstart                                         │
│    │                                                                 │
│    ├─> FOR EACH MODULE:                                             │
│    │   └─> module->update()  ◄────────────────┐                    │
│    │                                           │                    │
│    ├─> FOR EACH MODULE:                        │                    │
│    │   └─> module->setParameters()             │                    │
│    │                                           │                    │
│    ├─> FOR EACH MODULE:                        │                    │
│    │   └─> module->optimize() ◄────────────────┼────┐               │
│    │       │                                   │    │               │
│    │       └─> Returns exit_code              │    │               │
│    │                                           │    │               │
│    └─> Build PlannerOutput                     │    │               │
│        └─> return {trajectory, success}        │    │               │
│                                                │    │               │
└────────────────────────────────────────────────┼────┼───────────────┘
                                                 │    │
                                                 │    │
┌────────────────────────────────────────────────┼────┼───────────────┐
│                  guidance_constraints.cpp      │    │               │
│                  (Parallel MPC Module)         │    │               │
├────────────────────────────────────────────────┼────┼───────────────┤
│                                                │    │               │
│  update() ◄────────────────────────────────────┘    │               │
│    ├─> Load obstacles as halfspaces                 │               │
│    ├─> Set start state & velocity                   │               │
│    ├─> Generate goal grid: setGoals()               │               │
│    ├─> Run guidance search: global_guidance_->Update()              │
│    │   ├─> Graph search in 2D/3D                    │               │
│    │   ├─> Find topology classes                    │               │
│    │   └─> Generate guidance splines                │               │
│    └─> Map trajectories to planners                 │               │
│        [Produces: n_paths_ guidance trajectories]   │               │
│                                                      │               │
│  optimize() ◄────────────────────────────────────────┘               │
│    ├─> Setup OpenMP (nested, 8 threads)                             │
│    │                                                                 │
│    ├─> PARALLEL FOR (8 threads):                                    │
│    │   ├─ Thread 0: LocalPlanner[0] ─┐                              │
│    │   ├─ Thread 1: LocalPlanner[1] ─┤                              │
│    │   ├─ Thread 2: LocalPlanner[2] ─┤                              │
│    │   ├─ Thread 3: LocalPlanner[3] ─┤                              │
│    │   ├─ Thread 4: LocalPlanner[4] ─┤  Each runs:                  │
│    │   ├─ Thread 5: LocalPlanner[5] ─┤  - Copy solver               │
│    │   ├─ Thread 6: LocalPlanner[6] ─┤  - Init with guidance        │
│    │   └─ Thread 7: T-MPC++ ─────────┤  - Update constraints        │
│    │                                  │  - Load parameters           │
│    │   [BARRIER - All threads sync]  │  - Solve MPC (15-40ms)       │
│    │                                  │  - Store result              │
│    │                                  └─> All complete               │
│    │                                                                 │
│    ├─> FindBestPlanner()                                            │
│    │   └─> Select min(objective) where success=true                 │
│    │                                                                 │
│    └─> Transfer winning solution to main solver                     │
│        └─> return exit_code                                         │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

## 2. Detailed optimize() Execution Flow

```
optimize() Entry
     │
     ├─ Phase 1: Setup (< 1ms)
     │   ├─> omp_set_nested(1)
     │   ├─> omp_set_max_active_levels(2)
     │   └─> omp_set_dynamic(0)
     │
     ├─ Phase 2: Parallel Optimization Loop (~20-40ms)
     │   │
     │   #pragma omp parallel for num_threads(8)
     │   for (planner : planners_) {
     │   │
     │   ├─ 2.1: Determine if active
     │   │   └─> disabled if id >= num_guidance_trajectories
     │   │       (except T-MPC++ which is always active)
     │   │
     │   ├─ 2.2: Copy main solver
     │   │   └─> *local_solver = *_solver
     │   │
     │   ├─ 2.3: Construct constraints
     │   │   ├─ if is_original_planner:
     │   │   │   └─> No guidance constraints (empty data)
     │   │   │       Safety constraints only
     │   │   │
     │   │   └─ else (guided planner):
     │   │       ├─> if has existing_guidance:
     │   │       │   └─> Warmstart from previous MPC
     │   │       │
     │   │       └─> else:
     │   │           └─> initializeSolverWithGuidance()
     │   │               ├─> Get guidance spline
     │   │               ├─> Sample positions at k*dt
     │   │               ├─> Set x[k], y[k]
     │   │               └─> Compute psi, v from velocity
     │   │
     │   ├─ 2.4: Load parameters for all timesteps
     │   │   for k = 0 to N:
     │   │       ├─> guidance_constraints->setParameters(k)
     │   │       │   └─> Populate halfspace constraints
     │   │       └─> safety_constraints->setParameters(k)
     │   │           └─> Populate obstacle constraints
     │   │
     │   ├─ 2.5: Set adaptive timeout
     │   │   └─> timeout = planning_time - used_time - 0.006s
     │   │
     │   ├─ 2.6: SOLVE MPC (15-40ms each, parallel)
     │   │   ├─> local_solver->loadWarmstart()
     │   │   └─> exit_code = local_solver->solve()
     │   │       │
     │   │       └─> Interior point / SQP iterations
     │   │           ├─> Evaluate dynamics
     │   │           ├─> Evaluate constraints
     │   │           ├─> Compute gradient & Hessian
     │   │           ├─> Solve QP subproblem
     │   │           └─> Check convergence
     │   │
     │   └─ 2.7: Process results
     │       ├─> success = (exit_code == 1)
     │       ├─> objective = solver->_info.pobj
     │       ├─> guidance_ID = topology class
     │       │
     │       └─> if previously_selected:
     │           └─> objective *= consistency_weight (0.8)
     │   }
     │   [Implicit barrier - all threads complete]
     │
     ├─ Phase 3: Decision Making (< 1ms)
     │   │
     │   └─> FindBestPlanner()
     │       │
     │       ├─ For each planner:
     │       │   if (!disabled && success && obj < best_obj):
     │       │       best_index = i
     │       │       best_obj = obj
     │       │
     │       └─ Return best_index (or -1 if none succeeded)
     │
     └─ Phase 4: Solution Transfer (< 1ms)
         ├─> Get best_planner[best_index]
         ├─> Notify guidance: OverrideSelectedTrajectory()
         ├─> Transfer to main solver:
         │   ├─> _solver->_output = best_solver->_output
         │   ├─> _solver->_info = best_solver->_info
         │   └─> _solver->_params = best_solver->_params
         │
         └─> return best_planner.result.exit_code
```

## 3. Parallel Thread Execution Timeline

```
Time (ms) →
0         10        20        30        40        50
│─────────│─────────│─────────│─────────│─────────│
│
├─ Setup
│  [Config OpenMP, check guidance]
│
Thread 0: ├══════════════════════════════════╪══╡ Planner[0] (guided) - Cost: 95
Thread 1: ├════════════════════════════╪═════════╡ Planner[1] (guided) - Cost: 102
Thread 2: ├═══════════════════════════════════╪══╡ Planner[2] (guided) - Cost: 88 ★ BEST
Thread 3: ├════════════════════════════════╪═════╡ Planner[3] (disabled)
Thread 4: ├════════════════════════════════╪═════╡ Planner[4] (disabled)
Thread 5: ├════════════════════════════════╪═════╡ Planner[5] (disabled)
Thread 6: ├════════════════════════════════╪═════╡ Planner[6] (disabled)
Thread 7: ├═══════════════════════════════════════════╪╡ T-MPC++ - Cost: 110
                                                    │
                                            [Barrier]
                                                    │
                                                    ├─ Decision: Select Planner[2]
                                                    ├─ Transfer solution
                                                    └─ Return success

Legend:
═══ Solving MPC (expensive)
╪   Solver completed
★   Selected solution
```

## 4. Data Structures Relationship

```
┌──────────────────────────────────────────────────────────┐
│              GuidanceConstraints                         │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  std::vector<LocalPlanner> planners_                     │
│      │                                                   │
│      ├─> LocalPlanner[0] ─────────────────┐              │
│      │   ├─ id = 0                        │              │
│      │   ├─ local_solver (Solver*)        │              │
│      │   │   ├─ _output (trajectory)      │              │
│      │   │   ├─ _info (objective, iters)  │              │
│      │   │   └─ _params (N, dt, ...)      │              │
│      │   ├─ guidance_constraints           │              │
│      │   │   └─ LinearizedConstraints      │              │
│      │   │       └─ Halfspaces from        │              │
│      │   │           guidance trajectory   │              │
│      │   ├─ safety_constraints              │              │
│      │   │   └─ DecompConstraints or       │              │
│      │   │       EllipsoidalConstraints    │              │
│      │   │       └─ Obstacle avoidance     │              │
│      │   ├─ result (SolverResult)          │              │
│      │   │   ├─ exit_code: 1               │ ◄────────────┤ These results
│      │   │   ├─ objective: 95.3            │              │ are compared
│      │   │   ├─ success: true              │              │ to select
│      │   │   ├─ guidance_ID: 42            │              │ the best
│      │   │   └─ color: 0                   │              │ solution
│      │   ├─ is_original_planner: false     │              │
│      │   └─ disabled: false                │              │
│      │                                      │              │
│      ├─> LocalPlanner[1] ─────────────────┤              │
│      ├─> LocalPlanner[2] ─────────────────┤              │
│      ├─> ...                               │              │
│      └─> LocalPlanner[n] (T-MPC++)────────┘              │
│          └─ is_original_planner: true                    │
│                                                          │
│  int best_planner_index_ ───> Points to winning planner │
│                                                          │
│  std::shared_ptr<GlobalGuidance> global_guidance_       │
│      └─> Provides guidance trajectories                 │
│          ├─ Spline trajectory[0] (topology class 42)    │
│          ├─ Spline trajectory[1] (topology class 17)    │
│          └─ Spline trajectory[2] (topology class 5)     │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 5. Consistency Bonus Mechanism

```
Timestep t:
    Planners optimize in parallel
    │
    ├─ Planner A (topology 42) → objective = 100.0
    ├─ Planner B (topology 17) → objective = 95.0
    └─ Planner C (topology 5)  → objective = 110.0
    │
    Decision: Select B (lowest cost 95.0)
    ├─> Store: previously_selected = topology 17
    └─> Execute trajectory B

Timestep t+1:
    Planners optimize again
    │
    ├─ Planner A (topology 42) → objective = 98.0
    │   └─> NOT previously selected
    │       Effective cost = 98.0 * 1.0 = 98.0
    │
    ├─ Planner B (topology 17) → objective = 100.0
    │   └─> WAS previously selected! ★
    │       Effective cost = 100.0 * 0.8 = 80.0  [BONUS!]
    │
    └─ Planner C (topology 5)  → objective = 105.0
        └─> NOT previously selected
            Effective cost = 105.0 * 1.0 = 105.0
    │
    Decision: Select B (effective cost 80.0 < 98.0)
    └─> Maintain topology 17 even though 98.0 < 100.0
        (Prevents unnecessary switching)

Why this works:
    - Prevents oscillation between similar-cost topologies
    - Ensures smooth, consistent behavior
    - Only switches when new topology is significantly better
    - Typical consistency_weight: 0.7 - 0.9
```

## 6. Constraint Architecture

```
Each LocalPlanner has TWO constraint modules:

┌─────────────────────────────────────────────────────┐
│             LocalPlanner[i]                         │
├─────────────────────────────────────────────────────┤
│                                                     │
│  1. Guidance Constraints (LinearizedConstraints)   │
│     ┌───────────────────────────────────────────┐  │
│     │ Purpose: Keep in same homotopy class     │  │
│     │                                           │  │
│     │ For each timestep k:                     │  │
│     │   Get point on guidance at k*dt          │  │
│     │   Linearize into halfspace:              │  │
│     │     A^T * x ≤ b                          │  │
│     │                                           │  │
│     │ Effect: Restricts which side of          │  │
│     │         obstacles trajectory can pass    │  │
│     │                                           │  │
│     │ Example: Guidance goes LEFT of obstacle  │  │
│     │   → Constraint forces MPC to stay LEFT   │  │
│     └───────────────────────────────────────────┘  │
│                                                     │
│  2. Safety Constraints (Decomp or Ellipsoidal)     │
│     ┌───────────────────────────────────────────┐  │
│     │ Purpose: Avoid collisions                │  │
│     │                                           │  │
│     │ For each obstacle j, timestep k:         │  │
│     │   Get predicted obstacle position        │  │
│     │   Create convex constraint:              │  │
│     │     ||robot[k] - obs[k]|| ≥ r_safe      │  │
│     │                                           │  │
│     │ Effect: Ensures collision-free           │  │
│     │         trajectory                        │  │
│     │                                           │  │
│     │ Works WITH guidance constraints          │  │
│     │   (both must be satisfied)               │  │
│     └───────────────────────────────────────────┘  │
│                                                     │
└─────────────────────────────────────────────────────┘

T-MPC++ (Original Planner) is special:
    ├─ Guidance Constraints: ✗ DISABLED (empty data)
    │   └─> Free to explore any topology
    │
    └─ Safety Constraints: ✓ ACTIVE
        └─> Still avoids collisions

Result: T-MPC++ acts as fallback when guidance fails
```

## 7. Homology ID Storage Flow (Recommended Implementation)

```
┌──────────────────────────────────────────────────────────┐
│          GuidanceConstraints::optimize()                 │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  best_planner = planners_[best_index]                    │
│                                                          │
│  // Transfer solution (existing)                        │
│  _solver->_output = best_solver->_output                 │
│                                                          │
│  // NEW: Store metadata                                 │
│  module_data.selected_topology_id = guidance_ID          │
│  module_data.used_guidance = !is_original_planner        │
│  module_data.trajectory_cost = objective                 │
│                                                          │
└──────────────┬───────────────────────────────────────────┘
               │ module_data passed to caller
               ↓
┌──────────────────────────────────────────────────────────┐
│               Planner::solveMPC()                        │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  // After optimize loop                                 │
│  if (exit_flag == 1) {                                  │
│      _output.success = true;                            │
│                                                          │
│      // NEW: Transfer from module_data                  │
│      _output.selected_topology_id = module_data...      │
│      _output.used_guidance = module_data...             │
│      _output.trajectory_cost = module_data...           │
│                                                          │
│      // Build trajectory                                │
│      for (k = 0; k < N; k++)                            │
│          _output.trajectory.add(...)                    │
│  }                                                       │
│                                                          │
│  return _output;                                        │
│                                                          │
└──────────────┬───────────────────────────────────────────┘
               │ PlannerOutput with topology info
               ↓
┌──────────────────────────────────────────────────────────┐
│       jules_ros1_jackalplanner.cpp                       │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  auto output = _planner->solveMPC(_state, _data);       │
│                                                          │
│  if (output.success) {                                  │
│      // NOW AVAILABLE:                                  │
│      int topo = output.selected_topology_id;            │
│      bool guided = output.used_guidance;                │
│      double cost = output.trajectory_cost;              │
│                                                          │
│      // Use for:                                        │
│      LOG_INFO("Topology: " << topo);                    │
│      publishTopologyInfo(topo);                         │
│      trackTopologySwitches(topo);                       │
│      coordinateWithOtherRobots(topo);                   │
│  }                                                       │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 8. Complete System Timing Breakdown

```
One Control Cycle (50ms @ 20Hz):
│
├─ [0-2ms] ROS callbacks & data preparation
│   ├─ Robot state update
│   ├─ Obstacle predictions
│   └─ Path updates
│
├─ [2-7ms] Planner::solveMPC() setup
│   ├─ Check data readiness
│   ├─ Initialize warmstart
│   └─ Set initial state
│
├─ [7-12ms] Module updates (ALL modules)
│   ├─ MPCBaseModule::update()
│   ├─ Contouring::update()
│   └─ GuidanceConstraints::update()  [1-5ms]
│       └─> global_guidance_->Update()
│           ├─ Graph search
│           ├─ Topology classification
│           └─ Spline generation
│
├─ [12-14ms] Module setParameters (ALL modules)
│   └─> Minimal work for GuidanceConstraints
│
├─ [14-47ms] GuidanceConstraints::optimize()
│   ├─ [14-15ms] Setup & solver copy
│   │
│   ├─ [15-45ms] PARALLEL optimization
│   │   ├─ Thread 0-7 solve simultaneously
│   │   │   └─> Each: 15-40ms
│   │   │       └─> max(all) ≈ 30ms
│   │   └─ Implicit barrier
│   │
│   └─ [45-47ms] Decision & transfer
│       ├─ FindBestPlanner(): <1ms
│       └─ Solution transfer: <1ms
│
├─ [47-48ms] Build PlannerOutput
│   └─ Extract trajectory from solver
│
└─ [48-50ms] Command extraction & publishing
    ├─ getSolution(1, "v")
    ├─ getSolution(0, "w")
    ├─ Publish trajectory
    └─ Visualization

Total: 48-50ms (within 50ms deadline) ✓

Note: Timing varies based on:
    - Problem complexity
    - Number of obstacles
    - Convergence speed
    - Hardware (CPU cores, frequency)
```

---

These diagrams provide visual understanding of the guidance constraints module's operation. For detailed textual explanations, see `guidance_constraints_documentation.md`.
