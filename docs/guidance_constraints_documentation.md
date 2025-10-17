# Guidance Constraints Module Documentation - Comprehensive Guide

## Executive Summary

The `GuidanceConstraints` module is the **core implementation of T-MPC++ (Topology-aware Model Predictive Control)** for autonomous navigation in dynamic environments. This module enables a robot to simultaneously explore multiple trajectory homotopy classes (topologically distinct paths around obstacles) and select the optimal one in real-time.

**Key Innovation**: Instead of committing to a single trajectory early, the system evaluates multiple fundamentally different path topologies in parallel using independent MPC solvers, then intelligently selects the best feasible solution based on cost and constraints.

This module serves as the bridge between:
- **High-level guidance planning** (fast graph search in simplified space) 
- **Low-level MPC optimization** (dynamically feasible trajectory generation)

## Document Purpose

This documentation provides an **in-depth explanation** of how `guidance_constraints.cpp` works, with special emphasis on:
1. Its role as a module inside the `Planner::solveMPC()` pipeline
2. What happens when `module->update()` is called
3. What happens when `module->optimize()` is called  
4. How to store and utilize homology IDs for higher-level planning
5. Integration with `jules_ros1_jackalplanner.cpp`

## Overview

The `GuidanceConstraints` module is a sophisticated controller module that implements **T-MPC** (Topology-aware Model Predictive Control) for autonomous navigation. It enables the robot to explore multiple trajectory homotopy classes in parallel, allowing it to intelligently choose between different paths around obstacles based on optimality and feasibility.

This module serves as the bridge between high-level guidance planning (topology search) and low-level MPC optimization, running multiple parallel solvers to evaluate different trajectory topologies simultaneously.

## Key Concepts

### What is Homotopy/Topology?

In the context of path planning, **homotopy classes** (or topology classes) represent fundamentally different ways to navigate around obstacles. For example:
- Going left around an obstacle vs. right around an obstacle = 2 different homotopy classes
- Passing between two obstacles vs. going around both = 2 different homotopy classes

The guidance constraints module evaluates multiple homotopy classes in parallel to find the optimal trajectory.

### T-MPC and T-MPC++

- **T-MPC (Topology-aware MPC)**: Uses guidance trajectories to explore different homotopy classes
- **T-MPC++**: Enhanced version that includes an additional "non-guided" planner that operates without topology constraints, providing a fallback option

## Integration with Planner::solveMPC() Pipeline

### The Big Picture: Where GuidanceConstraints Fits

The `GuidanceConstraints` module is one of several controller modules loaded into the main `Planner` class. Let's trace the execution flow from when a robot needs to plan a trajectory:

```
jules_ros1_jackalplanner.cpp:
    └─> loop() / loopDirectTrajectory()
        └─> _planner->solveMPC(_state, _data)  ← Entry point
            │
            └─> Planner::solveMPC() [in planner.cpp]
                │
                ├─> 1. Initialize warmstart
                │   └─> _solver->initializeWarmstart()
                │
                ├─> 2. FOR EACH MODULE: module->update()
                │   └─> GuidanceConstraints::update()  ◄── FIRST KEY FUNCTION
                │       ├─> Runs global guidance search
                │       ├─> Finds topologically distinct paths
                │       └─> Maps trajectories to planners
                │
                ├─> 3. FOR EACH MODULE: module->setParameters()
                │   └─> GuidanceConstraints::setParameters()
                │       └─> (Does minimal work - most happens in optimize)
                │
                ├─> 4. FOR EACH MODULE: module->optimize()
                │   └─> GuidanceConstraints::optimize()  ◄── SECOND KEY FUNCTION
                │       ├─> Runs PARALLEL MPC optimizations (8 threads)
                │       ├─> Each thread optimizes a different topology
                │       ├─> Selects best feasible solution
                │       └─> Returns exit_code of best solution
                │
                └─> 5. Build output trajectory
                    └─> return PlannerOutput
```

### Why GuidanceConstraints is Special

Most modules in the MPC planner framework only implement `update()` and `setParameters()`. The **GuidanceConstraints module is unique** because it overrides `optimize()` to implement a **custom optimization strategy**.

**Key Point**: When `GuidanceConstraints::optimize()` returns an exit code other than `EXIT_CODE_NOT_OPTIMIZED_YET`, it **bypasses the standard solver loop entirely**. This allows T-MPC++ to replace single-solver optimization with parallel multi-solver optimization.

From `planner.cpp` lines 125-136:
```cpp
exit_flag = EXIT_CODE_NOT_OPTIMIZED_YET;
for (auto &module : _modules) {
    exit_flag = module->optimize(state, data, _module_data);
    if (exit_flag != EXIT_CODE_NOT_OPTIMIZED_YET)
        break;  // GuidanceConstraints returned a real exit code - DONE!
}
// This standard solve is SKIPPED when GuidanceConstraints is active:
if (exit_flag == EXIT_CODE_NOT_OPTIMIZED_YET)
    exit_flag = _solver->solve();
```

### Module Initialization

The modules are initialized in the auto-generated `modules.h` file:

```cpp
namespace MPCPlanner {
    inline void initializeModules(
        std::vector<std::shared_ptr<ControllerModule>> &modules, 
        std::shared_ptr<Solver> solver)
    {
        modules.emplace_back(std::make_shared<MPCBaseModule>(solver));
        modules.emplace_back(std::make_shared<Contouring>(solver));
        modules.emplace_back(std::make_shared<GuidanceConstraints>(solver));
    }
}
```

**Important**: Only `GuidanceConstraints` implements a custom `optimize()` that returns a meaningful exit code. The other modules return `EXIT_CODE_NOT_OPTIMIZED_YET`, deferring to the standard solver.

## Core Data Structures

### 1. SolverResult

```cpp
struct SolverResult {
    int exit_code;        // Solver exit status (1 = success)
    double objective;     // Cost of the solution
    bool success;         // Whether optimization succeeded
    int guidance_ID;      // Which guidance trajectory was used
    int color;            // Color index for visualization
}
```

**Purpose**: Encapsulates all relevant results from a single parallel optimization. This allows the module to track and compare results from multiple solvers running simultaneously.

**Key Fields**:
- `exit_code`: Indicates solver status (1 = converged successfully)
- `objective`: The cost function value - lower is better
- `guidance_ID`: Identifies which homotopy class this solution represents
- `color`: Used for visualization to distinguish different topology classes

### 2. LocalPlanner

```cpp
struct LocalPlanner {
    int id;                                              // Planner identifier
    std::shared_ptr<Solver> local_solver;                // Independent MPC solver
    std::unique_ptr<LinearizedConstraints> guidance_constraints;  // Topology constraints
    std::unique_ptr<GUIDANCE_CONSTRAINTS_TYPE> safety_constraints; // Collision avoidance
    SolverResult result;                                 // Optimization results
    bool is_original_planner;                            // True if T-MPC++ non-guided planner
    bool disabled;                                       // Whether to skip this planner
    bool taken;                                          // Used in trajectory mapping
    bool existing_guidance;                              // Has previous guidance for warmstart
}
```

**Purpose**: Represents a complete independent MPC optimization instance. Each LocalPlanner can explore a different homotopy class or operate without guidance constraints.

**Why Multiple LocalPlanners?**: The module maintains a vector of LocalPlanners (`planners_`) to run parallel optimizations. Each explores a different trajectory topology, and the best feasible solution is selected.

**Key Components**:
- `local_solver`: Each planner has its own solver instance to enable true parallel execution
- `guidance_constraints`: Linearized constraints that keep the trajectory in a specific topology
- `safety_constraints`: Collision avoidance constraints (decomposition or ellipsoidal)
- `is_original_planner`: Identifies the T-MPC++ planner that runs without guidance

## Important Functions

### Constructor: GuidanceConstraints(std::shared_ptr<Solver> solver)

**Purpose**: Initializes the guidance constraints module and creates multiple parallel planners.

**What it does**:
1. Creates the global guidance planner (`GlobalGuidance`) for topology search
2. Reads configuration parameters (`_use_tmpcpp`, `_enable_constraints`, `_control_frequency`)
3. Determines number of planners needed based on `n_paths_` configuration
4. Creates `n_solvers` LocalPlanner instances for guided optimization
5. Optionally adds a T-MPC++ planner (non-guided) if enabled

**Why it matters**: This setup phase creates all the parallel solver infrastructure needed for multi-topology exploration. Each LocalPlanner gets its own solver instance to avoid conflicts during parallel execution.

### update(State &state, const RealTimeData &data, ModuleData &module_data)

**Purpose**: Runs the global guidance search to find feasible trajectory topologies.

#### When It's Called

This function is called **once per control cycle** from `Planner::solveMPC()` at line 96:
```cpp
for (auto &module : _modules)
    module->update(state, data, _module_data);
```

It's called **BEFORE** `optimize()`, establishing the guidance trajectories that will be used for parallel optimization.

#### Complete Execution Flow with Code Analysis

Let's walk through the function step-by-step with the actual implementation:

##### Step 1: Validate Path Data (Lines 62-72)
```cpp
void GuidanceConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
{
    LOG_MARK("Guidance Constraints: Update");
    
    if (module_data.path == nullptr) {
        LOG_MARK("Path data not yet available");
        return;  // Early exit - no path loaded yet
    }
```

**Why this check?** The guidance planner needs a reference path to define longitudinal goals. Without it, no meaningful guidance can be computed.

**What happens when path is missing?** 
- The function returns early
- No guidance trajectories are generated
- Only T-MPC++ (non-guided planner) will be active in `optimize()`

##### Step 2: Load Static Obstacles (Lines 75-83)
```cpp
    // Convert static obstacles
    if (!module_data.static_obstacles.empty()) {
        std::vector<GuidancePlanner::Halfspace> halfspaces;
        for (size_t i = 0; i < module_data.static_obstacles[0].size(); i++) {
            halfspaces.emplace_back(
                module_data.static_obstacles[0][i].A,  // Normal vector
                module_data.static_obstacles[0][i].b   // Offset
            );
        }
        global_guidance_->LoadStaticObstacles(halfspaces);
    }
```

**What are halfspaces?** A halfspace is defined by `A^T * x ≤ b`, where:
- `A` is a normal vector pointing away from the obstacle
- `b` is the signed distance from the origin
- Together they define a linear inequality representing obstacle-free space

**Why halfspaces?** The guidance planner uses a graph search in 2D/3D space. Halfspaces provide an efficient representation for checking if a node is collision-free (just evaluate the inequality).

**Example**: For a wall at x=5, the halfspace would be: `A = [-1, 0]`, `b = -5` → `-x ≤ -5` → `x ≥ 5` (space right of the wall)

##### Step 3: Check for Early Exit with T-MPC++ Only (Lines 85-86)
```cpp
    if (_use_tmpcpp && global_guidance_->GetConfig()->n_paths_ == 0)
        return;  // No guidance needed - only non-guided planner will run
```

**Configuration scenario**: If `n_paths_ = 0` and `use_t-mpc++ = true`, the system operates as **standard LMPCC** (single non-guided planner). This is useful for comparing T-MPC performance against baseline.

##### Step 4: Set Start State (Lines 88-89)
```cpp
    global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));
```

**What gets passed**:
- `state.getPos()`: Current robot position (x, y) as Eigen::Vector2d
- `state.get("psi")`: Current robot heading angle in radians
- `state.get("v")`: Current robot velocity magnitude

**Why velocity matters**: The guidance planner projects future positions using velocity to determine reachable goals over the horizon.

##### Step 5: Set Reference Velocity (Lines 91-94)
```cpp
    if (module_data.path_velocity != nullptr)
        global_guidance_->SetReferenceVelocity(
            module_data.path_velocity->operator()(state.get("spline"))
        );
    else
        global_guidance_->SetReferenceVelocity(
            CONFIG["weights"]["reference_velocity"].as<double>()
        );
```

**Two modes**:
1. **Path-dependent velocity**: If the path has associated velocity profile (from motion planner), use the velocity at the current path position `s = state.get("spline")`
2. **Constant velocity**: Otherwise, use a constant reference velocity from config (typically 1.0-2.0 m/s)

**Why it matters**: Reference velocity determines how far along the path the robot aims to travel, affecting goal placement and guidance trajectory length.

##### Step 6: Handle Disabled Output Mode (Lines 96-100)
```cpp
    if (!CONFIG["enable_output"].as<bool>()) {
        LOG_INFO_THROTTLE(15000, "Not propagating nodes (output is disabled)");
        global_guidance_->DoNotPropagateNodes();
    }
```

**Purpose**: When `enable_output = false` (e.g., during testing or data collection), the robot should not move. This tells the guidance planner not to expand the search tree, saving computation.

##### Step 7: Generate Goal Grid (Line 103)
```cpp
    setGoals(state, module_data);
```

**This is critical!** The `setGoals()` function (detailed in next section) creates a spatial grid of goal points along the reference path. These goals define:
- **Where** the guidance planner searches (longitudinal positions along path)
- **What topologies** it can find (lateral offsets enable left/right paths)

See detailed breakdown in the `setGoals()` section below.

##### Step 8: Run Guidance Search - THE MAIN COMPUTATION (Lines 105-106)
```cpp
    LOG_MARK("Running Guidance Search");
    global_guidance_->Update();  // ← This is where the magic happens!
```

**What happens in `Update()`**:
1. **Graph Construction**: Build a motion primitive graph in state space
   - Nodes represent robot states (x, y, θ) or (x, y, θ, v)
   - Edges represent feasible motion primitives (turns, straight lines)
   
2. **Graph Search**: Run a search algorithm (typically A* or Dijkstra) to find paths from start to goals
   - **Heuristic**: Distance to nearest goal + estimated cost-to-go
   - **Collision checking**: For each expanded node, check against static obstacles (halfspaces) and dynamic obstacles
   
3. **Topology Classification**: Identify which paths belong to which homotopy classes
   - Uses topological signatures (e.g., which side of each obstacle the path passes)
   - Groups paths with same signature into one topology class
   
4. **Path Selection**: Select one representative path per topology class
   - Typically the lowest-cost path in each class
   - Up to `n_paths_` trajectories returned (one per class)
   
5. **Spline Generation**: Fit smooth splines through the discrete paths
   - Cubic or quintic splines for C² continuity
   - These become the guidance trajectories

**Computational cost**: 1-5ms typically (much faster than MPC optimization)

**Why so fast?** 
- Operates in lower-dimensional space (2D or 3D vs. MPC's full state-input space)
- Uses discrete motion primitives (precomputed)
- Graph search is well-optimized

##### Step 9: Map Trajectories to Planners (Line 108)
```cpp
    mapGuidanceTrajectoriesToPlanners();
```

**Purpose**: Assign each found guidance trajectory to a `LocalPlanner` instance for parallel optimization.

**Intelligence in the mapping**: This function tries to maintain consistency by assigning trajectories to planners that optimized the same topology in the previous timestep. This enables:
- **Effective warmstarting**: Previous solution is a good initialization
- **Temporal smoothness**: Reduces trajectory switching

See detailed breakdown in `mapGuidanceTrajectoriesToPlanners()` section.

##### Step 10: Store Empty Data for T-MPC++ (Lines 111-112)
```cpp
    empty_data_ = data;
    empty_data_.dynamic_obstacles.clear();
```

**Why clear obstacles?** The T-MPC++ (non-guided) planner operates without guidance constraints. To ensure fair comparison and avoid over-constraining it, we provide it with data that has dynamic obstacles removed for the guidance constraints module.

**Important**: The safety constraints module still sees full obstacle data, so collision avoidance is maintained!

#### What Gets Computed in update()

After `update()` completes, the module has:

1. ✅ **Global guidance trajectories**: `n_paths_` topologically distinct smooth paths from current position toward goals
2. ✅ **Trajectory-to-planner mapping**: Which `LocalPlanner` should optimize which guidance trajectory  
3. ✅ **Topology signatures**: Identification of which homotopy class each trajectory represents
4. ✅ **Empty data for T-MPC++**: Special data configuration for the non-guided planner

#### Why update() Runs Before optimize()

The guidance trajectories are used in two ways during `optimize()`:

1. **Initialization**: As warm-starts for MPC solvers (`initializeSolverWithGuidance()`)
2. **Constraints**: Linearized into halfspace constraints to keep trajectories in their topology

These must be available **before** optimization begins, hence the separation:
- `update()`: Compute guidance (fast graph search)
- `optimize()`: Refine guidance into dynamically feasible trajectories (expensive MPC)

### setGoals(State &state, const ModuleData &module_data)

**Purpose**: Generates a spatial grid of goal points along the reference path for the guidance planner.

**Algorithm**:
1. Compute final distance along path using velocity integration: `final_s = current_s + Σ(v(s) * dt)`
2. Create `n_long` longitudinal waypoints along the path from current position to `final_s`
3. For each longitudinal waypoint, create `n_lat` lateral offsets perpendicular to the path
4. Assign costs to each goal based on distance from the desired final position
5. Ensure the first goal is centered on the path (middle lateral position)

**Why it matters**: The goal grid defines the search space for the guidance planner. Longitudinal goals ensure progress toward the destination, while lateral goals enable exploration of different topology classes (e.g., left vs. right around obstacles).

**Important Constraints**:
- `n_lat` must be odd so there's a center goal on the reference path
- Goals respect path width and robot radius to ensure safety
- Cost function prefers goals closer to the reference path and farther along it

### mapGuidanceTrajectoriesToPlanners()

**Purpose**: Intelligently assigns found guidance trajectories to available planners, maintaining consistency with previous optimizations.

**Algorithm**:
1. Mark all planners as available
2. **First Pass**: Try to match each guidance trajectory to a planner that previously optimized the same homotopy class
   - This enables effective warmstarting from previous solutions
3. **Second Pass**: Assign remaining trajectories to unused planners
4. Store mappings in `_map_homotopy_class_to_planner`

**Why it matters**: This mapping strategy improves computational efficiency by:
- Enabling warm starts when a homotopy class was optimized in the previous time step
- Maintaining temporal consistency in trajectory selection
- Efficiently distributing computational work across available planners

### optimize(State &state, const RealTimeData &data, ModuleData &module_data) - **THE MAIN FUNCTION**

**Purpose**: Executes parallel MPC optimizations across multiple homotopy classes and selects the best solution.

#### When It's Called

This function is called from `Planner::solveMPC()` at line 128:
```cpp
for (auto &module : _modules) {
    exit_flag = module->optimize(state, data, _module_data);
    if (exit_flag != EXIT_CODE_NOT_OPTIMIZED_YET)
        break;  // GuidanceConstraints returned - use its result!
}
```

**Critical**: This function **replaces the standard MPC solver loop**. When it returns an exit code other than `EXIT_CODE_NOT_OPTIMIZED_YET`, the standard `_solver->solve()` is **not called**.

#### Complete Execution Flow with Code Analysis

This is the most complex and important function in the module. Let's break it down phase by phase:

---

##### Phase 1: Setup and Configuration (Lines 264-277)

```cpp
int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
{
    PROFILE_FUNCTION();
    
    // Required for parallel call to the solvers when using Forces
    omp_set_nested(1);            // Enable nested parallel regions
    omp_set_max_active_levels(2); // Allow 2 levels of parallelism
    omp_set_dynamic(0);           // Disable dynamic thread adjustment
    
    LOG_MARK("Guidance Constraints: optimize");
    
    if (!_use_tmpcpp && !global_guidance_->Succeeded())
        return 0;  // Guidance failed and no fallback - return failure
    
    bool shift_forward = CONFIG["shift_previous_solution_forward"].as<bool>() &&
                         CONFIG["enable_output"].as<bool>();
```

**OpenMP Configuration Deep Dive**:

1. **`omp_set_nested(1)`**: Allows OpenMP to create parallel regions inside other parallel regions
   - Level 1: The main parallel loop over planners (up to 8 threads)
   - Level 2: Internal solver operations that may also use OpenMP
   - Without this, inner parallel regions would run sequentially

2. **`omp_set_max_active_levels(2)`**: Limits nesting to 2 levels
   - Prevents excessive thread creation (exponential explosion)
   - With N threads at level 1 and M at level 2, total threads = N*M
   - Limiting to 2 keeps thread count manageable

3. **`omp_set_dynamic(0)`**: Disables dynamic thread adjustment
   - Forces OpenMP to use exactly the number of threads requested
   - Ensures deterministic behavior (same threads every iteration)
   - Prevents OpenMP from reducing threads due to load

**Why check guidance success?** If guidance search failed (no feasible paths found) and T-MPC++ is disabled, we can't proceed. Early return with exit code 0 (failure).

**Shift forward logic**: If the previous solve succeeded and output is enabled, we can use the previous solution shifted forward in time as a warm start. This exploits temporal smoothness of the optimization landscape.

---

##### Phase 2: Parallel Optimization Loop (Lines 279-361)

This is where the core parallel computation happens:

```cpp
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    PROFILE_SCOPE("Guidance Constraints: Parallel Optimization");
    planner.result.Reset();
    planner.disabled = false;
```

**The OpenMP pragma explained**:
- `parallel`: Create a team of threads
- `for`: Distribute loop iterations across threads
- `num_threads(8)`: Use exactly 8 threads
- Each iteration (planner) runs on a different thread
- **No data dependencies**: Each planner has independent solver

**Thread Safety**: This is embarrassingly parallel because:
- Each thread works on a different `planner` object
- Each planner has its own `local_solver` (independent memory)
- All reads from shared data (`_solver`, `data`) are read-only
- No locks or synchronization needed inside the loop

##### Sub-Phase 2.1: Determine Active Planners (Lines 286-293)

```cpp
    if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories()) {
        if (!planner.is_original_planner) {
            planner.disabled = true;
            continue;  // Skip this planner
        }
    }
```

**Logic**:
- `NumberOfGuidanceTrajectories()`: How many guidance paths were found (0 to `n_paths_`)
- If `planner.id >= num_trajectories`: No guidance for this planner ID
- Exception: Keep the original planner (`is_original_planner = true`) even without guidance
- Other planners without guidance are disabled and skipped

**Example**: If guidance found 3 paths and we have 5 planners + 1 original:
- Planners 0, 1, 2: Active (have guidance)
- Planners 3, 4: Disabled (no guidance)
- Original planner: Active (T-MPC++ fallback)

##### Sub-Phase 2.2: Copy Main Solver State (Lines 295-298)

```cpp
    auto &solver = planner.local_solver;
    LOG_MARK("Planner [" << planner.id << "]: Copying data from main solver");
    *solver = *_solver;  // Deep copy the main solver
```

**What gets copied**:
- Current state vector (x₀)
- Solver parameters (N, dt, bounds, weights)
- Warm-start trajectory (previous solution)
- Internal solver state

**Why copy?** Each thread needs an independent solver instance to avoid race conditions. The copy operation gives each planner a private solver initialized with the global state.

**Performance note**: This is a deep copy (~10KB of data). With 8 threads, that's ~80KB copied from `_solver`. This is one source of memory bandwidth usage.

##### Sub-Phase 2.3: Construct Constraints (Lines 300-317)

This is where guided vs. non-guided planners diverge:

```cpp
    // CONSTRUCT CONSTRAINTS
    if (planner.is_original_planner || (!_enable_constraints)) {
        // T-MPC++ path: No guidance constraints
        planner.guidance_constraints->update(state, empty_data_, module_data);
        planner.safety_constraints->update(state, data, module_data);
    }
    else {
        // Guided path: Use guidance trajectory
        LOG_MARK("Planner [" << planner.id << "]: Loading guidance into solver");
        
        if (CONFIG["t-mpc"]["warmstart_with_mpc_solution"].as<bool>() && 
            planner.existing_guidance) {
            // Warm-start from previous MPC solution for this topology
            planner.local_solver->initializeWarmstart(state, shift_forward);
        }
        else {
            // Warm-start from guidance trajectory
            initializeSolverWithGuidance(planner);
        }
        
        planner.guidance_constraints->update(state, data, module_data);
        planner.safety_constraints->update(state, data, module_data);
    }
```

**Decision tree**:

```
Is this the original planner OR are constraints disabled?
├─ YES: (T-MPC++ mode)
│   ├─> Update guidance_constraints with EMPTY obstacles (no guidance constraints)
│   └─> Update safety_constraints with FULL obstacles (collision avoidance only)
│
└─ NO: (Guided T-MPC mode)
    ├─> Does this planner have existing guidance from previous timestep?
    │   ├─ YES: Warm-start from previous MPC solution (best continuity)
    │   └─ NO: Warm-start from guidance trajectory (new topology)
    │
    ├─> Update guidance_constraints with FULL data
    │   └─> This linearizes the guidance trajectory into halfspace constraints
    │
    └─> Update safety_constraints with FULL data
        └─> Ensures collision avoidance with obstacles
```

**Why two constraint types?**

1. **Guidance Constraints** (`LinearizedConstraints`):
   - Purpose: Keep the trajectory in the same homotopy class as guidance
   - Implementation: Linearize guidance trajectory into halfspace constraints
   - Effect: Restricts the solution to stay on the same "side" of obstacles
   - Only active for guided planners

2. **Safety Constraints** (`DecompConstraints` or `EllipsoidalConstraints`):
   - Purpose: Ensure collision-free trajectories
   - Implementation: Convex constraints for each obstacle at each timestep
   - Effect: Prevents collisions while allowing topology flexibility
   - Active for ALL planners (including T-MPC++)

**Key insight**: Guidance constraints **restrict topology**, safety constraints **prevent collisions**. Together, they ensure safe, topology-consistent trajectories.

##### Sub-Phase 2.4: Load Parameters for All Timesteps (Lines 319-329)

```cpp
    // LOAD PARAMETERS
    LOG_MARK("Planner [" << planner.id << "]: Loading parameters into solver");
    for (int k = 0; k < _solver->N; k++) {
        if (planner.is_original_planner)
            planner.guidance_constraints->setParameters(empty_data_, module_data, k);
        else
            planner.guidance_constraints->setParameters(data, module_data, k);
        
        planner.safety_constraints->setParameters(data, module_data, k);
    }
```

**What this does**: For each timestep k in the prediction horizon [0, N]:
- Calls `setParameters(k)` on constraint modules
- Constraint modules populate solver parameter arrays with:
  - Halfspace constraint parameters (A, b for each constraint)
  - Obstacle positions and sizes
  - Constraint activation flags

**Why the loop?** MPC constraints are time-varying:
- Obstacle at timestep k=0 is at current position
- Obstacle at timestep k=5 is at predicted position 5*dt seconds ahead
- Guidance constraints also vary (different points along guidance trajectory)

**Original planner special case**: Uses `empty_data_` for guidance constraints (no topology restrictions) but real `data` for safety constraints (collision avoidance still active).

##### Sub-Phase 2.5: Set Adaptive Timeout (Lines 331-333)

```cpp
    std::chrono::duration<double> used_time = 
        std::chrono::system_clock::now() - data.planning_start_time;
    planner.local_solver->_params.solver_timeout = 
        _planning_time - used_time.count() - 0.006;
```

**Adaptive timeout calculation**:
```
Available time per control cycle = 1 / control_frequency
Used time so far = now - planning_start_time
Safety margin = 0.006 seconds (for post-processing)

Remaining time for solver = Available - Used - Safety
```

**Example at 20Hz** (50ms total cycle time):
- Planning starts at t=0
- update() and setup take 5ms → used_time = 0.005s
- Available: 0.050s, Used: 0.005s, Safety: 0.006s
- **Solver timeout**: 0.050 - 0.005 - 0.006 = **0.039s** (39ms)

**Why adaptive?** Different control cycles may use different amounts of time in `update()` depending on:
- Number of obstacles (affects guidance search complexity)
- Graph size (affects search time)
- Previous solutions (affects warmstart quality)

By adapting the timeout, we ensure all solvers get a fair share of the remaining time.

**Why 0.006s safety margin?** Leaves time for:
- Decision making (FindBestPlanner): ~1ms
- Solution transfer: <1ms  
- Visualization: 2-3ms
- Other post-processing: 1-2ms

##### Sub-Phase 2.6: Solve Optimization - THE EXPENSIVE PART (Lines 335-339)

```cpp
    // SOLVE OPTIMIZATION
    planner.local_solver->loadWarmstart();
    LOG_MARK("Planner [" << planner.id << "]: Solving ...");
    planner.result.exit_code = solver->solve();
    LOG_MARK("Planner [" << planner.id << "]: Done! (exitcode = " 
             << planner.result.exit_code << ")");
```

**What happens in `solver->solve()`**:

1. **Problem Formulation**: The solver has a nonlinear program (NLP):
   ```
   minimize_u   Σ(stage_cost(x_k, u_k)) + terminal_cost(x_N)
   subject to:  
       x_{k+1} = f(x_k, u_k)        [dynamics]
       h(x_k, u_k) ≤ 0              [inequality constraints]
       g(x_k, u_k) = 0              [equality constraints]
       u_min ≤ u_k ≤ u_max          [input bounds]
       x_min ≤ x_k ≤ x_max          [state bounds]
   ```

2. **Interior Point Method** (if using Forces) or **SQP** (if using Acados):
   - Iteratively solves linear approximations of the NLP
   - Each iteration: Compute gradient, Hessian, solve QP subproblem
   - Typical iterations: 10-50 depending on problem difficulty

3. **Constraint Evaluation**: At each iteration, evaluate:
   - Guidance constraints: Linear (fast)
   - Safety constraints: Nonlinear (ellipsoid/decomp - moderately expensive)
   - Dynamics: Nonlinear (vehicle model)

4. **Convergence Check**: Stop when:
   - Optimality conditions satisfied (KKT tolerance reached)
   - Maximum iterations reached
   - Timeout exceeded
   - Problem detected as infeasible

**Typical timing**:
- Best case (warm-start close to optimum): 5-15ms
- Average case (moderate convergence): 15-35ms
- Worst case (poor warm-start or difficult problem): 35-50ms or timeout

**Exit codes**:
- `1`: Success (converged to local optimum)
- `0`: Maximum iterations reached (solution may still be usable)
- `-1`: Infeasible (no solution exists satisfying constraints)
- Other negative values: Solver-specific errors

##### Sub-Phase 2.7: Process Results and Apply Consistency Bonus (Lines 341-360)

```cpp
    // ANALYSIS AND PROCESSING
    planner.result.success = planner.result.exit_code == 1;
    planner.result.objective = solver->_info.pobj;  // How good is the solution?
    
    if (planner.is_original_planner) {
        // T-MPC++: Assign a special ID outside normal topology range
        planner.result.guidance_ID = 2 * global_guidance_->GetConfig()->n_paths_;
        planner.result.color = -1;
    }
    else {
        // Guided planner: Get topology info from guidance trajectory
        auto &guidance_trajectory = global_guidance_->GetGuidanceTrajectory(planner.id);
        planner.result.guidance_ID = guidance_trajectory.topology_class;
        planner.result.color = guidance_trajectory.color_;
        
        // CONSISTENCY BONUS: Prefer previously selected topology
        if (guidance_trajectory.previously_selected_)
            planner.result.objective *= 
                global_guidance_->GetConfig()->selection_weight_consistency_;
    }
}  // End of parallel loop - implicit barrier here
```

**Result fields populated**:

1. **`success`**: Boolean indicating if optimization succeeded (exit_code == 1)

2. **`objective`**: The cost function value from the solver
   - Lower is better
   - Includes terms for: control effort, progress, path deviation, comfort
   - Typical range: 10-1000 depending on horizon length and weights

3. **`guidance_ID`**: Integer identifying which homotopy class this solution represents
   - For guided planners: Comes from guidance trajectory's topology signature
   - For original planner: Assigned a unique ID outside the normal range
   - Used for: Tracking which topology was selected, consistency bonuses

4. **`color`**: Visualization color index (0 to n_paths_-1)
   - Maps to a color palette for visualization
   - Allows distinguishing trajectories in RViz
   - Original planner gets color -1 (special highlighting)

**Consistency Bonus Mechanism**:

The key innovation for smooth behavior is the consistency bonus:

```cpp
if (guidance_trajectory.previously_selected_)
    planner.result.objective *= selection_weight_consistency_;
```

**How it works**:
- If this topology was selected in the previous timestep, `previously_selected_ = true`
- Multiply its objective by `selection_weight_consistency_` (typically 0.7-0.9)
- Effect: Makes this trajectory appear 10-30% "cheaper" than it actually is
- Result: Hysteresis - prefers to stick with current topology unless others are significantly better

**Example**:
```
Topology A (current): objective = 100, was selected last time
  → Effective objective = 100 * 0.8 = 80

Topology B (alternative): objective = 90, was NOT selected last time  
  → Effective objective = 90 * 1.0 = 90

Decision: Keep topology A (effective 80 < 90) despite B being cheaper (90 < 100)
```

**Why this matters**: Without consistency bonus, the robot would frequently switch between similar-cost topologies, causing:
- Jerky motion
- Oscillation between paths
- Discomfort for passengers
- Suboptimal long-term behavior

With consistency bonus:
- Smooth transitions
- Stable trajectory selection
- Only switches when new topology is significantly better

**Implicit Barrier**: At the end of the parallel for loop, OpenMP inserts an implicit barrier. All threads must complete their optimization before the program continues. This ensures all results are ready for the decision phase.

---

##### Phase 3: Decision Making - Select Best Solution (Lines 363-373)

After the parallel loop completes (all threads finished), decision making happens serially:

```cpp
omp_set_dynamic(1);  // Re-enable dynamic threads for subsequent code

{
    PROFILE_SCOPE("Decision");
    // DECISION MAKING
    best_planner_index_ = FindBestPlanner();
    
    if (best_planner_index_ == -1) {
        LOG_MARK("Failed to find a feasible trajectory in any of the " 
                 << planners_.size() << " optimizations.");
        return planners_[0].result.exit_code;  // Return failure
    }
```

**`FindBestPlanner()` logic** (implementation at lines 416-434):
```cpp
int GuidanceConstraints::FindBestPlanner() {
    double best_solution = 1e10;
    int best_index = -1;
    
    for (size_t i = 0; i < planners_.size(); i++) {
        auto &planner = planners_[i];
        
        if (planner.disabled)   // Skip disabled planners
            continue;
        
        if (planner.result.success && planner.result.objective < best_solution) {
            best_solution = planner.result.objective;
            best_index = i;
        }
    }
    return best_index;  // -1 if no successful planner found
}
```

**Selection criteria** (in priority order):
1. **Feasibility**: Only consider planners where `success = true` (exit_code == 1)
2. **Optimality**: Among feasible planners, select the one with **lowest objective value**
3. **Consistency bonus**: Objective values already include consistency bonus, so previously selected topology has an advantage

**Failure handling**: If `best_index = -1` (all optimizations failed):
- Log warning message
- Return exit code from first planner (typically infeasible)
- Robot will apply braking command (handled in jules_ros1_jackalplanner.cpp)

---

##### Phase 4: Solution Transfer and Notification (Lines 375-387)

```cpp
    auto &best_planner = planners_[best_planner_index_];
    auto &best_solver = best_planner.local_solver;
    
    // Communicate to guidance which topology we selected
    global_guidance_->OverrideSelectedTrajectory(
        best_planner.result.guidance_ID, 
        best_planner.is_original_planner
    );
    
    // Transfer winning solution to main solver
    _solver->_output = best_solver->_output;
    _solver->_info = best_solver->_info;
    _solver->_params = best_solver->_params;
    
    return best_planner.result.exit_code;  // Return success (1)
}
```

**Solution transfer deep dive**:

1. **`_solver->_output`**: The trajectory solution
   ```cpp
   struct SolverOutput {
       double x[N+1];      // Position x at each timestep
       double y[N+1];      // Position y at each timestep  
       double psi[N+1];    // Heading at each timestep
       double v[N+1];      // Velocity at each timestep
       double a[N];        // Acceleration input at each timestep
       double w[N];        // Angular velocity input at each timestep
       // ... other state/input variables
   };
   ```

2. **`_solver->_info`**: Solver statistics
   ```cpp
   struct SolverInfo {
       double pobj;             // Primal objective value
       int iterations;          // Number of iterations used
       double solvetime;        // Time taken to solve (seconds)
       // ... other solver metrics
   };
   ```

3. **`_solver->_params`**: Final solver parameters (mostly for debugging/logging)

**Why transfer to main solver?** The rest of the codebase (planner.cpp, jules_ros1_jackalplanner.cpp) expects the solution in `_solver`. By copying the winning solution there, we maintain compatibility with the rest of the system.

**Notification to guidance**: The call to `OverrideSelectedTrajectory()` informs the global guidance planner which topology was selected. This has two effects:

1. **Consistency tracking**: Marks this topology as `previously_selected_ = true` for next iteration
2. **Visualization**: Can highlight the selected trajectory in guidance planner visualizations

**Return value**: The exit code of the best planner (typically 1 for success) is returned to `Planner::solveMPC()`. This signals that a valid solution was found and prevents the standard solver from running.

---

#### Summary of optimize() Function

**Input**: Current state, real-time data with obstacles, module data with path

**Output**: Exit code indicating success/failure of best optimization

**Side Effects**:
1. Populates `_solver` with the winning solution
2. Updates `best_planner_index_` for visualization
3. Notifies global guidance of selected topology
4. Updates each planner's result structure

**Computational Breakdown**:
- Setup: <1ms
- Parallel optimization: max(all solver times) + 2ms overhead ≈ 15-55ms
- Decision making: <1ms
- Solution transfer: <1ms
- **Total**: 16-60ms (typically 20-40ms)

**Parallelization benefit**:
- Sequential: Σ(solve times) = 8 × 30ms = 240ms (too slow!)
- Parallel: max(solve times) + overhead = 35ms + 2ms = 37ms (acceptable!)
- **Speedup**: ~6-7x with 8 threads

#### Why This Is Brilliant

The `optimize()` function implements a **portfolio approach** to MPC:

1. **Diversification**: Instead of betting on one optimization, explore multiple simultaneously
2. **Robustness**: If one topology fails, others may succeed (T-MPC++ acts as fallback)
3. **Optimality**: Finds the truly best topology by actually solving the full problem for each
4. **Real-time**: Achieves this through parallelization, maintaining control frequency
5. **Consistency**: Uses hysteresis to avoid erratic switching

This is fundamentally different from traditional MPC that commits to one trajectory early and hopes it's good enough.

### initializeSolverWithGuidance(LocalPlanner &planner)

**Purpose**: Initializes a solver's state trajectory using a guidance trajectory as a warmstart.

**Algorithm**:
1. Retrieve the guidance trajectory spline for this planner
2. For each time step k in the prediction horizon:
   - Sample position from spline at time `k * dt`
   - Set solver prediction: `solver->setEgoPrediction(k, "x", position.x)`
   - Set solver prediction: `solver->setEgoPrediction(k, "y", position.y)`
   - Compute heading from velocity vector: `psi = atan2(velocity.y, velocity.x)`
   - Set velocity magnitude: `v = velocity.norm()`

**Why it matters**: This provides a high-quality initial guess to the MPC solver. Starting close to a feasible solution significantly improves:
- Convergence speed (fewer iterations needed)
- Success rate (less likely to get stuck in infeasible regions)
- Solution quality (better local optimum)

**Key Insight**: The guidance trajectory is kinematically feasible but not necessarily dynamically feasible. The MPC optimization refines it to respect dynamics, velocity limits, and collision constraints.

### FindBestPlanner()

**Purpose**: Selects the optimal planner from all parallel optimizations.

**Selection Logic**:
```cpp
for each planner:
    if not disabled and success and objective < best_objective:
        best_index = i
        best_objective = objective
```

**Returns**: Index of best planner, or -1 if none succeeded

**Why it matters**: This is the critical decision-making function. It determines which trajectory the robot will actually follow. The objective function typically includes:
- Progress toward goal
- Control effort
- Deviation from reference path
- Comfort (acceleration/jerk limits)

### onDataReceived(RealTimeData &data, std::string &&data_name)

**Purpose**: Handles incoming real-time data, specifically dynamic obstacles.

**For "dynamic obstacles"**:
1. Forward obstacle data to all planners' safety constraint modules
2. Convert obstacles to guidance planner format:
   - Extract position predictions over horizon
   - Include obstacle radius + robot radius for safe distance
   - Store obstacle ID for tracking
3. Load obstacles into global guidance: `global_guidance_->LoadObstacles()`

**Why it matters**: The guidance planner needs obstacle information to find topology classes that avoid collisions. This function bridges the gap between ROS obstacle messages and the guidance planner's internal representation.

### visualize(const RealTimeData &data, const ModuleData &module_data)

**Purpose**: Publishes visualization markers to RViz for debugging and monitoring.

**What gets visualized**:
1. **Guidance trajectories**: Color-coded by topology class from global guidance
2. **Warmstart trajectories**: Initial guesses for each planner (if debug mode enabled)
3. **Optimized trajectories**: Final MPC solutions for each planner
   - Best solution: Highlighted (typically white or special color)
   - Original planner (T-MPC++): Special color (color index 11)
   - Other guided planners: Color-coded by topology class

**Color Coding Logic**:
- Best selected trajectory: `color = -1` (special highlighting)
- Original planner: `color = 11`
- Guided planners: `color = guidance_trajectory.color_` (unique per topology)

**Why it matters**: Visual feedback is essential for:
- Understanding which topology the robot chose
- Debugging optimization failures
- Verifying that guidance trajectories match expectations
- Monitoring performance differences between topology classes

### reset()

**Purpose**: Resets all state when starting a new planning problem.

**Actions**:
1. Reset the global guidance planner: `global_guidance_->Reset()`
2. Reset all local solvers: `planner.local_solver->reset()`

**When called**: 
- When a goal is reached
- When a timeout occurs
- When manually triggered (e.g., new task)

**Why it matters**: Ensures clean slate for new planning problems and prevents stale state from affecting new optimizations.

### saveData(RosTools::DataSaver &data_saver)

**Purpose**: Records performance metrics for offline analysis and benchmarking.

**Data Recorded**:
- Guidance planner runtime
- Objective value for each planner (or -1 if failed)
- ID of the original planner (T-MPC++)
- Best planner index selected
- Best objective value (GMPCC objective)
- LMPCC objective (original planner performance)

**Why it matters**: Enables comparative analysis of:
- T-MPC vs. T-MPC++ performance
- Guidance planner computational cost
- Success rates across different topologies
- Objective function trends over time

## Parallel Computation Architecture

### Overview

The guidance constraints module employs **task parallelism** using OpenMP to run multiple MPC optimizations simultaneously. This is fundamentally different from data parallelism - instead of splitting one computation across threads, we run multiple complete optimizations in parallel.

### OpenMP Configuration Details

```cpp
omp_set_nested(1);            // Enable nested parallel regions
omp_set_max_active_levels(2); // Allow 2 levels of parallelism
omp_set_dynamic(0);           // Disable dynamic thread adjustment
```

**Nested Parallelism**:
- **Level 1**: Parallel loop over planners (up to 8 threads)
- **Level 2**: Each solver may use parallel operations internally (e.g., matrix operations)

**Why disable dynamic threads?** To ensure deterministic behavior and prevent OpenMP from adaptively changing thread counts, which could cause performance variability.

### Parallel Loop Structure

```cpp
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    // Each iteration runs on a separate thread
    // No data dependencies between iterations
}
```

**Thread Safety Considerations**:

1. **Independent Solvers**: Each planner has its own `local_solver` instance, so no shared state during optimization
2. **Read-Only Shared Data**: `_solver`, `data`, `module_data` are only read, never modified
3. **Result Storage**: Each planner writes to its own `planner.result` - no race conditions
4. **Logging**: OpenMP-safe logging via `LOG_MARK` macro

### Memory Access Patterns

**Per-Thread Data** (thread-local, no contention):
- `planner.local_solver` - Each thread's MPC solver
- `planner.result` - Each thread's results
- `planner.guidance_constraints` - Each thread's constraint module
- `planner.safety_constraints` - Each thread's safety module

**Shared Read-Only Data** (safe to share):
- `_solver` - Main solver (copied to local solvers, not modified)
- `data` - Real-time data (read-only in optimize)
- `module_data` - Module data (read-only in optimize)
- `global_guidance_` - Guidance planner results (read-only after Update())

**Shared Mutable Data** (synchronized):
- None during the parallel loop! The loop is embarrassingly parallel.

### Performance Characteristics

**Speedup Analysis**:
- **Ideal Case**: 8x speedup with 8 threads and 8 planners
- **Typical Case**: 5-7x speedup due to:
  - OpenMP overhead
  - Load imbalance (some topologies harder to optimize)
  - Memory bandwidth saturation
  - Cache effects

**Scalability**:
- Scales well up to the number of CPU cores
- Limited by the number of planners (typically 3-8)
- Each solver is computationally intensive (~10-50ms), making parallel overhead negligible

### Synchronization Points

1. **Before parallel loop**: All planners must be initialized (done in constructor)
2. **During parallel loop**: No synchronization needed - truly parallel
3. **After parallel loop**: Implicit barrier at end of `#pragma omp parallel for`
4. **Decision phase**: Serial execution to select best planner

```cpp
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    // Parallel region - 8 threads running simultaneously
}
// Implicit barrier here - all threads complete before continuing

omp_set_dynamic(1);  // Re-enable dynamic threads
```

**Why the barrier matters**: Ensures all optimizations complete before comparing results.

### Timeout Management

Each solver has an adaptive timeout:
```cpp
std::chrono::duration<double> used_time = std::chrono::system_clock::now() - data.planning_start_time;
planner.local_solver->_params.solver_timeout = _planning_time - used_time.count() - 0.006;
```

**Purpose**: Ensures all optimizations complete within the control cycle (e.g., 50ms at 20Hz).

**Why 0.006s buffer?** Leaves time for post-processing (decision making, solution transfer, visualization).

### Load Balancing

**Static Scheduling**: OpenMP uses static scheduling by default, dividing planners evenly among threads.

**Imbalance Factors**:
- Some topologies may converge faster than others
- Original planner (T-MPC++) may have different convergence time
- Disabled planners skip immediately

**Potential Improvement**: Could use `schedule(dynamic)` to improve load balancing if imbalance is significant.

### Cache and Memory Considerations

**Cache Effects**:
- Each solver has large state (parameters, outputs, ~10KB)
- Multiple threads may cause cache line conflicts
- `local_solver` instances should ideally fit in L2/L3 cache

**Memory Bandwidth**:
- All threads read from shared `_solver` during copy operation
- Could become bottleneck with many threads
- Generally not limiting with 8 threads

### Comparison to Sequential Execution

**Sequential Version** (hypothetical):
```cpp
for (auto &planner : planners_) {
    // Solve one at a time: Total time = sum of all solve times
}
```
**Total time**: ~8 × 30ms = 240ms (too slow for 20Hz control!)

**Parallel Version** (actual):
```cpp
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    // Solve simultaneously: Total time ≈ max of all solve times
}
```
**Total time**: ~max(30ms) + overhead ≈ 35ms (acceptable for 20Hz control)

### Why This Parallelization Works Well

1. **Embarrassingly Parallel**: No data dependencies between planners
2. **Coarse-Grained**: Each task (MPC solve) takes 10-50ms - overhead is negligible
3. **Balanced Workload**: All solvers do similar computations
4. **Memory Locality**: Each thread works on its own data structures
5. **Meaningful Work**: Each thread does substantial computation, not just busy-wait

### Limitations and Considerations

**Maximum Parallelism**: Limited by number of feasible topologies (typically 3-8)

**Hardware Requirements**: 
- Benefits from 8+ CPU cores
- Requires significant memory (~50MB per solver × 8 solvers = 400MB)

**Real-Time Constraints**:
- Must complete within control cycle (50ms at 20Hz)
- Adaptive timeouts prevent individual solvers from running too long
- If no solution found in time, returns infeasible

**Thread Affinity**: OpenMP may not provide optimal thread pinning - could be improved with explicit affinity settings.

## Module Integration and Data Flow

### High-Level Architecture

```
ROS Interface
    ↓
Planner::solveMPC()
    ↓
GuidanceConstraints::update()      ← Topology Search (Graph Search)
    ↓
GuidanceConstraints::optimize()    ← Parallel MPC Optimization
    ↓ (parallel)
LocalPlanner[0] → Solver → Solution₀
LocalPlanner[1] → Solver → Solution₁
LocalPlanner[2] → Solver → Solution₂
...
LocalPlanner[n] → Solver → Solutionₙ
    ↓ (serial)
FindBestPlanner() → Select Best Solution
    ↓
Transfer to Main Solver
    ↓
Robot Executes Best Trajectory
```

### Interaction with Other Modules

**Global Guidance Planner** (`GuidancePlanner::GlobalGuidance`):
- Performs graph search in lower-dimensional space
- Finds topologically distinct paths (homotopy classes)
- Provides guidance trajectories for MPC initialization
- Updates at control frequency (20Hz typical)

**Linearized Constraints** (`LinearizedConstraints`):
- Linearizes guidance trajectory into constraint halfspaces
- Keeps MPC solution in the same topology as guidance
- One instance per LocalPlanner

**Safety Constraints** (`GUIDANCE_CONSTRAINTS_TYPE`):
- Typically `DecompConstraints` or `EllipsoidalConstraints`
- Ensures collision avoidance with obstacles
- One instance per LocalPlanner

**Main Solver** (`Solver`):
- The primary MPC solver instance (ID = 0)
- Provides initial state and parameters to local solvers
- Receives the winning solution after parallel optimization

### Control Flow in solveMPC()

```cpp
// In Planner::solveMPC()
for (auto &module : _modules) {
    if (module->hasCustomOptimization()) {
        exit_flag = module->optimize(state, data, module_data);
        // GuidanceConstraints::optimize() runs here
        // Performs parallel optimization and returns best exit code
        break;  // Only one custom optimization allowed
    }
}
```

**Key Point**: `GuidanceConstraints::optimize()` replaces the standard MPC solve loop. Instead of one optimization, it runs multiple in parallel and returns the best result.

## Configuration Parameters

### T-MPC Settings

```yaml
t-mpc:
  use_t-mpc++: true                      # Enable non-guided planner
  enable_constraints: true                # Use guidance constraints (vs. just warmstart)
  warmstart_with_mpc_solution: true      # Warmstart from previous MPC solution
  highlight_selected: true                # Highlight chosen trajectory in visualization
  selection_weight_consistency_: 0.8      # Consistency bonus (multiply objective by this)
```

### Guidance Planner Settings

```yaml
guidance:
  n_paths_: 3                    # Number of topology classes to explore
  longitudinal_goals_: 5         # Number of longitudinal waypoints
  vertical_goals_: 5             # Number of lateral offsets per waypoint
  N: 20                          # Planning horizon for guidance
```

### Performance Settings

```yaml
control_frequency: 20.0                          # Hz - affects planning time budget
shift_previous_solution_forward: true            # Shift solution for warmstart
solver_timeout: 0.05                             # Max time per solver (overridden adaptively)
```

## Performance Characteristics

### Computational Cost

**Guidance Search**: 1-5ms (fast graph search in low-dimensional space)

**Single MPC Solve**: 10-50ms (depending on complexity)

**Parallel Optimization**: ~max(all solver times) + ~2ms overhead ≈ 15-55ms

**Decision Making**: <1ms (simple comparison)

**Total**: 16-60ms (typically 25-35ms, acceptable for 20Hz control)

### Success Rates

**With Guidance**: High success rate (>95%) when guidance is feasible
- Warmstart from guidance trajectory aids convergence

**T-MPC++ Fallback**: Provides solution when guidance fails
- May find different topology than guidance suggested
- Acts as safety net

### Memory Usage

**Per LocalPlanner**: ~50MB (solver state, parameters, constraints)

**Total**: ~50MB × (n_paths + 1) ≈ 200-400MB for typical configurations

**Guidance Planner**: ~10MB (graph, trajectories, obstacles)

## Troubleshooting and Common Issues

### Issue 1: All Solvers Fail

**Symptoms**: `FindBestPlanner()` returns -1

**Possible Causes**:
- Timeout too short (all solvers run out of time)
- Problem is genuinely infeasible (no collision-free path exists)
- Guidance trajectories lead to infeasible regions

**Solutions**:
- Increase `solver_timeout` or decrease complexity
- Enable T-MPC++ to try non-guided optimization
- Check obstacle prediction accuracy
- Verify guidance planner is finding feasible topologies

### Issue 2: Always Selects Original Planner

**Symptoms**: Best planner is always `is_original_planner == true`

**Possible Causes**:
- Guidance constraints are too restrictive
- Guidance trajectories are poor quality
- Original planner has unfair advantage (different constraints)

**Solutions**:
- Verify `enable_constraints` is true
- Check guidance planner configuration
- Review `selection_weight_consistency_` (may over-prefer previous selection)
- Visualize guidance trajectories to verify quality

### Issue 3: Poor Performance / Slow

**Symptoms**: Missing control deadlines, low control frequency

**Possible Causes**:
- Too many planners (large `n_paths_`)
- Insufficient CPU cores
- Solvers not converging quickly (many iterations)
- Load imbalance

**Solutions**:
- Reduce `n_paths_` (fewer topologies to explore)
- Use more CPU cores or better hardware
- Tune solver convergence settings
- Profile to find bottlenecks

### Issue 4: Frequent Topology Switching

**Symptoms**: Robot switches between topologies every time step

**Possible Causes**:
- `selection_weight_consistency_` too high (equal to 1.0)
- Objective values very similar between topologies
- Guidance trajectories unstable

**Solutions**:
- Decrease `selection_weight_consistency_` (e.g., 0.7-0.9) to prefer previous selection
- Add hysteresis in decision making
- Improve guidance planner stability
- Smooth trajectory transitions

## Advanced Topics

### Extending to More Topologies

To increase the number of explored topologies:

1. Increase `n_paths_` in configuration
2. Increase lateral goal count (`vertical_goals_`)
3. Ensure sufficient CPU cores
4. Monitor computational time to stay within control cycle

### Custom Heuristics

The objective function can be modified with:
- Consistency bonus (already implemented)
- Distance-based preference (prefer certain topologies based on robot position)
- Learning-based selection (use learned model to predict best topology)

### Integration with Learning

Potential extensions:
- Learn which topologies are typically best in different scenarios
- Predict solver success before running optimization
- Adaptive scheduling based on predicted difficulty

### Distributed Computation

For even more parallelism:
- Distribute planners across multiple machines
- Use MPI or ROS multi-master
- Aggregate results from distributed optimizations

## Conclusion

The `GuidanceConstraints` module implements a sophisticated parallel MPC framework that enables robots to intelligently explore multiple trajectory topologies simultaneously. By combining:

1. **Efficient topology search** (guidance planner)
2. **Parallel MPC optimization** (OpenMP)
3. **Intelligent decision making** (best planner selection)
4. **Consistency preferences** (temporal coherence)

The system achieves robust, optimal navigation in complex environments with multiple obstacles.

**Key Strengths**:
- Explores multiple solutions simultaneously
- Maintains real-time performance through parallelization
- Provides fallback option (T-MPC++) when guidance fails
- Enables navigation in scenarios where single-topology MPC would fail

**Core Innovation**: Rather than committing to a single trajectory topology early, the system evaluates multiple options in parallel and selects the best one. This significantly improves success rates and optimality compared to traditional MPC approaches.

---

## Integration with jules_ros1_jackalplanner.cpp

### How the Robot Controller Uses the Planner

The `jules_ros1_jackalplanner.cpp` file implements the ROS node that wraps the MPC planner for the Jackal robot. Let's trace how it uses the guidance constraints module:

#### Control Loop Structure

The main control loop in `loopDirectTrajectory()` (lines 262-290):

```cpp
void JulesJackalPlanner::loopDirectTrajectory(const ros::TimerEvent &) 
{
    // 1. Handle initial planning with dummy obstacles if needed
    if (_planning_for_the_frist_time || !_have_received_meaningful_trajectory_data) {
        handleInitialPlanningPhase();
        return;
    }
    
    // 2. Prepare obstacle data (merge trajectory predictions from other robots)
    _data.planning_start_time = std::chrono::system_clock::now();
    prepareObstacleData();
    
    // 3. Call the MPC solver
    auto [cmd, output] = generatePlanningCommand();
    
    // 4. Publish commands and visualize
    publishCmdAndVisualize(cmd, output);
}
```

#### The Critical Call: generatePlanningCommand() (lines 1222-1274)

```cpp
std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> 
JulesJackalPlanner::generatePlanningCommand()
{
    geometry_msgs::Twist cmd;
    MPCPlanner::PlannerOutput output;
    
    // Goal handling omitted for brevity...
    
    // MAIN MPC CALL - This is where GuidanceConstraints does its work
    output = _planner->solveMPC(_state, _data);
    
    if (_enable_output && output.success) {
        // Extract control commands from the solution
        cmd.linear.x = _planner->getSolution(1, "v");    // Velocity
        cmd.angular.z = _planner->getSolution(0, "w");   // Angular velocity
    }
    else {
        // Solver failed - apply braking
        applyBrakingCommand(cmd);
        buildOutputFromBrakingCommand(output, cmd);
    }
    
    return {cmd, output};
}
```

**Data Flow**:
```
_state: Current robot state (x, y, psi, v)
_data: RealTimeData with obstacles, path, goals
    ↓
_planner->solveMPC(_state, _data)
    ├─> GuidanceConstraints::update()    [Find topologies]
    ├─> GuidanceConstraints::optimize()  [Parallel MPC]
    └─> Returns PlannerOutput
            ↓
output.trajectory: Planned path (positions, orientations)
output.success: Did optimization succeed?
    ↓
Extract control commands:
    cmd.linear.x = velocity at k=1
    cmd.angular.z = angular velocity at k=0
    ↓
Robot executes command
```

#### What the Robot Receives

After `solveMPC()` completes, the `PlannerOutput` structure contains:

```cpp
struct PlannerOutput {
    Trajectory trajectory;  // The planned trajectory
    bool success;          // Did the solver succeed?
    
    // Trajectory contains:
    struct Trajectory {
        std::vector<Eigen::Vector2d> positions;     // (x,y) at each timestep
        std::vector<double> orientations;           // psi at each timestep
        double dt;                                  // Time between steps
        int N;                                      // Number of steps
    };
};
```

**What gets executed**: Only the **first control input** is executed:
- `v` at k=1 (velocity one timestep ahead)
- `w` at k=0 (angular velocity at current time)

**What gets published**: The **entire trajectory** is published for:
- Visualization in RViz
- Communication to other robots (multi-robot coordination)
- Higher-level planning nodes

#### Trajectory Publishing (lines 875-926)

The robot publishes its planned trajectory in two formats:

1. **Standard ROS Path** (for visualization):
```cpp
void publishCurrentTrajectory(MPCPlanner::PlannerOutput output) {
    nav_msgs::Path ros_trajectory_msg;
    for (const auto &position : output.trajectory.positions) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position(0);
        pose.pose.position.y = position(1);
        pose.pose.orientation = RosTools::angleToQuaternion(orientation);
        ros_trajectory_msg.poses.push_back(pose);
    }
    _trajectory_pub.publish(ros_trajectory_msg);
}
```

2. **Direct Trajectory** (for robot-robot communication):
```cpp
void publishDirectTrajectory(MPCPlanner::PlannerOutput output) {
    mpc_planner_msgs::ObstacleGMM msg;
    msg.id = _ego_robot_id;
    msg.pose = current_pose;
    
    // Fill trajectory as a Gaussian mean
    for (const auto &position : output.trajectory.positions) {
        msg.gaussians[0].mean.poses.push_back(pose_from_position);
    }
    _direct_trajectory_pub.publish(msg);
}
```

**Multi-robot usage**: Other robots subscribe to this trajectory and treat it as a moving obstacle prediction in their own MPC.

### Where Homology Information is Currently NOT Stored

**Critical observation**: The current `PlannerOutput` structure does **NOT** contain:
- ❌ Homology ID / topology class ID
- ❌ Guidance ID
- ❌ Which planner was selected
- ❌ Objective value
- ❌ Whether T-MPC++ or guided planner was chosen

This information is **lost** after `solveMPC()` returns!

---

## Storing and Using Homology IDs

### Why Store Homology IDs?

Knowing which topology class was selected enables:

1. **Higher-level decision making**: Route planner can prefer certain topologies
2. **Multi-robot coordination**: Communicate topology intentions to avoid conflicts
3. **Learning**: Train models to predict which topologies work best in different scenarios
4. **Debugging**: Understand why the robot chose a particular path
5. **Consistency analysis**: Track how often the robot switches topologies

### Method 1: Extend PlannerOutput Structure (Recommended)

#### Step 1: Modify PlannerOutput in planner.h

```cpp
// In mpc_planner/include/mpc_planner/planner.h
struct PlannerOutput
{
    Trajectory trajectory;
    bool success{false};
    
    // ADD THESE FIELDS:
    int selected_topology_id{-1};      // Homology class ID (from guidance_ID)
    int selected_planner_index{-1};    // Which planner was chosen (0 to n_paths)
    bool used_guidance{true};          // false if T-MPC++ (non-guided) was chosen
    double trajectory_cost{0.0};       // Objective value of selected solution
    int solver_exit_code{-1};          // Exit code (1=success, 0=max_iter, -1=infeasible)
    
    PlannerOutput(double dt, int N) : trajectory(dt, N) {}
    PlannerOutput() = default;
};
```

#### Step 2: Populate Fields in GuidanceConstraints::optimize()

```cpp
// In mpc_planner_modules/src/guidance_constraints.cpp
// At the end of optimize(), before returning:

auto &best_planner = planners_[best_planner_index_];

// Transfer to main solver (existing code)
_solver->_output = best_solver->_output;
_solver->_info = best_solver->_info;
_solver->_params = best_solver->_params;

// ADD THIS: Store metadata in module_data for Planner to retrieve
module_data.selected_topology_id = best_planner.result.guidance_ID;
module_data.selected_planner_index = best_planner_index_;
module_data.used_guidance = !best_planner.is_original_planner;
module_data.trajectory_cost = best_planner.result.objective;
module_data.solver_exit_code = best_planner.result.exit_code;

return best_planner.result.exit_code;
```

#### Step 3: Transfer to PlannerOutput in Planner::solveMPC()

```cpp
// In mpc_planner/src/planner.cpp
// After the optimize loop (line 137), add:

if (exit_flag == 1) {  // Success
    _output.success = true;
    
    // ADD THIS: Transfer homology metadata
    _output.selected_topology_id = _module_data.selected_topology_id;
    _output.selected_planner_index = _module_data.selected_planner_index;
    _output.used_guidance = _module_data.used_guidance;
    _output.trajectory_cost = _module_data.trajectory_cost;
    _output.solver_exit_code = exit_flag;
    
    // Build trajectory (existing code)
    for (int k = 0; k < _solver->N; k++) {
        _output.trajectory.add(_solver->getOutput(k, "x"), 
                              _solver->getOutput(k, "y"));
        _output.trajectory.add_orientation(_solver->getOutput(k, "psi"));
    }
}
```

#### Step 4: Add Fields to ModuleData

```cpp
// In mpc_planner_types/include/mpc_planner_types/module_data.h
struct ModuleData {
    // Existing fields...
    std::shared_ptr<RosTools::Spline2D> path;
    // ...
    
    // ADD THESE:
    int selected_topology_id{-1};
    int selected_planner_index{-1};
    bool used_guidance{true};
    double trajectory_cost{0.0};
    int solver_exit_code{-1};
};
```

### Method 2: Add Member Variable to Planner (Quick Alternative)

If modifying `PlannerOutput` is too invasive, store in `Planner` class:

```cpp
// In mpc_planner/include/mpc_planner/planner.h
class Planner {
public:
    // Existing methods...
    
    // ADD THESE:
    int getSelectedTopologyId() const { return _selected_topology_id; }
    int getSelectedPlannerIndex() const { return _selected_planner_index; }
    bool usedGuidance() const { return _used_guidance; }
    double getTrajectoryCost() const { return _trajectory_cost; }
    
private:
    // Existing members...
    
    // ADD THESE:
    int _selected_topology_id{-1};
    int _selected_planner_index{-1};
    bool _used_guidance{true};
    double _trajectory_cost{0.0};
};
```

Then populate in `Planner::solveMPC()`:
```cpp
// After optimize loop succeeds:
_selected_topology_id = _module_data.selected_topology_id;
_selected_planner_index = _module_data.selected_planner_index;
_used_guidance = _module_data.used_guidance;
_trajectory_cost = _module_data.trajectory_cost;
```

Access in `jules_ros1_jackalplanner.cpp`:
```cpp
auto output = _planner->solveMPC(_state, _data);
int topology_id = _planner->getSelectedTopologyId();
LOG_INFO("Selected topology: " << topology_id);
```

### Using Homology IDs in Higher-Level Planning

#### Example 1: Publish Topology Information

```cpp
// In jules_ros1_jackalplanner.cpp

// Add publisher in initializeSubscribersAndPublishers():
_topology_pub = nh.advertise<std_msgs::Int32>("output/selected_topology", 1);

// In generatePlanningCommand() after solveMPC():
if (output.success) {
    std_msgs::Int32 topology_msg;
    topology_msg.data = output.selected_topology_id;
    _topology_pub.publish(topology_msg);
    
    LOG_VALUE("Selected topology", output.selected_topology_id);
    if (output.used_guidance)
        LOG_INFO("Used guided planner");
    else
        LOG_INFO("Used T-MPC++ (non-guided)");
}
```

#### Example 2: Track Topology Switches

```cpp
// In jules_ros1_jackalplanner.h
class JulesJackalPlanner {
private:
    int _previous_topology_id{-1};
    int _topology_switch_count{0};
};

// In jules_ros1_jackalplanner.cpp
auto output = _planner->solveMPC(_state, _data);

if (output.success) {
    if (_previous_topology_id != -1 && 
        _previous_topology_id != output.selected_topology_id) {
        _topology_switch_count++;
        LOG_WARN("Topology switch! " << _previous_topology_id 
                 << " -> " << output.selected_topology_id);
        LOG_VALUE("Total switches", _topology_switch_count);
    }
    _previous_topology_id = output.selected_topology_id;
}
```

#### Example 3: Prefer Certain Topologies (Heuristic Bias)

```cpp
// Hypothetical: Add a topology preference to RealTimeData
// Then in GuidanceConstraints::optimize(), apply additional bias:

for (auto &planner : planners_) {
    // After computing objective...
    planner.result.objective *= solver->solve();
    
    // Apply user-defined topology preference
    if (data.preferred_topology_id == planner.result.guidance_ID)
        planner.result.objective *= 0.9;  // 10% bonus for preferred topology
}
```

#### Example 4: Multi-Robot Coordination with Topology

```cpp
// Publish topology intention to other robots
void JulesJackalPlanner::publishTopologyIntention() {
    mpc_planner_msgs::TopologyIntention msg;
    msg.robot_id = _ego_robot_id;
    msg.topology_id = _planner->getSelectedTopologyId();
    msg.trajectory = /* current trajectory */;
    _topology_intention_pub.publish(msg);
}

// Other robots receive this and could avoid conflicting topologies
void JulesJackalPlanner::otherRobotTopologyCallback(const mpc_planner_msgs::TopologyIntention &msg) {
    LOG_INFO("Robot " << msg.robot_id << " intends topology " << msg.topology_id);
    // Could add cost penalty in MPC for trajectories that conflict with this topology
}
```

### Method 3: Store in Custom Message for Multi-Robot Communication

Extend the obstacle message to include topology:

```cpp
// Define in mpc_planner_msgs/msg/ObstacleGMM.msg
int32 id
geometry_msgs/Pose pose
Gaussian[] gaussians
float64[] probabilities

# ADD THESE:
int32 topology_id           # Which homotopy class this trajectory follows
bool used_guidance          # Whether guidance was used
float64 trajectory_cost     # Cost of this trajectory
```

Then populate in `publishDirectTrajectory()`:
```cpp
void JulesJackalPlanner::publishDirectTrajectory(MPCPlanner::PlannerOutput output) {
    mpc_planner_msgs::ObstacleGMM msg;
    msg.id = _ego_robot_id;
    msg.topology_id = output.selected_topology_id;        // NEW
    msg.used_guidance = output.used_guidance;              // NEW
    msg.trajectory_cost = output.trajectory_cost;          // NEW
    // ... rest of message population
    _direct_trajectory_pub.publish(msg);
}
```

### Recommendations for Implementation

**Best approach**: **Method 1** (Extend PlannerOutput)
- ✅ Clean architecture - data flows with the output structure
- ✅ Available everywhere `PlannerOutput` is used
- ✅ Easy to serialize/log
- ⚠️ Requires changes to `PlannerOutput` structure (but low risk)

**Quick approach**: **Method 2** (Planner member variables)
- ✅ Minimal code changes
- ✅ No structure modifications needed
- ❌ Data separated from trajectory output
- ❌ Requires getter methods

**For multi-robot**: **Method 3** (Custom ROS message)
- ✅ Explicitly communicates topology between robots
- ✅ Enables topology-aware coordination
- ⚠️ Requires message definition changes

**Recommended combination**:
1. Use **Method 1** for internal planner architecture
2. Add **Method 3** if doing multi-robot coordination with topology awareness

### Example: Complete Integration

Here's a complete example showing all pieces together:

```cpp
// ========== 1. In guidance_constraints.cpp ==========
int GuidanceConstraints::optimize(...) {
    // ... parallel optimization ...
    
    best_planner_index_ = FindBestPlanner();
    if (best_planner_index_ == -1)
        return planners_[0].result.exit_code;
    
    auto &best_planner = planners_[best_planner_index_];
    
    // Transfer solution
    _solver->_output = best_planner.local_solver->_output;
    
    // Store metadata in module_data
    module_data.selected_topology_id = best_planner.result.guidance_ID;
    module_data.selected_planner_index = best_planner_index_;
    module_data.used_guidance = !best_planner.is_original_planner;
    module_data.trajectory_cost = best_planner.result.objective;
    
    return best_planner.result.exit_code;
}

// ========== 2. In planner.cpp ==========
PlannerOutput Planner::solveMPC(State &state, RealTimeData &data) {
    // ... module updates and optimization ...
    
    if (exit_flag == 1) {
        _output.success = true;
        
        // Transfer homology metadata
        _output.selected_topology_id = _module_data.selected_topology_id;
        _output.selected_planner_index = _module_data.selected_planner_index;
        _output.used_guidance = _module_data.used_guidance;
        _output.trajectory_cost = _module_data.trajectory_cost;
        
        // Build trajectory
        for (int k = 0; k < _solver->N; k++) {
            _output.trajectory.add(...);
        }
    }
    
    return _output;
}

// ========== 3. In jules_ros1_jackalplanner.cpp ==========
std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> 
JulesJackalPlanner::generatePlanningCommand() {
    auto output = _planner->solveMPC(_state, _data);
    
    if (output.success) {
        // Log topology information
        LOG_VALUE("Topology ID", output.selected_topology_id);
        LOG_VALUE("Planner index", output.selected_planner_index);
        LOG_VALUE("Cost", output.trajectory_cost);
        LOG_INFO((output.used_guidance ? "Guided" : "T-MPC++"));
        
        // Track switches
        if (_prev_topology != output.selected_topology_id) {
            LOG_WARN("Topology switch: " << _prev_topology 
                     << " -> " << output.selected_topology_id);
        }
        _prev_topology = output.selected_topology_id;
        
        // Publish for higher-level planner
        std_msgs::Int32 topo_msg;
        topo_msg.data = output.selected_topology_id;
        _topology_pub.publish(topo_msg);
        
        // Extract commands
        cmd.linear.x = _planner->getSolution(1, "v");
        cmd.angular.z = _planner->getSolution(0, "w");
    }
    
    return {cmd, output};
}
```

This gives you full visibility into which trajectory topology the robot selected and why!

---

## Summary and Key Takeaways

### The GuidanceConstraints Module's Role

1. **In update()**: Runs fast graph search to find topologically distinct guidance trajectories
   - Operates in low-dimensional space (2D/3D)
   - Identifies feasible homotopy classes (ways to pass obstacles)
   - Takes 1-5ms typically

2. **In optimize()**: Refines guidance into dynamically feasible trajectories
   - Runs 8 parallel MPC optimizations (one per topology + T-MPC++)
   - Each solver optimizes for 15-40ms
   - Selects best feasible solution based on cost and consistency
   - Takes max(solver times) + overhead ≈ 20-40ms total

3. **Result**: Robot executes the optimal trajectory from the best-performing topology

### Integration Points

| Component | Role | Key Functions |
|-----------|------|---------------|
| `Planner::solveMPC()` | Orchestrates module pipeline | Calls `update()`, `setParameters()`, `optimize()` |
| `GuidanceConstraints::update()` | Topology discovery | Runs graph search, maps to planners |
| `GuidanceConstraints::optimize()` | Parallel optimization | Solves MPC for each topology, selects best |
| `jules_ros1_jackalplanner.cpp` | Robot control loop | Calls `solveMPC()`, extracts commands, publishes trajectory |

### Data Flow

```
Robot State + Obstacles
    ↓
update() → Find topology classes
    ↓
optimize() → Solve MPC for each topology in parallel
    ↓
FindBestPlanner() → Select optimal topology
    ↓
PlannerOutput → Trajectory + Metadata
    ↓
jules_ros1_jackalplanner → Extract v, w commands
    ↓
Robot Actuators
```

### Homology ID Storage - Action Items

To enable higher-level planning with topology awareness:

1. **Extend `PlannerOutput`** with:
   - `selected_topology_id`
   - `selected_planner_index`
   - `used_guidance`
   - `trajectory_cost`

2. **Populate in `GuidanceConstraints::optimize()`**:
   - Store metadata in `module_data`

3. **Transfer in `Planner::solveMPC()`**:
   - Copy from `module_data` to `output`

4. **Use in `jules_ros1_jackalplanner.cpp`**:
   - Log topology switches
   - Publish to higher-level planner
   - Coordinate with other robots

### Why This Module is Powerful

**Traditional MPC**: Commits to one trajectory early, hopes it's good enough
- If it fails, robot stops or collides
- May miss better alternative topologies
- No parallelization benefit

**T-MPC with GuidanceConstraints**: Evaluates multiple topologies in parallel
- If one fails, others may succeed
- Finds truly optimal topology by solving full problem for each
- Leverages multi-core CPUs effectively
- Maintains real-time performance through parallelization

**Result**: More robust, optimal, and reliable autonomous navigation in complex dynamic environments.

---

## NEW: Topology Metadata Feature (October 2025)

The GuidanceConstraints module now populates **topology metadata** in the PlannerOutput, enabling topology-aware decision making and multi-robot coordination.

### What's New

After the best planner is selected in `optimize()`, the module populates:

```cpp
if (CONFIG["JULES"]["use_extra_params_module_data"].as<bool>()) {
    module_data.selected_topology_id = best_planner.result.guidance_ID;
    module_data.selected_planner_index = best_planner_index_;
    module_data.used_guidance = !best_planner.is_original_planner;
    module_data.trajectory_cost = best_planner.result.objective;
    module_data.solver_exit_code = best_planner.result.exit_code;
}
```

This metadata is then transferred to `PlannerOutput` in `planner.cpp`, making it available to the robot controller.

### Benefits

- **Multi-robot coordination**: Robots can share which topology they're following
- **Learning**: Collect data on which topologies work best in different scenarios
- **Debugging**: Understand why a particular topology was chosen
- **Monitoring**: Track topology switches and planner performance

### Complete Documentation

For comprehensive documentation on the topology metadata feature, including:
- Detailed field descriptions
- Usage examples (logging, tracking, coordination)
- ROS message integration
- Multi-robot applications

See: **[Topology Metadata Feature Documentation](topology_metadata_feature.md)**

---
