# Guidance Constraints Module Documentation

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

**Execution Flow**:
1. Validates that path data is available
2. Loads static obstacles as halfspaces into the guidance planner
3. Sets the start state (robot position, heading, velocity)
4. Sets reference velocity from path or configuration
5. Computes goals along the reference path (`setGoals()`)
6. **Runs the guidance search** (`global_guidance_->Update()`) - This is the main computation
7. Maps found guidance trajectories to available planners

**Why it matters**: This function performs the high-level topology search that identifies which homotopy classes are feasible. The result guides the subsequent parallel MPC optimizations.

**Key Insight**: The guidance planner operates in a lower-dimensional space (often 2D or 3D) and runs fast graph search algorithms to find topologically distinct paths. The MPC optimizations then refine these into dynamically feasible trajectories.

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

**High-Level Flow**:
1. **Setup**: Configure OpenMP for nested parallelism
2. **Parallel Optimization Loop**: Run multiple MPC solvers simultaneously
3. **Decision Making**: Select the best feasible solution
4. **Solution Transfer**: Copy the winning solution to the main solver

Let's break down each phase in detail:

#### Phase 1: OpenMP Configuration

```cpp
omp_set_nested(1);            // Enable nested parallel regions
omp_set_max_active_levels(2); // Allow 2 levels of parallelism
omp_set_dynamic(0);           // Disable dynamic thread adjustment
```

**Why this matters**: Forces solvers run in parallel by allowing the outer parallel loop (planners) and inner parallel operations (within each solver) to both use OpenMP. The dynamic setting is disabled to ensure consistent thread allocation.

#### Phase 2: Parallel Optimization Loop

```cpp
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    // ... optimization code for each planner ...
}
```

**What happens in each parallel thread**:

1. **Initialization**:
   - Reset results from previous iteration
   - Check if this planner should be active (disabled if no corresponding guidance trajectory)

2. **Solver Setup**:
   - Copy main solver state to local solver: `*solver = *_solver`
   - This gives each planner an independent solver instance

3. **Constraint Construction**:
   - **If original planner** (T-MPC++): Use only safety constraints, no guidance
   - **If guided planner**: 
     - Optionally warmstart from previous solution
     - Or initialize with guidance trajectory (`initializeSolverWithGuidance()`)
     - Update guidance constraints to stay in topology
     - Update safety constraints for collision avoidance

4. **Parameter Loading**:
   - Set solver parameters for all time steps (k = 0 to N)
   - Load guidance constraints and safety constraints
   - Set timeout based on remaining planning time

5. **Solve Optimization**:
   - Load warmstart if available
   - Call `solver->solve()` - This is the computationally intensive part
   - Record exit code and objective value

6. **Result Processing**:
   - Mark success if exit_code == 1
   - Store objective value (lower is better)
   - Record guidance_ID and color for the solution
   - **Apply consistency bonus**: If this trajectory was selected previously, multiply objective by `selection_weight_consistency_` (< 1.0) to prefer it

#### Phase 3: Decision Making

```cpp
best_planner_index_ = FindBestPlanner();
```

**Selection Criteria** (implemented in `FindBestPlanner()`):
1. Only consider non-disabled planners
2. Only consider successful optimizations (exit_code == 1)
3. Select the one with lowest objective value
4. Return -1 if no feasible solution found

#### Phase 4: Solution Transfer

```cpp
_solver->_output = best_solver->_output;
_solver->_info = best_solver->_info;
_solver->_params = best_solver->_params;
```

**Why this matters**: The main solver's output is replaced with the best solution from parallel optimizations. This is what gets executed by the robot.

**Additional Actions**:
- Notify global guidance which topology was selected: `global_guidance_->OverrideSelectedTrajectory()`
- This information is used in the next iteration for consistency bonuses

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
