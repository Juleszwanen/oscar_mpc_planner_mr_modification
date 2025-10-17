# Guidance Constraints Quick Reference Guide

This is a condensed reference for developers working with the `guidance_constraints.cpp` module. For detailed explanations, see `guidance_constraints_documentation.md`.

## Quick Overview

**What**: Parallel multi-topology MPC optimization for autonomous navigation
**Why**: Find optimal paths by evaluating multiple fundamentally different trajectories simultaneously
**How**: Graph search finds topologies → Parallel MPC refines them → Select best

## Key Function Calls

### 1. update() - Called from Planner::solveMPC()

**Timing**: ~1-5ms
**Purpose**: Find guidance trajectories

```cpp
void GuidanceConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
```

**What it does**:
1. Load static obstacles as halfspaces
2. Set start state and reference velocity
3. Generate spatial goal grid along path
4. **Run guidance search** (`global_guidance_->Update()`)
5. Map found trajectories to planners

**Output**: `n_paths_` guidance trajectories (one per homotopy class)

### 2. optimize() - Called from Planner::solveMPC()

**Timing**: ~20-40ms (with parallelization)
**Purpose**: Solve MPC for each topology and select best

```cpp
int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
```

**What it does**:
1. Setup OpenMP for nested parallelism
2. **Parallel loop** (8 threads):
   - Copy main solver state
   - Initialize with guidance or previous solution
   - Update constraints (guidance + safety)
   - Load parameters for all timesteps
   - **Solve MPC** (15-40ms per solver)
   - Apply consistency bonus if previously selected
3. **Select best** feasible solution (lowest cost)
4. Transfer winning solution to main solver

**Output**: Exit code (1=success), updates `_solver` with best solution

## Call Sequence from Robot Controller

```
jules_ros1_jackalplanner.cpp:
    loopDirectTrajectory() [every 50ms @ 20Hz]
        ↓
    generatePlanningCommand()
        ↓
    _planner->solveMPC(_state, _data)
        ↓
        │
        ├─ FOR EACH MODULE:
        │  └─ GuidanceConstraints::update()
        │     └─ Find topology classes [1-5ms]
        │
        ├─ FOR EACH MODULE:
        │  └─ module->setParameters()
        │
        └─ FOR EACH MODULE:
           └─ GuidanceConstraints::optimize()
              └─ Parallel MPC [20-40ms]
                 └─ Returns exit code
        
        Returns PlannerOutput {trajectory, success, ...}
        ↓
    Extract: cmd.linear.x = getSolution(1, "v")
             cmd.angular.z = getSolution(0, "w")
        ↓
    Robot executes command
```

## Data Structures

### LocalPlanner (One per topology)
```cpp
struct LocalPlanner {
    int id;                                 // 0 to n_paths-1 (or n_paths for T-MPC++)
    std::shared_ptr<Solver> local_solver;   // Independent MPC solver
    LinearizedConstraints* guidance_constraints;  // Keep in same topology
    GUIDANCE_CONSTRAINTS_TYPE* safety_constraints; // Avoid collisions
    SolverResult result;                    // Exit code, objective, guidance_ID
    bool is_original_planner;               // true for T-MPC++
};
```

### SolverResult
```cpp
struct SolverResult {
    int exit_code;        // 1=success, 0=max_iter, -1=infeasible
    double objective;     // Cost (lower is better)
    bool success;         // exit_code == 1
    int guidance_ID;      // Topology class ID
    int color;            // Visualization color
};
```

### PlannerOutput
```cpp
struct PlannerOutput {
    Trajectory trajectory;  // Positions, orientations, dt, N
    bool success;
    
    // RECOMMENDED TO ADD:
    int selected_topology_id;
    int selected_planner_index;
    bool used_guidance;
    double trajectory_cost;
};
```

## Configuration Parameters

**File**: `config/settings.yaml`

```yaml
t-mpc:
  use_t-mpc++: true                    # Enable non-guided fallback planner
  enable_constraints: true              # Use guidance constraints (vs just warmstart)
  warmstart_with_mpc_solution: true    # Use previous solution vs guidance
  selection_weight_consistency_: 0.8   # Consistency bonus multiplier

guidance:
  n_paths_: 3                          # Number of topology classes to explore
  longitudinal_goals_: 5               # Grid points along path
  vertical_goals_: 5                   # Grid points perpendicular to path

control_frequency: 20.0                # Hz
N: 20                                  # MPC horizon steps
integrator_step: 0.1                   # Time between steps (seconds)
```

## Performance Metrics

| Metric | Typical Value | Notes |
|--------|---------------|-------|
| Guidance search | 1-5ms | Fast graph search |
| Single MPC solve | 15-40ms | Depends on problem complexity |
| Parallel optimization | 20-45ms | max(all solves) + overhead |
| Total planning | 25-50ms | Fits in 50ms @ 20Hz |
| Speedup from parallelization | 6-7x | With 8 threads |

## Common Issues & Solutions

### Issue: All solvers fail
**Symptom**: `FindBestPlanner()` returns -1
**Solutions**:
- Enable T-MPC++ (`use_t-mpc++: true`)
- Increase solver timeout
- Check obstacle predictions
- Verify guidance found feasible paths

### Issue: Always selects T-MPC++ (non-guided)
**Symptom**: Best planner has `is_original_planner = true`
**Solutions**:
- Verify `enable_constraints: true`
- Check guidance trajectory quality (visualize)
- Review consistency weight (may over-prefer previous)

### Issue: Frequent topology switching
**Symptom**: Topology changes every timestep
**Solutions**:
- Lower `selection_weight_consistency_` (e.g., 0.7-0.85)
- Add topology switch penalty
- Improve guidance planner stability

### Issue: Real-time deadline misses
**Symptom**: Planning takes >50ms @ 20Hz
**Solutions**:
- Reduce `n_paths_` (fewer topologies)
- Reduce horizon `N`
- Increase `integrator_step` (larger dt)
- Use more CPU cores

## Code Snippets

### Store Homology ID (Method 1: Extend PlannerOutput)

**1. In guidance_constraints.cpp:**
```cpp
// At end of optimize():
module_data.selected_topology_id = best_planner.result.guidance_ID;
module_data.used_guidance = !best_planner.is_original_planner;
module_data.trajectory_cost = best_planner.result.objective;
```

**2. In planner.cpp:**
```cpp
if (exit_flag == 1) {
    _output.selected_topology_id = _module_data.selected_topology_id;
    _output.used_guidance = _module_data.used_guidance;
    _output.trajectory_cost = _module_data.trajectory_cost;
}
```

**3. In jules_ros1_jackalplanner.cpp:**
```cpp
auto output = _planner->solveMPC(_state, _data);
if (output.success) {
    LOG_INFO("Selected topology: " << output.selected_topology_id);
    if (output.used_guidance)
        LOG_INFO("Used guided planner");
    else
        LOG_INFO("Used T-MPC++");
}
```

### Track Topology Switches

```cpp
// In jules_ros1_jackalplanner.h:
int _previous_topology_id{-1};

// In generatePlanningCommand():
if (output.success) {
    if (_previous_topology_id != -1 && 
        _previous_topology_id != output.selected_topology_id) {
        LOG_WARN("Topology switch: " << _previous_topology_id 
                 << " -> " << output.selected_topology_id);
    }
    _previous_topology_id = output.selected_topology_id;
}
```

### Publish Topology for Higher-Level Planner

```cpp
// In jules_ros1_jackalplanner.cpp:

// Add publisher:
_topology_pub = nh.advertise<std_msgs::Int32>("output/selected_topology", 1);

// Publish after solveMPC:
if (output.success) {
    std_msgs::Int32 msg;
    msg.data = output.selected_topology_id;
    _topology_pub.publish(msg);
}
```

## Topology-Aware Communication (Multi-Robot)

### Overview
**Feature**: Only communicate trajectory updates when topology changes (not every iteration)
**Benefit**: 60-80% reduction in communication messages
**Safety**: Always communicates on failure or behavior changes

### Enable in Configuration

```yaml
# In settings.yaml:
JULES:
    use_extra_params_module_data: true  # Required for topology tracking
    communicate_on_topology_switch_only: true  # Enable conditional communication
    n_paths: 4  # Number of guided planners
```

### Communication Decision Logic

| Scenario | Communicate? | Reason |
|----------|--------------|--------|
| MPC failed | ✅ Yes | Safety critical |
| Topology switch detected | ✅ Yes | Behavior changed |
| Switched to non-guided | ✅ Yes | Unpredictable behavior |
| Staying in non-guided | ✅ Yes | Always unpredictable |
| Same guided topology | ❌ No | Predictable behavior |

### Implementation Example

The `publishCmdAndVisualize()` function handles conditional communication:

```cpp
void JulesJackalPlanner::publishCmdAndVisualize(const geometry_msgs::Twist &cmd, 
                                                const MPCPlanner::PlannerOutput &output)
{
    bool should_communicate = true;
    
    if (_communicate_on_topology_switch_only)
    {
        const int non_guided_topology_id = 2 * CONFIG["JULES"]["n_paths"].as<int>();
        
        if (!output.success) {
            should_communicate = true;  // Always on failure
        }
        else if (output.following_new_topology) {
            should_communicate = true;  // Topology switched
        }
        else if (output.selected_topology_id == non_guided_topology_id) {
            should_communicate = true;  // In non-guided mode
        }
        else {
            should_communicate = false; // Same guided topology
        }
    }
    
    _cmd_pub.publish(cmd);  // Always publish velocity command
    
    if (should_communicate) {
        this->publishDirectTrajectory(output);     // Conditional
        this->publishCurrentTrajectory(output);    // Conditional
    }
    
    _planner->visualize(_state, _data);  // Always visualize
}
```

### New PlannerOutput Fields

```cpp
struct PlannerOutput {
    // Existing fields...
    Trajectory trajectory;
    bool success;
    
    // New topology tracking fields:
    int selected_topology_id{-1};       // Current topology ID
    int previous_topology_id{-1};       // Previous topology ID
    bool following_new_topology{true};  // Did topology switch?
    int selected_planner_index{-1};     // Which planner (0 to n_paths)
    bool used_guidance{true};           // false if non-guided
    double trajectory_cost{0.0};        // Objective value
    int solver_exit_code{-1};           // Solver result
};
```

### Topology ID Convention

- Guided planners: `0` to `n_paths - 1` (e.g., 0, 1, 2, 3 for n_paths=4)
- Non-guided planner: `2 * n_paths` (e.g., 8 for n_paths=4)
- Invalid/None: `-1`

### Monitor Communication Activity

```bash
# Watch trajectory publications (should be sparse with feature enabled)
rostopic hz /jackal1/direct_trajectory

# View communication decisions in logs
rqt_console  # Filter for "Communicating" or "NOT communicating"

# Compare bandwidth
# Without feature: ~15-20 messages/sec
# With feature: ~2-5 messages/sec (typical)
```

### Debugging Commands

### Enable verbose logging
```yaml
# In settings.yaml:
debug_output: true
debug_visuals: true
```

### Visualize in RViz
Subscribe to:
- `/guidance_constraints/optimized_trajectories` - All computed paths (color-coded)
- `/guidance_constraints/warmstart_trajectories` - Initial guesses
- `/planned_trajectory` - Selected trajectory

### Check computation time
```cpp
// In your code:
#include <ros_tools/profiling.h>

PROFILE_SCOPE("MyFunction");
// ... your code ...
// Time will be logged automatically
```

## Key Insights

1. **Parallelization is critical**: Sequential execution would take ~240ms (8 solvers × 30ms), parallel takes ~37ms
2. **Consistency bonus prevents oscillation**: Without it, robot switches topologies erratically
3. **T-MPC++ is a safety net**: When guidance fails, non-guided planner provides fallback
4. **Guidance is fast**: Graph search in 2D/3D space much faster than full MPC
5. **Each planner is independent**: True parallelism with no locks or synchronization

## Related Files

| File | Purpose |
|------|---------|
| `mpc_planner_modules/src/guidance_constraints.cpp` | Main implementation |
| `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h` | Header |
| `mpc_planner/src/planner.cpp` | Orchestrates module pipeline |
| `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp` | Robot controller |
| `mpc_planner_modules/include/mpc_planner_modules/modules.h` | Module initialization |
| `config/settings.yaml` | Configuration |
| `docs/guidance_constraints_documentation.md` | **Full detailed documentation** |

## Next Steps

1. **Read full documentation**: See `guidance_constraints_documentation.md` for in-depth explanations
2. **Modify PlannerOutput**: Add homology ID fields (see detailed guide in main doc)
3. **Integrate with your planner**: Use code examples above
4. **Test and tune**: Adjust `selection_weight_consistency_` and `n_paths_` for your application
5. **Visualize**: Enable RViz visualization to understand topology selection

---

**For comprehensive explanations, code walkthroughs, and advanced topics, see the full documentation in `guidance_constraints_documentation.md`**
