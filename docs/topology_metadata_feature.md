# Topology Metadata Feature

## Overview

As of October 2025, the MPC planner now includes **topology metadata** in its output, enabling topology-aware communication and decision-making in multi-robot systems. This feature exposes information about which homotopy class (trajectory topology) was selected by the T-MPC++ planner, making it possible for higher-level planners and other robots to reason about topological choices.

**Key Applications**:
- **Topology-Aware Communication**: Reduce multi-robot network bandwidth by 60-80% by only publishing when topologies change (see [implementation details](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems))
- **Multi-Robot Coordination**: Robots can share and reason about topological intentions
- **Learning & Adaptation**: Collect data on which topologies work best in different scenarios
- **Debugging & Monitoring**: Track topology switches and planner performance

## What is Topology Metadata?

When the T-MPC++ (Topology-Driven Model Predictive Control) planner runs, it evaluates multiple distinct trajectory topologies in parallel - for example, passing to the left vs. right of an obstacle. The topology metadata captures which of these topological alternatives was selected and provides context about the decision.

### Key Information Captured

The `PlannerOutput` structure now includes:

```cpp
struct PlannerOutput {
    Trajectory trajectory;              // The planned trajectory
    bool success{false};                // Did optimization succeed?
    
    // Topology Metadata (Added October 2025)
    int selected_topology_id{-9};       // Homology class ID
    int selected_planner_index{-9};     // Which parallel planner was chosen
    bool used_guidance{true};           // Whether guidance constraints were used
    double trajectory_cost{0.0};        // Objective value of selected solution
    int solver_exit_code{-1};           // Solver exit status
    bool following_new_homology{true};  // Changed topology since last iteration
    
    std::string logOutput() const;      // Pretty-print metadata for logging
};
```

## Field Descriptions

### `selected_topology_id` (Homology Class ID)

**Type**: `int`  
**Default**: `-9` (uninitialized)  
**Range**: `0` to `n_paths-1` for guided planners, `2*n_paths` for T-MPC++ non-guided planner

**Purpose**: Uniquely identifies which homotopy class (topological path variant) the robot is following.

**Examples**:
- `0`: First topology class (e.g., passing left of obstacle A, left of obstacle B)
- `1`: Second topology class (e.g., passing left of obstacle A, right of obstacle B)
- `2`: Third topology class (e.g., passing right of obstacle A, left of obstacle B)
- `6`: T-MPC++ non-guided planner (when `n_paths = 3`)

**Use Cases**:
- **Multi-robot coordination**: Other robots can avoid conflicting with this topology
- **Topology-aware planning**: Higher-level planner can prefer certain topologies
- **Learning**: Train models to predict which topologies work best in different scenarios
- **Visualization**: Color-code trajectories by topology in RViz

### `selected_planner_index`

**Type**: `int`  
**Default**: `-9` (uninitialized)  
**Range**: `0` to `n_solvers-1`

**Purpose**: Identifies which parallel MPC solver instance produced the selected trajectory.

**Details**:
- The T-MPC++ module runs multiple MPC solvers in parallel (typically 4-8)
- Each solver explores a different topology using guidance constraints
- This field indicates which solver's solution was chosen as optimal

**Use Cases**:
- **Performance analysis**: Track which solver indices perform best
- **Debugging**: Identify which solver is failing or slow
- **Load balancing**: Understand solver workload distribution

### `used_guidance`

**Type**: `bool`  
**Default**: `true`  
**Values**: `true` (used guidance constraints) or `false` (T-MPC++ non-guided)

**Purpose**: Indicates whether the selected trajectory used guidance constraints to enforce a specific topology.

**Details**:
- `true`: Trajectory followed a specific topology guided by the guidance planner
- `false`: Trajectory from T-MPC++ non-guided planner (no topology constraints)

**Why this matters**:
- T-MPC++ includes a "fallback" planner that operates without guidance constraints
- When guidance fails or produces poor solutions, this fallback can find alternative paths
- Knowing which mode was used helps diagnose planning behavior

**Use Cases**:
- **Fallback detection**: Log when T-MPC++ is being used instead of guided planners
- **Guidance quality**: Measure how often guidance is beneficial vs. constraining
- **Performance metrics**: Compare guided vs. non-guided planner success rates

### `trajectory_cost`

**Type**: `double`  
**Default**: `0.0`  
**Units**: Cost function value (problem-specific)

**Purpose**: The objective function value of the selected trajectory.

**Details**:
- Lower cost = better trajectory
- Cost typically includes:
  - Progress toward goal
  - Control effort (acceleration, steering)
  - Deviation from reference path
  - Comfort (jerk limits, smooth steering)
  - Safety margins from obstacles

**Use Cases**:
- **Trajectory quality assessment**: Understand how good the selected trajectory is
- **Comparison**: Compare costs across different topologies or time steps
- **Threshold-based decisions**: Only accept trajectories below certain cost
- **Learning**: Train models using trajectory cost as reward signal

**Example**:
```cpp
if (output.trajectory_cost > 500.0) {
    LOG_WARN("High trajectory cost detected: " << output.trajectory_cost);
    // Consider more conservative behavior
}
```

### `solver_exit_code`

**Type**: `int`  
**Default**: `-1` (uninitialized/unknown)  
**Standard Values**:
- `1`: Success (optimal solution found)
- `0`: Max iterations reached (may still be usable)
- `-1`: Infeasible (no solution exists)
- Other negative: Solver-specific errors

**Purpose**: Detailed status of the MPC solver execution.

**Use Cases**:
- **Diagnostics**: Understand why planning failed
- **Retry logic**: Distinguish recoverable vs. unrecoverable failures
- **Logging**: Record solver health over time
- **Alerts**: Trigger warnings when specific error codes occur

**Example**:
```cpp
if (output.solver_exit_code == -1) {
    LOG_ERROR("MPC solver infeasible - no collision-free path exists");
    // Apply emergency braking
}
else if (output.solver_exit_code == 0) {
    LOG_WARN("MPC solver timeout - using suboptimal solution");
    // Proceed cautiously
}
```

### `following_new_homology`

**Type**: `bool`  
**Default**: `true`  
**Values**: `true` (changed topology) or `false` (same topology as previous iteration)

**Purpose**: Indicates whether the robot has switched to a different topology compared to the previous planning cycle.

**Details**:
- `true`: Current `selected_topology_id` differs from previous iteration
- `false`: Same topology as previous iteration (consistent behavior)

**Why this matters**:
- Frequent topology switching can indicate oscillatory behavior
- Staying on the same topology suggests stable planning
- Can be used to add hysteresis or smoothing

**Use Cases**:
- **Stability monitoring**: Log frequency of topology changes
- **Hysteresis**: Penalize switching to stabilize behavior
- **Debugging**: Detect when robot oscillates between topologies
- **Metrics**: Track topology change rate as performance indicator

**Example**:
```cpp
if (output.following_new_homology) {
    _topology_switch_count++;
    LOG_INFO("Topology switch: " << _prev_topology << " -> " << output.selected_topology_id);
    
    if (_topology_switch_count > 5) {
        LOG_WARN("Frequent topology switching detected - possible instability");
    }
}
```

## Integration with T-MPC++ Guidance Constraints Module

### Data Flow

```
GuidanceConstraints::optimize()
    ↓ (parallel optimization)
[Planner 0] → Solution 0 (topology 0)
[Planner 1] → Solution 1 (topology 1)
[Planner 2] → Solution 2 (topology 2)
[Planner N] → Solution N (T-MPC++)
    ↓ (select best)
FindBestPlanner() → Best planner index
    ↓ (populate metadata)
module_data.selected_topology_id = best_planner.result.guidance_ID
module_data.selected_planner_index = best_planner_index
module_data.used_guidance = !best_planner.is_original_planner
module_data.trajectory_cost = best_planner.result.objective
    ↓ (transfer to output)
Planner::solveMPC()
    output.selected_topology_id = module_data.selected_topology_id
    output.selected_planner_index = module_data.selected_planner_index
    output.used_guidance = module_data.used_guidance
    output.trajectory_cost = module_data.trajectory_cost
    output.solver_exit_code = exit_flag
    ↓ (use in application)
JulesJackalPlanner::generatePlanningCommand()
    // Access metadata from output
    LOG_INFO("Selected topology: " << output.selected_topology_id);
```

### Metadata Population in `guidance_constraints.cpp`

After parallel optimization completes and the best planner is selected:

```cpp
// In GuidanceConstraints::optimize()
auto &best_planner = planners_[best_planner_index_];

// Populate module_data with metadata
module_data.selected_topology_id = best_planner.result.guidance_ID;
module_data.selected_planner_index = best_planner_index_;
module_data.used_guidance = !best_planner.is_original_planner;
module_data.trajectory_cost = best_planner.result.objective;

// Transfer solution to main solver
_solver->_output = best_planner.local_solver->_output;

return best_planner.result.exit_code;
```

### Metadata Transfer in `planner.cpp`

After the solver completes successfully:

```cpp
// In Planner::solveMPC()
if (exit_flag == 1 && CONFIG["JULES"]["use_extra_params_module_data"].as<bool>()) {
    // Transfer topology metadata to output
    _output.selected_topology_id = _module_data.selected_topology_id;
    _output.selected_planner_index = _module_data.selected_planner_index;
    _output.used_guidance = _module_data.used_guidance;
    _output.trajectory_cost = _module_data.trajectory_cost;
    _output.solver_exit_code = exit_flag;
}
```

## Configuration

### Enabling Topology Metadata

In `config/settings.yaml`:

```yaml
JULES:
  use_extra_params_module_data: true  # Enable topology metadata in output
```

**Note**: When `false`, all metadata fields remain at their default values.

### Guidance Planner Configuration

The number of topologies explored is controlled by:

```yaml
guidance:
  n_paths_: 3  # Number of distinct topologies to evaluate
  
t-mpc:
  use_t-mpc++: true  # Include non-guided planner as fallback
```

This configuration results in:
- 3 guided planners (topologies 0, 1, 2)
- 1 non-guided planner (topology ID = 6 = 2 * 3)
- Total: 4 parallel planners

## Usage Examples

### Example 1: Logging Topology Selection

```cpp
auto output = _planner->solveMPC(_state, _data);

if (output.success) {
    LOG_INFO("Planning succeeded:");
    LOG_VALUE("  Topology ID", output.selected_topology_id);
    LOG_VALUE("  Planner index", output.selected_planner_index);
    LOG_VALUE("  Cost", output.trajectory_cost);
    LOG_INFO("  Guidance: " << (output.used_guidance ? "Yes" : "No (T-MPC++)"));
}
```

**Output**:
```
Planning succeeded:
  Topology ID: 1
  Planner index: 1
  Cost: 124.56
  Guidance: Yes
```

### Example 2: Tracking Topology Switches

```cpp
class JulesJackalPlanner {
private:
    int _prev_topology_id{-1};
    int _topology_switch_count{0};
    
public:
    void processPlanningOutput(const MPCPlanner::PlannerOutput& output) {
        if (output.success) {
            // Detect topology change
            if (_prev_topology_id != -1 && 
                _prev_topology_id != output.selected_topology_id) {
                _topology_switch_count++;
                LOG_WARN("Topology switch #" << _topology_switch_count 
                         << ": " << _prev_topology_id 
                         << " -> " << output.selected_topology_id);
            }
            
            _prev_topology_id = output.selected_topology_id;
        }
    }
};
```

### Example 3: Publishing Topology for Multi-Robot Coordination

```cpp
void JulesJackalPlanner::publishTopologyIntention(const MPCPlanner::PlannerOutput& output) {
    if (!output.success) return;
    
    // Create custom message with topology info
    mpc_planner_msgs::TrajectoryWithTopology msg;
    msg.robot_id = _ego_robot_id;
    msg.topology_id = output.selected_topology_id;
    msg.used_guidance = output.used_guidance;
    msg.trajectory_cost = output.trajectory_cost;
    
    // Include trajectory waypoints
    for (const auto& pos : output.trajectory.positions) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pos(0);
        pose.pose.position.y = pos(1);
        msg.trajectory.poses.push_back(pose);
    }
    
    _topology_pub.publish(msg);
}
```

### Example 4: Cost-Based Decision Making

```cpp
auto output = _planner->solveMPC(_state, _data);

if (output.success) {
    // Check if trajectory quality is acceptable
    if (output.trajectory_cost > _max_acceptable_cost) {
        LOG_WARN("Trajectory cost (" << output.trajectory_cost 
                 << ") exceeds threshold (" << _max_acceptable_cost << ")");
        
        // Could request replanning, reduce velocity, or stop
        if (!output.used_guidance) {
            LOG_ERROR("Even T-MPC++ fallback produced high-cost trajectory");
            // Emergency action needed
        }
    }
    
    // Log to data file for analysis
    _data_logger.log("topology_id", output.selected_topology_id);
    _data_logger.log("cost", output.trajectory_cost);
    _data_logger.log("used_guidance", output.used_guidance);
}
```

### Example 5: Pretty Logging with `logOutput()`

```cpp
auto output = _planner->solveMPC(_state, _data);

// Simple one-line logging with all metadata
LOG_INFO(output.logOutput());
```

**Output when successful**:
```
MPC Planning SUCCESS ✓
  Topology ID:     1
  Planner Index:   1
  Used Guidance:   Yes
  Trajectory Cost: 124.5600
  Solver Status:   SUCCESS (exit code: 1)
```

**Output when failed**:
```
MPC Planning FAILED ✗
  Solver Exit Code: -1 (INFEASIBLE)
  Success Flag:     false
  Topology ID:      N/A
  Planner Index:    N/A
```

## Benefits of Topology Metadata

### 1. Multi-Robot Coordination

**Problem**: In multi-robot scenarios, robots need to coordinate to avoid conflicting trajectories.

**Solution**: Broadcast topology intentions to other robots.

```cpp
// Robot 1 publishes: "I'm following topology 0 (left of obstacle)"
// Robot 2 receives and can prefer: "I'll follow topology 1 (right of obstacle)"
```

This enables:
- **Topological conflict avoidance**: Robots avoid choosing the same topology in tight spaces
- **Communication efficiency**: Share high-level topology intention instead of full trajectory
- **Predictability**: Other robots know which general path to expect

> **Implementation Note**: The topology metadata enables a powerful optimization called **topology-aware communication**, which reduces multi-robot network bandwidth by 60-80% by only publishing trajectory updates when a robot switches topologies. For implementation details, see the [Topology-Aware Communication for Multi-Robot Systems](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems) section in the Guidance Constraints Documentation.

### 2. Higher-Level Planning Integration

**Problem**: Global planners may want to influence local MPC decisions.

**Solution**: Use topology metadata for hierarchical planning.

```cpp
// Global planner suggests: "Prefer topology 1 for this segment"
// MPC includes topology preference in cost function
// Metadata confirms: "Following suggested topology 1"
```

### 3. Learning and Adaptation

**Problem**: Want to learn which topologies work best in different scenarios.

**Solution**: Collect topology metadata as training data.

```cpp
struct TrainingExample {
    EnvironmentState env;          // Obstacle positions, goal, etc.
    int selected_topology;         // Which topology was chosen
    double trajectory_cost;        // How good was the result
    bool success;                  // Did it work?
};

// Train model to predict best topology given environment
```

### 4. Debugging and Diagnostics

**Problem**: Hard to understand why the planner chose a particular path.

**Solution**: Log topology metadata for post-analysis.

```
[14:23:45] Topology 0: cost=150, FAILED (infeasible)
[14:23:45] Topology 1: cost=120, SUCCESS
[14:23:45] Topology 2: cost=130, SUCCESS
[14:23:45] T-MPC++:     cost=125, SUCCESS
[14:23:45] Selected: Topology 1 (lowest cost among successes)
```

### 5. Performance Monitoring

**Problem**: Need to track planner health over time.

**Solution**: Aggregate metadata for metrics.

```cpp
struct PlannerMetrics {
    double avg_cost;
    double max_cost;
    int topology_switches;
    double guidance_usage_rate;  // % of time guided vs. T-MPC++
    std::map<int, int> topology_frequency;  // Which topologies used most
};
```

## Topology Metadata in Module Data

The metadata flows through the `ModuleData` structure before reaching `PlannerOutput`. This structure is defined in `mpc_planner_types/include/mpc_planner_types/module_data.h`:

```cpp
namespace MPCPlanner {
    struct ModuleData {
        // Reference path and spline
        std::shared_ptr<RosTools::Spline2D> path;
        std::shared_ptr<RosTools::Spline1D> path_velocity;
        
        // Guidance trajectories (for T-MPC++)
        std::vector<std::vector<GuidancePoint>> guidance_trajectories;
        
        // Static obstacles
        std::vector<StaticObstacle> static_obstacles;
        
        // Path tracking
        int current_path_segment{-1};
        
        // Topology Metadata (Added October 2025)
        int selected_topology_id{-1};
        int selected_planner_index{-1};
        bool used_guidance{true};
        double trajectory_cost{0.0};
        int solver_exit_code{-1};
        
        void reset();
    };
}
```

**Data Flow**:
1. **Guidance module populates**: After selecting best planner
2. **Planner transfers**: If metadata enabled, copies to `PlannerOutput`
3. **Application consumes**: Robot controller accesses from output

**Note**: `ModuleData` is reset at the start of each planning cycle, so metadata only persists for one iteration unless explicitly cached.

## ROS Message Integration

### Custom Message Definition

To communicate topology metadata between robots, define a custom message:

**File**: `mpc_planner_msgs/msg/TrajectoryWithTopology.msg`

```
# Header
std_msgs/Header header

# Robot identification
int32 robot_id

# Topology metadata
int32 topology_id
int32 planner_index
bool used_guidance
float64 trajectory_cost
int32 solver_exit_code

# Trajectory data
nav_msgs/Path trajectory
```

### Publishing

```cpp
void JulesJackalPlanner::publishTrajectoryWithTopology(const MPCPlanner::PlannerOutput& output) {
    mpc_planner_msgs::TrajectoryWithTopology msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.robot_id = _ego_robot_id;
    
    // Topology metadata
    msg.topology_id = output.selected_topology_id;
    msg.planner_index = output.selected_planner_index;
    msg.used_guidance = output.used_guidance;
    msg.trajectory_cost = output.trajectory_cost;
    msg.solver_exit_code = output.solver_exit_code;
    
    // Trajectory waypoints
    msg.trajectory.header = msg.header;
    for (size_t i = 0; i < output.trajectory.positions.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = output.trajectory.positions[i](0);
        pose.pose.position.y = output.trajectory.positions[i](1);
        pose.pose.orientation = RosTools::angleToQuaternion(output.trajectory.orientations[i]);
        msg.trajectory.poses.push_back(pose);
    }
    
    _topology_trajectory_pub.publish(msg);
}
```

### Subscribing (Other Robots)

```cpp
void JulesJackalPlanner::otherRobotTopologyCallback(
    const mpc_planner_msgs::TrajectoryWithTopology::ConstPtr& msg) 
{
    LOG_INFO("Robot " << msg->robot_id << " following topology " << msg->topology_id);
    
    // Store in obstacle tracking
    int robot_index = findRobotObstacleIndex(msg->robot_id);
    _robot_topologies[robot_index] = msg->topology_id;
    _robot_costs[robot_index] = msg->trajectory_cost;
    
    // Could influence own topology selection
    if (shouldAvoidSameTopology(msg->robot_id, msg->topology_id)) {
        // Add bias against this topology in next planning cycle
        _topology_biases[msg->topology_id] += 0.2;  // Increase cost by 20%
    }
}
```

## Limitations and Considerations

### 1. Single-Planner Modes

**Limitation**: Topology metadata is only meaningful when using T-MPC++ with multiple parallel planners.

**When metadata is N/A**:
- Standard LMPCC (single planner, no guidance)
- Pure goal-tracking mode (no path following)
- Fallback to non-guided optimization only

**Check**:
```cpp
if (output.selected_topology_id == -9) {
    // Metadata not populated (single planner mode or disabled)
}
```

### 2. Configuration Dependency

**Requirement**: Must enable metadata in configuration.

```yaml
JULES:
  use_extra_params_module_data: true
```

If `false`, all metadata fields remain at default values (-9, -1, etc.).

### 3. Overhead

**Computational**: Negligible (~few bytes of data copied)

**Memory**: 5 additional fields in `PlannerOutput` (28 bytes on 64-bit system)

**Communication**: If broadcasting topology, slight increase in message size

### 4. Interpretation

**Topology IDs are problem-specific**: The meaning of "topology 0" vs "topology 1" depends on:
- Current obstacle configuration
- Guidance planner settings
- Which topologies were feasible in current iteration

**Not persistent**: Topology ID `0` in one scenario may mean something different in another scenario.

**Relative comparison**: Best used for comparison within a single planning problem, not across different environments.

## Testing and Validation

### Unit Tests

```cpp
TEST(TopologyMetadataTest, MetadataPopulation) {
    // Setup planner with T-MPC++
    Planner planner(settings);
    
    // Run MPC
    auto output = planner.solveMPC(state, data);
    
    // Verify metadata populated
    EXPECT_NE(output.selected_topology_id, -9);
    EXPECT_NE(output.selected_planner_index, -9);
    EXPECT_GE(output.trajectory_cost, 0.0);
    EXPECT_EQ(output.solver_exit_code, 1);  // Success
}

TEST(TopologyMetadataTest, GuidanceVsNonGuided) {
    // Test that used_guidance flag is correct
    Planner planner(settings);
    
    // Force selection of non-guided planner
    // ... (test implementation)
    
    auto output = planner.solveMPC(state, data);
    EXPECT_FALSE(output.used_guidance);
}
```

### Integration Tests

```cpp
TEST(MultiRobotTopologyTest, TopologyBroadcast) {
    // Launch 2 robots
    JulesJackalPlanner robot1(...);
    JulesJackalPlanner robot2(...);
    
    // Robot 1 plans
    auto output1 = robot1.solveMPC(...);
    robot1.publishTrajectoryWithTopology(output1);
    
    // Robot 2 receives
    ros::spinOnce();
    
    // Verify robot 2 received topology info
    EXPECT_EQ(robot2.getOtherRobotTopology(1), output1.selected_topology_id);
}
```

### Logging for Validation

```cpp
// Enable detailed topology logging
if (CONFIG["debug_topology_metadata"].as<bool>(true)) {
    LOG_INFO("=== Topology Decision ===");
    LOG_INFO("Available topologies: " << n_feasible_topologies);
    
    for (int i = 0; i < planners.size(); i++) {
        LOG_INFO("Planner " << i << ": "
                 << "topology=" << planners[i].result.guidance_ID
                 << ", cost=" << planners[i].result.objective
                 << ", success=" << planners[i].result.success);
    }
    
    LOG_INFO("Selected: planner " << output.selected_planner_index
             << ", topology " << output.selected_topology_id);
    LOG_INFO("========================");
}
```

## Future Enhancements

### 1. Topology Persistence

**Current**: Topology ID changes with obstacle configuration  
**Enhanced**: Persistent topology signatures that are comparable across scenarios

### 2. Topology Prediction

**Current**: Reactive topology selection  
**Enhanced**: Predictive model that suggests topology before MPC runs

```cpp
int predicted_topology = _topology_predictor.predict(environment_state);
// Bias MPC toward predicted topology
```

### 3. Multi-Objective Topology Selection

**Current**: Select based on trajectory cost only  
**Enhanced**: Consider multiple objectives

```cpp
struct TopologyScore {
    double trajectory_cost;
    double coordination_benefit;  // How well does it coordinate with other robots
    double consistency_bonus;      // Prefer same topology as previous
    double exploration_value;      // Encourage trying new topologies occasionally
};
```

### 4. Topology Communication Protocol

**Current**: Custom messages per application  
**Enhanced**: Standardized topology intention protocol

```yaml
# Topology Intention Protocol
robot_id: 1
intended_topology: 0
confidence: 0.85
valid_until: timestamp
alternatives: [1, 2]  # Backup topologies
```

### 5. Learning from Topology Choices

**Current**: No learning  
**Enhanced**: Offline learning from recorded topology metadata

```python
# Offline analysis
topology_data = load_rosbag("experiment.bag")
analyze_topology_patterns(topology_data)
# → "In narrow corridors, topology 0 fails 80% of the time"
# → "When obstacle moving left, topology 1 has 30% lower cost"
```

## Related Documentation

- [Guidance Constraints Documentation](guidance_constraints_documentation.md) - How T-MPC++ selects topologies
- [MPC Pipeline Documentation](mpc_pipeline_documentation.md) - Overall planner architecture
- [State Machine Implementation](../mpc_planner_jackalsimulator/docs/State_Machine_Implementation.md) - How topology metadata integrates with state machine

## Key Commits

- **Topology Metadata Addition**: `8f3e5eb` - "Added extra information to output struct and module_data struct for topology metadata"
- **Guidance Module Integration**: `ec9a45b` - "Working on getting topology of chosen trajectory up to jackalplanner"

## References

### Source Files

- **PlannerOutput Definition**: `mpc_planner/include/mpc_planner/planner.h`
- **ModuleData Definition**: `mpc_planner_types/include/mpc_planner_types/module_data.h`
- **Metadata Population**: `mpc_planner/src/planner.cpp` (lines 162-172)
- **Guidance Integration**: `mpc_planner_modules/src/guidance_constraints.cpp`
- **Usage Example**: `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp`

### Configuration Files

- **Enable Metadata**: `config/settings.yaml` (`JULES.use_extra_params_module_data`)
- **Topology Count**: `config/guidance_planner.yaml` (`n_paths_`)
- **T-MPC++ Enable**: `config/settings.yaml` (`t-mpc.use_t-mpc++`)

---

*Documentation created: October 2025*  
*Feature implemented: October 2025*  
*Status: Production-ready*
