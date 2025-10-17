# State Machine Implementation in JulesJackalPlanner

## Overview

As of October 2025, the JulesJackalPlanner has been refactored to use a **finite state machine (FSM)** architecture replacing the previous boolean flag-based state management. This implementation significantly improves:

- **Clarity**: System state is explicitly defined at all times
- **Safety**: Transitions are validated and logged
- **Maintainability**: Easier to understand and debug complex state flows
- **Robustness**: Prevents invalid state transitions and data access

## State Machine Design

### State Definitions

The planner operates in one of 12 distinct states, defined in `mpc_planner_types/include/mpc_planner_types/data_types.h`:

```cpp
enum class PlannerState
{
    UNINITIALIZED,                        // Just constructed, waiting for essential data
    TIMER_STARTUP,                        // Wait before starting (startup delay)
    WAITING_FOR_FIRST_EGO_POSE,          // Wait for first pose from localization
    INITIALIZING_OBSTACLES,               // Setting up robot-robot obstacle tracking
    WAITING_FOR_OTHER_ROBOTS_FIRST_POSES,// Wait for other robots' first poses
    WAITING_FOR_SYNC,                     // Waiting for other robots (if enabled)
    WAITING_FOR_TRAJECTORY_DATA,          // Have structure, waiting for valid trajectories
    PLANNING_ACTIVE,                      // Normal MPC planning operation
    JUST_REACHED_GOAL,                    // Just reached goal, ready to brake
    GOAL_REACHED,                         // At goal, may rotate or stop
    RESETTING,                            // Transitioning to new task
    ERROR_STATE                           // Unrecoverable error occurred
};
```

### State Transition Diagram

```
UNINITIALIZED
    ↓
TIMER_STARTUP (startup timer completes)
    ↓
WAITING_FOR_FIRST_EGO_POSE (receive first pose)
    ↓
INITIALIZING_OBSTACLES (initialize robot obstacles)
    ↓
WAITING_FOR_OTHER_ROBOTS_FIRST_POSES (receive all robot poses)
    ↓
WAITING_FOR_SYNC (all robots ready - if multi-robot sync enabled)
    ↓
WAITING_FOR_TRAJECTORY_DATA (receive trajectory data from other robots)
    ↓
PLANNING_ACTIVE (normal operation)
    ↓
JUST_REACHED_GOAL (detect goal proximity)
    ↓
GOAL_REACHED (stopped at goal)
    ↓
RESETTING (reset signal received)
    ↓
(back to WAITING_FOR_TRAJECTORY_DATA or WAITING_FOR_SYNC)

From any state → ERROR_STATE (on unrecoverable error)
```

## Implementation Details

### Core State Machine Functions

#### `transitionTo(PlannerState& current_state, PlannerState new_state, string ego_robot_ns)`

**Purpose**: Safely transition between states with validation and logging.

**Algorithm**:
1. Check if transition is valid using `canTransitionTo()`
2. If invalid, log error and optionally move to ERROR_STATE
3. If valid:
   - Log the transition
   - Update current state
   - Call `onStateEnter()` for state-specific initialization

**Example Usage**:
```cpp
transitionTo(_current_state, PlannerState::PLANNING_ACTIVE, _ego_robot_ns);
```

#### `canTransitionTo(PlannerState current_state, PlannerState new_state, string ego_robot_ns)`

**Purpose**: Validate if a state transition is allowed.

**Returns**: `true` if transition is valid, `false` otherwise.

**Transition Rules**:

| From State | Valid Next States |
|-----------|-------------------|
| UNINITIALIZED | TIMER_STARTUP |
| TIMER_STARTUP | WAITING_FOR_FIRST_EGO_POSE |
| WAITING_FOR_FIRST_EGO_POSE | INITIALIZING_OBSTACLES |
| INITIALIZING_OBSTACLES | WAITING_FOR_OTHER_ROBOTS_FIRST_POSES |
| WAITING_FOR_OTHER_ROBOTS_FIRST_POSES | WAITING_FOR_SYNC, WAITING_FOR_TRAJECTORY_DATA |
| WAITING_FOR_SYNC | WAITING_FOR_TRAJECTORY_DATA |
| WAITING_FOR_TRAJECTORY_DATA | PLANNING_ACTIVE, GOAL_REACHED |
| PLANNING_ACTIVE | JUST_REACHED_GOAL, RESETTING |
| JUST_REACHED_GOAL | GOAL_REACHED |
| GOAL_REACHED | RESETTING |
| RESETTING | WAITING_FOR_TRAJECTORY_DATA, WAITING_FOR_SYNC |
| ERROR_STATE | (terminal state) |

**Note**: Any state can transition to ERROR_STATE.

#### `onStateEnter(PlannerState current_state, PlannerState new_state, string ego_robot_ns)`

**Purpose**: Execute state-specific initialization when entering a new state.

**Actions by State**:

- **TIMER_STARTUP**: Start startup timer
- **INITIALIZING_OBSTACLES**: Initialize robot obstacle structures
- **WAITING_FOR_SYNC**: Log waiting for synchronization
- **PLANNING_ACTIVE**: Enable MPC solver, reset failure counts
- **GOAL_REACHED**: Stop robot, publish goal reached event
- **RESETTING**: Clear cached data, reset planner state
- **ERROR_STATE**: Stop robot, disable planner

### Main Control Loop: `loopDirectTrajectoryStateMachine()`

The new state machine-based control loop replaces the previous loop functions. It operates at the configured control frequency and dispatches to state-specific handlers.

**High-Level Structure**:
```cpp
void JulesJackalPlanner::loopDirectTrajectoryStateMachine(const ros::TimerEvent&)
{
    switch (_current_state) {
        case PlannerState::TIMER_STARTUP:
            // Wait for startup timer
            if (startup_timer_expired()) {
                transitionTo(_current_state, PlannerState::WAITING_FOR_FIRST_EGO_POSE, _ego_robot_ns);
            }
            break;
            
        case PlannerState::WAITING_FOR_FIRST_EGO_POSE:
            // Check if ego pose received
            if (_state.isInitialized()) {
                transitionTo(_current_state, PlannerState::INITIALIZING_OBSTACLES, _ego_robot_ns);
            }
            break;
            
        case PlannerState::INITIALIZING_OBSTACLES:
            // Initialize robot obstacles
            if (initializeOtherRobotsAsObstacles(...)) {
                transitionTo(_current_state, PlannerState::WAITING_FOR_OTHER_ROBOTS_FIRST_POSES, _ego_robot_ns);
            }
            break;
            
        case PlannerState::WAITING_FOR_OTHER_ROBOTS_FIRST_POSES:
            // Check if all robot poses received
            if (allRobotPosesReceived()) {
                auto next_state = _multi_robot_sync_enabled ? 
                    PlannerState::WAITING_FOR_SYNC : PlannerState::WAITING_FOR_TRAJECTORY_DATA;
                transitionTo(_current_state, next_state, _ego_robot_ns);
            }
            break;
            
        case PlannerState::WAITING_FOR_SYNC:
            // Wait for synchronization signal
            if (_all_robots_ready) {
                transitionTo(_current_state, PlannerState::WAITING_FOR_TRAJECTORY_DATA, _ego_robot_ns);
            }
            break;
            
        case PlannerState::WAITING_FOR_TRAJECTORY_DATA:
            // Check if trajectory data received
            if (haveReceivedMeaningfulTrajectoryData()) {
                transitionTo(_current_state, PlannerState::PLANNING_ACTIVE, _ego_robot_ns);
            }
            break;
            
        case PlannerState::PLANNING_ACTIVE:
            // Normal MPC planning
            prepareObstacleData();
            auto [cmd, output] = generatePlanningCommand(_current_state);
            
            // Check goal reached
            if (objectiveReached(_state, _data)) {
                transitionTo(_current_state, PlannerState::JUST_REACHED_GOAL, _ego_robot_ns);
            }
            
            publishCmdAndVisualize(cmd, output);
            break;
            
        case PlannerState::JUST_REACHED_GOAL:
            // Apply final braking
            applyBrakingCommand(cmd);
            publishCmdAndVisualize(cmd, empty_output);
            transitionTo(_current_state, PlannerState::GOAL_REACHED, _ego_robot_ns);
            break;
            
        case PlannerState::GOAL_REACHED:
            // Optional: rotate or hold position
            if (_rotate_at_goal) {
                rotatePiRadiansCw(cmd);
            }
            publishCmdAndVisualize(cmd, empty_output);
            break;
            
        case PlannerState::RESETTING:
            // Reset planner for new task
            _planner->reset();
            auto next_state = _multi_robot_sync_enabled ? 
                PlannerState::WAITING_FOR_SYNC : PlannerState::WAITING_FOR_TRAJECTORY_DATA;
            transitionTo(_current_state, next_state, _ego_robot_ns);
            break;
            
        case PlannerState::ERROR_STATE:
            // Stop and log error
            applyBrakingCommand(cmd);
            publishCmdAndVisualize(cmd, empty_output);
            LOG_ERROR("Planner in ERROR_STATE, stopped");
            break;
            
        default:
            LOG_ERROR("Unknown state: " << stateToString(_current_state));
            transitionTo(_current_state, PlannerState::ERROR_STATE, _ego_robot_ns);
    }
}
```

## Key Changes from Previous Implementation

### Removed Boolean Flags

The following boolean flags have been **removed** and replaced by state machine states:

| Removed Flag | Replaced By State(s) |
|--------------|---------------------|
| `_goal_reached` | `JUST_REACHED_GOAL`, `GOAL_REACHED` |
| `_have_received_meaningful_trajectory_data` | `WAITING_FOR_TRAJECTORY_DATA`, `PLANNING_ACTIVE` |
| `_planning_for_the_first_time` | `INITIALIZING_OBSTACLES`, `WAITING_FOR_OTHER_ROBOTS_FIRST_POSES` |
| `_all_robots_ready` | `WAITING_FOR_SYNC` (condition check) |
| `_startup_timer_expired` | `TIMER_STARTUP` |

### Simplified Functions

Several functions were simplified or removed:

- **Removed**: `handleInitialPlanningPhase()` - logic integrated into state handlers
- **Removed**: `applyDummyObstacleCommand()` - no longer needed
- **Simplified**: `generatePlanningCommand()` - now takes state as parameter
- **Refactored**: `initializeOtherRobotsAsObstacles()` - now returns bool for success

### New Utility Functions

**From Multi-Robot Utilities** (`mpc_planner_util/include/mpc_planner_util/multi_robot_utility_functions.h`):

```cpp
namespace MultiRobot {
    // Extract robot ID from namespace (e.g., "/jackal1" → 1)
    int extractRobotIdFromNamespace(const std::string& ns);
    
    // Find other robots in environment
    std::set<std::string> identifyOtherRobotNamespaces(
        const std::vector<std::string>& all_namespaces, 
        const std::string& ego_robot_ns);
    
    // Debug logging of real-time data state
    std::string dataToString(const MPCPlanner::RealTimeData& data, 
                            const std::string& ego_robot_ns);
}
```

## State-Specific Behavior

### PLANNING_ACTIVE State

This is the primary operational state where MPC planning occurs:

**Conditions to Enter**:
- Robot pose initialized
- Other robot obstacles initialized
- Trajectory data received from all robots (or timeout reached)
- Synchronization complete (if enabled)

**Behavior**:
1. Prepare obstacle data from robot trajectories
2. Call MPC solver: `_planner->solveMPC(_state, _data)`
3. Extract control commands from solution
4. Check if goal reached → transition to JUST_REACHED_GOAL
5. Publish commands and visualizations

**Exit Conditions**:
- Goal reached → JUST_REACHED_GOAL
- Reset signal → RESETTING
- Critical error → ERROR_STATE

### WAITING_FOR_TRAJECTORY_DATA State

Ensures all robot trajectory data is available before planning:

**Purpose**: Prevent MPC from operating with incomplete obstacle information

**Behavior**:
- Check if trajectory data received from all other robots
- Use timeout mechanism to proceed even if some data missing
- Log warning if proceeding with incomplete data

**Data Validation**:
```cpp
bool haveReceivedMeaningfulTrajectoryData() {
    for (const auto& obstacle : _data.trajectory_dynamic_obstacles) {
        if (obstacle.type == ObstacleType::ROBOT) {
            if (obstacle.prediction.empty()) {
                return false; // Missing trajectory data
            }
        }
    }
    return true;
}
```

### INITIALIZING_OBSTACLES State

Sets up the robot obstacle tracking structures:

**Actions**:
1. Identify other robots in the environment
2. Create DynamicObstacle entries for each robot
3. Set type to `ObstacleType::ROBOT`
4. Initialize with invalid positions (will be updated on first callback)

**Implementation**:
```cpp
bool initializeOtherRobotsAsObstacles(
    const std::set<std::string>& other_robot_nss, 
    MPCPlanner::RealTimeData& data,
    const double robot_radius)
{
    for (const auto& ns : other_robot_nss) {
        int robot_id = MultiRobot::extractRobotIdFromNamespace(ns);
        
        // Create obstacle with invalid initial position
        DynamicObstacle robot_obstacle(
            robot_id, 
            robot_radius, 
            ObstacleType::ROBOT
        );
        
        data.trajectory_dynamic_obstacles.push_back(robot_obstacle);
    }
    return true;
}
```

## Benefits of State Machine Architecture

### 1. Explicit State Management

**Before**:
```cpp
if (!_planning_for_the_first_time && _have_received_meaningful_trajectory_data && !_goal_reached) {
    // Do planning
}
```

**After**:
```cpp
if (_current_state == PlannerState::PLANNING_ACTIVE) {
    // Do planning
}
```

### 2. Validated Transitions

**Before**: Direct flag manipulation
```cpp
_goal_reached = true; // Could be set from anywhere
```

**After**: Validated state transition
```cpp
transitionTo(_current_state, PlannerState::GOAL_REACHED, _ego_robot_ns);
// Logged, validated, with state-entry actions
```

### 3. Clear Startup Sequence

The startup sequence is now explicitly defined in the state transitions:
```
UNINITIALIZED → TIMER_STARTUP → WAITING_FOR_FIRST_EGO_POSE → 
INITIALIZING_OBSTACLES → WAITING_FOR_OTHER_ROBOTS_FIRST_POSES → 
[WAITING_FOR_SYNC] → WAITING_FOR_TRAJECTORY_DATA → PLANNING_ACTIVE
```

### 4. Improved Debugging

State logging provides clear trace of system behavior:
```
[jackal0] Transitioning: WAITING_FOR_TRAJECTORY_DATA -> PLANNING_ACTIVE
[jackal0] Entering state: PLANNING_ACTIVE
[jackal0] MPC planning active, solver enabled
```

### 5. Safe Data Access

Data is only accessed when the state guarantees it's been initialized:

| Data | Required State | Guaranteed By |
|------|---------------|---------------|
| `_state` (ego pose) | >= WAITING_FOR_FIRST_EGO_POSE | State transition validation |
| Robot obstacles | >= INITIALIZING_OBSTACLES | State entry action |
| Robot trajectories | >= PLANNING_ACTIVE | Transition condition check |

## Multi-Robot Synchronization

### Synchronization States

For multi-robot experiments with coordinated startup:

**WAITING_FOR_SYNC**:
- All robots wait at this state until sync signal received
- Uses topic: `all_robots_planner_ready`
- Each robot publishes readiness when reaching this state
- Central aggregator monitors and sends "all ready" signal

**Reset Synchronization**:
- After all robots reach goals, aggregator sends reset signal
- All robots transition to RESETTING
- Coordinate return to synchronized state (WAITING_FOR_SYNC or WAITING_FOR_TRAJECTORY_DATA)

### Sync Flowchart

```
Robot 1                    Robot 2                    Robot N           Aggregator
   |                          |                          |                  |
   |---[ready]--------------->|                          |                  |
   |                          |---[ready]--------------->|                  |
   |                          |                          |---[ready]------->|
   |                          |                          |                  |
   |                          |                          |    [wait for all]
   |                          |                          |                  |
   |<------------------------[all_ready]------------------------------------|
   |                          |<--------------------------------------------|
   |                          |                          |<-----------------|
   |                          |                          |                  |
[PLANNING_ACTIVE]        [PLANNING_ACTIVE]         [PLANNING_ACTIVE]       |
```

## Error Handling

### ERROR_STATE

Entered when:
- Invalid state transition attempted
- Critical initialization failure
- Unrecoverable MPC solver error
- Missing essential data after timeout

**Behavior in ERROR_STATE**:
- Stop robot (apply braking)
- Log error details
- Remain in ERROR_STATE (terminal state)
- Requires manual intervention or node restart

### Graceful Degradation

**Missing Trajectory Data**:
- Timeout mechanism allows proceeding to PLANNING_ACTIVE even if some data missing
- MPC solver handles missing obstacles with default empty predictions
- Logged as warning, not error

**Solver Failure**:
- Remains in PLANNING_ACTIVE
- Applies braking command
- Retries on next cycle
- Only transitions to ERROR_STATE after repeated failures

## Configuration

### State Machine Parameters

In `config/settings.yaml` or launch files:

```yaml
# Multi-robot synchronization
multi_robot_sync_enabled: true    # Enable WAITING_FOR_SYNC state
all_robots_ready_topic: "/all_robots_planner_ready"

# Startup timing
startup_wait_time: 3.0            # TIMER_STARTUP duration (seconds)
trajectory_data_timeout: 10.0     # Max wait in WAITING_FOR_TRAJECTORY_DATA

# State transitions
allow_planning_without_full_data: true  # Proceed even if trajectory data incomplete
rotate_at_goal: true              # Enable rotation in GOAL_REACHED state
```

## Testing and Validation

### State Transition Testing

Verify all valid transitions:
```cpp
// Unit test example
TEST(StateMachineTest, ValidTransitions) {
    JulesJackalPlanner planner(...);
    
    // Test initialization sequence
    EXPECT_TRUE(planner.canTransitionTo(
        PlannerState::UNINITIALIZED, 
        PlannerState::TIMER_STARTUP));
    
    // Test invalid transition
    EXPECT_FALSE(planner.canTransitionTo(
        PlannerState::UNINITIALIZED, 
        PlannerState::PLANNING_ACTIVE));
}
```

### Integration Testing

Multi-robot scenario test:
1. Launch N robots with sync enabled
2. Verify all reach WAITING_FOR_SYNC
3. Send sync signal
4. Verify all transition to PLANNING_ACTIVE
5. Verify coordinated goal reaching
6. Verify synchronized reset

### State Logging

Enable detailed state logging:
```cpp
// In constructor
if (CONFIG["debug_state_machine"].as<bool>(true)) {
    _log_state_transitions = true;
}
```

Logs:
```
[jackal0][14:23:45.123] State: UNINITIALIZED -> TIMER_STARTUP
[jackal0][14:23:48.124] State: TIMER_STARTUP -> WAITING_FOR_FIRST_EGO_POSE
[jackal0][14:23:48.456] Received first ego pose (x: 0.0, y: 0.0)
[jackal0][14:23:48.457] State: WAITING_FOR_FIRST_EGO_POSE -> INITIALIZING_OBSTACLES
...
```

## Future Enhancements

### Potential Additional States

1. **WAITING_FOR_PATH**: Separate state for reference path availability
2. **REPLANNING**: Triggered when environment changes significantly
3. **COLLISION_DETECTED**: Safety state when collision is imminent
4. **DEGRADED_MODE**: Operating with reduced functionality

### Advanced Features

1. **State Timeouts**: Automatic transition to ERROR_STATE if stuck in a state too long
2. **State History**: Track last N states for debugging
3. **State Metrics**: Measure time spent in each state for performance analysis
4. **Conditional Transitions**: More complex transition logic based on multiple conditions

## References

### Related Files

- **State Definitions**: `mpc_planner_types/include/mpc_planner_types/data_types.h`
- **State Machine Logic**: `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp`
- **Utility Functions**: `mpc_planner_util/include/mpc_planner_util/multi_robot_utility_functions.h`

### Related Documentation

- [JulesJackalPlanner_Architecture.md](JulesJackalPlanner_Architecture.md) - Overall architecture
- [Refactoring_Strategy.md](Refactoring_Strategy.md) - Original refactoring plan
- [state_machine_diagram.md](state_machine_diagram.md) - Visual diagrams

### Key Commits

- **State Machine Implementation**: `b906df9` - "Implemented the state machine logic and refactored the code"
- **State Enum Addition**: `2a93c73` - "Adding PlannerState enum to data_types.h"
- **Cleanup**: `555018b` - "Removing unnecessary code from older versions"
- **Enhanced Comments**: `471e55d` - "Enhanced comments for more explainability"

---

*Documentation updated: October 2025*
*Implementation status: Complete and deployed*
