# Refactoring Strategy for jules_ros1_jackalplanner.cpp

## Executive Summary

This document provides a comprehensive strategy to refactor `jules_ros1_jackalplanner.cpp` to make it more maintainable, readable, and robust. The current implementation has grown organically to support multi-robot coordination, leading to complex state management with numerous configuration flags and uninitialized data handling issues.

**Key Goals:**
1. Simplify state management and reduce configuration variables
2. Introduce a clear state machine for lifecycle management
3. Improve data initialization and validation
4. Enhance code readability and maintainability
5. Make debugging and extension easier

---

## Current Issues Analysis

### 1. Complex State Management

**Problem:** The planner uses multiple boolean flags to track system state, making behavior prediction difficult:

```cpp
// Current flags scattered throughout the class:
bool _goal_received{false};
bool _goal_reached{false};
bool _received_obstacle_callback_first_time{true};
bool _have_received_meaningful_trajectory_data{false};
bool _stop_when_reached_goal{false};
bool _enable_output{true};
```

**Impact:**
- Hard to understand current system state
- Difficult to debug state transitions
- Risk of inconsistent state combinations
- Complex conditional logic in planning loop

### 2. Asynchronous Data Initialization

**Problem:** Multiple asynchronous callbacks populate `_state` and `_data` with no guaranteed order:

```cpp
// Data arrives from different callbacks at different times:
- statePoseCallback() -> updates _state (x, y, psi, v)
- goalCallback() -> updates _data.goal
- pathCallback() -> updates _data.reference_path
- obstacleCallback() -> updates _data.dynamic_obstacles
- trajectoryCallback() -> updates _data.trajectory_dynamic_obstacles
- poseOtherRobotCallback() -> updates robot obstacle positions
```

**Impact:**
- Planning may start with uninitialized data
- Race conditions during startup
- Solver calls with incomplete information
- Crashes when accessing uninitialized trajectory obstacles

### 3. Startup and Reset Complexity

**Problem:** Multiple overlapping mechanisms control startup behavior:

```cpp
// Startup mechanisms:
1. _startup_timer (20 second delay)
2. waitForAllRobotsReady() (synchronization barrier)
3. _have_received_meaningful_trajectory_data flag
4. _validated_trajectory_robots set
5. initializeOtherRobotsAsObstacles() (dummy obstacles)
```

**Impact:**
- Unclear when planning can safely begin
- Different behavior in first loop vs subsequent loops
- Complex reset logic with partial state clearing
- Difficult to add new robots or restart missions

### 4. Uninitialized Data Access

**Problem:** `_planner->solveMPC(_state, _data)` may be called with uninitialized obstacle data:

```cpp
// In loopDirectTrajectory():
if (!_have_received_meaningful_trajectory_data) {
    applyDummyObstacleCommand();  // Creates temporary dummy obstacles
    _data.dynamic_obstacles.clear();  // Then clears them!
}
// Later: updateRobotObstaclesFromTrajectories() may access uninitialized obstacles
```

**Impact:**
- Potential crashes when accessing trajectory_dynamic_obstacles
- Inconsistent obstacle data between planning cycles
- Dummy obstacles added and removed, wasteful
- Race conditions between pose and trajectory callbacks

### 5. Three Different Loop Functions

**Problem:** Three different planning loop implementations with overlapping logic:

```cpp
void loop(const ros::TimerEvent &event);              // Original version
void loopDirectTrajectory(const ros::TimerEvent &event);  // Direct P2P communication
void loopWithService(const ros::TimerEvent &event);   // Service-based aggregator
```

**Impact:**
- Code duplication across loops
- Inconsistent behavior between modes
- Maintenance burden (fix bugs in 3 places)
- Unclear which loop is active

---

## Proposed Solution: State Machine Architecture

### Overview

Introduce a clear **Finite State Machine (FSM)** to manage the planner lifecycle with well-defined states and transitions.

### State Definitions

```cpp
enum class PlannerState {
    UNINITIALIZED,           // Just constructed, waiting for essential data
    WAITING_FOR_SYNC,        // Waiting for other robots (if enabled)
    INITIALIZING_OBSTACLES,  // Setting up robot-robot obstacle tracking
    WAITING_FOR_DATA,        // Have structure, waiting for first valid trajectories
    PLANNING_ACTIVE,         // Normal MPC planning operation
    GOAL_REACHED,            // At goal, may rotate or stop
    RESETTING,               // Transitioning to new task
    ERROR_STATE              // Unrecoverable error occurred
};
```

### State Transitions

```
UNINITIALIZED
    ↓ (constructor completes)
WAITING_FOR_SYNC
    ↓ (all robots ready OR sync disabled)
INITIALIZING_OBSTACLES
    ↓ (trajectory_dynamic_obstacles created)
WAITING_FOR_DATA
    ↓ (first valid trajectory received)
PLANNING_ACTIVE
    ↓ (goal reached)
GOAL_REACHED
    ↓ (all robots reached goal)
RESETTING
    ↓ (reset complete)
WAITING_FOR_SYNC (cycle repeats)
```

### Benefits

1. **Clear System State**: One variable shows current state
2. **Predictable Behavior**: Each state has specific allowed actions
3. **Easy Debugging**: Log state transitions
4. **Maintainable**: Add new states without touching existing code
5. **Testable**: Can unit test state transitions

---

## Detailed Refactoring Strategy

### Phase 1: Introduce State Machine Core

**Goal:** Replace boolean flags with explicit state machine

**Steps:**

1. **Add State Enum and Variable**
   ```cpp
   // In jules_ros1_jackalplanner.h
   enum class PlannerState {
       UNINITIALIZED,
       WAITING_FOR_SYNC,
       INITIALIZING_OBSTACLES,
       WAITING_FOR_DATA,
       PLANNING_ACTIVE,
       GOAL_REACHED,
       RESETTING,
       ERROR_STATE
   };
   
   private:
       PlannerState _current_state{PlannerState::UNINITIALIZED};
       
       // Helper methods
       void transitionTo(PlannerState new_state);
       bool canTransitionTo(PlannerState new_state) const;
       std::string stateToString(PlannerState state) const;
   ```

2. **Implement State Transition Logic**
   ```cpp
   void JulesJackalPlanner::transitionTo(PlannerState new_state) {
       if (!canTransitionTo(new_state)) {
           LOG_ERROR("Invalid state transition from " + 
                     stateToString(_current_state) + 
                     " to " + stateToString(new_state));
           _current_state = PlannerState::ERROR_STATE;
           return;
       }
       
       LOG_INFO(_ego_robot_ns + ": State transition: " + 
                stateToString(_current_state) + " -> " + 
                stateToString(new_state));
       
       _current_state = new_state;
       onStateEnter(new_state);
   }
   ```

3. **Define State-Specific Behaviors**
   ```cpp
   void JulesJackalPlanner::onStateEnter(PlannerState state) {
       switch (state) {
           case PlannerState::WAITING_FOR_SYNC:
               if (_wait_for_sync_enabled) {
                   waitForAllRobotsReady(_nh);
               }
               transitionTo(PlannerState::INITIALIZING_OBSTACLES);
               break;
               
           case PlannerState::INITIALIZING_OBSTACLES:
               initializeOtherRobotsAsObstacles(_other_robot_nss, _data, 
                                                CONFIG["robot_radius"].as<double>());
               transitionTo(PlannerState::WAITING_FOR_DATA);
               break;
               
           case PlannerState::WAITING_FOR_DATA:
               _startup_timer->start();
               break;
               
           // ... other states
       }
   }
   ```

### Phase 2: Data Structure Initialization

**Goal:** Ensure all data structures are properly initialized before use

**Steps:**

1. **Create Data Readiness Tracker**
   ```cpp
   struct DataReadiness {
       bool state_received{false};
       bool goal_received{false};
       bool path_received{false};
       bool static_obstacles_received{false};
       
       // Per-robot trajectory data
       std::map<std::string, bool> robot_trajectory_ready;
       
       bool isEssentialDataReady() const {
           return state_received && goal_received && path_received;
       }
       
       bool areAllRobotTrajectoriesReady() const {
           return std::all_of(robot_trajectory_ready.begin(),
                              robot_trajectory_ready.end(),
                              [](const auto& pair) { return pair.second; });
       }
       
       void reset() {
           state_received = false;
           goal_received = false;
           path_received = false;
           static_obstacles_received = false;
           robot_trajectory_ready.clear();
       }
   };
   
   private:
       DataReadiness _data_readiness;
   ```

2. **Initialize Obstacle Structures Properly**
   ```cpp
   void JulesJackalPlanner::initializeOtherRobotsAsObstacles(
       const std::set<std::string>& other_robot_namespaces,
       MPCPlanner::RealTimeData& data,
       const double radius) {
       
       // Clear existing structures
       data.trajectory_dynamic_obstacles.clear();
       _data_readiness.robot_trajectory_ready.clear();
       
       for (const auto& robot_ns : other_robot_namespaces) {
           // Create obstacle entry with VALID initialization
           // Position far away but not (0,0) to avoid collision with world origin
           const Eigen::Vector2d safe_init_position(1000.0, 1000.0);
           
           auto obstacle = MPCPlanner::DynamicObstacle(
               extractRobotIdFromNamespace(robot_ns),
               safe_init_position,
               0.0,  // angle
               radius
           );
           
           // Initialize with empty but VALID prediction
           obstacle.prediction = MPCPlanner::Prediction(
               MPCPlanner::PredictionType::DETERMINISTIC
           );
           
           // Store in trajectory map
           data.trajectory_dynamic_obstacles[robot_ns] = obstacle;
           
           // Mark as not ready yet
           _data_readiness.robot_trajectory_ready[robot_ns] = false;
           
           LOG_INFO(_ego_robot_ns + " initialized obstacle for " + robot_ns);
       }
   }
   ```

3. **Update Callbacks to Set Readiness**
   ```cpp
   void JulesJackalPlanner::trajectoryCallback(
       const mpc_planner_msgs::ObstacleGMM::ConstPtr& msg,
       const std::string ns) {
       
       // Validate that obstacle structure exists
       auto it = _data.trajectory_dynamic_obstacles.find(ns);
       if (it == _data.trajectory_dynamic_obstacles.end()) {
           LOG_ERROR(_ego_robot_ns + ": Received trajectory from " + ns + 
                     " before initialization!");
           return;
       }
       
       // ... process trajectory data ...
       
       // Mark this robot's data as ready
       if (!msg->gaussians.empty() && !msg->gaussians[0].mean.poses.empty()) {
           _data_readiness.robot_trajectory_ready[ns] = true;
           
           // Check if we can transition to active planning
           if (_current_state == PlannerState::WAITING_FOR_DATA &&
               _data_readiness.areAllRobotTrajectoriesReady()) {
               transitionTo(PlannerState::PLANNING_ACTIVE);
           }
       }
   }
   ```

### Phase 3: Unified Planning Loop

**Goal:** Consolidate three loop functions into one with clear state-based dispatch

**Steps:**

1. **Create Main Loop Entry Point**
   ```cpp
   void JulesJackalPlanner::mainLoop(const ros::TimerEvent& event) {
       // State-based dispatch
       switch (_current_state) {
           case PlannerState::UNINITIALIZED:
           case PlannerState::WAITING_FOR_SYNC:
           case PlannerState::INITIALIZING_OBSTACLES:
               // Do nothing, state transitions handle these
               return;
               
           case PlannerState::WAITING_FOR_DATA:
               handleWaitingForData();
               return;
               
           case PlannerState::PLANNING_ACTIVE:
               handleActivePlanning();
               return;
               
           case PlannerState::GOAL_REACHED:
               handleGoalReached();
               return;
               
           case PlannerState::RESETTING:
               handleReset();
               return;
               
           case PlannerState::ERROR_STATE:
               handleError();
               return;
       }
   }
   ```

2. **Implement State Handlers**
   ```cpp
   void JulesJackalPlanner::handleWaitingForData() {
       // Check if we've timed out waiting
       if (_startup_timer->hasFinished()) {
           LOG_WARN(_ego_robot_ns + 
                    ": Timeout waiting for trajectory data, using defaults");
           transitionTo(PlannerState::PLANNING_ACTIVE);
           return;
       }
       
       // Publish dummy command to keep robot alive
       geometry_msgs::Twist cmd;
       cmd.linear.x = 0.0;
       cmd.angular.z = 0.0;
       _cmd_pub.publish(cmd);
       
       LOG_INFO_THROTTLE(1000, _ego_robot_ns + 
                         ": Waiting for trajectory data from " +
                         std::to_string(_data_readiness.robot_trajectory_ready.size()) +
                         " robots");
   }
   
   void JulesJackalPlanner::handleActivePlanning() {
       // Validate data before planning
       if (!_data_readiness.isEssentialDataReady()) {
           LOG_ERROR(_ego_robot_ns + ": Lost essential data during planning!");
           transitionTo(PlannerState::ERROR_STATE);
           return;
       }
       
       // Update timestamps
       _data.planning_start_time = std::chrono::system_clock::now();
       
       // Prepare obstacles (merge trajectory and static)
       prepareObstacleData();
       
       // Check goal status
       if (objectiveReached(_state, _data)) {
           transitionTo(PlannerState::GOAL_REACHED);
           return;
       }
       
       // Run MPC solver
       auto output = _planner->solveMPC(_state, _data);
       
       // Generate commands
       geometry_msgs::Twist cmd = generateCommand(output);
       
       // Publish and visualize
       publishCmdAndVisualize(cmd, output);
   }
   ```

### Phase 4: Robust Reset Mechanism

**Goal:** Clean, predictable reset behavior

**Steps:**

1. **Centralized Reset Function**
   ```cpp
   void JulesJackalPlanner::performReset(bool success) {
       LOG_INFO(_ego_robot_ns + ": Starting reset (success=" + 
                std::to_string(success) + ")");
       
       // Transition to reset state
       transitionTo(PlannerState::RESETTING);
       
       // Reset planner internals
       _planner->reset(_state, _data, success);
       
       // Clear trajectory tracking
       _validated_trajectory_robots.clear();
       
       // Reset data readiness
       _data_readiness.reset();
       
       // Re-initialize robot obstacles
       initializeOtherRobotsAsObstacles(_other_robot_nss, _data,
                                        CONFIG["robot_radius"].as<double>());
       
       // Publish roadmap reverse
       std_msgs::Empty empty_msg;
       _reverse_roadmap_pub.publish(empty_msg);
       
       // Reset goal tracking
       _goal_reached = false;
       
       // Start fresh startup period
       _startup_timer->setDuration(2.0);
       _startup_timer->start();
       
       // Transition back to waiting for data
       transitionTo(PlannerState::WAITING_FOR_DATA);
       
       LOG_INFO(_ego_robot_ns + ": Reset complete, ready for new mission");
   }
   ```

2. **Reset Callback Simplification**
   ```cpp
   void JulesJackalPlanner::allRobotsReachedObjectiveCallback(
       const std_msgs::Bool::ConstPtr& msg) {
       
       if (!msg->data) {
           LOG_ERROR(_ego_robot_ns + 
                     ": Invalid reset signal from aggregator");
           return;
       }
       
       if (_current_state != PlannerState::GOAL_REACHED) {
           LOG_WARN(_ego_robot_ns + 
                    ": Received reset signal but not at goal!");
           return;
       }
       
       performReset(true);
   }
   ```

### Phase 5: Data Validation and Safety Checks

**Goal:** Add comprehensive validation to prevent crashes

**Steps:**

1. **Obstacle Data Validator**
   ```cpp
   class ObstacleDataValidator {
   public:
       static bool validateDynamicObstacles(
           const std::vector<MPCPlanner::DynamicObstacle>& obstacles) {
           
           for (const auto& obs : obstacles) {
               // Check position is finite
               if (!std::isfinite(obs.position.x()) || 
                   !std::isfinite(obs.position.y())) {
                   LOG_ERROR("Invalid obstacle position: non-finite values");
                   return false;
               }
               
               // Check radius is positive
               if (obs.radius <= 0.0) {
                   LOG_ERROR("Invalid obstacle radius: " + 
                             std::to_string(obs.radius));
                   return false;
               }
               
               // Check prediction is valid
               if (!validatePrediction(obs.prediction)) {
                   return false;
               }
           }
           return true;
       }
       
       static bool validatePrediction(
           const MPCPlanner::Prediction& pred) {
           
           if (pred.modes.empty()) {
               LOG_ERROR("Prediction has no modes");
               return false;
           }
           
           for (const auto& mode : pred.modes) {
               for (const auto& step : mode) {
                   if (!std::isfinite(step.position.x()) || 
                       !std::isfinite(step.position.y())) {
                       LOG_ERROR("Invalid prediction step position");
                       return false;
                   }
               }
           }
           return true;
       }
   };
   ```

2. **Safe Obstacle Update**
   ```cpp
   void JulesJackalPlanner::prepareObstacleData() {
       // Update robot obstacles from trajectories
       updateRobotObstaclesFromTrajectories();
       
       // Validate before sending to solver
       if (!ObstacleDataValidator::validateDynamicObstacles(
               _data.dynamic_obstacles)) {
           LOG_ERROR(_ego_robot_ns + 
                     ": Invalid obstacle data, using safe defaults");
           _data.dynamic_obstacles.clear();
       }
       
       // Ensure correct size
       MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);
       
       // Final validation
       if (!ObstacleDataValidator::validateDynamicObstacles(
               _data.dynamic_obstacles)) {
           LOG_ERROR(_ego_robot_ns + 
                     ": Obstacle data still invalid after padding!");
           transitionTo(PlannerState::ERROR_STATE);
           return;
       }
       
       // Notify planner
       _planner->onDataReceived(_data, "dynamic obstacles");
   }
   ```

### Phase 6: Configuration Consolidation

**Goal:** Group related configuration into logical structures

**Steps:**

1. **Create Configuration Structures**
   ```cpp
   struct PlannerConfig {
       struct Timing {
           double control_frequency{20.0};
           double startup_timeout{20.0};
           double goal_tolerance{0.8};
       } timing;
       
       struct Robot {
           double radius{0.5};
           double deceleration_at_infeasible{1.0};
       } robot;
       
       struct MultiRobot {
           bool wait_for_sync{false};
           bool enable_robot_to_robot_comm{true};
       } multi_robot;
       
       struct Behavior {
           bool enable_output{true};
           bool stop_when_reached_goal{false};
       } behavior;
       
       void loadFromROS(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
           pnh.param("control_frequency", timing.control_frequency, 
                     timing.control_frequency);
           pnh.param("goal_tolerance", timing.goal_tolerance, 
                     timing.goal_tolerance);
           nh.param("/wait_for_sync", multi_robot.wait_for_sync, 
                    multi_robot.wait_for_sync);
           // ... etc
       }
   };
   
   private:
       PlannerConfig _config;
   ```

2. **Use Configuration Structures**
   ```cpp
   // Instead of:
   if (_enable_output && output.success)
   
   // Use:
   if (_config.behavior.enable_output && output.success)
   ```

---

## Implementation Roadmap

### Week 1: Preparation and Infrastructure
- [ ] Create comprehensive unit tests for current behavior
- [ ] Document current state transitions with diagrams
- [ ] Set up feature branch for refactoring
- [ ] Create backup of working version

### Week 2: Core State Machine
- [ ] Implement PlannerState enum and transition logic
- [ ] Add state logging and diagnostics
- [ ] Update constructor to use state machine
- [ ] Test state transitions in isolation

### Week 3: Data Initialization
- [ ] Implement DataReadiness tracker
- [ ] Update all callbacks to set readiness flags
- [ ] Add validation to obstacle initialization
- [ ] Test with delayed/missing callbacks

### Week 4: Loop Consolidation
- [ ] Create mainLoop() with state dispatch
- [ ] Migrate loopDirectTrajectory() logic
- [ ] Remove old loop functions
- [ ] Verify behavior matches original

### Week 5: Reset and Recovery
- [ ] Implement performReset() function
- [ ] Update reset callback
- [ ] Add error state handling
- [ ] Test reset scenarios

### Week 6: Validation and Safety
- [ ] Add ObstacleDataValidator
- [ ] Implement safe defaults
- [ ] Add comprehensive error handling
- [ ] Test edge cases

### Week 7: Configuration
- [ ] Create PlannerConfig structures
- [ ] Migrate all config loading
- [ ] Update code to use new structures
- [ ] Verify no regressions

### Week 8: Testing and Documentation
- [ ] Integration testing with multiple robots
- [ ] Performance testing
- [ ] Update architecture documentation
- [ ] Code review and cleanup

---

## Testing Strategy

### Unit Tests

1. **State Machine Tests**
   ```cpp
   TEST(PlannerStateTest, ValidTransitions) {
       // Test all valid state transitions
       // Verify invalid transitions are rejected
       // Check state entry/exit callbacks
   }
   ```

2. **Data Readiness Tests**
   ```cpp
   TEST(DataReadinessTest, EssentialData) {
       // Test individual readiness flags
       // Test combined readiness logic
       // Test reset functionality
   }
   ```

3. **Validation Tests**
   ```cpp
   TEST(ObstacleValidatorTest, InvalidData) {
       // Test with NaN positions
       // Test with negative radius
       // Test with empty predictions
       // Verify rejection
   }
   ```

### Integration Tests

1. **Multi-Robot Startup**
   - Launch 3 robots with sync enabled
   - Verify all reach PLANNING_ACTIVE state
   - Check obstacle data is complete

2. **Asynchronous Data Arrival**
   - Delay different callbacks
   - Verify graceful handling
   - Check no crashes

3. **Reset Behavior**
   - Reach goal with all robots
   - Trigger reset
   - Verify clean restart

### Simulation Tests

1. **Full Mission Cycle**
   - Start → Plan → Goal → Reset → Repeat
   - Monitor for memory leaks
   - Check performance metrics

2. **Failure Scenarios**
   - Robot loses connection
   - Solver fails repeatedly
   - Invalid obstacle data
   - Verify recovery

---

## Migration Path

### Backward Compatibility

To ensure smooth migration, maintain backward compatibility during transition:

1. **Feature Flags**
   ```cpp
   #define USE_STATE_MACHINE 1  // Toggle new architecture
   
   void JulesJackalPlanner::loop(const ros::TimerEvent& event) {
   #if USE_STATE_MACHINE
       mainLoop(event);
   #else
       loopDirectTrajectory(event);
   #endif
   }
   ```

2. **A/B Testing**
   - Run old and new versions side-by-side
   - Compare trajectories and timing
   - Verify identical behavior

3. **Gradual Rollout**
   - Week 1-2: State machine only
   - Week 3-4: Add data validation
   - Week 5-6: Migrate loop logic
   - Week 7-8: Full migration

---

## Benefits After Refactoring

### For Developers

1. **Easier Debugging**
   - Clear state at any moment
   - Predictable behavior
   - Better error messages

2. **Simpler Extensions**
   - Add new states easily
   - Modify behavior per state
   - Clear entry points

3. **Better Testing**
   - Isolated unit tests
   - Reproducible scenarios
   - Coverage metrics

### For Users

1. **More Reliable**
   - No crashes from uninitialized data
   - Graceful error recovery
   - Predictable startup

2. **Better Diagnostics**
   - Clear status messages
   - State visualization in RViz
   - Debugging tools

3. **Easier Configuration**
   - Organized parameters
   - Clear documentation
   - Sensible defaults

---

## Risk Mitigation

### Potential Risks

1. **Regression Bugs**
   - *Mitigation:* Comprehensive test suite
   - *Mitigation:* A/B testing against original

2. **Performance Impact**
   - *Mitigation:* Profile before/after
   - *Mitigation:* Optimize hot paths

3. **Deployment Disruption**
   - *Mitigation:* Feature flags
   - *Mitigation:* Gradual rollout

4. **Team Learning Curve**
   - *Mitigation:* Documentation
   - *Mitigation:* Code reviews
   - *Mitigation:* Training sessions

---

## Success Metrics

### Code Quality

- [ ] Reduce number of boolean state flags from 6 to 1 (state enum)
- [ ] Reduce cyclomatic complexity by 50%
- [ ] Achieve 80%+ test coverage
- [ ] Zero crashes in 100-hour stress test

### Maintainability

- [ ] New developer can understand code in < 2 hours
- [ ] Bug fix time reduced by 50%
- [ ] Feature addition time reduced by 30%

### Reliability

- [ ] Zero uninitialized data access
- [ ] 100% successful startup in multi-robot scenarios
- [ ] Graceful handling of all callback orders

---

## Appendix A: Current Data Flow Diagram

```
Constructor
    ↓
waitForAllRobotsReady() (optional)
    ↓
initializeOtherRobotsAsObstacles()
    ↓
Timer → loopDirectTrajectory()
    ↓
[Wait for _have_received_meaningful_trajectory_data]
    ↓
    ↓ (async) trajectoryCallback() sets flag
    ↓
[Normal Planning]
    ↓
updateRobotObstaclesFromTrajectories()
    ↓
ensureObstacleSize()
    ↓
solveMPC()
    ↓
publishCommands()
```

## Appendix B: Proposed Data Flow Diagram

```
Constructor
    ↓
State: UNINITIALIZED
    ↓
State: WAITING_FOR_SYNC (if enabled)
    ↓
State: INITIALIZING_OBSTACLES
    ↓ [create obstacle structures]
State: WAITING_FOR_DATA
    ↓ [callbacks populate data]
    ↓ (trajectoryCallback sets readiness)
State: PLANNING_ACTIVE
    ↓ [validate data]
    ↓ [prepare obstacles]
    ↓ [solve MPC]
    ↓ [publish commands]
    ↓ (goal reached)
State: GOAL_REACHED
    ↓ (all robots ready)
State: RESETTING
    ↓ [clean reset]
State: WAITING_FOR_DATA (repeat)
```

## Appendix C: Key Code Patterns

### Before: Boolean Flag Hell
```cpp
if (!_have_received_meaningful_trajectory_data) {
    if (_startup_timer->hasFinished()) {
        if (_goal_received && _enable_output) {
            // Do something
        }
    }
}
```

### After: Clear State Dispatch
```cpp
switch (_current_state) {
    case PlannerState::WAITING_FOR_DATA:
        handleWaitingForData();
        break;
    case PlannerState::PLANNING_ACTIVE:
        handleActivePlanning();
        break;
}
```

---

## Conclusion

This refactoring strategy addresses the core issues in `jules_ros1_jackalplanner.cpp`:
- Replaces complex boolean logic with a clear state machine
- Ensures robust data initialization and validation
- Consolidates three loop functions into one
- Simplifies reset and startup behavior
- Improves code readability and maintainability

The phased approach with comprehensive testing ensures a safe migration path while the new architecture provides a solid foundation for future development.

**Estimated Effort:** 8 weeks with 1 developer
**Risk Level:** Medium (mitigated by testing and gradual rollout)
**Impact:** High (significantly improved maintainability and reliability)
