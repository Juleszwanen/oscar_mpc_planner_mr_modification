# State Machine Diagrams for jules_ros1_jackalplanner

## Current Architecture (Before Refactoring)

```
┌─────────────────────────────────────────────────────────────────┐
│                         CONSTRUCTOR                              │
│  - Load config                                                   │
│  - Create planner                                               │
│  - Init subscribers/publishers                                  │
│  - initializeOtherRobotsAsObstacles()                          │
│  - Start timer (if wait_for_sync, wait first)                  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    LOOP FUNCTION CALLED                          │
│                                                                  │
│  Flags consulted:                                               │
│  • _have_received_meaningful_trajectory_data  ────────┐         │
│  • _startup_timer->hasFinished()                      │         │
│  • _goal_reached                                      │         │
│  • _enable_output                                     │         │
│  • _received_obstacle_callback_first_time             │         │
│  • _goal_received                                     │         │
│                                                       │         │
└───────────────────────────────────────────────────────┼─────────┘
                                                        │
                    ┌───────────────────────────────────┘
                    │
    ┌───────────────▼────────────────┐
    │ if (!_have_received_           │
    │  meaningful_trajectory_data)   │
    └───────────────┬────────────────┘
                    │
        ┌───────────▼───────────┐
        │ applyDummyObstacle    │
        │ Command()             │
        │ • Create dummy obs    │
        │ • Call solver         │
        │ • Clear dummy obs     │
        └───────────┬───────────┘
                    │
                    └──────────┐
                               │
    ┌──────────────────────────▼────────────────────────────┐
    │ else (normal planning)                                 │
    │                                                        │
    │  ┌─────────────────────────────────────────────┐      │
    │  │ if (!_goal_reached &&                       │      │
    │  │     objectiveReached())                     │      │
    │  │   → applyBrakingCommand()                   │      │
    │  │   → _goal_reached = true                    │      │
    │  └─────────────────────────────────────────────┘      │
    │                                                        │
    │  ┌─────────────────────────────────────────────┐      │
    │  │ else if (_goal_reached)                     │      │
    │  │   → cmd = (0, 0)                            │      │
    │  └─────────────────────────────────────────────┘      │
    │                                                        │
    │  ┌─────────────────────────────────────────────┐      │
    │  │ else (normal MPC)                           │      │
    │  │   → updateRobotObstacles...()               │      │
    │  │   → ensureObstacleSize()                    │      │
    │  │   → output = solveMPC(_state, _data)        │      │
    │  │   → if (success) extract command            │      │
    │  │   → else applyBrakingCommand()              │      │
    │  └─────────────────────────────────────────────┘      │
    │                                                        │
    │  → publish commands                                    │
    │  → visualize                                           │
    └────────────────────────────────────────────────────────┘

PROBLEMS:
❌ 6+ boolean flags create complex decision tree
❌ No clear initialization sequence
❌ Race conditions on startup
❌ Uninitialized data access possible
❌ Hard to understand current system state
❌ Difficult to add new behaviors
```

## Proposed Architecture (After Refactoring)

```
┌─────────────────────────────────────────────────────────────────┐
│                         CONSTRUCTOR                              │
│  - Load config                                                   │
│  - Create planner                                               │
│  - Init subscribers/publishers                                  │
│  - state = UNINITIALIZED                                        │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    transitionTo(WAITING_FOR_SYNC)
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    STATE: WAITING_FOR_SYNC                       │
│  On entry:                                                       │
│  • if (wait_for_sync_enabled)                                   │
│      waitForAllRobotsReady()                                    │
│  • transitionTo(INITIALIZING_OBSTACLES)                         │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                STATE: INITIALIZING_OBSTACLES                     │
│  On entry:                                                       │
│  • initializeOtherRobotsAsObstacles()                           │
│  • Create trajectory_dynamic_obstacles map                      │
│  • Initialize data_readiness tracker                            │
│  • transitionTo(WAITING_FOR_DATA)                               │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   STATE: WAITING_FOR_DATA                        │
│  Loop behavior:                                                  │
│  • Check data_readiness.areAllRobotTrajectoriesReady()         │
│  • Check startup_timer timeout                                  │
│  • Publish zero velocity command                                │
│  • Log waiting status                                           │
│                                                                  │
│  Callbacks:                                                      │
│  • trajectoryCallback() → sets data_readiness flags             │
│  • When all ready → transitionTo(PLANNING_ACTIVE)               │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   STATE: PLANNING_ACTIVE                         │
│  Loop behavior:                                                  │
│  • Validate essential data                                      │
│  • Update planning timestamp                                    │
│  • prepareObstacleData()                                        │
│    - updateRobotObstaclesFromTrajectories()                     │
│    - validate with ObstacleDataValidator                        │
│    - ensureObstacleSize()                                       │
│    - final validation                                           │
│  • if (objectiveReached())                                      │
│      transitionTo(GOAL_REACHED)                                 │
│  • output = solveMPC(_state, _data)                             │
│  • cmd = generateCommand(output)                                │
│  • publishCmdAndVisualize(cmd, output)                          │
└─────────────────────────────────────────────────────────────────┘
                              ↓ (goal reached)
┌─────────────────────────────────────────────────────────────────┐
│                     STATE: GOAL_REACHED                          │
│  Loop behavior:                                                  │
│  • if (stop_when_reached_goal)                                  │
│      → publish zero velocity                                    │
│      → publishObjectiveReachedEvent()                           │
│  • else                                                          │
│      → rotatePiRadians()                                        │
│      → when complete, publishObjectiveReachedEvent()            │
│                                                                  │
│  Callbacks:                                                      │
│  • allRobotsReachedObjectiveCallback()                          │
│      → transitionTo(RESETTING)                                  │
└─────────────────────────────────────────────────────────────────┘
                              ↓ (all robots ready)
┌─────────────────────────────────────────────────────────────────┐
│                      STATE: RESETTING                            │
│  On entry:                                                       │
│  • _planner->reset(_state, _data, success)                      │
│  • _data_readiness.reset()                                      │
│  • _validated_trajectory_robots.clear()                         │
│  • initializeOtherRobotsAsObstacles()                           │
│  • publish roadmap reverse                                      │
│  • _startup_timer.start()                                       │
│  • transitionTo(WAITING_FOR_DATA)                               │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    (cycle back to WAITING_FOR_DATA)


                    ┌─────────────────────────────────┐
                    │      STATE: ERROR_STATE          │
                    │  • Log error                     │
                    │  • Publish zero velocity         │
                    │  • Wait for manual intervention  │
                    └─────────────────────────────────┘
                         ↑ (on validation failure)

BENEFITS:
✅ Single state variable, clear current status
✅ Well-defined initialization sequence
✅ No race conditions on startup
✅ All data validated before use
✅ Easy to understand and debug
✅ Simple to extend with new states
```

## State Transition Matrix

| From State               | To State                 | Trigger                           | Validation                    |
|-------------------------|--------------------------|-----------------------------------|-------------------------------|
| UNINITIALIZED           | WAITING_FOR_SYNC         | Constructor completes             | None                          |
| WAITING_FOR_SYNC        | INITIALIZING_OBSTACLES   | All robots ready OR sync disabled | None                          |
| INITIALIZING_OBSTACLES  | WAITING_FOR_DATA         | Obstacles initialized             | trajectory_dynamic_obstacles created |
| WAITING_FOR_DATA        | PLANNING_ACTIVE          | All trajectory data ready         | data_readiness.areAllReady()  |
| WAITING_FOR_DATA        | PLANNING_ACTIVE          | Startup timeout                   | timeout expired               |
| PLANNING_ACTIVE         | GOAL_REACHED             | Objective reached                 | objectiveReached() == true    |
| PLANNING_ACTIVE         | ERROR_STATE              | Data validation fails             | Invalid obstacle data         |
| GOAL_REACHED            | RESETTING                | All robots at goal                | All objective events received |
| RESETTING               | WAITING_FOR_DATA         | Reset complete                    | Data structures cleared       |
| ERROR_STATE             | RESETTING                | Manual recovery                   | User intervention             |

## Data Readiness Tracking

```
┌─────────────────────────────────────────────────────────────────┐
│                    DataReadiness Structure                       │
│                                                                  │
│  Essential Data:                                                 │
│  ├─ state_received         (from statePoseCallback)            │
│  ├─ goal_received          (from goalCallback)                 │
│  ├─ path_received          (from pathCallback)                 │
│  └─ static_obstacles_received                                  │
│                                                                  │
│  Per-Robot Trajectory Data:                                     │
│  ├─ robot_trajectory_ready["/jackal1"] = false                 │
│  ├─ robot_trajectory_ready["/jackal2"] = false                 │
│  └─ robot_trajectory_ready["/jackal3"] = false                 │
│                                                                  │
│  Methods:                                                        │
│  ├─ isEssentialDataReady() → bool                              │
│  ├─ areAllRobotTrajectoriesReady() → bool                      │
│  └─ reset() → void                                              │
└─────────────────────────────────────────────────────────────────┘
```

## Callback Flow in New Architecture

```
Callback Events                    State Machine Response
─────────────────                  ──────────────────────

statePoseCallback()
    ↓
    data_readiness.state_received = true
    ↓
    (no state transition)


goalCallback()
    ↓
    data_readiness.goal_received = true
    ↓
    (no state transition)


trajectoryCallback(ns)
    ↓
    Validate: obstacle exists in map?
    ↓
    Update trajectory_dynamic_obstacles[ns]
    ↓
    data_readiness.robot_trajectory_ready[ns] = true
    ↓
    if (current_state == WAITING_FOR_DATA &&
        data_readiness.areAllRobotTrajectoriesReady())
        ↓
        transitionTo(PLANNING_ACTIVE)


allRobotsReachedObjectiveCallback()
    ↓
    Validate: current_state == GOAL_REACHED?
    ↓
    transitionTo(RESETTING)
```

## Comparison: Before vs After

| Aspect                  | Before (Current)                       | After (Proposed)                    |
|------------------------|----------------------------------------|-------------------------------------|
| **State Tracking**     | 6+ boolean flags                       | 1 enum PlannerState                 |
| **Initialization**     | Implicit, scattered                    | Explicit state machine              |
| **Data Validation**    | Minimal, ad-hoc                        | Comprehensive, structured           |
| **Startup Sequence**   | Unclear, race conditions possible      | Well-defined, deterministic         |
| **Error Handling**     | Braking commands, no recovery          | ERROR_STATE with recovery path      |
| **Debugging**          | Print flags, guess state               | Log state, know state exactly       |
| **Testing**            | Integration tests only                 | Unit test state transitions         |
| **Extension**          | Modify conditionals throughout         | Add new state, implement handler    |
| **Code Complexity**    | O(n) conditionals                      | O(1) state dispatch                 |
| **Maintainability**    | Low (hard to understand)               | High (clear structure)              |

## Example: Handling Late Trajectory Data

### Before (Current):
```cpp
void loopDirectTrajectory() {
    if (!_have_received_meaningful_trajectory_data) {
        applyDummyObstacleCommand();  // Creates dummy, plans, clears
        return;
    }
    
    // ... normal planning but might still have incomplete data
    updateRobotObstaclesFromTrajectories();  // May crash if not initialized
}

void trajectoryCallback(msg, ns) {
    auto it = _data.trajectory_dynamic_obstacles.find(ns);
    if (it == end()) {  // Race condition: obstacle not initialized!
        LOG_WARN("Received trajectory before initialization");
        return;  // Data lost!
    }
    // ... update
    _have_received_meaningful_trajectory_data = true;  // Side effect
}
```

### After (Proposed):
```cpp
void mainLoop() {
    switch (_current_state) {
        case WAITING_FOR_DATA:
            handleWaitingForData();  // Publish zero velocity, log status
            return;
        case PLANNING_ACTIVE:
            handleActivePlanning();  // Data guaranteed to be ready
            return;
    }
}

void trajectoryCallback(msg, ns) {
    // Obstacle guaranteed to exist (created in INITIALIZING_OBSTACLES state)
    auto& obstacle = _data.trajectory_dynamic_obstacles[ns];
    
    // Update data
    updateObstacleFromMessage(obstacle, msg);
    
    // Mark ready
    _data_readiness.robot_trajectory_ready[ns] = true;
    
    // Trigger transition if all data ready
    if (_current_state == WAITING_FOR_DATA &&
        _data_readiness.areAllRobotTrajectoriesReady()) {
        transitionTo(PLANNING_ACTIVE);
    }
}
```

---

## Key Takeaways

1. **State Machine eliminates flag complexity**
   - 6+ flags → 1 state enum
   - Complex conditionals → Simple dispatch

2. **Explicit initialization prevents crashes**
   - Guaranteed structure creation
   - Validated data before planning
   - No race conditions

3. **Clear lifecycle improves debugging**
   - Know exact state at any moment
   - Log state transitions
   - Reproducible behavior

4. **Easier to extend and maintain**
   - Add new states without touching old code
   - State-specific behavior isolated
   - Unit testable

5. **Robust error handling**
   - ERROR_STATE for recovery
   - Validation at every step
   - Graceful degradation
