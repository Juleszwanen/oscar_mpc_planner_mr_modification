# JulesJackalPlanner Architecture Documentation

## Overview

The `JulesJackalPlanner` class serves as a ROS wrapper for the MPC (Model Predictive Control) planner, specifically designed for multi-robot Jackal navigation scenarios. It bridges ROS communication with the underlying MPC solver and handles real-time control loops.

## Class Architecture

### Core Components

- **MPC Planner**: `std::unique_ptr<MPCPlanner::Planner> _planner` - The main MPC solver
- **State Management**: `MPCPlanner::State _state` - Current robot state (x, y, psi, v)
- **Real-time Data**: `MPCPlanner::JulesRealTimeData _data` - Dynamic obstacles, goals, reference paths
- **ROS Communication**: Publishers, subscribers, service clients for system integration

### Key Design Principles

1. **Namespace-Friendly**: All topics are relative, enabling multi-robot deployment
2. **Timer-Driven Control**: Fixed-rate MPC solving for predictable latency
3. **Fail-Safe Design**: Braking commands when solver fails or goals are reached
4. **Modular Integration**: Clean separation between ROS I/O and MPC logic

## Constructor Flow

The constructor follows a specific initialization order for proper MPC setup:

1. **Node Identity Logging**: Extract robot namespace for multi-robot coordination
2. **Robot Discovery**: Find other robots in the system for obstacle management
3. **Configuration Loading**: Initialize CONFIG from YAML files before planner construction
4. **Robot Footprint Setup**: Define collision geometry from configuration
5. **Parameter Reading**: Load node-specific parameters with launch-time overrides
6. **Planner Construction**: Create MPC solver after configuration is ready
7. **ROS I/O Setup**: Wire up all publishers and subscribers
8. **Synchronization**: Wait for all robots to be ready (optional)
9. **Timer Start**: Begin the periodic control loop

## ROS Communication

### Input Topics (Subscribers)

| Topic | Message Type | Purpose | Queue Size |
|-------|-------------|---------|------------|
| `input/state` | `nav_msgs/Odometry` | Optional velocity estimates | 5 |
| `input/state_pose` | `geometry_msgs/PoseStamped` | Ground truth pose (encoded format) | 1 |
| `input/goal` | `geometry_msgs/PoseStamped` | Navigation goals from RViz/planner | 1 |
| `input/reference_path` | `nav_msgs/Path` | Reference path for MPCC modules | 1 |
| `input/obstacles` | `mpc_planner_msgs/ObstacleArray` | Dynamic obstacles from aggregator | 1 |

### Output Topics (Publishers)

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `output/command` | `geometry_msgs/Twist` | Velocity commands to robot |
| `output/pose` | `geometry_msgs/PoseStamped` | Current pose for aggregator/RViz |
| `output/current_trajectory` | `nav_msgs/Path` | Planned trajectory for coordination |
| `events/objective_reached` | `std_msgs/Bool` | Goal completion events |

### Service Clients

| Service | Type | Purpose |
|---------|------|---------|
| `/get_other_robot_obstacles_srv` | `GetOtherTrajectories` | Alternative to pub/sub for obstacle data |

## Core Callback Functions

### State Management

#### `stateCallback(nav_msgs::Odometry)`
- Processes standard ROS odometry messages
- Updates internal state with position, orientation, and velocity
- Includes flip detection for safety

#### `statePoseCallback(geometry_msgs::PoseStamped)`
- **Special Encoded Format**: Uses non-standard encoding where:
  - `pose.orientation.z` contains yaw angle (NOT quaternion)
  - `pose.position.z` contains linear velocity
- Primary state input from `mobile_robot_state_publisher`
- Includes flip detection and reset capability

### Navigation Control

#### `goalCallback(geometry_msgs::PoseStamped)`
- Handles 2D navigation goals (typically from RViz)
- Updates internal goal state and flags
- Resets goal-reached status for new objectives

#### `pathCallback(nav_msgs::Path)`
- Processes reference paths for MPCC/contouring modules
- Implements fast-path checking to avoid unnecessary reprocessing
- Extracts XY waypoints and notifies dependent modules

### Obstacle Management

#### `obstacleCallback(mpc_planner_msgs::ObstacleArray)`
- Primary obstacle data processing from central aggregator
- Supports both deterministic and Gaussian prediction modes
- Handles single-mode predictions with uncertainty ellipses
- Pads/truncates to fixed obstacle count for solver consistency
- Optional uncertainty propagation over prediction horizon

#### `obstacleServiceCallback(mpc_planner_msgs::ObstacleArray)`
- Service-based alternative to pub/sub obstacle communication
- Identical processing logic to `obstacleCallback`
- Used with `loopWithService` for client-server architecture

## Control Loop Architecture

### Main Control Loop (`loop()`)

The timer-driven control loop executes at `_control_frequency` Hz:

1. **Profiling Setup**: Instrument the planning cycle
2. **State Validation**: Check for first-time execution
3. **Goal Status Check**: Handle goal-reached transitions
4. **MPC Solving**: Run the optimization with latest state/data
5. **Command Extraction**: Extract velocity commands from solution
6. **Fail-Safe Logic**: Apply braking on solver failure
7. **Command Publishing**: Send velocity commands to robot
8. **Visualization**: Update RViz markers and trajectories

### Alternative Service Loop (`loopWithService()`)

- Uses service calls instead of subscribers for obstacle data
- Requests trajectories from central service before each MPC solve
- Otherwise identical to main loop architecture

### Command Extraction Convention

- `cmd.angular.z = getSolution(0, "w")` - Angular velocity from stage 0
- `cmd.linear.x = getSolution(1, "v")` - Linear velocity from stage 1

## State Transitions and Fail-Safe Logic

### Goal Management States

1. **No Goal**: `_goal_received = false` - Robot maintains current behavior
2. **Active Navigation**: Goal received, not reached - Normal MPC solving
3. **Goal Reached**: Transition to braking, publish completion event
4. **Maintaining Stop**: Continue stopped state at goal location

### Fail-Safe Braking Logic

Applied when:
- Solver fails (infeasible/no convergence)
- Output is disabled via configuration
- Goal is reached

Braking law: `v_next = max(v - decel * dt, 0)`

## Multi-Robot Coordination

### Robot Discovery and Communication

#### `findOtherRobots()`
- Extracts other robot namespaces from global parameter `/robot_ns_list`
- Populates `_other_robot_nss` set for obstacle subscription

#### `createObstacleRobotsSubs()`
- Creates dynamic subscribers for each other robot's pose and trajectory
- Enables direct robot-to-robot communication for obstacle avoidance

#### `waitForAllRobotsReady()`
- Synchronization mechanism using ROS parameters
- Each robot sets `/<namespace>/ready_to_start = true`
- Blocks until all robots signal readiness
- Prevents timing issues in multi-robot startup

### Trajectory Publishing

#### `publishCurrentTrajectory()`
- Publishes planned trajectory for other robots to use as obstacles
- Converts MPC solution to `nav_msgs::Path` format
- Includes timing information in pose timestamps

## Utility Functions

### Visualization

#### `visualize()`
- Publishes heading ray using ros_tools VISUALS system
- Simple line from robot position extending 1m in heading direction
- Useful for debugging orientation and state feedback

#### `publishPose()`
- Publishes current robot pose in encoded format
- Maintains consistency with `mobile_robot_state_publisher` convention
- Used by aggregator and visualization systems

### Path Comparison

#### `isPathTheSame()`
- Fast O(1) comparison to avoid unnecessary path reprocessing
- Compares path length and first few waypoints
- Uses planner's internal tolerance via `pointInPath()`

### Goal Evaluation

#### `objectiveReached()`
- Delegates to planner's internal goal checking logic
- Enables modular goal criteria (distance, heading, time-based)
- Returns boolean for goal completion status

## Configuration and Parameters

### Key Configuration Sources

1. **YAML Configuration**: Loaded via `SYSTEM_CONFIG_PATH(__FILE__, "settings")`
2. **ROS Parameters**: Node-specific overrides via private parameter server
3. **Global Parameters**: Multi-robot coordination via `/robot_ns_list`

### Critical Parameters

- `control_frequency`: Timer rate for MPC loop (typically 20 Hz)
- `enable_output`: Global enable/disable for command output
- `deceleration_at_infeasible`: Braking rate for fail-safe scenarios
- `goal_tolerance`: Distance threshold for goal completion
- `obstacle_radius`: Default radius for dynamic obstacles
- `max_obstacles`: Fixed obstacle count for solver consistency

## Threading and Safety

### Threading Model

- **Single-Threaded Spinner**: Default ROS spinning mode
- **Callback Serialization**: State updates and control loop are serialized
- **No Mutex Required**: Safe data access under single-threaded execution

### Multi-Threading Considerations

If switching to `MultiThreadedSpinner`:
- Add mutex protection around `_data` and `_state` access
- Consider lock-free patterns for high-frequency state updates
- Profile for potential contention in obstacle processing

## Extension Points

### Adding New Input Sources

1. Create subscriber in `initializeSubscribersAndPublishers()`
2. Implement callback to update `_data` or `_state`
3. Call `_planner->onDataReceived()` if modules need notification

### Custom Obstacle Processing

1. Extend `obstacleCallback()` for new prediction types
2. Add support for multi-modal predictions
3. Implement custom uncertainty propagation

### Alternative Communication Patterns

1. Implement additional service-based interfaces
2. Add support for action servers for long-running goals
3. Integrate with tf2 for coordinate frame transformations

## Performance Considerations

### Optimization Strategies

- **Fast-Path Checking**: Avoid reprocessing unchanged data
- **Fixed-Size Obstacles**: Consistent solver parameter dimensions
- **Profiling Integration**: Built-in timing instrumentation
- **Memory Reservation**: Pre-allocate vectors in hot paths

### Monitoring and Debugging

- **Extensive Logging**: Configurable debug output levels
- **Profiling Scopes**: Performance measurement integration
- **Visualization Support**: Real-time RViz feedback
- **Event Publishing**: External monitoring of goal completion

## Integration with MPC Library

### Data Flow Integration

1. **State Updates**: Callbacks write to `_state` (x, y, psi, v)
2. **Reference Data**: Callbacks populate `_data` (goal, path, obstacles)
3. **Module Notification**: `onDataReceived()` updates solver parameters
4. **Solving**: `solveMPC()` optimizes with current state/data
5. **Solution Extraction**: `getSolution()` retrieves control commands

### Configuration Dependencies

- MPC library expects `CONFIG` to be initialized before `Planner` construction
- Robot footprint must be defined before planner creation
- Module-specific parameters loaded from YAML configuration

This architecture provides a robust, extensible foundation for multi-robot MPC navigation with proper fail-safes, visualization, and coordination mechanisms.