# RViz Visualization Documentation

## Overview

This document provides a comprehensive guide to all RViz visualizations available in the MPC Planner system. The visualizations are essential for debugging, monitoring, and understanding the behavior of the Model Predictive Controller during autonomous navigation in dynamic environments.

The visualization system is built on top of the `ros_tools` library, using `ROSMarkerPublisher` to publish `visualization_msgs/MarkerArray` messages that can be displayed in RViz.

---

## Table of Contents

1. [Visualization Architecture](#visualization-architecture)
2. [Core Planner Visualizations](#core-planner-visualizations)
3. [Module-Specific Visualizations](#module-specific-visualizations)
4. [Guidance Planner Visualizations](#guidance-planner-visualizations)
5. [Debug Visualizations](#debug-visualizations)
6. [Configuration Reference](#configuration-reference)
7. [RViz Setup Guide](#rviz-setup-guide)
8. [rqt GUI Concept](#rqt-gui-concept)

---

## Visualization Architecture

### File Locations

| Component | File Path |
|-----------|-----------|
| Core Visualization Functions | `mpc_planner_util/src/data_visualization.cpp` |
| Visualization Headers | `mpc_planner_util/include/mpc_planner_util/data_visualization.h` |
| Planner Visualization | `mpc_planner/src/planner.cpp` |
| Module Visualizations | `mpc_planner_modules/src/*.cpp` |
| RViz Configuration | `mpc_planner_jackalsimulator/rviz/*.rviz` |

### Topic Naming Convention

All visualization topics follow this naming pattern:
```
/<planner_namespace>/<module_name>/<visualization_name>
```

Example:
```
/jackalsimulator_planner/contouring/path
/jackalsimulator_planner/guidance_constraints/optimized_trajectories
```

---

## Core Planner Visualizations

### 1. Planned Trajectory

**Name:** Planned Trajectory

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 262)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeTrajectory`)

**Purpose:**
Displays the optimal trajectory computed by the MPC solver for the current planning cycle. This is the trajectory that the robot will execute (first control input applied, rest is a prediction).

**What It Shows:**
- **Cylinders**: Circular footprints at each predicted position along the trajectory
- **Lines**: Connecting lines between consecutive predicted positions
- **Z-offset**: Elevated slightly (0.0-0.05m) for visibility

**Interpretation:**
- **Dark magenta/purple trajectory**: The selected optimal trajectory (elevated when `color_index = -1`)
- **Trajectory length**: Spans from current position to N time steps ahead
- **Cylinder size**: Robot radius (2 × `robot_radius`)

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/planned_trajectory` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `robot_radius` | `settings.yaml` | Cylinder diameter |
| `N` | `settings.yaml` | Number of prediction steps shown |
| `alpha` | Code (default 0.2) | Transparency |

**When to Enable / Disable:**
- **Enable**: Always recommended for basic operation monitoring
- **Disable**: Never - this is essential for understanding robot behavior

---

### 2. Robot Area (Circular)

**Name:** Robot Area

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 269)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeRobotArea`)

**Purpose:**
Shows the current robot footprint as a set of circular discs. The robot may be modeled as multiple discs for more accurate collision checking.

**What It Shows:**
- **Cylinders**: One or more circular discs representing the robot's collision footprint
- **Position**: At current robot position with correct orientation

**Interpretation:**
- Each cylinder represents a collision disc
- The discs' combined area shows what volume must remain collision-free
- Color is indexed (default color index 0)

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/robot_area` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `n_discs` | `settings.yaml` | Number of discs |
| `robot_radius` | `settings.yaml` | Disc radius |

**When to Enable / Disable:**
- **Enable**: When debugging collision shapes or robot modeling
- **Disable**: For cleaner visualization when not needed

---

### 3. Robot Rectangular Area

**Name:** Robot Rectangular Area

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 275-277)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeRectangularRobotArea`)

**Purpose:**
Displays the robot's rectangular footprint (length × width) for platforms like the Jackal where a rectangular bounding box is more accurate than circles.

**What It Shows:**
- **Cube marker**: Rectangle at current position and orientation
- **Dimensions**: Robot length and width

**Interpretation:**
- Shows the actual physical dimensions of the robot
- Oriented according to current heading (psi)

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/robot_rect_area` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `robot.length` | `settings.yaml` | Rectangle length (m) |
| `robot.width` | `settings.yaml` | Rectangle width (m) |

**When to Enable / Disable:**
- **Enable**: For accurate footprint visualization on rectangular robots
- **Disable**: When using disc-based collision checking

---

### 4. Robot Area Trajectory

**Name:** Robot Area Trajectory

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 279)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeRobotAreaTrajectory`)

**Purpose:**
Shows the predicted robot footprint at each step along the planned trajectory. Essential for understanding if the planned trajectory is collision-free.

**What It Shows:**
- **Cylinders**: Robot footprint discs at each prediction step k=1 to N-1
- **Semi-transparent**: Alpha = 0.1 to avoid visual clutter

**Interpretation:**
- Shows the "swept volume" of the robot along the trajectory
- If any cylinder overlaps with an obstacle, collision may occur
- Useful for verifying collision avoidance constraints

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/robot_area_trajectory` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `robot_radius` | `settings.yaml` | Disc radius |
| `alpha` | Code (0.1) | Transparency |

**When to Enable / Disable:**
- **Enable**: When debugging collision avoidance or verifying safety margins
- **Disable**: For cleaner visualization (can be visually dense)

---

### 5. Dynamic Obstacles

**Name:** Dynamic Obstacles

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 267)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeObstacles`)

**Purpose:**
Displays the current positions of dynamic obstacles (pedestrians, other robots, moving objects) as detected by the perception system.

**What It Shows:**
- **3D model markers**: Human pedestrian models or other object representations
- **Color**: Color-coded by obstacle index using BRUNO colormap
- **Orientation**: Shows heading direction of each obstacle

**Interpretation:**
- Each marker represents one dynamic obstacle
- Only DYNAMIC type obstacles are shown (STATIC obstacles are handled separately)
- Position reflects the current detected position

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/obstacles` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `max_obstacles` | `settings.yaml` | Maximum number of tracked obstacles |
| `obstacle_radius` | `settings.yaml` | Default obstacle size |

**When to Enable / Disable:**
- **Enable**: Always - essential for understanding environment
- **Disable**: Never

---

### 6. Obstacle Predictions

**Name:** Obstacle Predictions

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (lines 285-294)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeObstaclePredictions`, `visualizeObstaclePredictionsWithTime`)

**Purpose:**
Shows the predicted future positions of dynamic obstacles over the planning horizon. This is what the MPC optimizes against.

**What It Shows:**
- **Cylinders**: Obstacle footprints at each predicted time step
- **Color**: Indexed by obstacle ID using BRUNO colormap
- **Z-position**: Optional time-based elevation (z = k × dt for temporal visualization)

**Interpretation:**
- Each set of cylinders shows one obstacle's predicted trajectory
- Helps verify that obstacle predictions are reasonable
- The robot's planned trajectory should avoid these predicted positions

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/obstacle_predictions` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `integrator_step` | `settings.yaml` | dt for time-based z-offset |
| `alpha` | Code (0.8) | Transparency |

**When to Enable / Disable:**
- **Enable**: When debugging obstacle prediction integration or collision avoidance
- **Disable**: Default off due to visual density

---

### 7. Warmstart Trajectory (Debug)

**Name:** Warmstart Trajectory

**File / Package Location:**
- Implementation: `mpc_planner/src/planner.cpp` (line 264-265)
- Visualization function: `mpc_planner_util/src/data_visualization.cpp` (`visualizeTrajectory`)

**Purpose:**
Shows the initial guess provided to the MPC solver (from the previous solution or a braking trajectory).

**What It Shows:**
- Same as Planned Trajectory but shows the warmstart input
- Color-coded differently to distinguish from optimized trajectory

**Interpretation:**
- Compare with final planned trajectory to see how much the solver refined it
- Large differences indicate the warmstart was far from optimal
- Small differences indicate good temporal consistency

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/warmstart_trajectory` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `debug_visuals` | `settings.yaml` | Must be `true` to enable |

**When to Enable / Disable:**
- **Enable**: Only when debugging solver convergence or warmstart quality
- **Disable**: Normal operation (only visible when `debug_visuals: true`)

---

## Module-Specific Visualizations

### 8. Reference Path

**Name:** Reference Path (Contouring Module)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 331-337)
- Visualization functions: `visualizePathPoints()`, `visualizeSpline()`

**Purpose:**
Displays the global reference path that the robot should follow (from the global planner or user input).

**What It Shows:**
- **Points (CYLINDER markers)**: Small black cylinders at each input path waypoint
- **Lines**: Smooth spline curve connecting the waypoints

**Interpretation:**
- Black dots: Original input waypoints
- Colored line: Interpolated spline used for contouring
- Robot tries to minimize contouring error (distance to this path)

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/path` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| Color index | Code (5 of 10) | Path line color |
| Point size | Code (0.15) | Waypoint marker size |

**When to Enable / Disable:**
- **Enable**: Always recommended to understand path following
- **Disable**: Never during normal operation

---

### 9. Road Boundary Constraints

**Name:** Road Boundary Constraints (Contouring Module)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 339-356)
- Visualization function: `visualizeLinearConstraint()`

**Purpose:**
Shows the linear constraints that keep the robot within the road boundaries.

**What It Shows:**
- **Lines**: Halfspace boundaries (linear constraints) at each time step
- **Color**: Time-indexed (rainbow from k=0 to k=N)

**Interpretation:**
- Each line is one side of a constraint: robot must stay on the "inside"
- Two constraints per time step (left and right boundaries)
- Useful for verifying road geometry is correct

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/road_boundary_constraints` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `road.width` | `settings.yaml` | Distance between boundaries |
| `road.two_way` | `settings.yaml` | If true, asymmetric road bounds |
| `alpha` | Code (0.5) | Line transparency |
| `thickness` | Code (0.1) | Line thickness |

**When to Enable / Disable:**
- **Enable**: When debugging road constraint issues
- **Disable**: Normal operation (can be visually cluttered)

---

### 10. Current Segment (Debug)

**Name:** Current Segment (Contouring Module)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 290-302)

**Purpose:**
Shows the closest point on the reference path to the robot.

**What It Shows:**
- **Cube marker**: At the start of the current spline segment

**Interpretation:**
- Indicates which part of the reference path is being tracked
- Useful for verifying closest point computation

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/current` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `debug_visuals` | `settings.yaml` | Must be `true` |

**When to Enable / Disable:**
- **Enable**: Only when debugging path tracking issues
- **Disable**: Normal operation

---

### 11. Tracked Path Section (Debug)

**Name:** Tracked Path Section (Contouring Module)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 304-328)

**Purpose:**
Shows the segment of the path currently being tracked by the MPC.

**What It Shows:**
- **Line**: The `n_segments` spline sections being optimized against

**Interpretation:**
- Shows exactly which path segment the solver sees
- Helps debug "which segment am I on" issues

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/tracked_path` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `contouring.num_segments` | `settings.yaml` | Number of segments shown |
| `debug_visuals` | `settings.yaml` | Must be `true` |

**When to Enable / Disable:**
- **Enable**: Debugging contouring module
- **Disable**: Normal operation

---

### 12. Linearized Constraints

**Name:** Linearized Constraints

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/linearized_constraints.cpp` (lines 218-234)
- Visualization function: `visualizeLinearConstraint()`

**Purpose:**
Displays the linearized halfspace constraints used for obstacle avoidance (when using linearized collision constraints instead of ellipsoidal).

**What It Shows:**
- **Lines**: Linear separating hyperplanes between robot and obstacles
- **Color**: Time-indexed (k=0 to k=N)

**Interpretation:**
- Each line separates the robot from one obstacle at one time step
- Robot position must satisfy: `a1*x + a2*y <= b`
- Normal vector points from obstacle to robot

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/linearized_constraints` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `debug_visuals` | `settings.yaml` | Hidden unless `true` for guidance constraints |

**When to Enable / Disable:**
- **Enable**: When debugging linearized collision avoidance
- **Disable**: Normal operation (cluttered visualization)

---

### 13. Gaussian Constraints (Uncertainty Ellipses)

**Name:** Gaussian Constraints

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/gaussian_constraints.cpp` (lines 107-133)

**Purpose:**
Visualizes the uncertainty ellipses for probabilistic obstacle avoidance. Shows how obstacle uncertainty grows over the prediction horizon.

**What It Shows:**
- **Cylinders (elliptical)**: Uncertainty regions around obstacle predictions
- **Color**: Time-indexed

**Interpretation:**
- Larger ellipses at later time steps = more uncertainty
- Robot must stay outside these inflated regions with probability >= (1 - risk)
- The `chi` factor scales the ellipse based on risk tolerance

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/gaussian_constraints` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `probabilistic.risk` | `settings.yaml` | Risk tolerance (smaller = larger ellipses) |
| `visualization.draw_every` | `settings.yaml` | Only draw every N-th step |

**When to Enable / Disable:**
- **Enable**: When using chance-constrained MPC
- **Disable**: When using deterministic constraints

---

### 14. Free Space (Decomposition Constraints)

**Name:** Free Space Polygons

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/decomp_constraints.cpp` (lines 220-278)

**Purpose:**
Visualizes the convex free-space corridors computed by the decomposition utility for static obstacle avoidance.

**What It Shows:**
- **Polygon lines**: Convex polygon boundaries at each time step
- **Color**: Time-indexed

**Interpretation:**
- Robot must stay inside these convex polygons
- Larger polygons = more freedom of movement
- Computed based on costmap occupied cells

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/free_space` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `decomp.range` | `settings.yaml` | Search radius for obstacles |
| `visualization.draw_every` | `settings.yaml` | Visualization sampling rate |

**When to Enable / Disable:**
- **Enable**: When using decomposition-based static obstacle avoidance
- **Disable**: When not using costmap-based constraints

---

### 15. Goal Marker

**Name:** Goal Marker (Goal Module)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/goal_module.cpp` (lines 58-72)

**Purpose:**
Shows the target goal position that the robot is navigating toward.

**What It Shows:**
- **Sphere**: At the goal (x, y) position

**Interpretation:**
- Robot tries to minimize distance to this point
- Only shown when goal has been received

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/goal_module` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| Color index | Code (5) | Sphere color |
| Scale | Code (0.4) | Sphere diameter |

**When to Enable / Disable:**
- **Enable**: When using goal-based navigation
- **Disable**: When using path following only

---

## Guidance Planner Visualizations

The T-MPC++ system uses a guidance planner to generate topologically distinct trajectories. These visualizations help understand the topology exploration.

### 16. Start and Goals (Guidance Planner)

**Name:** Start and Goals

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Shows the start position and goal grid used by the guidance planner.

**What It Shows:**
- **Markers**: Start position and goal points along the reference path

**Interpretation:**
- Goals form a grid: longitudinal (along path) × lateral (perpendicular)
- The guidance planner searches from start to these goals

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/start_and_goals` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: When debugging guidance planner goal generation
- **Disable**: Normal operation

---

### 17. Graph (Guidance Planner)

**Name:** Graph

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Shows the motion primitive graph searched by the guidance planner.

**What It Shows:**
- **Lines/nodes**: The graph structure used for path search

**Interpretation:**
- Nodes: Discrete states in space-time
- Edges: Feasible motion primitives
- Search finds paths through this graph

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/graph` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: Only for detailed guidance planner debugging
- **Disable**: Normal operation (very dense visualization)

---

### 18. Geometric Paths (Guidance Planner)

**Name:** Geometric Paths

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Shows the raw geometric paths found by the graph search before smoothing.

**What It Shows:**
- **Lines**: Piecewise linear paths through the graph

**Interpretation:**
- Each path represents one topology class
- These are converted to smooth splines for MPC initialization

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/geometric_paths` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: When debugging topology discovery
- **Disable**: Normal operation

---

### 19. Guidance Trajectories (Guidance Planner)

**Name:** Guidance Trajectories

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Shows the smoothed guidance trajectories used to initialize the parallel MPC solvers.

**What It Shows:**
- **Lines/curves**: Smooth spline trajectories, one per topology class
- **Color**: Indexed by topology class

**Interpretation:**
- Each trajectory represents a different way to pass obstacles
- One MPC solver is initialized with each trajectory
- Color consistency helps track which topology is being followed

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/trajectories` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: Highly recommended for T-MPC understanding
- **Disable**: Only if visualization is too cluttered

---

### 20. 3D Obstacles (Guidance Planner)

**Name:** 3D Obstacles

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Shows obstacles in 3D space-time representation used by the guidance planner.

**What It Shows:**
- **3D volumes**: Obstacle "tubes" in (x, y, t) space

**Interpretation:**
- Z-axis represents time
- Trajectories must avoid these space-time volumes
- Useful for understanding obstacle motion predictions in guidance

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/obstacles_3d` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: For detailed space-time debugging
- **Disable**: Normal operation (can be confusing in 2D view)

---

### 21. Homology (Guidance Planner)

**Name:** Homology

**File / Package Location:**
- Implementation: `guidance_planner` package (external)

**Purpose:**
Visualizes the homology/topology class information for paths.

**What It Shows:**
- **Markers**: Indicating which homotopy class each path belongs to

**Interpretation:**
- Paths with same homology pass obstacles on the same sides
- Different homology = fundamentally different routing decisions

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_planner/homology` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: When debugging topology classification
- **Disable**: Normal operation

---

### 22. Optimized Trajectories (T-MPC)

**Name:** Optimized Trajectories

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/guidance_constraints.cpp` (lines 586-675)
- Visualization function: `visualizeTrajectory()`

**Purpose:**
Shows all the optimized MPC trajectories from parallel solvers, including the selected one.

**What It Shows:**
- **Multiple trajectory lines**: One per successful parallel solver
- **Highlighted trajectory**: The selected best trajectory (elevated, different color)
- **Non-guided markers** (optional): Yellow cubes (selected) or cyan spheres (not selected) for T-MPC++ planner

**Interpretation:**
- **Elevated dark magenta line**: The SELECTED trajectory that will be executed
- **Colored lines**: Other feasible trajectories from different topology classes
- **Yellow cubes on trajectory**: Non-guided (T-MPC++) planner was selected
- **Cyan spheres**: Non-guided planner trajectory (not selected)

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_constraints/optimized_trajectories` | `visualization_msgs/MarkerArray` |
| `/<ns>/guidance_constraints/non_guided_markers` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `t-mpc.highlight_selected` | `settings.yaml` | Highlight best trajectory |
| `JULES.have_non_guided_standout` | `settings.yaml` | Add special markers for non-guided |

**When to Enable / Disable:**
- **Enable**: Essential for understanding T-MPC decision making
- **Disable**: Never when using T-MPC++

---

### 23. Warmstart Trajectories (T-MPC Debug)

**Name:** Warmstart Trajectories

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/guidance_constraints.cpp` (lines 600-606)

**Purpose:**
Shows the initial guess trajectories provided to each parallel solver.

**What It Shows:**
- **Multiple lines**: One per planner, showing what each solver started with

**Interpretation:**
- Compare with optimized trajectories to see solver convergence
- Good warmstarts lead to faster convergence

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/guidance_constraints/warmstart_trajectories` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `debug_visuals` | `settings.yaml` | Must be `true` |

**When to Enable / Disable:**
- **Enable**: When debugging T-MPC warmstart quality
- **Disable**: Normal operation

---

## Debug Visualizations

### 24. Road Boundary Points (Debug)

**Name:** Road Boundary Points

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 358-393)

**Purpose:**
Shows the computed boundary points used for road constraint generation.

**What It Shows:**
- **Cubes**: Path centerline points, left boundary points, right boundary points

**Interpretation:**
- Helps verify road geometry computation
- Shows where boundaries are placed relative to centerline

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/road_boundary_points` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: Only when debugging road constraint issues
- **Disable**: Normal operation

---

### 25. Glued Spline Points (Debug)

**Name:** Glued Spline Points

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 395-469)

**Purpose:**
Debug visualization showing how multiple spline segments are blended together.

**What It Shows:**
- **Points**: The result of sigmoid blending between adjacent spline segments

**Interpretation:**
- Shows the exact path points the solver sees internally
- Useful for debugging contouring error computation

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/glued_spline_points` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: Only for advanced contouring debugging
- **Disable**: Normal operation

---

### 26. Spline Variables (Debug)

**Name:** Spline Variables

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/contouring.cpp` (lines 471-489)

**Purpose:**
Shows the spline parameter (s) values at each prediction step.

**What It Shows:**
- **Points**: Path positions corresponding to the spline variable at each k

**Interpretation:**
- Shows where along the path the solver expects to be at each time step
- Helps debug progress tracking

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/contouring/spline_variables` | `visualization_msgs/MarkerArray` |

**When to Enable / Disable:**
- **Enable**: When debugging spline parameter integration
- **Disable**: Normal operation

---

### 27. Costmap Points (Debug)

**Name:** Map (Occupied Cells)

**File / Package Location:**
- Implementation: `mpc_planner_modules/src/decomp_constraints.cpp` (lines 265-277)

**Purpose:**
Visualizes the occupied cells from the costmap used for decomposition.

**What It Shows:**
- **Cubes**: Each occupied cell in the costmap

**Interpretation:**
- Black cubes represent obstacles in the costmap
- These are what the decomposition algorithm avoids

**Relevant ROS Topics & Message Types:**
| Topic | Message Type |
|-------|--------------|
| `/<ns>/map` | `visualization_msgs/MarkerArray` |

**Configurable Parameters:**
| Parameter | Location | Effect |
|-----------|----------|--------|
| `debug_visuals` | `settings.yaml` | Must be `true` |

**When to Enable / Disable:**
- **Enable**: When debugging costmap integration
- **Disable**: Normal operation

---

## Configuration Reference

### Key Configuration Parameters

Located in `mpc_planner_<system>/config/settings.yaml`:

```yaml
# Enable/disable debug visualizations
debug_output: false        # Console debug output
debug_limits: false        # Show when limits are hit
debug_visuals: true        # Enable extra visualizations

# Visualization sampling
visualization:
  draw_every: 5            # Draw every N-th time step

# Robot parameters (affect footprint visualization)
robot_radius: 0.325        # Collision disc radius
robot:
  length: 0.65             # Rectangle length
  width: 0.65              # Rectangle width

# T-MPC visualization
t-mpc:
  highlight_selected: true # Highlight best trajectory

# Custom visualization options
JULES:
  have_non_guided_standout: true  # Add markers for non-guided planner
```

### Enabling Debug Visualizations

To enable all debug visualizations, set in `settings.yaml`:
```yaml
debug_visuals: true
```

This enables:
- Warmstart trajectories
- Road boundary points
- Glued spline points
- Spline variables
- Costmap points
- Extended constraint visualizations

---

## RViz Setup Guide

### Loading Pre-configured RViz

The repository includes pre-configured RViz files:

```bash
# For ROS1:
rosrun rviz rviz -d $(rospack find mpc_planner_jackalsimulator)/rviz/ros1.rviz

# For ROS2:
rviz2 -d $(ros2 pkg prefix mpc_planner_jackalsimulator)/share/mpc_planner_jackalsimulator/rviz/ros2.rviz
```

### Available RViz Configurations

| File | Purpose |
|------|---------|
| `ros1.rviz` | Standard 2D top-down view |
| `ros1_3d.rviz` | 3D perspective view |
| `ros1_2dcamera.rviz` | Fixed 2D camera view |
| `hsignature.rviz` | Homology signature visualization |
| `prm.rviz` | Probabilistic roadmap visualization |

### Adding Visualization Topics Manually

1. In RViz, click "Add" in the Displays panel
2. Select "By topic" tab
3. Navigate to `/<planner_namespace>/`
4. Select the desired `MarkerArray` topic
5. Click OK

### Recommended Display Settings

For best visualization experience:

| Setting | Recommended Value |
|---------|-------------------|
| Fixed Frame | `world` or `odom` |
| Background Color | Light gray (230, 230, 230) |
| Grid | Enabled, 1m cell size |
| Decay Time (MarkerArray) | 0.0 (instant updates) |

---

## rqt GUI Concept

### Overview

This section presents a concept for an rqt GUI plugin that displays key runtime metrics for the MPC planner, including communication strategies and performance statistics.

### Proposed Plugin: MPC Planner Dashboard

**Plugin Name:** `mpc_planner_dashboard`

**Purpose:** Provide real-time monitoring of MPC planner performance, topology selection, and multi-robot communication strategies.

### GUI Layout Concept

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        MPC Planner Dashboard                             │
├─────────────────────────────────────────────────────────────────────────┤
│  Robot: [jackal1 ▼]                                    Status: ● RUNNING │
├──────────────────────────────┬──────────────────────────────────────────┤
│     SOLVER PERFORMANCE       │       COMMUNICATION METRICS              │
├──────────────────────────────┼──────────────────────────────────────────┤
│                              │                                          │
│  Solve Time: ██████░░ 35ms   │  Last Trigger: TOPOLOGY_CHANGE          │
│  Success Rate: 98.5%         │                                          │
│  Iterations: 12              │  Communication Triggers Distribution:    │
│                              │  ┌────────────────────────────────────┐  │
│  Exit Code: 1 (SUCCESS)      │  │ TIME         ████████████  45%     │  │
│                              │  │ TOPOLOGY     ██████       25%     │  │
│  Objective: 142.35           │  │ GEOMETRIC    ████         15%     │  │
│                              │  │ NON_GUIDED   ██           10%     │  │
│                              │  │ INFEASIBLE   █             5%     │  │
│                              │  └────────────────────────────────────┘  │
├──────────────────────────────┼──────────────────────────────────────────┤
│     TOPOLOGY SELECTION       │      TRAJECTORY STATISTICS              │
├──────────────────────────────┼──────────────────────────────────────────┤
│                              │                                          │
│  Current Topology: 2         │  Trajectory Cost: 142.35                 │
│  Previous Topology: 2        │  Horizon Length: 30 steps                │
│  Topology Switch: NO         │  Time Step: 0.2s                         │
│                              │                                          │
│  Using Guidance: YES         │  [▶ Trajectory Plot over time ───────]   │
│  Planner Index: 2            │                                          │
│                              │  Communication Savings: 78%              │
│  Topology Distribution:      │  Messages Sent: 1,234                    │
│  ┌────────────────────────┐  │  Messages Saved: 4,321                   │
│  │ T0  ████████    35%    │  │                                          │
│  │ T1  ██████      25%    │  │                                          │
│  │ T2  ████████    35%    │  │                                          │
│  │ T++  ██          5%    │  │                                          │
│  └────────────────────────┘  │                                          │
├──────────────────────────────┴──────────────────────────────────────────┤
│                          TIMELINE VIEW                                   │
├──────────────────────────────────────────────────────────────────────────┤
│  Time: 0s      10s      20s      30s      40s      50s      60s         │
│        │        │        │        │        │        │        │          │
│  Topo: ████2████████2██████████3██████████2████████2████████2████       │
│  Comm: │  C  │     │  C  │  C  │     │     │     │  C  │     │          │
│                                                                          │
│  C = Communication event    Colors = Topology class                      │
└──────────────────────────────────────────────────────────────────────────┘
```

### Key Metrics Displayed

#### 1. Solver Performance Panel
- **Solve Time**: Real-time bar graph showing MPC solve duration
- **Success Rate**: Percentage of successful optimizations
- **Iterations**: Number of solver iterations used
- **Exit Code**: Current solver status with human-readable explanation
- **Objective Value**: Cost function value of selected trajectory

#### 2. Communication Metrics Panel
- **Last Trigger**: Most recent communication trigger reason
- **Trigger Distribution**: Pie/bar chart showing breakdown of why communications occur:
  - `NO_COMMUNICATION` - Skipped (same topology)
  - `INFEASIBLE` - Solver failed
  - `INFEASIBLE_TO_FEASIBLE` - Recovery from failure
  - `TOPOLOGY_CHANGE` - Switched guided topology
  - `GEOMETRIC` - Trajectory deviation exceeded threshold
  - `TIME` - Heartbeat timer triggered
  - `NON_GUIDED_HOMOLOGY_FAIL` - Using non-guided planner

#### 3. Topology Selection Panel
- **Current/Previous Topology**: Track topology switches
- **Using Guidance**: Whether guided or non-guided planner selected
- **Planner Index**: Which parallel solver was chosen
- **Topology Distribution**: How often each topology class is selected

#### 4. Trajectory Statistics Panel
- **Trajectory Cost**: Objective value of current plan
- **Communication Savings**: Percentage reduction in messages
- **Messages Sent/Saved**: Absolute counts

#### 5. Timeline View
- **Topology over time**: Color-coded timeline of selected topologies
- **Communication events**: Markers when trajectory was broadcast

### Implementation Approach

#### Required ROS Topics (to be added)

```cpp
// New publishers for rqt GUI (add to planner wrapper)
ros::Publisher metrics_pub_;  // Custom message with all metrics

// Message definition (mpc_planner_msgs/MPCMetrics.msg)
Header header
string robot_name

# Solver metrics
float64 solve_time_ms
float64 success_rate
int32 iterations
int32 exit_code
float64 objective_value

# Topology metrics  
int32 current_topology_id
int32 previous_topology_id
bool topology_switch
bool used_guidance
int32 selected_planner_index

# Communication metrics
string last_communication_trigger
int32 messages_sent_total
int32 messages_saved_total
float64 communication_savings_percent

# Distribution arrays
int32[] topology_selection_counts
string[] communication_trigger_counts
```

#### Python rqt Plugin Structure

```python
# File: rqt_mpc_dashboard/src/rqt_mpc_dashboard/mpc_dashboard.py

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout
from mpc_planner_msgs.msg import MPCMetrics

class MPCDashboard(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('MPCDashboard')
        
        # Create main widget
        self._widget = QWidget()
        context.add_widget(self._widget)
        
        # Subscribe to metrics
        self._metrics_sub = rospy.Subscriber(
            'mpc_metrics', 
            MPCMetrics, 
            self._metrics_callback
        )
    
    def _metrics_callback(self, msg):
        # Update GUI elements with new metrics
        self._update_solver_panel(msg)
        self._update_communication_panel(msg)
        self._update_topology_panel(msg)
        self._update_timeline(msg)
```

### Configuration Options

The rqt GUI should support configuration via ROS parameters or a config file:

```yaml
# rqt_mpc_dashboard/config/dashboard.yaml
dashboard:
  robot_namespace: "/jackal1"
  
  panels:
    solver_performance:
      enabled: true
      show_histogram: true
      
    communication_metrics:
      enabled: true
      show_distribution: true
      
    topology_selection:
      enabled: true
      max_topologies: 8
      
    timeline:
      enabled: true
      history_seconds: 60
```

### Benefits of This GUI

1. **Real-time Monitoring**: Immediate feedback on planner performance
2. **Communication Analysis**: Understand when and why robots communicate
3. **Topology Insights**: Track how often different paths are selected
4. **Debugging**: Quickly identify solver failures or unexpected behavior
5. **Multi-Robot Coordination**: Monitor communication efficiency across robot team
6. **Performance Tuning**: Identify bottlenecks and optimize parameters

### Future Extensions

1. **Multi-Robot View**: Show metrics for all robots simultaneously
2. **Recording/Playback**: Record metrics for offline analysis
3. **Alert System**: Highlight when metrics exceed thresholds
4. **Parameter Tuning**: Allow live adjustment of configuration parameters
5. **Network Statistics**: Monitor message latency and bandwidth usage

---

## Summary

This documentation covers all 27+ visualization types available in the MPC Planner system. The visualizations are organized hierarchically:

1. **Core Planner** (7 visualizations): Essential trajectory and robot state displays
2. **Module-Specific** (8 visualizations): Contouring, constraints, and goal tracking
3. **Guidance Planner** (7 visualizations): T-MPC topology exploration
4. **Debug** (5+ visualizations): Internal solver state for troubleshooting

For optimal debugging workflow:
1. Start with core visualizations enabled
2. Add module-specific visualizations as needed
3. Enable debug visualizations only when investigating specific issues
4. Use the proposed rqt GUI for runtime metric monitoring

The combination of RViz visualizations and the rqt GUI concept provides comprehensive insight into the MPC planner's operation, enabling effective debugging, monitoring, and optimization of autonomous navigation in dynamic environments.
