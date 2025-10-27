# Obstacle Processing Documentation: `obstacleCallback()` and `parseObstacle()`

## Overview

This document explains the obstacle processing system in the MPC Jackal planner, focusing on how the `obstacleCallback()` and `parseObstacle()` functions work together to convert incoming obstacle data from the Vicon motion capture system into a format suitable for the MPC optimization solver.

## System Architecture

### Data Flow Pipeline
```
Vicon System → vicon_util → derived_object_msgs::ObjectArray → obstacleCallback() → MPC Planner
                ↓
            Bundle Obstacles
                ↓
        EKF Filtering & Prediction
                ↓
            Dynamic Objects
```

The obstacle processing is part of a larger perception pipeline:

1. **Vicon Motion Capture**: Provides real-time 6DOF poses of objects and robots
2. **vicon_util**: Bundles obstacles, runs EKF filters, and publishes `derived_object_msgs::ObjectArray`
3. **MPC Planner**: Receives obstacle array and converts to internal obstacle representation

## Function: `obstacleCallback()`

### Purpose
Converts incoming `derived_object_msgs::ObjectArray` messages into the MPC planner's internal `DynamicObstacle` representation, handling shape processing, velocity transformation, motion prediction, and obstacle classification.

### Input
```cpp
const derived_object_msgs::ObjectArray::ConstPtr &msg
```
- **Source**: `/input/obstacles` topic (remapped to `vicon_util/dynamic_objects`)
- **Content**: Array of tracked objects from Vicon with pose, twist, and shape information
- **Frequency**: Typically 10-20 Hz (controlled by Vicon frequency dividers)

### Processing Pipeline

#### 1. **Initialization and Reset**
```cpp
_data.dynamic_obstacles.clear();

std::vector<double> angles;
std::vector<Eigen::Vector2d> positions;
std::vector<double> radii;
std::vector<Eigen::Vector2d> twists;
std::vector<ObstacleType> types;
```

**Why clear obstacles each time?**
- Ensures fresh obstacle list (objects may disappear from Vicon tracking)
- Prevents accumulation of stale obstacle data
- Maintains consistent obstacle indexing

#### 2. **Object Filtering and Angle Calculation**
```cpp
for (auto &object : msg->objects)
{
    if (object.id == 0)
        continue;
        
    double object_angle = RosTools::quaternionToAngle(object.pose.orientation) +
                          std::atan2(object.twist.linear.y, object.twist.linear.x) +
                          M_PI_2;
    double velocity = std::sqrt(object.twist.linear.x * object.twist.linear.x +
                                object.twist.linear.y * object.twist.linear.y);
    if (velocity < 0.01) // Stop the visualization from rotating if standing still
        object_angle = RosTools::quaternionToAngle(object.pose.orientation);
```

**Object ID Filtering:**
- `object.id == 0`: Reserved ID, typically represents invalid/dummy objects
- Only processes objects with valid IDs (> 0)

**Angle Calculation Logic:**
The angle calculation combines three components:
1. **`object.pose.orientation`**: Object's current orientation from Vicon
2. **`atan2(twist.linear.y, twist.linear.x)`**: Direction of motion
3. **`M_PI_2`**: 90-degree offset for visualization alignment

**Why this complex angle calculation?**
- Aligns obstacle visualization with motion direction
- Provides better prediction visualization in RViz
- Falls back to pose orientation for stationary objects (velocity < 0.01 m/s)

#### 3. **Shape Processing with `parseObstacle()`**
```cpp
int prev_size = positions.size();
parseObstacle(object, object_angle, positions, radii);
int new_obstacles = positions.size() - prev_size;
```

**Multi-obstacle Support:**
- Single objects can be decomposed into multiple circular obstacles
- Tracks how many obstacles were added per object
- Used later for velocity and type assignment

#### 4. **Velocity Transformation**
```cpp
geometry_msgs::Twist global_twist = object.twist;
Eigen::Matrix2d rot_matrix = RosTools::rotationMatrixFromHeading(-RosTools::quaternionToAngle(object.pose.orientation));
Eigen::Vector2d twist_out = rot_matrix * Eigen::Vector2d(global_twist.linear.x, global_twist.linear.y);
```

**Coordinate Frame Transformation:**
- **Input**: Global frame velocity from Vicon
- **Transform**: Rotation matrix based on object orientation
- **Output**: Velocity in object's local coordinate frame
- **Negative sign**: Accounts for coordinate system conventions

**Why transform velocity?**
- MPC solver expects velocities in consistent coordinate frames
- Enables accurate motion prediction for oriented objects
- Handles different coordinate system conventions between Vicon and MPC

#### 5. **Obstacle Type Classification**
```cpp
// Assume obstacles consisting of multiple parts are static
if (new_obstacles > 1)
{
    for (int i = 0; i < new_obstacles; i++)
        types.push_back(ObstacleType::STATIC);
}
else
    types.push_back(ObstacleType::DYNAMIC);
```

**Classification Logic:**
- **Single obstacle**: Classified as `DYNAMIC` (can move)
- **Multiple obstacles**: Classified as `STATIC` (typically large objects decomposed into parts)

**Reasoning:**
- Large objects split into multiple circles are usually static structures
- Single-point objects are more likely to be moving people/robots
- Static vs. dynamic affects constraint generation in MPC solver

#### 6. **Dynamic Obstacle Creation and Prediction**
```cpp
for (int i = 0; i < positions.size(); i++)
{
    _data.dynamic_obstacles.emplace_back(
        i,
        positions[i],
        angles[i],
        radii[i],
        types[i]);

    auto &dynamic_obstacle = _data.dynamic_obstacles.back();

    dynamic_obstacle.prediction = getConstantVelocityPrediction(
        dynamic_obstacle.position,
        twists[i],
        CONFIG["integrator_step"].as<double>(),
        CONFIG["N"].as<int>());
}
```

**Obstacle Construction:**
Creates `DynamicObstacle` objects with:
- **ID**: Sequential index
- **Position**: 2D coordinates
- **Angle**: Orientation for visualization
- **Radius**: Collision boundary
- **Type**: STATIC or DYNAMIC classification

**Motion Prediction:**
- **Function**: `getConstantVelocityPrediction()`
- **Model**: Constant velocity motion model
- **Horizon**: `N` steps (prediction horizon length)
- **Time step**: `integrator_step` (typically 0.1-0.2 seconds)

#### 7. **Post-Processing**
```cpp
ensureObstacleSize(_data.dynamic_obstacles, _state);
propagatePredictionUncertainty(_data.dynamic_obstacles);

_planner->onDataReceived(_data, "dynamic obstacles");
```

**Obstacle Management:**
- **`ensureObstacleSize()`**: Ensures fixed number of obstacles for solver
  - Adds dummy obstacles if too few
  - Keeps closest obstacles if too many
  - Maintains consistent solver input size

**Uncertainty Propagation:**
- **`propagatePredictionUncertainty()`**: Adds prediction uncertainty over time
- Models increasing uncertainty for longer prediction horizons
- Used in probabilistic/stochastic MPC formulations

## Function: `parseObstacle()`

### Purpose
Converts different geometric shapes (cylinders, boxes) into circular obstacles suitable for the MPC solver's collision avoidance constraints.

### Input Parameters
```cpp
const derived_object_msgs::Object &object  // Object to process
double object_angle                         // Calculated orientation
std::vector<Eigen::Vector2d> &positions_out // Output positions
std::vector<double> &radii_out             // Output radii
```

### Shape Processing Logic

#### **Cylinder Handling**
```cpp
if (object.shape.type == object.shape.CYLINDER)
{
    positions_out.emplace_back(object.pose.position.x, object.pose.position.y);
    radii_out.push_back(object.shape.dimensions[1]);
}
```

**Simple Case:**
- Cylinders map directly to circles
- Position: Object center
- Radius: Cylinder radius (`dimensions[1]`)
- **Use case**: People, robots, cylindrical obstacles

#### **Box Handling - Square Case**
```cpp
else if (object.shape.type == object.shape.BOX)
{
    if (object.shape.dimensions[0] == object.shape.dimensions[1])
    {
        positions_out.emplace_back(object.pose.position.x, object.pose.position.y);
        radii_out.push_back(std::sqrt(2) * object.shape.dimensions[0]);
    }
```

**Square Boxes:**
- Treated as single circles
- Position: Box center
- Radius: `√2 × side_length` (circumscribes the square)
- **Reasoning**: Single circle provides conservative approximation

#### **Box Handling - Rectangular Case**
```cpp
else
{
    Eigen::Vector2d pos(object.pose.position.x, object.pose.position.y);

    double small_dim = std::min(object.shape.dimensions[0], object.shape.dimensions[1]);
    double large_dim = std::max(object.shape.dimensions[0], object.shape.dimensions[1]);
    double margin = 0.05;

    positions_out.emplace_back(pos + Eigen::Vector2d(std::cos(object_angle - M_PI_2),
                                                     std::sin(object_angle - M_PI_2)) *
                                         large_dim / 2);
    radii_out.emplace_back(std::sqrt(2) * small_dim + margin);

    positions_out.emplace_back(pos - Eigen::Vector2d(std::cos(object_angle - M_PI_2),
                                                     std::sin(object_angle - M_PI_2)) *
                                         large_dim / 2);
    radii_out.emplace_back(std::sqrt(2) * small_dim + margin);
}
```

**Rectangular Boxes - Two-Circle Approximation:**

**Why two circles?**
- Single circle would be too conservative (large radius)
- Two circles provide better approximation of rectangular shape
- Maintains convex constraints for MPC solver

**Circle Placement:**
- **Direction**: `object_angle - π/2` (perpendicular to object orientation)
- **Distance**: `large_dim / 2` (half the length along long axis)
- **Positions**: `center ± direction × distance`

**Circle Sizing:**
- **Base radius**: `√2 × small_dim` (circumscribes cross-section)
- **Margin**: Additional 5cm safety margin
- **Conservative**: Ensures coverage of rectangular area

**Geometric Derivation:**
```
Rectangle: width × length
Long axis: length direction
Short axis: width direction

Circle 1: center + (length/2) × direction_perpendicular
Circle 2: center - (length/2) × direction_perpendicular
Radius: √2 × width + margin
```

### Mathematical Background

#### **Coordinate Transformations**
```cpp
std::cos(object_angle - M_PI_2)  // X component of perpendicular direction
std::sin(object_angle - M_PI_2)  // Y component of perpendicular direction
```

**Angle offset reasoning:**
- `object_angle`: Object's heading direction
- `M_PI_2`: 90-degree offset to get perpendicular direction
- Places circles along object's length (perpendicular to heading)

#### **Conservative Approximation**
The `√2` factor ensures the circle circumscribes the square cross-section:
- Square diagonal = `√2 × side_length`
- Circle radius ≥ diagonal/2 = `√2 × side_length / 2`
- Using `√2 × side_length` provides additional safety margin

## Design Rationale

### **Why Circular Obstacles?**

1. **Solver Efficiency**: Circle-circle collision constraints are computationally simple
2. **Convexity**: Circular constraints maintain convex optimization problem
3. **Real-time Performance**: Fast distance calculations during MPC solving
4. **Robustness**: Conservative approximations ensure safety

### **Why Clear Obstacles Each Callback?**

1. **Dynamic Environment**: Objects may appear/disappear from tracking
2. **Consistent State**: Prevents accumulation of stale data
3. **ID Management**: Simplifies obstacle indexing
4. **Memory Management**: Prevents unbounded growth

### **Why Velocity Transformation?**

1. **Frame Consistency**: MPC solver expects consistent coordinate frames
2. **Prediction Accuracy**: Proper velocities enable accurate future state prediction
3. **Multi-robot Coordination**: Different robots may have different coordinate conventions

### **Why Static/Dynamic Classification?**

1. **Optimization Efficiency**: Different constraint handling for static vs. dynamic obstacles
2. **Prediction Models**: Static obstacles don't need motion prediction
3. **Safety Margins**: Different safety factors for different obstacle types

## Configuration Parameters

### **From CONFIG System:**
- `max_obstacles`: Maximum number of obstacles for solver
- `max_obstacle_distance`: Distance threshold for filtering distant obstacles
- `integrator_step`: Time step for motion prediction
- `N`: Prediction horizon length
- `probabilistic.enable`: Whether to use uncertainty propagation

### **Hard-coded Parameters:**
- `margin = 0.05`: Safety margin for rectangular obstacles (5cm)
- `velocity < 0.01`: Threshold for stationary objects
- `object.id == 0`: Reserved ID for invalid objects

## Common Issues and Debugging

### **Problem**: Obstacles not appearing in MPC
**Possible causes:**
1. Object ID = 0 (filtered out)
2. Object too far away (distance filtering)
3. Shape type not recognized
4. Transformation issues

### **Problem**: Robot colliding despite obstacles
**Possible causes:**
1. Insufficient safety margins
2. Prediction model inaccuracy
3. Delay in obstacle updates
4. Conservative circle approximation insufficient

### **Problem**: Performance issues
**Possible causes:**
1. Too many obstacles exceeding `max_obstacles`
2. High frequency obstacle updates
3. Complex shape decomposition
4. Uncertainty propagation overhead

## Future Improvements

### **Shape Approximation:**
- **Elliptical obstacles**: Better approximation of elongated objects
- **Polygon decomposition**: More accurate shape representation
- **Adaptive resolution**: More circles for larger objects

### **Motion Models:**
- **Acceleration-based prediction**: More realistic motion models
- **Intent estimation**: Predict human/robot behavior
- **Interaction models**: Account for multi-agent interactions

### **Performance Optimization:**
- **Spatial filtering**: Process only nearby obstacles
- **Temporal filtering**: Update obstacles at different frequencies
- **Hierarchical processing**: Different detail levels for near/far obstacles

This obstacle processing system provides a robust foundation for real-time collision avoidance in dynamic environments, balancing accuracy, safety, and computational efficiency for the MPC optimization framework.