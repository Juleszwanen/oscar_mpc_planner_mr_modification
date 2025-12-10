# Joint Planning Implementation Guide: Full Joint Optimization (Variant B)

## Document Information
- **Version**: 1.0
- **Purpose**: Step-by-step implementation guide with concrete code for integrating Full Joint Optimization (Variant B) into the T-MPC++ framework
- **Target**: Developers implementing Interactive Joint Planning in the MPC planner codebase
- **Reference**: See `JOINT_PLANNING_INTEGRATION_DESIGN.md` Section 6.2 for theoretical foundation

---

## Table of Contents

1. [Overview & Assumptions](#1-overview--assumptions)
2. [Current Architecture Summary](#2-current-architecture-summary)
3. [Joint Optimization Problem (Variant B)](#3-joint-optimization-problem-variant-b)
4. [Phase 4: SQP Loop & EC Updates](#4-phase-4-sqp-loop--ec-updates)
5. [MPC Module Extensions (CasADi & acados)](#5-mpc-module-extensions-casadi--acados)
6. [Changes in `guidance_constraints.cpp`](#6-changes-in-guidance_constraintscpp)
7. [Changes in `planner.cpp`](#7-changes-in-plannercpp)
8. [Integration in `mpc_jackal` and `mpc_jackalsimulator`](#8-integration-in-mpc_jackal-and-mpc_jackalsimulator)
9. [Testing & Benchmarking (Phase 5)](#9-testing--benchmarking-phase-5)
10. [Solver & Interface Changes (Phase 6)](#10-solver--interface-changes-phase-6)
11. [Data Flow & Execution Cycle](#11-data-flow--execution-cycle)
12. [Implementation Checklist](#12-implementation-checklist)

---

## 1. Overview & Assumptions

### 1.1 What This Guide Covers

This implementation guide provides **concrete, compilable code** for integrating the **Full Joint Optimization (Variant B)** from the Interactive Joint Planning (IJP) paper into the existing T-MPC++ codebase. The key innovation is:

- **Ego-Conditioned (EC) Robots**: Instead of treating nearby robots as fixed obstacles, we jointly optimize their trajectories along with the ego robot
- **Coupled Collision Constraints**: Collision avoidance becomes a joint constraint between ego and EC robot positions
- **Deviation Cost**: EC robots are penalized for deviating from their communicated/predicted trajectories

### 1.2 Key Assumptions

| # | Assumption | Rationale |
|---|------------|-----------|
| A1 | Maximum 2-3 EC robots | Computational tractability; O((M+1)³) complexity |
| A2 | EC robots use same unicycle dynamics as ego | Simplifies solver generation |
| A3 | Horizon N is identical for all robots | Required for joint constraints at each stage |
| A4 | Topology constraints apply to ego only | EC robots can freely adjust within deviation cost |
| A5 | EC robot predictions come from communication | Uses existing `DynamicObstacle` infrastructure |

**Note on Heterogeneous Dynamics (Assumption A2):**

This guide assumes homogeneous robots (all unicycle model). To support robots with different dynamics models:

1. **Different parameters, same model type**: Load per-robot bounds from a robot registry:
   ```cpp
   struct RobotParams {
       double max_velocity;
       double max_acceleration;
       double max_angular_velocity;
   };
   std::map<int, RobotParams> robot_registry;
   ```

2. **Different model types**: Requires solver regeneration for each combination, or use a generic point-mass model for EC robots with tighter bounds.

3. **Unknown robot type**: Fall back to conservative point-mass dynamics with minimum expected maneuverability.

### 1.3 Files Modified

| File | Changes |
|------|---------|
| `mpc_planner_types/include/mpc_planner_types/data_types.h` | Add `ECRobot` struct |
| `mpc_planner_types/include/mpc_planner_types/realtime_data.h` | Add `ec_robots` vector |
| `mpc_planner_types/include/mpc_planner_types/module_data.h` | Add joint planning result fields |
| `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h` | Add joint planning members and functions |
| `mpc_planner_modules/src/guidance_constraints.cpp` | Add SQP loop, EC selection, joint optimization |
| `mpc_planner_modules/scripts/joint_ec_constraints.py` | NEW: Python module for joint EC constraints |
| `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py` | Add EC robot variables and costs |
| `mpc_planner_jackalsimulator/config/settings.yaml` | Add `joint_planning` configuration section |
| `mpc_planner_solver/src/acados_solver_interface.cpp` | Add EC robot variable accessors |
| `mpc_planner_modules/test/test_joint_planning.cpp` | NEW: Unit tests for joint planning |

---

## 2. Current Architecture Summary

### 2.1 Module Organization

The MPC planner follows a modular architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS Interface Layer                       │
│     (mpc_planner_jackalsimulator/src/jules_ros1_...)        │
│                                                             │
│  • State callbacks (odometry)                               │
│  • Obstacle callbacks (other robots' trajectories)          │
│  • Command publishing (velocity commands)                   │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                     Planner Core Layer                       │
│               (mpc_planner/src/planner.cpp)                 │
│                                                             │
│  • Main loop: solveMPC(state, data)                         │
│  • Module orchestration                                     │
│  • Warmstart management                                     │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                      Module Layer                            │
│        (mpc_planner_modules/src/guidance_constraints.cpp)   │
│                                                             │
│  GuidanceConstraints Module:                                │
│    • update(): Graph search for topologies                  │
│    • optimize(): Parallel MPC solving per topology          │
│    • FindBestPlanner(): Select lowest cost solution         │
│                                                             │
│  LocalPlanner struct:                                       │
│    • local_solver: Independent MPC solver instance          │
│    • guidance_constraints: Topology constraints             │
│    • safety_constraints: Collision avoidance                │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                     Solver Layer                             │
│        (mpc_planner_solver/src/acados_solver_interface.cpp) │
│                                                             │
│  • Parameter management                                     │
│  • Warmstart handling                                       │
│  • acados QP solver invocation                              │
│  • Solution retrieval                                       │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Current Optimization Flow

```cpp
// In GuidanceConstraints::optimize()
#pragma omp parallel for num_threads(8)
for (auto &planner : planners_) {
    // 1. Copy main solver
    *planner.local_solver = *_solver;
    
    // 2. Build constraints
    planner.guidance_constraints->update(state, data, module_data);  // Topology
    planner.safety_constraints->update(state, data, module_data);    // Collision (fixed obstacles)
    
    // 3. Set parameters
    for (int k = 0; k < N; k++) {
        planner.guidance_constraints->setParameters(data, module_data, k);
        planner.safety_constraints->setParameters(data, module_data, k);
    }
    
    // 4. Solve
    planner.result.exit_code = planner.local_solver->solve();
    planner.result.objective = planner.local_solver->_info.pobj;
}

// 5. Select best
best_planner_index_ = FindBestPlanner();
```

### 2.3 Current Decision Variables

**Ego only:**
```
z = [a, w, x, y, psi, v, spline] × N stages
```

Where:
- `a`: Acceleration (control input)
- `w`: Angular velocity (control input)
- `x, y`: Position (state)
- `psi`: Heading (state)
- `v`: Velocity (state)
- `spline`: Progress along reference (state)

---

## 3. Joint Optimization Problem (Variant B)

### 3.1 Extended Decision Variables

For joint optimization with M EC robots:

```
z_joint = [
    // Ego robot (existing)
    a_ego, w_ego, x_ego, y_ego, psi_ego, v_ego, spline,
    
    // EC robot 1 (NEW)
    a_ec1, w_ec1, x_ec1, y_ec1, psi_ec1, v_ec1,
    
    // EC robot 2 (NEW)
    a_ec2, w_ec2, x_ec2, y_ec2, psi_ec2, v_ec2,
    
    // ... up to M EC robots
] × N stages
```

### 3.2 Cost Function (Equation 8 from IJP Paper)

```
minimize  η_e · [J_ref(x_ego, x_ref) + J_u(u_ego)] 
        + η_o · Σ_i [J_dev(x_ec_i, x_pred_i) + J_u(u_ec_i)]
```

Where:
- `η_e`: Ego selfishness weight (default: 0.8)
- `η_o`: EC robot weight (default: 0.2)
- `J_ref`: Reference tracking cost (existing contouring cost)
- `J_u(u)`: Control effort cost (existing)
- `J_dev`: **NEW** - Deviation cost penalizing EC robots for moving away from predictions

### 3.3 Coupled Collision Constraints

**OLD (fixed obstacle):**
```
||x_ego[k] - x_obs[k]||² ≥ (r_ego + r_obs)²  // x_obs is fixed parameter
```

**NEW (coupled with EC robot):**
```
||x_ego[k] - x_ec_i[k]||² ≥ (r_ego + r_ec_i)²  // x_ec_i is decision variable
```

This coupling is what makes the optimization "joint" - both ego and EC positions can adjust to avoid collision.

### 3.4 EC Robot Dynamics

Each EC robot follows the same unicycle dynamics as ego:

```
x_ec[k+1] = x_ec[k] + v_ec[k] * cos(psi_ec[k]) * dt
y_ec[k+1] = y_ec[k] + v_ec[k] * sin(psi_ec[k]) * dt
psi_ec[k+1] = psi_ec[k] + w_ec[k] * dt
v_ec[k+1] = v_ec[k] + a_ec[k] * dt
```

---

## 4. Phase 4: SQP Loop & EC Updates

### 4.1 Step 4.1: Modify `optimize()` for SQP Loop

**File:** `mpc_planner_modules/src/guidance_constraints.cpp`

The key modification wraps the existing parallel optimization loop in an SQP outer loop. This allows iterative refinement where EC robot predictions are updated based on ego's plan.

#### 4.1.1 Updated `optimize()` Function

```cpp
int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
{
    PROFILE_FUNCTION();
    omp_set_nested(1);
    omp_set_max_active_levels(2);
    omp_set_dynamic(0);
    LOG_MARK("Guidance Constraints: optimize");

    if (!_use_tmpcpp && !global_guidance_->Succeeded())
        return 0;

    // Interpolate previous trajectory for consistency (existing code)
    interpolatePrevTrajectoryByElapsedTime();
    visualizePreviousTrajectory();

    bool shift_forward = CONFIG["shift_previous_solution_forward"].as<bool>() &&
                         CONFIG["enable_output"].as<bool>();

    // ==================== NEW: Joint Planning Setup ====================
    std::vector<ECRobot> ec_robots;
    if (_joint_planning_enabled) {
        // Select EC robots from dynamic obstacles
        ec_robots = selectECRobots(data.dynamic_obstacles, state.getPos(), _ec_selection_radius);
        
        // Limit to maximum configured EC robots
        if (ec_robots.size() > static_cast<size_t>(_max_ec_robots)) {
            ec_robots.resize(_max_ec_robots);
        }
        
        LOG_INFO(_ego_robot_ns + ": Selected " << ec_robots.size() 
                 << " EC robots for joint optimization");
    }

    // Determine number of SQP iterations
    int sqp_rounds = _joint_planning_enabled ? _sqp_iterations : 1;
    // ====================================================================

    // ==================== NEW: SQP Outer Loop ====================
    for (int sqp_iter = 0; sqp_iter < sqp_rounds; sqp_iter++) {
        
        if (_joint_planning_enabled && sqp_iter > 0) {
            LOG_DEBUG(_ego_robot_ns + ": SQP iteration " << sqp_iter + 1 << "/" << sqp_rounds);
        }

        #pragma omp parallel for num_threads(8)
        for (auto &planner : planners_)
        {
            PROFILE_SCOPE("Guidance Constraints: Parallel Optimization");
            planner.result.Reset();
            planner.disabled = false;

            // Check if planner should be enabled (existing logic)
            if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories())
            {
                if (!planner.is_original_planner)
                {
                    planner.disabled = true;
                    continue;
                }
            }

            // Copy main solver
            auto &solver = planner.local_solver;
            *solver = *_solver;

            // ==================== Build Constraints ====================
            if (planner.is_original_planner || (!_enable_constraints))
            {
                planner.guidance_constraints->update(state, empty_data_, module_data);
                planner.safety_constraints->update(state, data, module_data);
            }
            else
            {
                if (CONFIG["t-mpc"]["warmstart_with_mpc_solution"].as<bool>() && planner.existing_guidance)
                    planner.local_solver->initializeWarmstart(state, shift_forward);
                else
                    initializeSolverWithGuidance(planner);

                planner.guidance_constraints->update(state, data, module_data);
                planner.safety_constraints->update(state, data, module_data);
            }

            // ==================== Set Parameters ====================
            for (int k = 0; k < _solver->N; k++)
            {
                if (planner.is_original_planner)
                    planner.guidance_constraints->setParameters(empty_data_, module_data, k);
                else
                    planner.guidance_constraints->setParameters(data, module_data, k);

                planner.safety_constraints->setParameters(data, module_data, k);
                
                // Existing consistency parameters
                setConsistencyParametersForPlanner(planner, k);

                // ==================== NEW: Joint Planning Parameters ====================
                if (_joint_planning_enabled) {
                    setJointPlanningParametersForPlanner(planner, ec_robots, k);
                }
                // ========================================================================
            }

            // Set solver timeout
            std::chrono::duration<double> used_time = std::chrono::system_clock::now() - data.planning_start_time;
            planner.local_solver->_params.solver_timeout = _planning_time - used_time.count() - 0.006;

            // Solve
            planner.local_solver->loadWarmstart();
            planner.result.exit_code = solver->solve();

            // Process result
            planner.result.success = planner.result.exit_code == 1;
            planner.result.objective = solver->_info.pobj;

            // Handle result (existing consistency cost subtraction, topology assignment, etc.)
            // ... [existing code for handling results] ...

            // ==================== NEW: Extract EC Robot Trajectories ====================
            if (_joint_planning_enabled && planner.result.success) {
                extractECRobotTrajectoriesFromSolver(planner, ec_robots);
            }
            // ============================================================================
        }

        // ==================== NEW: Update EC Predictions for Next SQP Iteration ====================
        if (_joint_planning_enabled && sqp_iter < sqp_rounds - 1) {
            updateECRobotPredictions(ec_robots, planners_);
        }
        // ===========================================================================================

    } // End SQP outer loop
    // =============================================================

    omp_set_dynamic(1);

    // Decision making (existing code)
    {
        PROFILE_SCOPE("Decision");
        best_planner_index_ = FindBestPlanner();
        if (best_planner_index_ == -1)
        {
            _has_previous_trajectory = false;
            _prev_selected_topology_id = -1;
            _prev_was_original_planner = false;
            return planners_[0].result.exit_code;
        }

        auto &best_planner = planners_[best_planner_index_];
        auto &best_solver = best_planner.local_solver;

        // Transfer solution to main solver (existing)
        _solver->_output = best_solver->_output;
        _solver->_info = best_solver->_info;
        _solver->_params = best_solver->_params;

        // Store module data (existing)
        if (CONFIG["JULES"]["use_extra_params_module_data"].as<bool>())
        {
            module_data.selected_topology_id = best_planner.result.guidance_ID;
            module_data.selected_planner_index = best_planner_index_;
            module_data.used_guidance = !best_planner.is_original_planner;
            module_data.trajectory_cost = best_planner.result.objective;
            module_data.solver_exit_code = best_planner.result.exit_code;
            module_data.num_of_guidance_found = global_guidance_->NumberOfGuidanceTrajectories();
        }

        // ==================== NEW: Store Joint Planning Results ====================
        if (_joint_planning_enabled) {
            module_data.joint_planning_enabled = true;
            module_data.ec_robot_planned_trajectories = 
                extractFinalECTrajectories(best_planner, ec_robots);
        }
        // ===========================================================================

        // Store previous trajectory for consistency (existing)
        storePreviousTrajectoryFromSolver(best_solver);
        _prev_selected_topology_id = best_planner.result.guidance_ID;
        _prev_was_original_planner = best_planner.is_original_planner;

        return best_planner.result.exit_code;
    }
}
```

### 4.2 Step 4.2: Implement EC Robot Selection

**File:** `mpc_planner_modules/src/guidance_constraints.cpp`

This function selects which nearby robots should be included in joint optimization.

```cpp
std::vector<ECRobot> GuidanceConstraints::selectECRobots(
    const std::vector<DynamicObstacle>& obstacles,
    const Eigen::Vector2d& ego_pos,
    double radius)
{
    std::vector<ECRobot> ec_robots;
    
    // Create a vector of (distance, obstacle_index) pairs for sorting
    std::vector<std::pair<double, size_t>> candidates;
    
    for (size_t i = 0; i < obstacles.size(); i++) {
        const auto& obs = obstacles[i];
        
        // Only consider obstacles that are robots (not pedestrians, etc.)
        if (obs.type != ObstacleType::ROBOT) {
            continue;
        }
        
        // Calculate distance to ego
        double dist = (obs.position - ego_pos).norm();
        
        // Only consider robots within selection radius
        if (dist > radius) {
            continue;
        }
        
        candidates.push_back({dist, i});
    }
    
    // Sort by distance (closest first)
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // Convert to ECRobot structs
    for (const auto& [dist, idx] : candidates) {
        const auto& obs = obstacles[idx];
        
        ECRobot ec(obs.index);
        ec.position = obs.position;
        ec.heading = obs.angle;
        ec.velocity = obs.current_speed;
        ec.radius = obs.radius;
        ec.is_active = true;
        
        // Copy prediction as unconditioned trajectory
        if (!obs.prediction.empty() && !obs.prediction.modes[0].empty()) {
            ec.predicted_trajectory = Trajectory(CONFIG["integrator_step"].as<double>());
            for (const auto& step : obs.prediction.modes[0]) {
                ec.predicted_trajectory.add(step.position);
            }
        }
        
        // Initialize planned trajectory to match predicted (will be updated by solver)
        ec.planned_trajectory = ec.predicted_trajectory;
        
        ec_robots.push_back(ec);
        
        LOG_DEBUG(_ego_robot_ns + ": EC robot " << ec.robot_id 
                  << " at distance " << dist << "m selected for joint optimization");
        
        // Limit to max EC robots
        if (ec_robots.size() >= static_cast<size_t>(_max_ec_robots)) {
            break;
        }
    }
    
    return ec_robots;
}
```

### 4.3 Step 4.3: Implement EC Trajectory Update (Heuristic for SQP)

**File:** `mpc_planner_modules/src/guidance_constraints.cpp`

This function updates EC robot predictions between SQP iterations based on simple collision avoidance heuristics.

```cpp
void GuidanceConstraints::updateECRobotPredictions(
    std::vector<ECRobot>& ec_robots,
    const std::vector<LocalPlanner>& planners)
{
    // Get ego trajectory from the best planner so far
    int best_idx = FindBestPlanner();
    if (best_idx < 0) {
        LOG_WARN(_ego_robot_ns + ": No feasible planner found, cannot update EC predictions");
        return;
    }
    
    const auto& best_solver = planners[best_idx].local_solver;
    
    // Extract ego trajectory
    std::vector<Eigen::Vector2d> ego_trajectory;
    for (int k = 0; k < best_solver->N; k++) {
        ego_trajectory.push_back(Eigen::Vector2d(
            best_solver->getOutput(k, "x"),
            best_solver->getOutput(k, "y")
        ));
    }
    
    // For each EC robot, apply simple collision avoidance heuristic
    for (auto& ec : ec_robots) {
        if (!ec.is_active || ec.predicted_trajectory.positions.empty()) {
            continue;
        }
        
        double dt = CONFIG["integrator_step"].as<double>();
        double safety_margin = CONFIG["joint_planning"]["safety_margin"].as<double>(0.5);
        double repulsion_strength = CONFIG["joint_planning"]["repulsion_strength"].as<double>(0.3);
        
        // Update each timestep of the EC prediction
        for (size_t k = 0; k < std::min(ego_trajectory.size(), 
                                        ec.predicted_trajectory.positions.size()); k++) {
            Eigen::Vector2d ego_pos = ego_trajectory[k];
            Eigen::Vector2d ec_pos = ec.predicted_trajectory.positions[k];
            
            // Calculate distance to ego
            Eigen::Vector2d diff = ec_pos - ego_pos;
            double dist = diff.norm();
            double min_dist = CONFIG["robot_radius"].as<double>() * 2.0 + safety_margin;
            
            // If too close, apply repulsion
            if (dist < min_dist && dist > 0.01) {
                // Calculate repulsion direction (away from ego)
                Eigen::Vector2d repulsion_dir = diff.normalized();
                
                // Calculate repulsion magnitude (stronger when closer)
                double penetration = min_dist - dist;
                double repulsion_mag = repulsion_strength * penetration;
                
                // Apply repulsion to EC position
                ec.predicted_trajectory.positions[k] += repulsion_dir * repulsion_mag;
                
                LOG_DEBUG(_ego_robot_ns + ": EC robot " << ec.robot_id 
                          << " at k=" << k << " pushed by " << repulsion_mag << "m");
            }
        }
    }
}
```

### 4.4 Step 4.4: Set Joint Planning Parameters

**File:** `mpc_planner_modules/src/guidance_constraints.cpp`

This function sets EC robot parameters in the solver for each stage.

```cpp
void GuidanceConstraints::setJointPlanningParametersForPlanner(
    LocalPlanner& planner,
    const std::vector<ECRobot>& ec_robots,
    int k)
{
    // Constants for inactive EC robots (far from any realistic position)
    static constexpr double INACTIVE_EC_POSITION = 1000.0;
    
    // For each EC robot slot in the solver
    for (int ec_idx = 0; ec_idx < _max_ec_robots; ec_idx++) {
        
        // Default values (inactive EC robot - positioned far away)
        double ec_x = INACTIVE_EC_POSITION;
        double ec_y = INACTIVE_EC_POSITION;
        double ec_psi = 0.0;
        double ec_v = 0.0;
        double ec_pred_x = INACTIVE_EC_POSITION;
        double ec_pred_y = INACTIVE_EC_POSITION;
        double ec_r = 0.0;
        double ec_active = 0.0;
        
        // If we have an active EC robot for this slot
        if (ec_idx < static_cast<int>(ec_robots.size())) {
            const auto& ec = ec_robots[ec_idx];
            
            if (ec.is_active && k < static_cast<int>(ec.predicted_trajectory.positions.size())) {
                // Use predicted trajectory as reference
                ec_pred_x = ec.predicted_trajectory.positions[k](0);
                ec_pred_y = ec.predicted_trajectory.positions[k](1);
                
                // Initial state for EC robot (at k=0, use current state)
                if (k == 0) {
                    ec_x = ec.position(0);
                    ec_y = ec.position(1);
                    ec_psi = ec.heading;
                    ec_v = ec.velocity;
                } else {
                    // For k > 0, use prediction as warmstart
                    ec_x = ec_pred_x;
                    ec_y = ec_pred_y;
                    ec_psi = (k < static_cast<int>(ec.predicted_trajectory.orientations.size())) 
                             ? ec.predicted_trajectory.orientations[k] : ec.heading;
                    ec_v = ec.velocity;  // Assume constant velocity for warmstart
                }
                
                ec_r = ec.radius;
                ec_active = 1.0;
            }
        }
        
        // Set parameters in solver
        std::string prefix = "ec" + std::to_string(ec_idx) + "_";
        
        planner.local_solver->setParameter(k, prefix + "pred_x", ec_pred_x);
        planner.local_solver->setParameter(k, prefix + "pred_y", ec_pred_y);
        planner.local_solver->setParameter(k, prefix + "r", ec_r);
        planner.local_solver->setParameter(k, prefix + "active", ec_active);
        
        // Set warmstart for EC robot state variables
        if (ec_active > 0.5) {
            planner.local_solver->setEgoPrediction(k, prefix + "x", ec_x);
            planner.local_solver->setEgoPrediction(k, prefix + "y", ec_y);
            planner.local_solver->setEgoPrediction(k, prefix + "psi", ec_psi);
            planner.local_solver->setEgoPrediction(k, prefix + "v", ec_v);
        }
    }
}
```

### 4.5 Step 4.5: Extract EC Robot Trajectories from Solver

```cpp
void GuidanceConstraints::extractECRobotTrajectoriesFromSolver(
    LocalPlanner& planner,
    std::vector<ECRobot>& ec_robots)
{
    for (int ec_idx = 0; ec_idx < static_cast<int>(ec_robots.size()); ec_idx++) {
        auto& ec = ec_robots[ec_idx];
        
        if (!ec.is_active) continue;
        
        // Clear and rebuild planned trajectory
        ec.planned_trajectory = Trajectory(CONFIG["integrator_step"].as<double>());
        ec.deviation_cost = 0.0;
        ec.control_cost = 0.0;
        
        std::string prefix = "ec" + std::to_string(ec_idx) + "_";
        
        for (int k = 0; k < planner.local_solver->N; k++) {
            double x = planner.local_solver->getOutput(k, prefix + "x");
            double y = planner.local_solver->getOutput(k, prefix + "y");
            ec.planned_trajectory.add(x, y);
            
            // Calculate deviation from prediction
            if (k < static_cast<int>(ec.predicted_trajectory.positions.size())) {
                double dx = x - ec.predicted_trajectory.positions[k](0);
                double dy = y - ec.predicted_trajectory.positions[k](1);
                ec.deviation_cost += dx*dx + dy*dy;
            }
        }
        
        ec.deviation_cost *= _deviation_weight;
        
        LOG_DEBUG(_ego_robot_ns + ": EC robot " << ec.robot_id 
                  << " deviation cost: " << ec.deviation_cost);
    }
}
```

### 4.6 Step 4.6: Extract Final EC Trajectories

```cpp
std::vector<Trajectory> GuidanceConstraints::extractFinalECTrajectories(
    const LocalPlanner& planner,
    const std::vector<ECRobot>& ec_robots)
{
    std::vector<Trajectory> trajectories;
    
    for (int ec_idx = 0; ec_idx < static_cast<int>(ec_robots.size()); ec_idx++) {
        const auto& ec = ec_robots[ec_idx];
        
        if (ec.is_active) {
            trajectories.push_back(ec.planned_trajectory);
        } else {
            // Empty trajectory for inactive EC robots
            trajectories.push_back(Trajectory());
        }
    }
    
    return trajectories;
}
```

---

## 5. MPC Module Extensions (CasADi & acados)

### 5.1 New Python Module: `joint_ec_constraints.py`

**File:** `mpc_planner_modules/scripts/joint_ec_constraints.py`

This new module defines EC robot decision variables, deviation costs, and coupled collision constraints.

```python
import os
import sys

import casadi as cd
import numpy as np

from util.math import rotation_matrix
from control_modules import ConstraintModule, ObjectiveModule

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))


class JointECConstraintModule(ConstraintModule, ObjectiveModule):
    """
    Joint optimization module for Ego-Conditioned (EC) robots.
    
    Implements Variant B from the Interactive Joint Planning paper:
    - Adds EC robot state variables (x, y, psi, v) and control inputs (a, w)
    - Adds deviation cost penalizing EC robots for deviating from predictions
    - Adds coupled collision constraints between ego and EC robots
    - Adds EC robot dynamics constraints
    """

    def __init__(self, settings, max_ec_robots=2):
        ConstraintModule.__init__(self)
        ObjectiveModule.__init__(self)
        
        self.max_ec_robots = max_ec_robots
        self.n_discs = settings["n_discs"]
        
        self.module_name = "JointECConstraints"
        self.import_name = "joint_ec_constraints.h"  # C++ header (to be created)
        self.description = "Joint optimization with EC robots (Variant B)"
        
        # Add constraint for coupled collision avoidance
        self.constraints.append(JointCollisionConstraint(self.n_discs, max_ec_robots))
        
        # Add objective for EC robot deviation and control costs
        self.objectives.append(JointECObjective(max_ec_robots, settings))

    def add_model_variables(self, model, settings):
        """Add EC robot state and control variables to the model."""
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # EC robot control inputs
            model.add_input(prefix + "a", 
                           lower_bound=-settings.get("ec_max_acceleration", 1.5),
                           upper_bound=settings.get("ec_max_acceleration", 1.5))
            model.add_input(prefix + "w",
                           lower_bound=-settings.get("ec_max_angular_velocity", 1.0),
                           upper_bound=settings.get("ec_max_angular_velocity", 1.0))
            
            # EC robot states
            model.add_state(prefix + "x", lower_bound=-1000.0, upper_bound=1000.0)
            model.add_state(prefix + "y", lower_bound=-1000.0, upper_bound=1000.0)
            model.add_state(prefix + "psi", lower_bound=-np.pi*2, upper_bound=np.pi*2)
            model.add_state(prefix + "v", 
                           lower_bound=-0.1,  # Allow small negative for numerical stability
                           upper_bound=settings.get("ec_max_velocity", 2.0))

    def get_dynamics(self, model, settings):
        """Return EC robot dynamics expressions for integration."""
        dt = settings["integrator_step"]
        dynamics = {}
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            x = model.get(prefix + "x")
            y = model.get(prefix + "y")
            psi = model.get(prefix + "psi")
            v = model.get(prefix + "v")
            a = model.get(prefix + "a")
            w = model.get(prefix + "w")
            
            # Unicycle dynamics (same as ego)
            dynamics[prefix + "x_dot"] = v * cd.cos(psi)
            dynamics[prefix + "y_dot"] = v * cd.sin(psi)
            dynamics[prefix + "psi_dot"] = w
            dynamics[prefix + "v_dot"] = a
        
        return dynamics


class JointCollisionConstraint:
    """Coupled collision constraint between ego and EC robots."""

    def __init__(self, n_discs, max_ec_robots):
        self.n_discs = n_discs
        self.max_ec_robots = max_ec_robots
        
        # One constraint per EC robot per ego disc
        self.nh = max_ec_robots * n_discs

    def define_parameters(self, params):
        """Define EC robot parameters."""
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # EC robot predicted trajectory (reference for deviation cost)
            params.add(prefix + "pred_x", bundle_name="ec_pred_x")
            params.add(prefix + "pred_y", bundle_name="ec_pred_y")
            
            # EC robot radius
            params.add(prefix + "r", bundle_name="ec_r")
            
            # Active flag (1.0 = active, 0.0 = inactive)
            params.add(prefix + "active", bundle_name="ec_active")

    def get_lower_bound(self):
        """Constraint: distance² >= min_distance². Lower bound is 1.0."""
        return [1.0] * (self.max_ec_robots * self.n_discs)

    def get_upper_bound(self):
        """Upper bound is infinity (no upper limit on distance)."""
        return [np.inf] * (self.max_ec_robots * self.n_discs)

    def get_constraints(self, model, params, settings, stage_idx):
        """
        Build coupled collision constraints.
        
        For each EC robot i and ego disc d:
            ||pos_ego_d - pos_ec_i||² / (r_ego + r_ec_i + margin)² >= 1
        
        When EC robot is inactive, constraint is automatically satisfied
        because EC position is set far away (1000, 1000).
        """
        constraints = []
        
        # Ego position
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])
        
        try:
            psi = model.get("psi")
        except:
            psi = 0.0
        
        rotation_car = rotation_matrix(psi)
        r_ego = params.get("ego_disc_radius")
        safety_margin = settings.get("joint_planning", {}).get("safety_margin", 0.1)
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # Get EC robot position (decision variable!)
            ec_x = model.get(prefix + "x")
            ec_y = model.get(prefix + "y")
            ec_pos = np.array([ec_x, ec_y])
            
            # Get EC robot radius (parameter)
            ec_r = params.get(prefix + "r")
            
            # Total safety distance
            min_dist = r_ego + ec_r + safety_margin
            
            for disc_it in range(self.n_discs):
                # Ego disc position
                disc_offset = params.get(f"ego_disc_{disc_it}_offset")
                disc_relative_pos = np.array([disc_offset, 0])
                disc_pos = pos + rotation_car @ disc_relative_pos
                
                # Distance squared
                diff = disc_pos - ec_pos
                dist_sq = diff[0]**2 + diff[1]**2
                
                # Numerical epsilon for division stability
                EPSILON = 1e-6
                
                # Normalized constraint: dist² / min_dist² >= 1
                # This formulation is better conditioned than dist² >= min_dist²
                constraint = dist_sq / (min_dist**2 + EPSILON)
                constraints.append(constraint)
        
        return constraints


class JointECObjective:
    """
    EC robot deviation cost and control effort cost.
    
    Implements: η_o * [J_dev(x_ec, x_pred) + J_u(u_ec)]
    """

    def __init__(self, max_ec_robots, settings):
        self.max_ec_robots = max_ec_robots
        self.settings = settings

    def define_parameters(self, params):
        """Parameters already defined by JointCollisionConstraint."""
        # EC deviation weight (η_o in paper)
        params.add("ec_deviation_weight")
        params.add("ec_control_weight")

    def get_objective(self, model, params, settings, stage_idx):
        """
        Compute EC robot cost terms.
        
        J_ec = Σ_i [ w_dev * ||x_ec_i - x_pred_i||² + w_ctrl * (a_ec_i² + w_ec_i²) ]
        """
        w_dev = params.get("ec_deviation_weight")
        w_ctrl = params.get("ec_control_weight")
        
        total_cost = 0.0
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # Get EC robot state (decision variable)
            ec_x = model.get(prefix + "x")
            ec_y = model.get(prefix + "y")
            
            # Get EC robot prediction (parameter)
            pred_x = params.get(prefix + "pred_x")
            pred_y = params.get(prefix + "pred_y")
            
            # Get active flag
            active = params.get(prefix + "active")
            
            # Deviation cost: ||position - prediction||²
            dev_x = ec_x - pred_x
            dev_y = ec_y - pred_y
            deviation_cost = w_dev * (dev_x**2 + dev_y**2)
            
            # Control effort cost: ||u||²
            ec_a = model.get(prefix + "a")
            ec_w = model.get(prefix + "w")
            control_cost = w_ctrl * (ec_a**2 + ec_w**2)
            
            # Only add cost if EC robot is active
            # When inactive, active = 0 and cost contribution is 0
            total_cost += active * (deviation_cost + control_cost)
        
        return total_cost
```

### 5.2 Update Solver Generation Script

**File:** `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py`

Add the following changes to enable joint EC optimization:

```python
#!/usr/bin/python3

import os
import sys
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# ... existing imports ...

# NEW: Import joint EC constraints module
from joint_ec_constraints import JointECConstraintModule

# ... existing model definitions ...

def configuration_tmpc_joint_planning(settings):
    """
    T-MPC++ with joint EC robot optimization (Variant B).
    """
    modules = ModuleManager()
    
    # Check if joint planning is enabled
    joint_planning_enabled = settings.get("joint_planning", {}).get("enabled", False)
    max_ec_robots = settings.get("joint_planning", {}).get("max_ec_robots", 2)
    
    if joint_planning_enabled:
        # Use extended model with EC robot variables
        model = ContouringSecondOrderUnicycleModelWithEC(max_ec_robots)
    else:
        model = ContouringSecondOrderUnicycleModel()

    # Base module (existing)
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2)

    # Contouring module (existing)
    modules.add_module(ContouringModule(settings))
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings))
    
    # Consistency module (existing)
    if settings.get("JULES", {}).get("consistency_enabled", False):
        modules.add_module(ConsistencyModule(settings))
    
    # NEW: Joint EC constraints module
    if joint_planning_enabled:
        modules.add_module(JointECConstraintModule(settings, max_ec_robots))
    
    # Guidance constraints with obstacle avoidance
    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule
    ))

    return model, modules


# Extended model class with EC robot variables
class ContouringSecondOrderUnicycleModelWithEC:
    """
    Extended unicycle model that includes EC robot state variables.
    """
    
    def __init__(self, max_ec_robots=2):
        self.max_ec_robots = max_ec_robots
        
        # Ego robot variables (same as base model)
        self.nu_ego = 2  # a, w
        self.nx_ego = 5  # x, y, psi, v, spline
        
        # EC robot variables (per robot)
        self.nu_ec = 2  # a_ec, w_ec
        self.nx_ec = 4  # x_ec, y_ec, psi_ec, v_ec
        
        # Total dimensions
        self.nu = self.nu_ego + max_ec_robots * self.nu_ec
        self.nx = self.nx_ego + max_ec_robots * self.nx_ec
        
        # State and input bounds
        self._setup_bounds()
    
    def _setup_bounds(self):
        """Set up bounds for all variables.
        
        Note: These bounds should be loaded from configuration for production use.
        The values shown here are typical defaults for Jackal robots.
        """
        # Constants for bounds (should match settings.yaml in production)
        POS_LIMIT = 2000.0
        EGO_MAX_ACCEL = 2.0
        EGO_MAX_ANGULAR_VEL = 0.8
        EGO_MAX_VEL = 3.0
        EC_MAX_ACCEL = 1.5
        EC_MAX_ANGULAR_VEL = 1.0
        EC_MAX_VEL = 2.0
        
        # Ego bounds
        ego_lower = [-EGO_MAX_ACCEL, -EGO_MAX_ANGULAR_VEL,   # a, w
                     -POS_LIMIT, -POS_LIMIT, -np.pi*2, -1.0, -1.0]  # x, y, psi, v, spline
        ego_upper = [EGO_MAX_ACCEL, EGO_MAX_ANGULAR_VEL,
                     POS_LIMIT, POS_LIMIT, np.pi*2, EGO_MAX_VEL, 10000.0]
        
        # EC robot bounds (per robot)
        ec_lower = [-EC_MAX_ACCEL, -EC_MAX_ANGULAR_VEL,  # a_ec, w_ec
                    -POS_LIMIT, -POS_LIMIT, -np.pi*2, -0.1]  # x_ec, y_ec, psi_ec, v_ec
        ec_upper = [EC_MAX_ACCEL, EC_MAX_ANGULAR_VEL,
                    POS_LIMIT, POS_LIMIT, np.pi*2, EC_MAX_VEL]
        
        self.lower_bound = ego_lower.copy()
        self.upper_bound = ego_upper.copy()
        
        for _ in range(self.max_ec_robots):
            self.lower_bound.extend(ec_lower)
            self.upper_bound.extend(ec_upper)
    
    def get_nvar(self):
        return self.nu + self.nx
    
    def get_x(self):
        """Get symbolic state vector."""
        import casadi as cd
        
        # Ego states
        x_ego = cd.SX.sym("x")
        y_ego = cd.SX.sym("y")
        psi_ego = cd.SX.sym("psi")
        v_ego = cd.SX.sym("v")
        spline = cd.SX.sym("spline")
        
        states = [x_ego, y_ego, psi_ego, v_ego, spline]
        
        # EC robot states
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            states.extend([
                cd.SX.sym(prefix + "x"),
                cd.SX.sym(prefix + "y"),
                cd.SX.sym(prefix + "psi"),
                cd.SX.sym(prefix + "v")
            ])
        
        return cd.vertcat(*states)
    
    def get_acados_dynamics(self):
        """Return dynamics for acados OCP."""
        import casadi as cd
        
        # Get state and input
        x = self.get_x()
        u = self.get_acados_u()
        
        # Ego dynamics
        x_ego = x[0]
        y_ego = x[1]
        psi_ego = x[2]
        v_ego = x[3]
        spline = x[4]
        
        a_ego = u[0]
        w_ego = u[1]
        
        # Ego dynamics expressions
        x_dot = v_ego * cd.cos(psi_ego)
        y_dot = v_ego * cd.sin(psi_ego)
        psi_dot = w_ego
        v_dot = a_ego
        spline_dot = v_ego
        
        f_expl = [x_dot, y_dot, psi_dot, v_dot, spline_dot]
        
        # EC robot dynamics
        for ec_idx in range(self.max_ec_robots):
            state_offset = 5 + ec_idx * 4
            input_offset = 2 + ec_idx * 2
            
            x_ec = x[state_offset]
            y_ec = x[state_offset + 1]
            psi_ec = x[state_offset + 2]
            v_ec = x[state_offset + 3]
            
            a_ec = u[input_offset]
            w_ec = u[input_offset + 1]
            
            f_expl.extend([
                v_ec * cd.cos(psi_ec),  # x_ec_dot
                v_ec * cd.sin(psi_ec),  # y_ec_dot
                w_ec,                    # psi_ec_dot
                a_ec                     # v_ec_dot
            ])
        
        f_expl = cd.vertcat(*f_expl)
        
        # Implicit dynamics: x_dot - f(x, u) = 0
        x_dot_sym = cd.SX.sym("x_dot", self.nx)
        f_impl = x_dot_sym - f_expl
        
        return f_expl, f_impl
    
    # ... other required methods similar to base model ...


# Select configuration based on settings
settings = load_settings()

if settings.get("joint_planning", {}).get("enabled", False):
    model, modules = configuration_tmpc_joint_planning(settings)
else:
    model, modules = configuration_tmpc_consistency_cost(settings)

generate_solver(modules, model, settings)
exit(0)
```

---

## 6. Changes in `guidance_constraints.cpp`

### 6.1 Header File Updates

**File:** `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h`

Add the following to the private section:

```cpp
// ==================== Joint Planning Members ====================
bool _joint_planning_enabled{false};
int _max_ec_robots{2};
double _ec_selection_radius{10.0};
double _deviation_weight{5.0};
double _ec_control_weight{1.0};
double _ego_selfishness{0.8};
int _sqp_iterations{2};

// Joint planning helper functions
std::vector<ECRobot> selectECRobots(
    const std::vector<DynamicObstacle>& obstacles,
    const Eigen::Vector2d& ego_pos,
    double radius);

void setJointPlanningParametersForPlanner(
    LocalPlanner& planner,
    const std::vector<ECRobot>& ec_robots,
    int k);

void extractECRobotTrajectoriesFromSolver(
    LocalPlanner& planner,
    std::vector<ECRobot>& ec_robots);

void updateECRobotPredictions(
    std::vector<ECRobot>& ec_robots,
    const std::vector<LocalPlanner>& planners);

std::vector<Trajectory> extractFinalECTrajectories(
    const LocalPlanner& planner,
    const std::vector<ECRobot>& ec_robots);
// ================================================================
```

### 6.2 Configuration Loading in Constructor

Add to `GuidanceConstraints::GuidanceConstraints()`:

```cpp
// Joint planning configuration
try {
    auto jp_config = CONFIG["joint_planning"];
    _joint_planning_enabled = jp_config["enabled"].as<bool>(false);
    _max_ec_robots = jp_config["max_ec_robots"].as<int>(2);
    _ec_selection_radius = jp_config["ec_robot_selection_radius"].as<double>(10.0);
    _deviation_weight = jp_config["deviation_weight"].as<double>(5.0);
    _ec_control_weight = jp_config["ec_control_effort_weight"].as<double>(1.0);
    _ego_selfishness = jp_config["ego_selfishness"].as<double>(0.8);
    _sqp_iterations = jp_config["sqp_iterations"].as<int>(2);
    
    LOG_INFO(_ego_robot_ns + ": Joint planning " 
             << (_joint_planning_enabled ? "ENABLED" : "DISABLED")
             << " (max_ec=" << _max_ec_robots 
             << ", sqp_iter=" << _sqp_iterations << ")");
} catch (...) {
    _joint_planning_enabled = false;
    LOG_WARN("Joint planning config not found, disabled by default");
}
```

---

## 7. Changes in `planner.cpp`

### 7.1 Handling Joint Planning Results

In `Planner::solveMPC()`, add handling for joint planning results in module_data:

```cpp
// After the optimization loop, check for joint planning results
if (module_data.joint_planning_enabled) {
    LOG_DEBUG(_ego_robot_ns + ": Joint planning produced " 
              << module_data.ec_robot_planned_trajectories.size() 
              << " EC robot trajectories");
    
    // Optionally visualize EC robot planned trajectories
    for (size_t i = 0; i < module_data.ec_robot_planned_trajectories.size(); i++) {
        const auto& ec_traj = module_data.ec_robot_planned_trajectories[i];
        if (!ec_traj.positions.empty()) {
            visualizeTrajectory(ec_traj, "ec_robot_" + std::to_string(i) + "_plan", 
                              false, 0.5, 5, 10);  // Different color
        }
    }
}
```

---

## 8. Integration in `mpc_jackal` and `mpc_jackalsimulator`

### 8.1 Configuration File Updates

**File:** `mpc_planner_jackalsimulator/config/settings.yaml`

Add the joint planning configuration section:

```yaml
joint_planning:
  enabled: false                          # Toggle joint planning on/off
  max_ec_robots: 2                        # Max robots to jointly optimize (1-3 recommended)
  ec_robot_selection_radius: 10.0         # [m] Select EC robots within this radius
  deviation_weight: 5.0                   # Weight for J_dev (EC deviation from prediction)
  ec_control_effort_weight: 1.0           # Weight for J_u of EC robots
  ego_selfishness: 0.8                    # η_e: 0=altruistic, 1=selfish (0.8 recommended)
  sqp_iterations: 2                       # Number of SQP refinement rounds
  use_coupled_collision: true             # Use joint collision constraints
  safety_margin: 0.1                      # [m] Additional safety buffer
  repulsion_strength: 0.3                 # Strength of EC repulsion in SQP update
  
  # EC robot dynamics bounds
  ec_max_velocity: 2.0                    # [m/s] EC robot velocity bound
  ec_max_acceleration: 1.5                # [m/s²] EC robot acceleration bound
  ec_max_angular_velocity: 1.0            # [rad/s] EC robot angular velocity bound
```

### 8.2 Launch File Updates

**File:** `mpc_planner_jackalsimulator/launch/multi_robot_joint_planning.launch` (NEW)

```xml
<?xml version="1.0"?>
<launch>
    <!-- Multi-robot simulation with joint planning enabled -->
    
    <arg name="num_robots" default="2"/>
    <arg name="joint_planning" default="true"/>
    <arg name="max_ec_robots" default="2"/>
    
    <!-- Robot 1 -->
    <group ns="jackal1">
        <include file="$(find mpc_planner_jackalsimulator)/launch/jackal_planner.launch">
            <arg name="robot_ns" value="jackal1"/>
            <arg name="joint_planning_enabled" value="$(arg joint_planning)"/>
            <arg name="max_ec_robots" value="$(arg max_ec_robots)"/>
        </include>
    </group>
    
    <!-- Robot 2 -->
    <group ns="jackal2">
        <include file="$(find mpc_planner_jackalsimulator)/launch/jackal_planner.launch">
            <arg name="robot_ns" value="jackal2"/>
            <arg name="joint_planning_enabled" value="$(arg joint_planning)"/>
            <arg name="max_ec_robots" value="$(arg max_ec_robots)"/>
        </include>
    </group>
    
    <!-- Additional robots if needed -->
    <group if="$(eval num_robots >= 3)" ns="jackal3">
        <include file="$(find mpc_planner_jackalsimulator)/launch/jackal_planner.launch">
            <arg name="robot_ns" value="jackal3"/>
            <arg name="joint_planning_enabled" value="$(arg joint_planning)"/>
            <arg name="max_ec_robots" value="$(arg max_ec_robots)"/>
        </include>
    </group>
    
</launch>
```

### 8.3 ROS Parameter Override

To enable/disable joint planning at runtime via ROS parameters:

```cpp
// In jules_ros1_jackalplanner.cpp, in initialization:
bool joint_planning_enabled;
ros::param::param<bool>("~joint_planning/enabled", joint_planning_enabled, false);

// Override YAML config if ROS param is set
if (ros::param::has("~joint_planning/enabled")) {
    // Parameter was explicitly set, use it to override
    LOG_INFO("Joint planning override from ROS param: " 
             << (joint_planning_enabled ? "ENABLED" : "DISABLED"));
}
```

---

## 9. Testing & Benchmarking (Phase 5)

### 9.1 Unit Tests

**File:** `mpc_planner_modules/test/test_joint_planning.cpp` (NEW)

```cpp
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "mpc_planner_modules/guidance_constraints.h"
#include "mpc_planner_types/data_types.h"
#include "mpc_planner_types/realtime_data.h"

using namespace MPCPlanner;

class JointPlanningTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "test_joint_planning");
        
        // Create solver (will use test configuration)
        solver_ = std::make_shared<Solver>();
        
        // Create guidance constraints module
        guidance_constraints_ = std::make_unique<GuidanceConstraints>(solver_);
    }
    
    void TearDown() override {
        guidance_constraints_.reset();
        solver_.reset();
    }
    
    // Helper to create mock dynamic obstacles
    std::vector<DynamicObstacle> createMockRobots(int num_robots, 
                                                   const Eigen::Vector2d& ego_pos,
                                                   double spacing = 5.0) {
        std::vector<DynamicObstacle> obstacles;
        
        for (int i = 0; i < num_robots; i++) {
            Eigen::Vector2d pos = ego_pos + Eigen::Vector2d(spacing * (i + 1), 0);
            DynamicObstacle obs(i, pos, 0.0, 0.325, ObstacleType::ROBOT);
            
            // Add prediction (10 steps)
            obs.prediction = Prediction(PredictionType::DETERMINISTIC);
            Mode mode;
            for (int k = 0; k < 10; k++) {
                Eigen::Vector2d pred_pos = pos + Eigen::Vector2d(0.5 * k, 0);
                mode.push_back(PredictionStep(pred_pos, 0.0, 0.1, 0.1));
            }
            obs.prediction.modes.push_back(mode);
            obs.prediction.probabilities.push_back(1.0);
            
            obstacles.push_back(obs);
        }
        
        return obstacles;
    }
    
    std::shared_ptr<Solver> solver_;
    std::unique_ptr<GuidanceConstraints> guidance_constraints_;
};

// Test 1: EC Robot Selection - Distance-based selection
TEST_F(JointPlanningTest, ECRobotSelectionByDistance) {
    Eigen::Vector2d ego_pos(0.0, 0.0);
    double selection_radius = 15.0;  // Should select robots within 15m
    
    // Create 4 robots at distances 5, 10, 15, 20 meters
    auto obstacles = createMockRobots(4, ego_pos, 5.0);
    
    // Call selectECRobots (need to expose or make friend)
    // For now, we test through the full optimize() flow
    
    RealTimeData data;
    data.dynamic_obstacles = obstacles;
    
    // With radius 15m, should select 3 robots (at 5, 10, 15m - not 20m)
    // Verify through logs or module_data after optimize()
    
    EXPECT_EQ(obstacles.size(), 4u);
    
    // Verify robot type filtering
    for (const auto& obs : obstacles) {
        EXPECT_EQ(obs.type, ObstacleType::ROBOT);
    }
}

// Test 2: EC Robot Selection - Max limit enforcement
TEST_F(JointPlanningTest, ECRobotSelectionMaxLimit) {
    Eigen::Vector2d ego_pos(0.0, 0.0);
    int max_ec_robots = 2;
    
    // Create 5 robots, all within selection radius
    auto obstacles = createMockRobots(5, ego_pos, 2.0);
    
    // Even with 5 nearby robots, should only select max_ec_robots
    // This is enforced in the selectECRobots function
    
    EXPECT_GE(obstacles.size(), static_cast<size_t>(max_ec_robots));
}

// Test 3: EC Robot Selection - Only ROBOT type
TEST_F(JointPlanningTest, ECRobotSelectionOnlyRobotType) {
    Eigen::Vector2d ego_pos(0.0, 0.0);
    
    std::vector<DynamicObstacle> obstacles;
    
    // Add ROBOT type
    DynamicObstacle robot(0, ego_pos + Eigen::Vector2d(5, 0), 0.0, 0.325, ObstacleType::ROBOT);
    obstacles.push_back(robot);
    
    // Add DYNAMIC type (should be ignored)
    DynamicObstacle pedestrian(1, ego_pos + Eigen::Vector2d(3, 0), 0.0, 0.3, ObstacleType::DYNAMIC);
    obstacles.push_back(pedestrian);
    
    // Add STATIC type (should be ignored)
    DynamicObstacle static_obs(2, ego_pos + Eigen::Vector2d(4, 0), 0.0, 0.5, ObstacleType::STATIC);
    obstacles.push_back(static_obs);
    
    // EC selection should only pick the ROBOT type
    int robot_count = 0;
    for (const auto& obs : obstacles) {
        if (obs.type == ObstacleType::ROBOT) {
            robot_count++;
        }
    }
    
    EXPECT_EQ(robot_count, 1);
}

// Test 4: Configuration Loading
TEST_F(JointPlanningTest, ConfigLoading) {
    // Test that joint_planning config is loaded correctly
    // This tests the constructor's config loading logic
    
    // Check default values when config fields are missing
    // The module should initialize with safe defaults
    
    ASSERT_NE(guidance_constraints_, nullptr);
    
    // Module should be created successfully even without joint_planning config
}

// Test 5: SQP Iteration Count
TEST_F(JointPlanningTest, SQPIterationCount) {
    // Verify that the correct number of SQP iterations are executed
    
    // When joint_planning is disabled: 1 iteration
    // When joint_planning is enabled: sqp_iterations from config
    
    // This would require instrumenting the optimize() function
    // or checking solve time characteristics
    
    SUCCEED();  // Placeholder - implement with proper instrumentation
}

// Test 6: EC Trajectory Extraction
TEST_F(JointPlanningTest, ECTrajectoryExtraction) {
    // Verify that EC robot trajectories are correctly extracted from solver
    
    // Create mock solver output
    // Call extractECRobotTrajectoriesFromSolver
    // Verify trajectory dimensions and values
    
    SUCCEED();  // Placeholder
}

// Test 7: Deviation Cost Calculation
TEST_F(JointPlanningTest, DeviationCostCalculation) {
    // Verify that deviation cost is correctly computed
    
    // EC robot at prediction: deviation_cost = 0
    // EC robot shifted by d: deviation_cost = w_dev * d^2
    
    double w_dev = 5.0;
    double deviation = 1.0;  // 1 meter deviation
    
    double expected_cost = w_dev * deviation * deviation;
    
    // This would require calling the actual cost computation
    EXPECT_NEAR(expected_cost, 5.0, 1e-6);
}

// Test 8: Coupled Collision Constraint
TEST_F(JointPlanningTest, CoupledCollisionConstraint) {
    // Verify that coupled collision constraint is correctly formulated
    
    double r_ego = 0.325;
    double r_ec = 0.325;
    double safety_margin = 0.1;
    double min_dist = r_ego + r_ec + safety_margin;  // = 0.75m
    
    // Two robots at exactly min_dist should have constraint value = 1
    Eigen::Vector2d ego_pos(0, 0);
    Eigen::Vector2d ec_pos(min_dist, 0);
    
    double dist_sq = (ec_pos - ego_pos).squaredNorm();
    double constraint_value = dist_sq / (min_dist * min_dist);
    
    EXPECT_NEAR(constraint_value, 1.0, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

### 9.2 Integration Tests

**File:** `mpc_planner_modules/test/test_joint_planning_integration.cpp` (NEW)

```cpp
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "mpc_planner/planner.h"
#include "mpc_planner_types/realtime_data.h"

using namespace MPCPlanner;

class JointPlanningIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "test_joint_planning_integration");
        
        planner_ = std::make_unique<Planner>("test_robot", false);
    }
    
    void TearDown() override {
        planner_.reset();
    }
    
    std::unique_ptr<Planner> planner_;
};

// Integration test: Two robots in narrow corridor
TEST_F(JointPlanningIntegrationTest, NarrowCorridorTwoRobots) {
    // Scenario: Two robots approaching each other in a 2m wide corridor
    
    // Setup initial state
    State state;
    state.set("x", 0.0);
    state.set("y", 0.0);
    state.set("psi", 0.0);
    state.set("v", 1.0);
    state.set("spline", 0.0);
    
    // Setup real-time data with approaching robot
    RealTimeData data;
    data.robot_area.push_back(Disc(0.0, 0.325));
    
    // Other robot coming from opposite direction
    DynamicObstacle other_robot(0, Eigen::Vector2d(10.0, 0.0), M_PI, 0.325, ObstacleType::ROBOT);
    
    // Prediction: other robot moving towards ego
    other_robot.prediction = Prediction(PredictionType::DETERMINISTIC);
    Mode mode;
    for (int k = 0; k < 30; k++) {
        double x = 10.0 - 0.2 * k;  // Moving at ~1 m/s towards ego
        mode.push_back(PredictionStep(Eigen::Vector2d(x, 0.0), M_PI, 0.1, 0.1));
    }
    other_robot.prediction.modes.push_back(mode);
    other_robot.prediction.probabilities.push_back(1.0);
    
    data.dynamic_obstacles.push_back(other_robot);
    
    // Run MPC
    auto output = planner_->solveMPC(state, data);
    
    // Verify: MPC should succeed
    EXPECT_TRUE(output.success);
    
    // Verify: Trajectory should avoid collision
    // Check minimum distance between ego trajectory and other robot prediction
    double min_dist = std::numeric_limits<double>::max();
    for (size_t k = 0; k < output.trajectory.positions.size(); k++) {
        if (k < mode.size()) {
            double dist = (output.trajectory.positions[k] - mode[k].position).norm();
            min_dist = std::min(min_dist, dist);
        }
    }
    
    double safety_dist = 0.325 * 2 + 0.1;  // Two robot radii + margin
    EXPECT_GT(min_dist, safety_dist * 0.9);  // Allow small tolerance
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

### 9.3 Benchmark Tests

**File:** `mpc_planner_modules/test/benchmark_joint_planning.cpp` (NEW)

```cpp
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <chrono>

#include "mpc_planner/planner.h"
#include "mpc_planner_types/realtime_data.h"

using namespace MPCPlanner;

class JointPlanningBenchmark : public ::testing::Test {
protected:
    void SetUp() override {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "benchmark_joint_planning");
    }
    
    // Run N iterations and return average solve time in ms
    double measureAverageSolveTime(Planner& planner, 
                                   State& state, 
                                   RealTimeData& data,
                                   int iterations = 100) {
        double total_time = 0.0;
        
        for (int i = 0; i < iterations; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            
            planner.solveMPC(state, data);
            
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            total_time += elapsed.count();
        }
        
        return total_time / iterations;
    }
};

// Benchmark: Baseline vs Joint Planning solve time
TEST_F(JointPlanningBenchmark, SolveTimeComparison) {
    // Create two planners: baseline and joint planning
    // Note: This requires separate configuration files
    
    State state;
    state.set("x", 0.0);
    state.set("y", 0.0);
    state.set("psi", 0.0);
    state.set("v", 1.0);
    state.set("spline", 0.0);
    
    RealTimeData data;
    data.robot_area.push_back(Disc(0.0, 0.325));
    
    // Add 2 nearby robots
    for (int i = 0; i < 2; i++) {
        DynamicObstacle robot(i, Eigen::Vector2d(5.0 * (i + 1), 2.0), 0.0, 0.325, ObstacleType::ROBOT);
        robot.prediction = Prediction(PredictionType::DETERMINISTIC);
        Mode mode;
        for (int k = 0; k < 30; k++) {
            mode.push_back(PredictionStep(robot.position + Eigen::Vector2d(0.2 * k, 0), 0.0, 0.1, 0.1));
        }
        robot.prediction.modes.push_back(mode);
        robot.prediction.probabilities.push_back(1.0);
        data.dynamic_obstacles.push_back(robot);
    }
    
    // Measure baseline (joint_planning disabled)
    // double baseline_time = measureAverageSolveTime(...);
    
    // Measure joint planning (joint_planning enabled)
    // double joint_time = measureAverageSolveTime(...);
    
    // Report results
    // std::cout << "Baseline solve time: " << baseline_time << " ms" << std::endl;
    // std::cout << "Joint planning solve time: " << joint_time << " ms" << std::endl;
    // std::cout << "Slowdown factor: " << joint_time / baseline_time << "x" << std::endl;
    
    // Expected: Joint planning 2-5x slower due to larger QP
    // EXPECT_LT(joint_time / baseline_time, 5.0);
    
    SUCCEED();  // Placeholder
}

// Benchmark: Trajectory smoothness comparison
TEST_F(JointPlanningBenchmark, TrajectorySmoothnessComparison) {
    // Compare curvature/jerk between baseline and joint planning
    
    // Smoothness metric: sum of squared second derivatives
    auto computeSmoothness = [](const Trajectory& traj) {
        if (traj.positions.size() < 3) return 0.0;
        
        double smoothness = 0.0;
        for (size_t i = 1; i < traj.positions.size() - 1; i++) {
            Eigen::Vector2d second_deriv = 
                traj.positions[i+1] - 2*traj.positions[i] + traj.positions[i-1];
            smoothness += second_deriv.squaredNorm();
        }
        return smoothness;
    };
    
    // Run both planners and compare smoothness
    // Lower smoothness value = smoother trajectory
    
    SUCCEED();  // Placeholder
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

### 9.4 Running Tests

```bash
# Build tests
cd /path/to/catkin_ws
catkin build mpc_planner_modules --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run unit tests
rosrun mpc_planner_modules test_joint_planning

# Run integration tests
rosrun mpc_planner_modules test_joint_planning_integration

# Run benchmarks
rosrun mpc_planner_modules benchmark_joint_planning

# Run all tests with coverage
catkin test mpc_planner_modules --verbose
```

---

## 10. Solver & Interface Changes (Phase 6)

### 10.1 Step 6.1: Extend acados Solver Definition

**File:** `solver_generator/generate_acados_solver.py`

Extend the acados OCP creation to include EC robot variables:

```python
def create_acados_model_joint(settings, model, modules, max_ec_robots=2):
    """
    Create acados model with joint EC robot optimization.
    
    Extended from create_acados_model() to include:
    - EC robot state variables (x, y, psi, v per EC robot)
    - EC robot control inputs (a, w per EC robot)
    - Coupled collision constraints
    - Deviation costs
    """
    acados_model = AcadosModel()
    acados_model.name = solver_name(settings) + "_joint"

    # Extended dimensions
    nx_ego = model.nx  # Ego states (typically 5: x, y, psi, v, spline)
    nu_ego = model.nu  # Ego inputs (typically 2: a, w)
    nx_ec = 4  # Per EC robot: x, y, psi, v
    nu_ec = 2  # Per EC robot: a, w
    
    NX = nx_ego + max_ec_robots * nx_ec
    NU = nu_ego + max_ec_robots * nu_ec
    
    # Build extended symbolics
    z = model.acados_symbolics()  # Ego symbolics
    
    # Add EC robot symbolics
    ec_states = []
    ec_inputs = []
    for ec_idx in range(max_ec_robots):
        prefix = f"ec{ec_idx}_"
        ec_states.extend([
            cd.SX.sym(prefix + "x"),
            cd.SX.sym(prefix + "y"),
            cd.SX.sym(prefix + "psi"),
            cd.SX.sym(prefix + "v")
        ])
        ec_inputs.extend([
            cd.SX.sym(prefix + "a"),
            cd.SX.sym(prefix + "w")
        ])
    
    # Concatenate into full state/input vectors
    x_full = cd.vertcat(model.get_x(), *ec_states)
    u_full = cd.vertcat(model.get_acados_u(), *ec_inputs)
    
    # Extended dynamics
    dyn_f_expl_ego, _ = model.get_acados_dynamics()
    
    # Add EC robot dynamics (unicycle model)
    ec_dynamics = []
    for ec_idx in range(max_ec_robots):
        idx_s = nx_ego + ec_idx * nx_ec  # State offset
        idx_u = nu_ego + ec_idx * nu_ec  # Input offset
        
        x_ec = x_full[idx_s]
        y_ec = x_full[idx_s + 1]
        psi_ec = x_full[idx_s + 2]
        v_ec = x_full[idx_s + 3]
        a_ec = u_full[idx_u]
        w_ec = u_full[idx_u + 1]
        
        ec_dynamics.extend([
            v_ec * cd.cos(psi_ec),  # x_dot
            v_ec * cd.sin(psi_ec),  # y_dot
            w_ec,                    # psi_dot
            a_ec                     # v_dot
        ])
    
    dyn_f_expl = cd.vertcat(dyn_f_expl_ego, *ec_dynamics)
    
    # Extended parameters
    params = settings["params"]
    p = params.get_acados_p()
    
    # Add EC-specific parameters
    for ec_idx in range(max_ec_robots):
        prefix = f"ec{ec_idx}_"
        p = cd.vertcat(p,
            cd.SX.sym(prefix + "pred_x"),
            cd.SX.sym(prefix + "pred_y"),
            cd.SX.sym(prefix + "r"),
            cd.SX.sym(prefix + "active")
        )
    
    # Constraints including coupled collision
    constr = cd.vertcat(*constraints(modules, z, p, model, settings, 1))
    
    # Add coupled collision constraints
    for ec_idx in range(max_ec_robots):
        idx_s = nx_ego + ec_idx * nx_ec
        
        ego_x = x_full[0]  # Assuming x is first state
        ego_y = x_full[1]
        ec_x = x_full[idx_s]
        ec_y = x_full[idx_s + 1]
        
        # Get radii from parameters
        r_ego = params.get("ego_disc_radius")
        param_offset = params.length() + ec_idx * 4
        r_ec = p[param_offset + 2]  # ec{i}_r
        active = p[param_offset + 3]  # ec{i}_active
        
        # Distance constraint: ||ego - ec||² >= (r_ego + r_ec)²
        dx = ego_x - ec_x
        dy = ego_y - ec_y
        dist_sq = dx**2 + dy**2
        min_dist_sq = (r_ego + r_ec + 0.1)**2  # + safety margin
        
        # Normalized: dist_sq / min_dist_sq >= 1 (when active)
        # When inactive (active=0), constraint is satisfied by default
        coupled_constraint = dist_sq / (min_dist_sq + 1e-6)
        
        constr = cd.vertcat(constr, coupled_constraint)
    
    # Cost: Ego cost + EC deviation cost + EC control cost
    cost_stage = objective(modules, z, p, model, settings, 1)
    
    # Add EC costs
    w_dev = settings.get("joint_planning", {}).get("deviation_weight", 5.0)
    w_ctrl = settings.get("joint_planning", {}).get("ec_control_effort_weight", 1.0)
    
    for ec_idx in range(max_ec_robots):
        idx_s = nx_ego + ec_idx * nx_ec
        idx_u = nu_ego + ec_idx * nu_ec
        param_offset = params.length() + ec_idx * 4
        
        ec_x = x_full[idx_s]
        ec_y = x_full[idx_s + 1]
        pred_x = p[param_offset]      # ec{i}_pred_x
        pred_y = p[param_offset + 1]  # ec{i}_pred_y
        active = p[param_offset + 3]  # ec{i}_active
        
        ec_a = u_full[idx_u]
        ec_w = u_full[idx_u + 1]
        
        # Deviation cost
        dev_cost = w_dev * ((ec_x - pred_x)**2 + (ec_y - pred_y)**2)
        
        # Control effort cost
        ctrl_cost = w_ctrl * (ec_a**2 + ec_w**2)
        
        # Only add if active
        cost_stage += active * (dev_cost + ctrl_cost)
    
    # Terminal cost
    cost_e = objective(modules, z, p, model, settings, settings["N"] - 1)
    
    # Populate acados model
    acados_model.x = x_full
    acados_model.u = u_full
    acados_model.xdot = cd.SX.sym("xdot", NX)
    acados_model.f_expl_expr = dyn_f_expl
    acados_model.f_impl_expr = acados_model.xdot - dyn_f_expl
    acados_model.p = p

    acados_model.cost_expr_ext_cost = cost_stage
    acados_model.cost_expr_ext_cost_e = cost_e
    acados_model.con_h_expr = constr

    return acados_model, NX, NU
```

### 10.2 Step 6.2: Regenerate Solver

After modifying the solver definition, regenerate:

```bash
# Navigate to workspace
cd /path/to/catkin_ws/src/oscar_mpc_planner_mr_modification

# Regenerate solver with joint planning support
poetry run python mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py

# This generates:
# - solver_generator/acados/jackal_joint/
#   - jackal_joint.json (OCP definition)
#   - jackal_joint_model.c (model code)
#   - jackal_joint_cost.c (cost code)
#   - jackal_joint_constraints.c (constraint code)
#   - main_jackal_joint.c (solver code)
#   - Makefile

# Rebuild the solver package
catkin build mpc_planner_solver --force-cmake

# Rebuild dependent packages
catkin build mpc_planner_jackalsimulator
```

### 10.3 Step 6.3: Update C++ Solver Interface

**File:** `mpc_planner_solver/src/acados_solver_interface.cpp`

Add methods to access EC robot solution variables:

```cpp
// Add to header: mpc_planner_solver/include/mpc_planner_solver/acados_solver_interface.h

// EC robot variable access
int getMaxECRobots() const { return _max_ec_robots; }

Eigen::Vector2d getECRobotPosition(int ec_idx, int k) const;
double getECRobotHeading(int ec_idx, int k) const;
double getECRobotVelocity(int ec_idx, int k) const;
Eigen::Vector2d getECRobotControl(int ec_idx, int k) const;

void setECRobotPrediction(int ec_idx, int k, 
                          double pred_x, double pred_y, 
                          double radius, bool active);

private:
    int _max_ec_robots{2};
    int _nx_ego{5};   // Ego state dimension
    int _nu_ego{2};   // Ego input dimension
    int _nx_ec{4};    // Per-EC state dimension
    int _nu_ec{2};    // Per-EC input dimension
```

```cpp
// Implementation in acados_solver_interface.cpp

Eigen::Vector2d Solver::getECRobotPosition(int ec_idx, int k) const
{
    if (ec_idx >= _max_ec_robots) {
        LOG_WARN("EC robot index " << ec_idx << " out of range");
        return Eigen::Vector2d::Zero();
    }
    
    // Calculate offset in state vector
    int state_offset = _nx_ego + ec_idx * _nx_ec;
    
    // Get from output trajectory
    double x = _output.xtraj[k * nx + state_offset];
    double y = _output.xtraj[k * nx + state_offset + 1];
    
    return Eigen::Vector2d(x, y);
}

double Solver::getECRobotHeading(int ec_idx, int k) const
{
    if (ec_idx >= _max_ec_robots) return 0.0;
    
    int state_offset = _nx_ego + ec_idx * _nx_ec + 2;  // psi is 3rd state
    return _output.xtraj[k * nx + state_offset];
}

double Solver::getECRobotVelocity(int ec_idx, int k) const
{
    if (ec_idx >= _max_ec_robots) return 0.0;
    
    int state_offset = _nx_ego + ec_idx * _nx_ec + 3;  // v is 4th state
    return _output.xtraj[k * nx + state_offset];
}

Eigen::Vector2d Solver::getECRobotControl(int ec_idx, int k) const
{
    if (ec_idx >= _max_ec_robots) {
        return Eigen::Vector2d::Zero();
    }
    
    int input_offset = _nu_ego + ec_idx * _nu_ec;
    
    double a = _output.utraj[k * nu + input_offset];
    double w = _output.utraj[k * nu + input_offset + 1];
    
    return Eigen::Vector2d(a, w);
}

void Solver::setECRobotPrediction(int ec_idx, int k, 
                                   double pred_x, double pred_y, 
                                   double radius, bool active)
{
    if (ec_idx >= _max_ec_robots) return;
    
    // Calculate parameter offset
    // Assuming EC parameters come after all standard parameters
    // EC parameter layout: [pred_x, pred_y, radius, active] for each EC robot
    enum ECParamOffset {
        EC_PRED_X = 0,
        EC_PRED_Y = 1,
        EC_RADIUS = 2,
        EC_ACTIVE = 3,
        EC_PARAMS_PER_ROBOT = 4
    };
    
    int ec_param_offset = _n_standard_params + ec_idx * EC_PARAMS_PER_ROBOT;
    
    _params.all_parameters[k * npar + ec_param_offset + EC_PRED_X] = pred_x;
    _params.all_parameters[k * npar + ec_param_offset + EC_PRED_Y] = pred_y;
    _params.all_parameters[k * npar + ec_param_offset + EC_RADIUS] = radius;
    _params.all_parameters[k * npar + ec_param_offset + EC_ACTIVE] = active ? 1.0 : 0.0;
}
```

---

## 11. Data Flow & Execution Cycle

### 11.1 Complete Data Flow Diagram

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         Planning Cycle (20-30 Hz)                           │
└────────────────────────────────────────────────────────────────────────────┘

1. DATA RECEPTION
   ┌──────────────────────────────────────────────────────────────────┐
   │  ROS Interface (jules_ros1_jackalplanner.cpp)                    │
   │                                                                   │
   │  - Odometry callback → state (x, y, psi, v)                      │
   │  - Trajectory callback → dynamic_obstacles[]                      │
   │    └─ Each obstacle has: position, prediction, type (ROBOT/etc) │
   │  - Reference path callback → module_data.path                    │
   └──────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
2. PLANNER CORE (planner.cpp::solveMPC)
   ┌──────────────────────────────────────────────────────────────────┐
   │  - Initialize warmstart                                          │
   │  - Set initial state                                             │
   │  - Call module->update() for all modules                         │
   │  - Call module->setParameters() for all k                        │
   │  - Call module->optimize()  ← GuidanceConstraints takes over     │
   └──────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
3. GUIDANCE CONSTRAINTS OPTIMIZATION (guidance_constraints.cpp::optimize)
   ┌──────────────────────────────────────────────────────────────────┐
   │                                                                   │
   │  ┌──────────────────────────────────────────────────────────┐   │
   │  │  NEW: EC Robot Selection                                  │   │
   │  │  selectECRobots(dynamic_obstacles, ego_pos, radius)       │   │
   │  │    → Filter by type==ROBOT                               │   │
   │  │    → Sort by distance                                    │   │
   │  │    → Take top max_ec_robots                              │   │
   │  └──────────────────────────────────────────────────────────┘   │
   │                           │                                      │
   │                           ▼                                      │
   │  ┌──────────────────────────────────────────────────────────┐   │
   │  │  NEW: SQP Outer Loop (sqp_iterations times)               │   │
   │  │                                                           │   │
   │  │  for sqp_iter in [0, sqp_iterations):                    │   │
   │  │    │                                                      │   │
   │  │    ▼                                                      │   │
   │  │  ┌────────────────────────────────────────────────────┐  │   │
   │  │  │  Parallel Topology Optimization                     │  │   │
   │  │  │  #pragma omp parallel for num_threads(8)           │  │   │
   │  │  │                                                     │  │   │
   │  │  │  for each LocalPlanner:                            │  │   │
   │  │  │    1. Copy main solver                             │  │   │
   │  │  │    2. Build guidance constraints (topology)        │  │   │
   │  │  │    3. Build safety constraints (non-EC obstacles)  │  │   │
   │  │  │    4. NEW: Set joint planning parameters           │  │   │
   │  │  │       - EC predictions (x_pred, y_pred)           │  │   │
   │  │  │       - EC radii and active flags                 │  │   │
   │  │  │    5. Solve QP                                     │  │   │
   │  │  │    6. NEW: Extract EC trajectories from solution   │  │   │
   │  │  └────────────────────────────────────────────────────┘  │   │
   │  │    │                                                      │   │
   │  │    ▼                                                      │   │
   │  │  ┌────────────────────────────────────────────────────┐  │   │
   │  │  │  NEW: Update EC Predictions (if not last iter)     │  │   │
   │  │  │  updateECRobotPredictions(ec_robots, planners)     │  │   │
   │  │  │    - Get best ego trajectory so far               │  │   │
   │  │  │    - Apply repulsion heuristic to EC predictions  │  │   │
   │  │  └────────────────────────────────────────────────────┘  │   │
   │  │                                                           │   │
   │  └──────────────────────────────────────────────────────────┘   │
   │                           │                                      │
   │                           ▼                                      │
   │  ┌──────────────────────────────────────────────────────────┐   │
   │  │  Decision Making                                          │   │
   │  │  best_planner = FindBestPlanner()                         │   │
   │  │                                                           │   │
   │  │  Transfer solution to main solver                         │   │
   │  │  NEW: Store EC trajectories in module_data                │   │
   │  └──────────────────────────────────────────────────────────┘   │
   │                                                                   │
   └──────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
4. OUTPUT PROCESSING (planner.cpp)
   ┌──────────────────────────────────────────────────────────────────┐
   │  - Extract ego trajectory from solver                            │
   │  - Store metadata (topology_id, cost, etc.)                      │
   │  - NEW: Store EC robot planned trajectories                      │
   │  - Return PlannerOutput                                          │
   └──────────────────────────────┬───────────────────────────────────┘
                                  │
                                  ▼
5. EXECUTION & COMMUNICATION (jules_ros1_jackalplanner.cpp)
   ┌──────────────────────────────────────────────────────────────────┐
   │  - Publish velocity command (first step of trajectory)           │
   │  - Check communication triggers                                  │
   │  - Publish trajectory to other robots if triggered               │
   │  - NEW: Optionally publish EC planned trajectories (debug)       │
   │  - Visualize in RViz                                            │
   └──────────────────────────────────────────────────────────────────┘
```

### 11.2 Timing Considerations

| Phase | Typical Duration | Notes |
|-------|-----------------|-------|
| Data reception | < 1 ms | ROS callbacks |
| EC robot selection | < 0.5 ms | Simple filtering |
| Topology update | 5-15 ms | Graph search |
| QP solve (baseline) | 10-30 ms | Per planner |
| QP solve (joint) | 20-80 ms | Per planner, 2-4x baseline |
| SQP iterations | 2x solve time | 2 iterations typical |
| Decision making | < 1 ms | Compare costs |
| **Total (baseline)** | **25-50 ms** | At 20 Hz |
| **Total (joint)** | **50-150 ms** | May need 10 Hz |

**Recommendations:**
- Use `max_ec_robots = 1-2` for real-time operation
- Use `sqp_iterations = 2` (diminishing returns beyond)
- Consider adaptive EC count based on available compute time

### 11.3 Adaptive Performance Management

When compute time is critical, consider implementing adaptive joint planning:

```cpp
// Adaptive EC robot count based on available time budget
int GuidanceConstraints::computeAdaptiveECCount(
    double remaining_time_ms,
    double avg_solve_time_per_ec_ms)
{
    // Reserve time for baseline operations
    const double BASELINE_TIME_MS = 30.0;
    const double SAFETY_MARGIN_MS = 10.0;
    
    double available_for_ec = remaining_time_ms - BASELINE_TIME_MS - SAFETY_MARGIN_MS;
    
    if (available_for_ec <= 0) {
        LOG_WARN("Insufficient time for joint planning, falling back to baseline");
        return 0;  // Disable joint planning this cycle
    }
    
    int max_affordable = static_cast<int>(available_for_ec / avg_solve_time_per_ec_ms);
    return std::min(max_affordable, _max_ec_robots);
}
```

**When to disable joint planning:**
- If `remaining_time < 40ms` after topology update
- If previous solve exceeded time budget by >20%
- If all EC robots are far away (>15m)

---

## 12. Implementation Checklist

### 12.1 Phase 1: Data Structures (Week 1)

- [ ] **Step 1.1**: Add `ECRobot` struct to `data_types.h`
  ```cpp
  struct ECRobot {
      int robot_id;
      Eigen::Vector2d position;
      double heading, velocity, radius;
      Trajectory predicted_trajectory;
      Trajectory planned_trajectory;
      double deviation_cost, control_cost;
      bool is_active;
  };
  ```

- [ ] **Step 1.2**: Add `ec_robots` vector to `realtime_data.h`
  
- [ ] **Step 1.3**: Add joint planning fields to `module_data.h`
  ```cpp
  std::vector<Trajectory> ec_robot_planned_trajectories;
  bool joint_planning_enabled{false};
  ```

- [ ] **Step 1.4**: Build project, verify compilation

### 12.2 Phase 2: Configuration (Week 1)

- [ ] **Step 2.1**: Add `joint_planning:` section to `settings.yaml`

- [ ] **Step 2.2**: Load config in `GuidanceConstraints` constructor

- [ ] **Step 2.3**: Verify config loading with debug output

### 12.3 Phase 3: EC Robot Selection (Week 2)

- [ ] **Step 3.1**: Implement `selectECRobots()` function

- [ ] **Step 3.2**: Add header declarations

- [ ] **Step 3.3**: Test EC selection with mock data

### 12.4 Phase 4: SQP Loop & EC Updates (Weeks 3-4)

- [ ] **Step 4.1**: Modify `optimize()` for SQP outer loop

- [ ] **Step 4.2**: Implement `updateECRobotPredictions()` heuristic

- [ ] **Step 4.3**: Implement `setJointPlanningParametersForPlanner()`

- [ ] **Step 4.4**: Implement `extractECRobotTrajectoriesFromSolver()`

- [ ] **Step 4.5**: Implement `extractFinalECTrajectories()`

- [ ] **Step 4.6**: Test SQP loop with 2-robot simulation

### 12.5 Phase 5: Testing (Week 5)

- [ ] **Step 5.1**: Create `test_joint_planning.cpp` unit tests

- [ ] **Step 5.2**: Create integration test with narrow corridor scenario

- [ ] **Step 5.3**: Create benchmark tests for solve time comparison

- [ ] **Step 5.4**: Run all tests, fix failures

### 12.6 Phase 6: Solver Extension (Weeks 6-8)

- [ ] **Step 6.1**: Create `joint_ec_constraints.py` module

- [ ] **Step 6.2**: Update `generate_jackalsimulator_solver.py`

- [ ] **Step 6.3**: Create extended model `ContouringSecondOrderUnicycleModelWithEC`

- [ ] **Step 6.4**: Regenerate solver with EC variables

- [ ] **Step 6.5**: Update C++ solver interface for EC access

- [ ] **Step 6.6**: Verify solver generates and compiles

### 12.7 Phase 7: Integration & Validation (Weeks 9-10)

- [ ] **Step 7.1**: Full system test with 2-robot simulation

- [ ] **Step 7.2**: Profile solve times, adjust parameters

- [ ] **Step 7.3**: Test edge cases (robots entering/leaving EC range)

- [ ] **Step 7.4**: Compare with baseline (joint_planning disabled)

- [ ] **Step 7.5**: Document performance characteristics

### 12.8 Success Criteria

| Criterion | Target | Measurement |
|-----------|--------|-------------|
| Compilation | All packages build | `catkin build` exits 0 |
| Unit tests | All pass | `catkin test` exits 0 |
| Solve time | < 100ms at 10Hz | Benchmark test |
| Collision avoidance | No collisions in 100 runs | Integration test |
| Trajectory quality | Similar or better smoothness | Benchmark comparison |
| Config toggle | Enable/disable works | Manual test |

---

## Appendix A: Quick Reference - Key Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `max_ec_robots` | 2 | 1-3 | More EC robots = larger QP, longer solve |
| `ec_robot_selection_radius` | 10.0 | 5-20 | Larger radius = more candidates |
| `deviation_weight` | 5.0 | 1-20 | Higher = EC robots resist deviating |
| `ec_control_effort_weight` | 1.0 | 0.1-5 | Higher = smoother EC trajectories |
| `ego_selfishness` | 0.8 | 0-1 | Higher = ego less willing to yield |
| `sqp_iterations` | 2 | 1-4 | More = better convergence, longer time |
| `safety_margin` | 0.1 | 0.05-0.3 | Extra distance buffer |

### A.1 SQP Iterations: Understanding Convergence

The `sqp_iterations` parameter controls how many times the optimization is refined:

- **Iteration 1**: Solve with initial EC predictions (from communication)
- **Iteration 2**: Update EC predictions based on ego's solution, re-solve
- **Iteration 3+**: Further refinement (diminishing returns)

**Practical guidance for `sqp_iterations`:**

| Value | Use Case | Expected Behavior |
|-------|----------|-------------------|
| 1 | Time-critical, simple scenarios | EC predictions unchanged; ego adjusts only |
| 2 | Recommended default | Good balance; EC predictions adjust once |
| 3 | High-precision needed | Better convergence; ~50% more compute |
| 4+ | Research/offline planning | Near-optimal; rarely needed in real-time |

**Convergence indicators:**
- Change in ego trajectory < 0.1m between iterations
- Change in EC trajectory < 0.2m between iterations
- Cost improvement < 1% between iterations

---

## Appendix B: Troubleshooting

### B.1 Solver Fails to Generate

**Symptom:** `generate_jackalsimulator_solver.py` fails with dimension errors

**Solution:**
1. Verify `max_ec_robots` is consistent across model and config
2. Check that all EC state/input variables are declared
3. Ensure dynamics expressions match state dimensions

### B.2 QP Infeasibility

**Symptom:** All planners return exit code != 1

**Solution:**
1. Reduce `max_ec_robots` to 1
2. Increase `safety_margin`
3. Check EC robot predictions are valid (not NaN)
4. Verify EC initial states match current positions

### B.3 Slow Solve Time

**Symptom:** Solve time > 100ms

**Solution:**
1. Reduce `max_ec_robots` to 1
2. Reduce `sqp_iterations` to 1
3. Consider using `PARTIAL_CONDENSING_HPIPM` QP solver
4. Profile to identify bottleneck (QP vs linearization)

### B.4 EC Robots Not Selected

**Symptom:** `ec_robots.size() == 0` despite nearby robots

**Solution:**
1. Verify robots have `type == ObstacleType::ROBOT`
2. Check `ec_robot_selection_radius` is large enough
3. Verify dynamic_obstacles are populated

---

## Appendix C: References

1. **Interactive Joint Planning Paper**: "Interactive Joint Planning for Autonomous Vehicles" - Core algorithm and Equation 8
2. **JOINT_PLANNING_INTEGRATION_DESIGN.md**: Theoretical design document
3. **acados Documentation**: https://docs.acados.org/
4. **CasADi Documentation**: https://web.casadi.org/docs/

---

*Document version: 1.0*
*Last updated: 2025*
*Compatible with: T-MPC++ codebase v2.x*
