# Design Document: Integration of Interactive Joint Planning into Topology-Aware MPC Framework

## Document Information
- **Purpose**: Step-by-step design document for integrating joint optimization (Interactive Joint Planning) into the existing topology-aware explicit communication framework
- **Target Audience**: Developers familiar with MPC, homotopy, and ROS but not with the specifics of the IJP paper or current codebase
- **Constraint**: Topology/homotopy class extraction mechanism should remain unchanged; focus is on adding joint optimization

---

## Table of Contents
1. [Executive Summary](#1-executive-summary)
2. [Part A: Understanding the Source Materials](#part-a-understanding-the-source-materials)
   - [2.1 Interactive Joint Planning Paper Analysis](#21-interactive-joint-planning-paper-analysis)
   - [2.2 Current Codebase Analysis](#22-current-codebase-analysis)
3. [Part B: Mapping Concepts Between Methods](#part-b-mapping-concepts-between-methods)
4. [Part C: Integrated Method Design](#part-c-integrated-method-design)
5. [Part D: Code-Level Integration](#part-d-code-level-integration)
6. [Part E: Trade-offs and Variants](#part-e-trade-offs-and-variants)
7. [Part F: Recommendations](#part-f-recommendations)

---

## 1. Executive Summary

This document describes how to integrate **Interactive Joint Planning (IJP)**, a joint optimization approach for autonomous vehicles, into an existing **Topology-Aware MPC** framework (T-MPC++) that uses explicit communication between robots.

### Key Insight

- **IJP's core idea**: Instead of treating other agents as obstacles with fixed predicted trajectories, IJP **jointly optimizes** trajectories for both the ego vehicle and nearby agents. This produces implicit "ego-conditioned predictions" without expensive ego-conditioned prediction models.

- **Current T-MPC++ approach**: Multiple parallel MPC solvers explore different homotopy classes independently, each robot plans its own trajectory while treating other robots as dynamic obstacles with received/predicted trajectories.

- **Integration goal**: Add joint optimization capability **on top of** the existing topology exploration mechanism, enabling robots to reason about mutual trajectory adjustments while preserving the topology-aware planning structure.

---

## Part A: Understanding the Source Materials

### 2.1 Interactive Joint Planning Paper Analysis

#### 2.1.1 Core Algorithm Summary

The **Interactive Joint Planning (IJP)** paper presents an MPC-based planner that jointly optimizes over:
1. The **ego vehicle's trajectory** (what we control)
2. **Nearby agents' trajectories** (what we influence but don't directly control)

**Key Algorithm Steps** (from Algorithm 1 in the paper):

```
procedure IJP(xref, C, L, x0_e, x0_o)
  1. {xpred_o,j} ← TRAJ_PRED(C)           // Get unconditioned predictions for agents
  2. {xsample_e,i} ← EGO_SAMPLING(x0_e, L)  // Sample ego trajectory candidates
  3. {(xe,k, xo,k, hk)} ← HOM_SEL(...)      // Select promising homotopy candidates
  4. for r = 1,...,R do                     // Sequential Quadratic Programming (SQP) rounds
  5.   for k = 1,...,K do                   // For each homotopy class
  6.     QPk ← LINEARIZE(...)               // Build QP with joint decision variables
  7.     xe,k, xo,k ← SOLVE_QP(QPk)          // Solve joint QP
  8.   end for
  9. end for
  10. return xe associated with best homotopy class
end procedure
```

#### 2.1.2 Joint Optimization Formulation

The joint MPC solves the following Quadratic Program (QP):

**Decision Variables:**
- `ue`: Ego vehicle inputs (acceleration, steering)
- `uo`: Other agents' inputs (acceleration, steering)
- `xe`: Ego vehicle states over horizon
- `xo`: Other agents' states over horizon

**Cost Function (Equation 8 in paper):**
```
min_{ue,uo,xe,xo} ηe(Jref(xe,xref) + Ju(ue)) + ηo(Jdev(xo,xpred) + Ju(uo))
```

Where:
- `Jref`: Penalty on ego tracking error w.r.t. reference trajectory
- `Ju(ue)`: Penalty on ego acceleration and jerk
- `Jdev`: Penalty on agents' deviation from their **unconditioned** prediction
- `Ju(uo)`: Penalty on agents' acceleration and jerk
- `ηe`, `ηo`: Weights determining selfishness vs. altruism

**Constraints:**
- Initial state constraints: `xe[0] = x0_e`, `xo[0] = x0_o`
- Dynamics: `xi[t+1] = Ai[t]xi[t] + Bi[t]ui[t] + Ci[t]` (linearized)
- Input/state bounds: `Gd_x,i[t]xi[t] + Gd_u,i[t]ui[t] ≤ gd[t]`
- **Joint safety constraints**: `Gs_e[t]xe[t] + Gs_o[t]xo[t] ≤ gs[t]` (collision avoidance between ego AND other agents)
- Homotopy constraints (optional, often implicitly satisfied by initialization)

#### 2.1.3 How Agent Interaction is Modeled

**Key mechanisms:**

1. **Deviation Cost (`Jdev`)**: Penalizes other agents' trajectories for deviating from their unconditioned predictions. This acts as a "stiffness" - other agents won't deviate much unless necessary to avoid collision.

2. **Joint Collision Avoidance**: Collision constraints couple ego and agent trajectories. If ego moves into an agent's path, the QP will push the agent trajectory aside (while incurring deviation cost).

3. **Implicit Ego-Conditioning**: The solved agent trajectories serve as "ego-conditioned predictions" - they show how agents *would* react to the ego's planned motion.

4. **Balance via `ηe` and `ηo`**: 
   - Large `ηe` → selfish ego (agents must yield)
   - Small `ηe` → altruistic ego (ego yields to agents)

#### 2.1.4 Free-End Homotopy

IJP introduces "free-end homotopy" to categorize trajectories that don't share the same endpoint. Key concept:

**Mode** `m(x, xo)` classifies relative motion as:
- **CW (Clockwise)**: Ego passes agent on the right
- **S (Stationary)**: Ego maintains relative position to agent  
- **CCW (Counter-Clockwise)**: Ego passes agent on the left

This is computed via angular distance:
```
Δθ(x, xo) = Σ [arctan((Yi+1 - Yo_i+1)/(Xi+1 - Xo_i+1)) - arctan((Yi - Yo_i)/(Xi - Xo_i))]
```

### 2.2 Current Codebase Analysis

#### 2.2.1 High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS Interface Layer                                  │
│            (mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp)   │
│                                                                             │
│  • State callbacks (odometry)                                               │
│  • Obstacle callbacks (other robots' trajectories)                          │
│  • Goal/path callbacks                                                      │
│  • Command publishing (velocity commands)                                   │
│  • Communication triggers (topology-aware trajectory sharing)               │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Planner Core Layer                                   │
│                      (mpc_planner/src/planner.cpp)                          │
│                                                                             │
│  • Main control loop: solveMPC(state, data)                                 │
│  • Module orchestration (update → setParameters → optimize)                 │
│  • Warmstart management                                                     │
│  • PlannerOutput construction with topology metadata                        │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Module Layer                                         │
│               (mpc_planner_modules/src/guidance_constraints.cpp)            │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              GuidanceConstraints Module (T-MPC++)                    │   │
│  │                                                                      │   │
│  │  update():                                                           │   │
│  │    • global_guidance_->Update() - Graph search for topologies       │   │
│  │    • mapGuidanceTrajectoriesToPlanners() - Assign to LocalPlanners  │   │
│  │                                                                      │   │
│  │  optimize():                                                         │   │
│  │    #pragma omp parallel for num_threads(8)                          │   │
│  │    for each LocalPlanner:                                            │   │
│  │      • Copy main solver: *solver = *_solver                         │   │
│  │      • Build constraints (guidance + safety)                        │   │
│  │      • solver->solve()                                              │   │
│  │      • Store result (exit_code, objective, guidance_ID)             │   │
│  │                                                                      │   │
│  │    Serial: FindBestPlanner() → Select lowest cost feasible planner  │   │
│  │    Transfer solution to main _solver                                 │   │
│  │    Return exit_code                                                  │   │
│  │                                                                      │   │
│  │  LocalPlanner struct:                                                │   │
│  │    • local_solver: Independent MPC solver instance                  │   │
│  │    • guidance_constraints: LinearizedConstraints (topology)         │   │
│  │    • safety_constraints: Collision avoidance (GUIDANCE_CONSTRAINTS_TYPE) │
│  │    • result: SolverResult (exit_code, objective, guidance_ID)       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  Other Modules: MPCBaseModule, Contouring, GoalModule, etc.                │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Solver Layer                                         │
│              (mpc_planner_solver/src/forces_solver_interface.cpp)           │
│                                                                             │
│  • Parameter management (setParameter, getOutput)                           │
│  • Warmstart handling                                                       │
│  • Solver invocation (Forces Pro / Acados)                                  │
│  • Solution retrieval                                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### 2.2.2 Where Topologies/Homotopy Classes Are Computed and Stored

**Location**: `mpc_planner_modules/src/guidance_constraints.cpp`

**Computation** (in `GuidanceConstraints::update()`):
```cpp
global_guidance_->Update();  // Graph search finds topologically distinct paths
mapGuidanceTrajectoriesToPlanners();  // Maps each topology to a LocalPlanner
```

**Storage**:
- `global_guidance_->GetGuidanceTrajectory(i).topology_class` - Topology ID for trajectory i
- `planners_[p].result.guidance_ID` - After optimization, which topology this planner followed
- `module_data.selected_topology_id` - Final selected topology for this planning cycle
- `PlannerOutput::selected_topology_id` - Exposed to external code

**Key Files**:
- `guidance_planner/` package (external) - Core topology search
- `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h` - LocalPlanner struct
- `mpc_planner_types/include/mpc_planner_types/module_data.h` - ModuleData struct

#### 2.2.3 How MPC Problems Are Formulated and Solved Per Robot

**Current formulation** (per LocalPlanner):

**Decision Variables**: Only ego robot states and inputs
- States: `[x, y, psi, v, ...]` for k = 0...N
- Inputs: `[a, w, ...]` for k = 0...N-1

**Cost Function** (built from stacked modules):
```
minimize Σ_k [ w_a * a²_k + w_ω * ω²_k + w_v * (v_k - v_ref)² + MPCC_cost_k ]
```

**Constraints**:
- Dynamics: `x_{k+1} = f(x_k, u_k)` (unicycle model)
- Guidance constraints (topology): Linearized halfspaces keeping trajectory in topology
- Safety constraints: Ellipsoid/linear constraints for each dynamic obstacle
  - **Other robots treated as obstacles with predicted trajectories**

**Solver Interface** (`mpc_planner_solver/src/forces_solver_interface.cpp`):
- `solver->solve()` - Invokes Forces Pro / Acados
- `solver->setParameter(k, "param_name", value)` - Set constraint/cost parameters
- `solver->getOutput(k, "x")` - Retrieve solution

#### 2.2.4 How Explicit Communication Is Implemented

**What is sent**:
- Full planned trajectory: `[x_0, y_0, psi_0], [x_1, y_1, psi_1], ...`
- Robot ID
- Published via `mpc_planner_msgs/ObstacleGMM` message

**When it is sent** (from `mpc_planner_communication/src/communication_triggers.cpp`):
- `checkInfeasible()` - MPC failed
- `checkTopologyChange()` - Switched to different homotopy class
- `checkGeometricDeviation()` - Trajectory deviates from last communicated
- `checkTime()` - Heartbeat timer expired
- `checkNonGuidedHomologyFail()` - Using non-guided fallback planner

**How it is used in planning**:
- Received trajectories stored in `data.dynamic_obstacles`
- Each robot's trajectory becomes a `DynamicObstacle` with `Prediction`
- Safety constraints avoid these predicted trajectories

**Key Files**:
- `mpc_planner_communication/include/mpc_planner_communication/communication_triggers.h`
- `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp` - `publishCmdAndVisualize()`

---

## Part B: Mapping Concepts Between Methods

### 3.1 Concept Correspondence Table

| IJP Concept | Current T-MPC++ Implementation | Correspondence |
|-------------|-------------------------------|----------------|
| Ego vehicle trajectory `xe` | `local_solver->getOutput(k, "x/y/psi/v")` | Direct match |
| Other agents' trajectories `xo` | `data.dynamic_obstacles[i].prediction` | **Mismatch**: IJP optimizes these; T-MPC++ treats as fixed |
| Homotopy classes | `global_guidance_->GetGuidanceTrajectory(i).topology_class` | Conceptually similar, different computation method |
| Free-end homotopy mode | Not implemented | **Gap**: Could enhance topology identification |
| Unconditioned prediction | Received trajectory from other robot | Match: Both use external prediction as reference |
| Deviation cost `Jdev` | Not present | **Gap**: No cost for pushing agents from nominal |
| Joint collision constraints | Independent per robot | **Gap**: Constraints don't couple ego/agent decisions |
| SQP iterative refinement | Single QP solve per planner | Partial match: Could iterate |
| `ηe`, `ηo` balance | Not present | **Gap**: No selfishness/altruism trade-off |

### 3.2 Independent vs. Jointly Optimizable Components

**Currently Independent (per robot):**
1. ✗ Ego trajectory optimization
2. ✗ Topology/homotopy class selection
3. ✗ Cost computation and planner selection
4. ✗ Collision avoidance constraint satisfaction

**Could Be Jointly Optimized:**
1. ✓ Ego + other robots' trajectories (main IJP contribution)
2. ✓ Coupled collision avoidance constraints
3. ? Joint topology selection (advanced, optional)
4. ? Coordinated communication scheduling

### 3.3 Parts That Can Be Reused As-Is

| Component | Location | Reusability |
|-----------|----------|-------------|
| Topology extraction | `guidance_planner/` package | ✅ **Keep unchanged** |
| GlobalGuidance graph search | `global_guidance_->Update()` | ✅ **Keep unchanged** |
| Topology-to-planner mapping | `mapGuidanceTrajectoriesToPlanners()` | ✅ **Keep unchanged** |
| Parallel optimization framework | OpenMP parallel for in `optimize()` | ✅ **Reuse structure** |
| Communication triggers | `CommunicationTriggers` class | ✅ **Keep, may extend** |
| Solver interface | `Solver` class | ⚠️ **Extend for joint variables** |
| ROS interface | `jules_ros1_jackalplanner.cpp` | ✅ **Keep, extend callbacks** |

### 3.4 Parts That Need Coupling Across Robots

| Component | Current State | Required Change |
|-----------|--------------|-----------------|
| **Decision variables** | `[x_ego, u_ego]` | Add `[x_other, u_other]` for EC robots |
| **Cost function** | `J(x_ego, u_ego)` | Add `Jdev(x_other, x_pred)` + `Ju(u_other)` |
| **Collision constraints** | `||x_ego - x_obs|| ≥ d_safe` | Couple: `||x_ego[k] - x_other[k]|| ≥ d_safe` |
| **Planning horizon** | Same `N` for all robots | Must align horizons for joint optimization |
| **Communication content** | Trajectory only | Add: current state, velocity, intended action |

---

## Part C: Integrated Method Design

### 4.1 Current Baseline Pipeline Recap

```
For each control cycle (20-30 Hz):

1. RECEIVE DATA
   - State: x_ego = [x, y, ψ, v]
   - Other robots: trajectory predictions from communication
   - Reference path/goal

2. UPDATE MODULES
   - GuidanceConstraints::update()
     └─> global_guidance_->Update()  // Find N topologies
     └─> mapGuidanceTrajectoriesToPlanners()

3. PARALLEL OPTIMIZATION (T-MPC++)
   #pragma omp parallel for
   for each LocalPlanner p in [0, n_paths + 1]:
     - Copy main solver
     - Build guidance constraints (keep in topology)
     - Build safety constraints (avoid other robots AS OBSTACLES)
     - solver->solve()
     - Store result

4. DECISION
   - FindBestPlanner() → select lowest-cost feasible
   - Transfer to main solver

5. EXECUTE
   - Publish velocity command
   - Conditionally publish trajectory (topology-aware)

6. REPEAT
```

### 4.2 New Joint-Planning Concept (High-Level Idea)

**Core Change**: Instead of treating other robots as obstacles with fixed predictions, include their trajectories as **decision variables** in the MPC, coupled via collision constraints and bounded by deviation costs.

**Conceptual Flow**:
```
1. RECEIVE DATA (same as before)
   + Identify "EC robots" (Ego-Conditioned) vs. "non-EC robots"
   + EC robots: Will be jointly optimized
   + non-EC robots: Treated as fixed obstacles (like before)

2. UPDATE MODULES (unchanged)
   - Topology extraction remains the same
   
3. JOINT PARALLEL OPTIMIZATION (new!)
   for each LocalPlanner p:
     DECISION VARIABLES: [x_ego, u_ego] + [x_EC_1, u_EC_1] + ... + [x_EC_M, u_EC_M]
     
     COST:
       ego_cost: Reference tracking + control effort
       ec_cost: Σ_i [ Jdev(x_EC_i, x_pred_i) + Ju(u_EC_i) ]
     
     CONSTRAINTS:
       Dynamics: ego + each EC robot
       Safety: ||x_ego[k] - x_EC_i[k]|| ≥ d_safe  (COUPLED!)
       Topology: Keep ego in homotopy class (unchanged)
       Bounds: State/input limits for all robots
       
     solver->solve_joint()

4. DECISION (mostly unchanged)
   - Select best topology based on ego cost only
   - (Optional: Consider EC robot costs too)

5. EXECUTE (same as before)
   + (Optional: Share "planned" EC trajectories)
```

### 4.3 Detailed Design Changes

#### 4.3.1 Problem Formulation Changes

**New Decision Variables**:
```
Original:
  z = [x_ego_0, ..., x_ego_N, u_ego_0, ..., u_ego_{N-1}]
  Size: (n_states * (N+1) + n_inputs * N) per planner

Joint:
  z = [x_ego_0, ..., x_ego_N, u_ego_0, ..., u_ego_{N-1},
       x_ec1_0, ..., x_ec1_N, u_ec1_0, ..., u_ec1_{N-1},
       ...,
       x_ecM_0, ..., x_ecM_N, u_ecM_0, ..., u_ecM_{N-1}]
  Size: (1 + M) * (n_states * (N+1) + n_inputs * N)
```

**New Cost Terms**:
```cpp
// In cost function definition (solver generation)
// For each EC robot i at each timestep k:
Jdev_i_k = w_dev * ||x_ec_i_k - x_pred_i_k||²   // Deviation from prediction
Ju_i_k = w_u * (a_ec_i_k² + ω_ec_i_k²)          // Control effort

// Total added cost:
J_joint = Σ_i Σ_k (Jdev_i_k + Ju_i_k)
```

**New Constraints**:

```cpp
// Coupled collision avoidance (replacing fixed-obstacle constraints)
// For each EC robot i, at each timestep k:
||x_ego_k - x_ec_i_k||² ≥ (r_ego + r_ec_i)²  // Joint constraint

// EC robot dynamics (simple unicycle/point mass)
x_ec_i_{k+1} = f(x_ec_i_k, u_ec_i_k)

// EC robot input bounds
u_min ≤ u_ec_i_k ≤ u_max
```

#### 4.3.2 Communication Scheme Changes

**Current**:
- Send: Own planned trajectory
- Receive: Other robots' planned trajectories → use as obstacle predictions

**With Joint Planning**:

**Option A: No Additional Communication (Minimal)**
- Use received trajectories as `x_pred` (unconditioned prediction)
- Joint optimization implicitly conditions on ego's plan
- Other robot continues following its own plan (may not match IJP's "planned" trajectory)

**Option B: Share Joint Solution (Coordinated)**
- After solving, broadcast `x_ec_i` (what ego thinks robot i should do)
- Robot i receives ego's suggestion and can consider it
- Requires new message type or field:
  ```
  mpc_planner_msgs/JointPlanSuggestion:
    int32 source_robot_id
    int32 target_robot_id
    nav_msgs/Path suggested_trajectory
    float64 deviation_from_current_plan
  ```

**Option C: Negotiate (Advanced)**
- Exchange suggested plans
- Iterate to find mutually acceptable solution
- Significantly more complex

**Recommendation**: Start with **Option A**, extend to **Option B** later.

#### 4.3.3 Solver Structure Changes

**Current**: Single-agent QP per LocalPlanner

**Joint**: Multi-agent QP with coupled constraints

**Implementation Options**:

1. **Monolithic QP** (Centralized Joint Solve)
   - Single large QP with all robots' variables
   - Pros: Globally optimal within linearization
   - Cons: Scales poorly O((M+1)³) in solve time

2. **Distributed but Coupled** (ADMM-style)
   - Each robot solves its own QP
   - Share constraint residuals
   - Iterate to consensus
   - Pros: Scales better
   - Cons: Convergence issues, communication overhead

3. **Iterative Coupling** (SQP with External Loop)
   - Ego solves QP with current EC predictions
   - Update EC robot "predictions" based on ego plan
   - Repeat for R rounds
   - Pros: Reuses existing solver; easy implementation
   - Cons: May not converge; quality depends on rounds

**Recommendation**: Start with **Option 3** (Iterative Coupling) for minimal changes.

#### 4.3.4 Data Structures and Interface Changes

**New/Modified Data Structures**:

```cpp
// In mpc_planner_types/include/mpc_planner_types/realtime_data.h

struct ECRobot {
    int robot_id;
    Eigen::Vector2d position;
    double heading;
    double velocity;
    Trajectory predicted_trajectory;   // Unconditioned prediction (received)
    Trajectory planned_trajectory;     // Result of joint optimization (computed)
    bool is_active;                    // Whether to include in joint optimization
};

struct RealTimeData {
    // ... existing fields ...
    
    // NEW: EC robots for joint optimization
    std::vector<ECRobot> ec_robots;
    int max_ec_robots;  // Config parameter
};
```

```cpp
// In mpc_planner_types/include/mpc_planner_types/module_data.h

struct ModuleData {
    // ... existing fields ...
    
    // NEW: Joint planning results
    std::vector<Trajectory> ec_robot_planned_trajectories;
    std::vector<double> ec_robot_deviation_costs;
};
```

**New Configuration Parameters** (settings.yaml):

```yaml
joint_planning:
  enabled: true
  max_ec_robots: 3                    # Max robots to jointly optimize
  ec_robot_selection_radius: 10.0     # [m] Select EC robots within this radius
  deviation_weight: 5.0               # Weight for Jdev
  ec_control_effort_weight: 1.0       # Weight for Ju of EC robots
  ego_selfishness: 0.8                # ηe: 0=altruistic, 1=selfish
  sqp_iterations: 2                   # Number of iterative refinement rounds
  use_coupled_collision: true         # Joint collision constraints vs. fixed
```

### 4.4 How Joint Optimization Interacts with Homotopy Classes / Topologies

**Current Interaction**: Each LocalPlanner independently explores one homotopy class for the ego robot. Other robots don't have associated homotopy constraints.

**With Joint Optimization**:

**Option A: Ego-Only Homotopy (Recommended)**
- Homotopy constraints apply only to ego trajectory
- EC robot trajectories are free to adjust (bounded by deviation cost)
- Preserves existing topology exploration mechanism
- Joint optimization finds how EC robots react within each ego topology

**Option B: Joint Homotopy**
- Define homotopy for ego+EC pair relative to third parties
- Much more complex; combinatorial explosion
- Not recommended initially

**Implementation for Option A**:
```cpp
// In GuidanceConstraints::optimize(), within parallel loop:

// Guidance constraints (topology) - UNCHANGED, ego only
planner.guidance_constraints->update(state, data, module_data);

// Safety constraints - MODIFIED for joint optimization
if (CONFIG["joint_planning"]["enabled"].as<bool>()) {
    // Joint collision: ego vs. EC robots (coupled decision variables)
    planner.joint_safety_constraints->update(state, data, module_data);
    // Fixed collision: ego vs. non-EC obstacles (unchanged)
    planner.fixed_safety_constraints->update(state, data, module_data);
} else {
    // Original: ego vs. all obstacles (fixed)
    planner.safety_constraints->update(state, data, module_data);
}
```

### 4.5 How Joint Optimization Interacts with Explicit Communication

**Current Flow**:
```
Robot A publishes trajectory → Robot B receives → Robot B avoids trajectory as obstacle
```

**With Joint Optimization (Option A - Minimal)**:
```
Robot A publishes trajectory → Robot B receives as x_pred
Robot B jointly optimizes → Plans x_ego and "hypothetical" x_A
Robot B executes x_ego; x_A is internal only
Robot B publishes trajectory → Robot A receives...
```

**With Joint Optimization (Option B - Coordinated)**:
```
Robot A publishes trajectory → Robot B receives as x_pred
Robot B jointly optimizes → Plans x_ego and x_A_suggested
Robot B publishes trajectory + x_A_suggested
Robot A receives both:
  - Uses B's trajectory as prediction for B
  - Considers B's suggestion (optional)
Robot A jointly optimizes → Plans x_A and x_B_suggested
...
```

**Communication Trigger Extensions**:
```cpp
// New trigger: Significant change in EC robot planned trajectory
bool CommunicationTriggers::checkECPlanChange(
    const std::vector<Trajectory>& ec_planned,
    const std::vector<Trajectory>& ec_predicted,
    double threshold)
{
    for (size_t i = 0; i < ec_planned.size(); i++) {
        if (ec_planned[i].geometricDeviation(ec_predicted[i]) > threshold)
            return true;
    }
    return false;
}
```

---

## Part D: Code-Level Integration

### 5.1 File-by-File Integration Points

| File | Changes Required |
|------|------------------|
| `mpc_planner_modules/src/guidance_constraints.cpp` | Major: Add joint optimization logic in `optimize()` |
| `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h` | Add ECRobot handling, new member variables |
| `mpc_planner_types/include/mpc_planner_types/realtime_data.h` | Add `ECRobot` struct, `ec_robots` vector |
| `mpc_planner_types/include/mpc_planner_types/module_data.h` | Add joint planning result fields |
| `mpc_planner_solver/src/forces_solver_interface.cpp` | Extend for joint variables (if monolithic QP) |
| `solver_generator/generate_forces_solver.py` | Modify solver generation for joint variables |
| `mpc_planner_modules/scripts/ellipsoid_constraints.py` | Add coupled constraint formulation |
| `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp` | EC robot selection, callback modifications |
| `mpc_planner_jackalsimulator/config/settings.yaml` | Add `joint_planning:` configuration section |
| `mpc_planner_communication/src/communication_triggers.cpp` | Optional: Add EC plan change trigger |

### 5.2 Detailed Changes: guidance_constraints.cpp

**Location**: `mpc_planner_modules/src/guidance_constraints.cpp`

**Function**: `GuidanceConstraints::optimize()`

**Current Structure**:
```cpp
int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
{
    // Setup OpenMP
    omp_set_nested(1);
    omp_set_max_active_levels(2);
    
    #pragma omp parallel for num_threads(8)
    for (auto &planner : planners_) {
        // 1. Copy main solver
        *planner.local_solver = *_solver;
        
        // 2. Build constraints
        planner.guidance_constraints->update(...);
        planner.safety_constraints->update(...);
        
        // 3. Set parameters
        for (int k = 0; k < N; k++) {
            planner.guidance_constraints->setParameters(..., k);
            planner.safety_constraints->setParameters(..., k);
        }
        
        // 4. Solve
        planner.result.exit_code = planner.local_solver->solve();
        
        // 5. Process result
        planner.result.objective = planner.local_solver->_info.pobj;
    }
    
    // Serial: Find best planner
    best_planner_index_ = FindBestPlanner();
    
    // Transfer solution
    _solver->_output = best_solver->_output;
    
    return best_planner.result.exit_code;
}
```

**Proposed Changes** (Iterative Coupling Approach):

```cpp
int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
{
    // Setup OpenMP (unchanged)
    omp_set_nested(1);
    omp_set_max_active_levels(2);
    
    // NEW: Select EC robots for joint optimization
    std::vector<ECRobot> ec_robots;
    if (_joint_planning_enabled) {
        ec_robots = selectECRobots(data.dynamic_obstacles, state.getPos(), _ec_selection_radius);
        if (ec_robots.size() > _max_ec_robots) {
            ec_robots.resize(_max_ec_robots);
        }
    }
    
    // NEW: SQP outer loop for iterative coupling
    int sqp_rounds = _joint_planning_enabled ? _sqp_iterations : 1;
    
    for (int sqp_iter = 0; sqp_iter < sqp_rounds; sqp_iter++) {
        
        #pragma omp parallel for num_threads(8)
        for (auto &planner : planners_) {
            // 1. Copy main solver (unchanged)
            *planner.local_solver = *_solver;
            
            // 2. Build constraints
            planner.guidance_constraints->update(state, data, module_data);
            
            if (_joint_planning_enabled) {
                // NEW: Joint collision constraints
                buildJointCollisionConstraints(planner, ec_robots, data);
                // Fixed collision for non-EC obstacles
                planner.safety_constraints->update(state, data, module_data);
            } else {
                // Original: all obstacles treated as fixed
                planner.safety_constraints->update(state, data, module_data);
            }
            
            // 3. Set parameters
            for (int k = 0; k < _solver->N; k++) {
                planner.guidance_constraints->setParameters(..., k);
                planner.safety_constraints->setParameters(..., k);
                
                if (_joint_planning_enabled) {
                    // NEW: Set EC robot prediction as deviation reference
                    setECRobotDeviationCost(planner, ec_robots, k);
                }
            }
            
            // 4. Solve
            planner.result.exit_code = planner.local_solver->solve();
            
            // 5. Process result
            planner.result.objective = planner.local_solver->_info.pobj;
            
            if (_joint_planning_enabled) {
                // NEW: Extract EC robot "planned" trajectories
                extractECRobotTrajectories(planner, ec_robots);
            }
        }
        
        // NEW: Update EC robot predictions for next SQP iteration
        if (_joint_planning_enabled && sqp_iter < sqp_rounds - 1) {
            updateECRobotPredictions(ec_robots, planners_);
        }
    }
    
    // Serial: Find best planner (unchanged)
    best_planner_index_ = FindBestPlanner();
    
    // Transfer solution (unchanged)
    auto &best_solver = planners_[best_planner_index_].local_solver;
    _solver->_output = best_solver->_output;
    
    // NEW: Store EC robot planned trajectories in module_data
    if (_joint_planning_enabled) {
        module_data.ec_robot_planned_trajectories = 
            extractFinalECTrajectories(planners_[best_planner_index_], ec_robots);
    }
    
    return planners_[best_planner_index_].result.exit_code;
}
```

### 5.3 New Functions to Implement

```cpp
// In guidance_constraints.h

private:
    // Joint planning configuration
    bool _joint_planning_enabled{false};
    int _max_ec_robots{3};
    double _ec_selection_radius{10.0};
    double _deviation_weight{5.0};
    double _ec_control_weight{1.0};
    double _ego_selfishness{0.8};
    int _sqp_iterations{2};
    
    // NEW: Joint planning helper functions
    std::vector<ECRobot> selectECRobots(
        const std::vector<DynamicObstacle>& obstacles,
        const Eigen::Vector2d& ego_pos,
        double radius);
    
    void buildJointCollisionConstraints(
        LocalPlanner& planner,
        const std::vector<ECRobot>& ec_robots,
        const RealTimeData& data);
    
    void setECRobotDeviationCost(
        LocalPlanner& planner,
        const std::vector<ECRobot>& ec_robots,
        int k);
    
    void extractECRobotTrajectories(
        LocalPlanner& planner,
        std::vector<ECRobot>& ec_robots);
    
    void updateECRobotPredictions(
        std::vector<ECRobot>& ec_robots,
        const std::vector<LocalPlanner>& planners);
    
    std::vector<Trajectory> extractFinalECTrajectories(
        const LocalPlanner& planner,
        const std::vector<ECRobot>& ec_robots);
```

### 5.4 High-Level API Sketch

```cpp
// ============================================================================
// ECRobot struct (mpc_planner_types/include/mpc_planner_types/data_types.h)
// ============================================================================

struct ECRobot {
    int robot_id;                          // Unique identifier
    Eigen::Vector2d position;              // Current position
    double heading;                        // Current heading
    double velocity;                       // Current velocity
    
    Trajectory predicted_trajectory;       // Received/predicted trajectory (unconditioned)
    Trajectory planned_trajectory;         // Joint optimization result (ego-conditioned)
    
    double deviation_cost;                 // Cost incurred by deviation from predicted
    double control_cost;                   // Cost of EC robot's control inputs
    
    bool is_active;                        // Whether included in current optimization
    
    ECRobot(int id) : robot_id(id), is_active(false) {}
    
    double totalCost() const { return deviation_cost + control_cost; }
};

// ============================================================================
// JointConstraintsModule (new module, could extend EllipsoidConstraints)
// ============================================================================

class JointConstraintsModule : public ControllerModule {
public:
    JointConstraintsModule(std::shared_ptr<Solver> solver);
    
    void update(State &state, const RealTimeData &data, 
                ModuleData &module_data) override;
    
    void setParameters(const RealTimeData &data, 
                       const ModuleData &module_data, int k) override;
    
    // NEW: Set joint decision variables for EC robots
    void setECRobotVariables(const std::vector<ECRobot>& ec_robots, int k);
    
    // NEW: Get optimized EC robot trajectory from solver output
    void extractECRobotTrajectory(int ec_index, Trajectory& out_trajectory);
    
    // NEW: Build coupled collision constraint between ego and EC robot
    void buildCoupledCollisionConstraint(
        const ECRobot& ec_robot,
        int k,
        double safety_margin);

private:
    std::vector<ECRobot> _ec_robots;
    int _max_ec_robots;
    double _safety_margin;
};

// ============================================================================
// Extended Solver Interface (solver generation changes)
// ============================================================================

// In solver_generator/generate_forces_solver.py, add:

def add_ec_robot_variables(model, settings, num_ec_robots=3):
    """Add decision variables for EC robots."""
    for i in range(num_ec_robots):
        # EC robot states
        model.add_variable(f"ec{i}_x", length=N+1)
        model.add_variable(f"ec{i}_y", length=N+1)
        model.add_variable(f"ec{i}_psi", length=N+1)
        model.add_variable(f"ec{i}_v", length=N+1)
        
        # EC robot inputs
        model.add_variable(f"ec{i}_a", length=N)
        model.add_variable(f"ec{i}_w", length=N)

def add_ec_robot_costs(model, settings, num_ec_robots=3):
    """Add deviation and control costs for EC robots."""
    w_dev = settings["joint_planning"]["deviation_weight"]
    w_ctrl = settings["joint_planning"]["ec_control_effort_weight"]
    
    for i in range(num_ec_robots):
        # Deviation cost: ||x_ec - x_pred||²
        for k in range(N+1):
            model.add_cost(
                w_dev * (model.var(f"ec{i}_x", k) - model.param(f"ec{i}_pred_x", k))**2 +
                w_dev * (model.var(f"ec{i}_y", k) - model.param(f"ec{i}_pred_y", k))**2
            )
        
        # Control cost: ||u_ec||²
        for k in range(N):
            model.add_cost(
                w_ctrl * model.var(f"ec{i}_a", k)**2 +
                w_ctrl * model.var(f"ec{i}_w", k)**2
            )

def add_coupled_collision_constraints(model, settings, num_ec_robots=3):
    """Add collision constraints between ego and EC robots."""
    safety_margin = settings["joint_planning"]["safety_margin"]
    
    for i in range(num_ec_robots):
        for k in range(N+1):
            # ||ego - ec_i||² >= (r_ego + r_ec)²
            dx = model.var("x", k) - model.var(f"ec{i}_x", k)
            dy = model.var("y", k) - model.var(f"ec{i}_y", k)
            dist_sq = dx**2 + dy**2
            min_dist_sq = (model.param("r_ego") + model.param(f"ec{i}_r"))**2
            
            model.add_constraint(dist_sq >= min_dist_sq - safety_margin)

// ============================================================================
// Configuration (settings.yaml)
// ============================================================================

joint_planning:
  enabled: true
  max_ec_robots: 3
  ec_robot_selection_radius: 10.0
  deviation_weight: 5.0
  ec_control_effort_weight: 1.0
  ego_selfishness: 0.8
  sqp_iterations: 2
  use_coupled_collision: true
  safety_margin: 0.1
  
  # EC robot dynamics bounds
  ec_max_velocity: 2.0
  ec_max_acceleration: 1.5
  ec_max_angular_velocity: 1.0
```

---

## Part E: Trade-offs and Variants

### 6.1 Variant A: Minimal, Low-Intrusion Integration

**Description**: Add iterative coupling with minimal changes to solver structure. EC robots remain as fixed obstacles during optimization, but their predictions are updated between SQP rounds based on ego's plan.

**What Changes**:
- Add SQP outer loop in `optimize()`
- Heuristically update EC robot predictions based on simple collision avoidance model
- No solver regeneration required
- Communication unchanged

**What Remains Unchanged**:
- ✅ Topology extraction
- ✅ Solver formulation (single-agent QP)
- ✅ Parallel optimization structure
- ✅ Communication protocol
- ✅ All constraint modules

**Implementation Effort**: ~2-3 weeks

**Advantages**:
- Minimal code changes
- No solver regeneration
- Easy to toggle on/off
- Maintains real-time performance
- Low risk

**Disadvantages**:
- Not true joint optimization (heuristic update)
- May not converge to good solution
- Limited interaction modeling
- No guarantee of collision avoidance under joint motion

**Runtime Impact**: ~1.2x increase (additional SQP rounds)

**Scalability**: Good - scales with number of obstacles, not EC robots specifically

### 6.2 Variant B: Full Joint Optimization

**Description**: Implement true joint QP with EC robot decision variables, coupled collision constraints, and deviation costs. Requires solver regeneration.

**What Changes**:
- Extend solver with EC robot variables (`ec_x`, `ec_y`, `ec_a`, `ec_w`)
- Add deviation cost `Jdev` and EC control cost `Ju`
- Replace fixed collision constraints with coupled constraints
- Modify `LocalPlanner` to handle joint solution extraction
- Potentially extend communication to share suggested EC plans

**What Remains Unchanged**:
- ✅ Topology extraction (ego only)
- ✅ Guidance constraints (ego homotopy)
- ✅ Overall parallel structure (though QPs are larger)
- ✅ Basic communication triggers

**Implementation Effort**: ~6-10 weeks

**Advantages**:
- True joint optimization
- Better modeling of interaction
- Implicit ego-conditioned prediction
- More robust collision avoidance
- Matches IJP paper more closely

**Disadvantages**:
- Solver regeneration required
- QP size increases O((M+1)×n_states)
- Solve time increases significantly
- More complex debugging
- Risk of infeasibility with many EC robots

**Runtime Impact**: ~2-5x increase depending on EC robot count

**Scalability**: Limited - solve time scales cubically with state dimension

### 6.3 Comparison Table

| Aspect | Variant A (Minimal) | Variant B (Full Joint) |
|--------|---------------------|------------------------|
| Implementation effort | Low (2-3 weeks) | High (6-10 weeks) |
| Solver changes | None | Major regeneration |
| Interaction quality | Heuristic | Optimal (within linearization) |
| Runtime impact | 1.2x | 2-5x |
| Scalability | Good | Limited |
| Risk | Low | Medium-High |
| Matches IJP paper | Partially | Closely |
| Recommended for | Initial integration, testing | Production, complex scenarios |

### 6.4 Migration Path

**Phase 1**: Implement Variant A
- Validate concept with minimal changes
- Establish benchmarks
- Identify edge cases

**Phase 2**: Implement Variant B (if Phase 1 shows promise)
- Start with single EC robot
- Gradually increase complexity
- Profile and optimize

**Phase 3**: Production Integration
- Fine-tune parameters (`ηe`, `ηo`, weights)
- Extend communication protocol
- Add monitoring and logging

---

## Part F: Recommendations

### 7.1 Recommended Integration Path

1. **Start with Variant A** (Minimal) for proof of concept
2. Implement in a **separate module** (`JointPlanningModule`) that wraps `GuidanceConstraints`
3. Make it **easily toggleable** via config parameter
4. **Profile extensively** before committing to Variant B
5. If pursuing Variant B, consider **Acados** solver which may handle larger QPs better

### 7.2 Key Design Decisions

| Decision | Recommendation | Rationale |
|----------|----------------|-----------|
| EC robot selection | Distance-based, max 3 robots | Balances computation with interaction coverage |
| Homotopy treatment | Ego-only, EC free | Preserves existing topology mechanism |
| Communication | Start with Option A (no additional) | Simpler; add Option B later if needed |
| Solver approach | Iterative coupling first | Minimal changes; true joint later |
| SQP iterations | 2-3 rounds | Diminishing returns beyond |

### 7.3 Testing Strategy

1. **Unit Tests**: EC robot selection, constraint building
2. **Integration Tests**: Single-robot with simulated EC robots
3. **Multi-Robot Simulation**: 2-4 robots with joint planning
4. **Comparison Benchmarks**: With/without joint planning
5. **Edge Cases**: Narrow passages, head-on encounters, overtaking

### 7.4 Future Extensions

1. **Game-Theoretic Extension**: Model EC robots as strategic agents
2. **Learning Integration**: Learn `ηe/ηo` balance from human demonstration
3. **Distributed Joint Planning**: Each robot jointly plans; merge solutions
4. **Prediction Quality Feedback**: Use joint plan deviation to improve prediction

### 7.5 Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Infeasibility with many EC robots | Limit max EC robots; add slack variables |
| Solve time explosion | Adaptive EC count; early termination |
| Inconsistent plans between robots | Clear communication protocol; fallback to non-joint |
| Integration complexity | Modular design; extensive logging; feature flags |

---

## Appendix A: Glossary

| Term | Definition |
|------|------------|
| **EC Robot** | Ego-Conditioned Robot - an agent whose trajectory is jointly optimized with ego |
| **Non-EC Robot** | Robot treated as obstacle with fixed predicted trajectory |
| **Homotopy Class** | Topologically distinct category of paths around obstacles |
| **Free-End Homotopy** | Homotopy concept for trajectories without fixed endpoints |
| **SQP** | Sequential Quadratic Programming - iterative QP solving |
| **T-MPC++** | Topology-aware MPC with non-guided fallback planner |
| **Deviation Cost (Jdev)** | Penalty for EC robot's deviation from prediction |
| **ηe, ηo** | Balance weights between ego and EC robot costs |

## Appendix B: File Quick Reference

| Purpose | File Location |
|---------|---------------|
| Main planner logic | `mpc_planner/src/planner.cpp` |
| Guidance/topology module | `mpc_planner_modules/src/guidance_constraints.cpp` |
| Collision constraints | `mpc_planner_modules/src/ellipsoid_constraints.cpp` |
| Data types | `mpc_planner_types/include/mpc_planner_types/data_types.h` |
| Module data | `mpc_planner_types/include/mpc_planner_types/module_data.h` |
| Real-time data | `mpc_planner_types/include/mpc_planner_types/realtime_data.h` |
| Solver interface | `mpc_planner_solver/include/mpc_planner_solver/forces_solver_interface.h` |
| Solver generation | `solver_generator/generate_forces_solver.py` |
| Communication triggers | `mpc_planner_communication/src/communication_triggers.cpp` |
| ROS interface | `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp` |
| Configuration | `mpc_planner_jackalsimulator/config/settings.yaml` |

---

*Document version: 1.0*
*Last updated: December 2024*
