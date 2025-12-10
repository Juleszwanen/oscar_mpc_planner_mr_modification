# Joint Planning Debug and Implementation Guide

## Document Information
- **Version**: 2.0
- **Purpose**: Corrected design and implementation for joint planning with proper module separation
- **Target**: Developers implementing Interactive Joint Planning (Variant B) in the MPC planner
- **Reference**: See `JOINT_PLANNING_INTEGRATION_DESIGN.md` for theoretical background

---

## Table of Contents

1. [Overview & Corrections](#1-overview--corrections)
2. [EC Dynamics Model (`solver_model.py`)](#2-ec-dynamics-model-solver_modelpy)
3. [Module Architecture (Constraint vs Objective)](#3-module-architecture-constraint-vs-objective)
4. [Implementation Files](#4-implementation-files)
5. [Solver Generation Updates](#5-solver-generation-updates)
6. [Configuration](#6-configuration)
7. [Integration Checklist](#7-integration-checklist)

---

## 1. Overview & Corrections

### 1.1 Issues with Previous Attempt

The previous joint planning implementation had two critical issues:

1. **Model Location**: `ContouringSecondOrderUnicycleModelWithEC` was defined in the wrong file
2. **Module Design**: `JointECConstraintModule` used dual inheritance (`ConstraintModule`, `ObjectiveModule`)

### 1.2 Corrections Made

#### Model Definition (Correction #1)
**Previous**: Model was defined elsewhere (or incorrectly structured)

**Corrected**: `ContouringSecondOrderUnicycleModelWithEC` is now defined in:
- **File**: `solver_generator/solver_model.py`
- **Rationale**: This follows the same pattern as all other dynamics models in the codebase (e.g., `SecondOrderUnicycleModel`, `ContouringSecondOrderUnicycleModel`, `BicycleModel2ndOrder`)
- **Inheritance**: Properly inherits from `DynamicsModel` base class

#### Module Design (Correction #2)
**Previous**: 
```python
class JointECConstraintModule(ConstraintModule, ObjectiveModule):  # BAD!
    ...
```

This causes a `self.type` conflict:
- `ConstraintModule.__init__()` sets `self.type = "constraint"`
- `ObjectiveModule.__init__()` sets `self.type = "objective"`

**Corrected**: Split into two separate modules:
1. `JointECObjectiveModule` (pure `ObjectiveModule`) - EC deviation and control costs
2. `JointECConstraintModule` (pure `ConstraintModule`) - Coupled collision constraints

---

## 2. EC Dynamics Model (`solver_model.py`)

### 2.1 Location and Rationale

**File**: `solver_generator/solver_model.py` (end of file)

**Why Here**:
- All dynamics models are defined in `solver_model.py`
- Follows established naming conventions
- Inherits from `DynamicsModel` base class
- Integrates with existing solver generation pipeline via `generate_acados_solver.py`

### 2.2 Model Implementation

```python
class ContouringSecondOrderUnicycleModelWithEC(DynamicsModel):
    """
    Extended unicycle model with EC (Ego-Conditioned) robot variables.
    
    Decision Variables per stage:
        Inputs:  [a, w, ec0_a, ec0_w, ec1_a, ec1_w, ...]
        States:  [x, y, psi, v, spline, ec0_x, ec0_y, ec0_psi, ec0_v, ...]
    
    Total dimensions:
        nu = 2 + max_ec_robots * 2
        nx = 5 + max_ec_robots * 4
    """
    
    def __init__(self, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        
        # Ego dimensions
        self.nu_ego = 2   # a, w
        self.nx_ego = 5   # x, y, psi, v, spline
        
        # EC robot dimensions (per robot)
        self.nu_ec = 2    # a_ec, w_ec
        self.nx_ec = 4    # x_ec, y_ec, psi_ec, v_ec
        
        # Total dimensions
        self.nu = self.nu_ego + max_ec_robots * self.nu_ec
        self.nx = self.nx_ego + max_ec_robots * self.nx_ec
        
        # Build state/input lists
        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]
        
        for ec_idx in range(max_ec_robots):
            prefix = f"ec{ec_idx}_"
            self.states.extend([prefix + "x", prefix + "y", 
                               prefix + "psi", prefix + "v"])
            self.inputs.extend([prefix + "a", prefix + "w"])
        
        self._setup_bounds()
```

### 2.3 Continuous Dynamics

```python
def continuous_model(self, x, u):
    """
    Ego + EC robot dynamics (all unicycle models).
    
    Ego:
        x_dot = v * cos(psi)
        y_dot = v * sin(psi)
        psi_dot = w
        v_dot = a
        spline_dot = v
    
    EC robot i:
        x_ec_dot = v_ec * cos(psi_ec)
        y_ec_dot = v_ec * sin(psi_ec)
        psi_ec_dot = w_ec
        v_ec_dot = a_ec
    """
    # Ego dynamics
    a_ego, w_ego = u[0], u[1]
    psi_ego, v_ego = x[2], x[3]
    
    ego_dynamics = [
        v_ego * cd.cos(psi_ego),  # x_dot
        v_ego * cd.sin(psi_ego),  # y_dot
        w_ego,                     # psi_dot
        a_ego,                     # v_dot
        v_ego                      # spline_dot
    ]
    
    # EC robot dynamics
    ec_dynamics = []
    for ec_idx in range(self.max_ec_robots):
        u_offset = self.nu_ego + ec_idx * self.nu_ec
        x_offset = self.nx_ego + ec_idx * self.nx_ec
        
        a_ec, w_ec = u[u_offset], u[u_offset + 1]
        psi_ec, v_ec = x[x_offset + 2], x[x_offset + 3]
        
        ec_dynamics.extend([
            v_ec * cd.cos(psi_ec),
            v_ec * cd.sin(psi_ec),
            w_ec,
            a_ec
        ])
    
    return np.array(ego_dynamics + ec_dynamics)
```

---

## 3. Module Architecture (Constraint vs Objective)

### 3.1 Existing Module Pattern

The codebase follows a clear separation:

```python
# control_modules.py

class Module:
    def __init__(self):
        self.module_name = "UNDEFINED"
        self.type = None  # Set by subclasses

class ConstraintModule(Module):
    def __init__(self):
        super().__init__()
        self.type = "constraint"      # <-- Fixed type
        self.constraints = []

class ObjectiveModule(Module):
    def __init__(self):
        super().__init__()
        self.type = "objective"       # <-- Fixed type
        self.objectives = []
```

### 3.2 Why Dual Inheritance Fails

```python
class JointECConstraintModule(ConstraintModule, ObjectiveModule):
    def __init__(self):
        ConstraintModule.__init__(self)  # Sets self.type = "constraint"
        ObjectiveModule.__init__(self)   # Overwrites! self.type = "objective"
        # Now self.type = "objective", but we have self.constraints!
```

In `solver_definition.py`:
```python
def define_parameters(modules, params, settings):
    for module in modules.modules:
        if module.type == "objective":  # EC module IS matched here
            module.define_parameters(params)  # But has .constraints, not .objectives!
    
    for module in modules.modules:
        if module.type == "constraint":  # EC module NOT matched here
            module.define_parameters(params)  # Never called for constraints!
```

### 3.3 Correct Design: Separate Modules

**JointECObjectiveModule** (pure ObjectiveModule):
```python
# joint_ec_objective.py

class JointECDeviationObjective(Objective):
    """J_dev = w_dev * ||x_ec - x_pred||²"""
    ...

class JointECControlObjective(Objective):
    """J_u = w_ctrl * (a_ec² + w_ec²)"""
    ...

class JointECObjectiveModule(ObjectiveModule):
    def __init__(self, settings, max_ec_robots=2):
        super().__init__()  # Sets self.type = "objective"
        self.objectives.append(JointECDeviationObjective(...))
        self.objectives.append(JointECControlObjective(...))
```

**JointECConstraintModule** (pure ConstraintModule):
```python
# joint_ec_constraints.py

class JointCollisionConstraint:
    """||x_ego - x_ec||² / (r_ego + r_ec)² >= 1"""
    ...

class JointECConstraintModule(ConstraintModule):
    def __init__(self, settings, max_ec_robots=2):
        super().__init__()  # Sets self.type = "constraint"
        self.constraints.append(JointCollisionConstraint(...))
```

---

## 4. Implementation Files

### 4.1 File Structure

```
oscar_mpc_planner_mr_modification/
├── solver_generator/
│   └── solver_model.py                 # + ContouringSecondOrderUnicycleModelWithEC
│
├── mpc_planner_modules/
│   └── scripts/
│       ├── joint_ec_objective.py       # NEW: EC deviation + control costs
│       └── joint_ec_constraints.py     # NEW: Coupled collision constraints
│
├── mpc_planner_jackalsimulator/
│   ├── config/
│   │   └── settings.yaml               # + joint_planning section
│   └── scripts/
│       └── generate_jackalsimulator_solver.py  # + joint planning configuration
│
└── docs/
    └── JOINT_PLANNING_DEBUG_AND_IMPLEMENTATION.md  # This file
```

### 4.2 Module Files

#### `joint_ec_objective.py`

```python
"""
EC Robot Objective Module (PURE ObjectiveModule)

Implements:
- JointECDeviationObjective: J_dev = w_dev * Σ ||x_ec - x_pred||²
- JointECControlObjective: J_u = w_ctrl * Σ (a_ec² + w_ec²)
"""

class JointECDeviationObjective(Objective):
    def define_parameters(self, params):
        params.add("ec_deviation_weight", add_to_rqt_reconfigure=True)
        for ec_idx in range(self.max_ec_robots):
            params.add(f"ec{ec_idx}_pred_x", bundle_name="ec_pred_x")
            params.add(f"ec{ec_idx}_pred_y", bundle_name="ec_pred_y")
            params.add(f"ec{ec_idx}_active", bundle_name="ec_active")
    
    def get_value(self, model, params, settings, stage_idx):
        w_dev = params.get("ec_deviation_weight")
        cost = 0.0
        for ec_idx in range(self.max_ec_robots):
            ec_x = model.get(f"ec{ec_idx}_x")
            ec_y = model.get(f"ec{ec_idx}_y")
            pred_x = params.get(f"ec{ec_idx}_pred_x")
            pred_y = params.get(f"ec{ec_idx}_pred_y")
            active = params.get(f"ec{ec_idx}_active")
            cost += active * w_dev * ((ec_x - pred_x)**2 + (ec_y - pred_y)**2)
        return cost
```

#### `joint_ec_constraints.py`

```python
"""
EC Robot Constraint Module (PURE ConstraintModule)

Implements coupled collision constraints:
    ||x_ego[k] - x_ec[k]||² >= (r_ego + r_ec)²

Key difference from standard constraints:
    Standard: x_obs is a PARAMETER (fixed)
    Joint:    x_ec is a DECISION VARIABLE (optimized)
"""

class JointCollisionConstraint:
    def __init__(self, n_discs, max_ec_robots):
        self.nh = max_ec_robots * n_discs  # One constraint per disc per EC robot
    
    def get_constraints(self, model, params, settings, stage_idx):
        constraints = []
        pos_ego = np.array([model.get("x"), model.get("y")])
        r_ego = params.get("ego_disc_radius")
        margin = params.get("joint_safety_margin")
        
        for ec_idx in range(self.max_ec_robots):
            ec_x = model.get(f"ec{ec_idx}_x")  # Decision variable!
            ec_y = model.get(f"ec{ec_idx}_y")  # Decision variable!
            ec_r = params.get(f"ec{ec_idx}_r")
            
            for disc_it in range(self.n_discs):
                disc_pos = self._get_disc_pos(pos_ego, disc_it, params, model)
                diff = disc_pos - np.array([ec_x, ec_y])
                dist_sq = diff[0]**2 + diff[1]**2
                min_dist_sq = (r_ego + ec_r + margin)**2
                
                # Normalized: dist²/min_dist² >= 1
                constraints.append(dist_sq / (min_dist_sq + 1e-6))
        
        return constraints
    
    def get_lower_bound(self):
        return [1.0] * self.nh
    
    def get_upper_bound(self):
        return [np.inf] * self.nh
```

---

## 5. Solver Generation Updates

### 5.1 Model Import

In `generate_jackalsimulator_solver.py`:

```python
from solver_model import (
    ContouringSecondOrderUnicycleModel,
    ContouringSecondOrderUnicycleModelWithSlack,
    ContouringSecondOrderUnicycleModelCurvatureAware,
    ContouringSecondOrderUnicycleModelWithEC  # NEW!
)

from joint_ec_objective import JointECObjectiveModule   # NEW!
from joint_ec_constraints import JointECConstraintModule  # NEW!
```

### 5.2 Configuration Function

```python
def configuration_tmpc_joint_planning(settings):
    """T-MPC++ with joint EC robot optimization."""
    
    joint_config = settings.get("joint_planning", {})
    enabled = joint_config.get("enabled", False)
    max_ec_robots = joint_config.get("max_ec_robots", 2)
    
    modules = ModuleManager()
    
    if enabled:
        model = ContouringSecondOrderUnicycleModelWithEC(max_ec_robots)
    else:
        model = ContouringSecondOrderUnicycleModel()
    
    # Base + Contouring modules (same as before)
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    modules.add_module(ContouringModule(settings))
    
    # Joint planning modules (when enabled)
    if enabled:
        modules.add_module(JointECObjectiveModule(settings, max_ec_robots))
        modules.add_module(JointECConstraintModule(settings, max_ec_robots))
    
    # Guidance constraints (obstacle avoidance for non-EC obstacles)
    modules.add_module(GuidanceConstraintModule(
        settings, constraint_submodule=EllipsoidConstraintModule
    ))
    
    return model, modules
```

### 5.3 Main Execution

```python
settings = load_settings()

if settings.get("joint_planning", {}).get("enabled", False):
    model, modules = configuration_tmpc_joint_planning(settings)
else:
    model, modules = configuration_tmpc_consistency_cost(settings)

generate_solver(modules, model, settings)
```

---

## 6. Configuration

### 6.1 Settings YAML

Add to `mpc_planner_jackalsimulator/config/settings.yaml`:

```yaml
joint_planning:
  enabled: false                          # Toggle joint planning
  max_ec_robots: 2                        # Max EC robots to jointly optimize
  
  # EC robot selection
  ec_robot_selection_radius: 10.0         # [m]
  
  # Cost weights (Equation 8 from IJP paper)
  deviation_weight: 5.0                   # J_dev weight
  ec_control_effort_weight: 1.0           # J_u weight for EC robots
  ego_selfishness: 0.8                    # η_e parameter
  
  # SQP iterations (for runtime, not solver generation)
  sqp_iterations: 2
  
  # Constraint parameters
  safety_margin: 0.1                      # [m] Additional buffer
  
  # EC robot dynamics bounds
  ec_max_velocity: 2.0                    # [m/s]
  ec_max_acceleration: 1.5                # [m/s²]
  ec_max_angular_velocity: 1.0            # [rad/s]
```

### 6.2 Enabling Joint Planning

1. Set `joint_planning.enabled: true` in settings.yaml
2. Regenerate the solver:
   ```bash
   cd /path/to/catkin_ws/src/oscar_mpc_planner_mr_modification
   poetry run python mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
   ```
3. Rebuild the package:
   ```bash
   catkin build mpc_planner_jackalsimulator
   ```

---

## 7. Integration Checklist

### Phase 1: Solver Generation (Python)
- [x] Add `ContouringSecondOrderUnicycleModelWithEC` to `solver_model.py`
- [x] Create `joint_ec_objective.py` with pure `ObjectiveModule`
- [x] Create `joint_ec_constraints.py` with pure `ConstraintModule`
- [x] Update `generate_jackalsimulator_solver.py` with imports and configuration
- [x] Add `joint_planning` section to `settings.yaml`

### Phase 2: C++ Runtime (Future Work)
- [ ] Implement EC robot selection in `guidance_constraints.cpp`
- [ ] Implement SQP outer loop for iterative coupling
- [ ] Add EC trajectory extraction from solver output
- [ ] Implement `setJointPlanningParametersForPlanner()`
- [ ] Update `module_data.h` with joint planning results

### Phase 3: Testing
- [ ] Verify solver generates with joint planning enabled
- [ ] Test with single EC robot
- [ ] Test with multiple EC robots
- [ ] Benchmark solve times
- [ ] Compare trajectory quality with/without joint planning

---

## Appendix A: Why This Design

### A.1 Model in `solver_model.py`

**Q: Why not put the EC model in `joint_ec_constraints.py`?**

A: The model defines the dynamics and state space for the entire optimization problem. It's used by:
1. `generate_acados_solver.py` to build the OCP
2. All modules to access state/input variables via `model.get()`

Putting it in a constraint module would break this pattern and make the dynamics dependent on a specific constraint being enabled.

### A.2 Separate Modules

**Q: Why not use a single module with both constraints and objectives?**

A: The solver generation pipeline iterates over modules twice:
```python
# First pass: objectives
for module in modules.modules:
    if module.type == "objective":
        module.define_parameters(params)

# Second pass: constraints
for module in modules.modules:
    if module.type == "constraint":
        module.define_parameters(params)
```

A module with `type = "objective"` (from dual inheritance) would only be processed in the first pass, leaving its constraints unregistered.

### A.3 Active Flag Pattern

**Q: Why use an `active` flag instead of dynamic EC robot count?**

A: The solver is generated with a fixed `max_ec_robots`. At runtime:
- Set `ec{i}_active = 1.0` for robots within selection radius
- Set `ec{i}_active = 0.0` for unused slots (positioned at 1000, 1000)

This allows the QP structure to remain fixed while handling variable numbers of nearby robots.

---

*Document version: 2.0*
*Last updated: 2025*
*Compatible with: T-MPC++ codebase*
