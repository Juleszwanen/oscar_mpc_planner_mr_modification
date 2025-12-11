# Joint Planning Debug and Implementation Guide

## Document Information
- **Version**: 3.0
- **Last Updated**: 2025
- **Purpose**: Complete design and implementation guide for joint planning with acados solver
- **Target**: Developers implementing Interactive Joint Planning (Variant B) in the MPC planner
- **Reference**: See `JOINT_PLANNING_INTEGRATION_DESIGN.md` for theoretical background
- **Solver**: acados (this guide is acados-specific as per project requirements)

---

## Table of Contents

1. [Overview & Key Design Decisions](#1-overview--key-design-decisions)
2. [EC Robot Dynamics Model](#2-ec-robot-dynamics-model)
3. [Module Architecture](#3-module-architecture)
4. [Complete Implementation](#4-complete-implementation)
5. [Solver Generation](#5-solver-generation)
6. [Configuration](#6-configuration)
7. [Integration Checklist](#7-integration-checklist)
8. [FAQ & Design Rationale](#8-faq--design-rationale)

---

## 1. Overview & Key Design Decisions

### 1.1 Purpose

This document describes the implementation of **Interactive Joint Planning (IJP)** for multi-robot coordination. The key idea is to jointly optimize trajectories for the ego robot and nearby "EC" (Ego-Conditioned) robots, rather than treating other robots as fixed obstacles.

### 1.2 Critical Design Decisions (Explicit Justifications)

The following design decisions are made **intentionally** and are documented here for clarity:

#### Decision 1: EC Robots Use `SecondOrderUnicycleModel` (No Spline Parameter)

**Why**: EC robots do not have spline information. We only receive their current geometric trajectory (position, heading, velocity) via communication. Therefore:

- **Ego robot**: Uses `ContouringSecondOrderUnicycleModel` dynamics with 5 states `[x, y, psi, v, spline]`
- **EC robots**: Use `SecondOrderUnicycleModel` dynamics with 4 states `[x, y, psi, v]` (NO spline)

This is correct because:
1. EC robots don't follow the ego's reference path (no shared spline)
2. EC robots' "prediction" is just geometric positions over time
3. The spline state is for path tracking, which only applies to ego

#### Decision 2: Model Defined in `solver_model.py`

**Why**: This follows the established codebase pattern where:
- All dynamics models inherit from `DynamicsModel`
- All models are defined in `solver_generator/solver_model.py`
- The solver generator imports models from this file

This is **intentional** and consistent with `SecondOrderUnicycleModel`, `ContouringSecondOrderUnicycleModel`, etc.

#### Decision 3: Separate Constraint and Objective Modules

**Why**: The codebase architecture requires modules to have a single `type`:
- `ConstraintModule` → `self.type = "constraint"`
- `ObjectiveModule` → `self.type = "objective"`

Multiple inheritance would cause conflicts. Instead, we use:
- `JointECObjectiveModule` (pure `ObjectiveModule`) for EC costs
- `JointECConstraintModule` (pure `ConstraintModule`) for coupled collision constraints

This is **intentional** to respect the existing module pattern.

---

## 2. EC Robot Dynamics Model

### 2.1 Model Location and Structure

**File**: `solver_generator/solver_model.py`
**Class**: `ContouringSecondOrderUnicycleModelWithEC`

### 2.2 Decision Variable Structure

```
Per MPC Stage k:
┌────────────────────────────────────────────────────────────────────┐
│ INPUTS (nu = 2 + M*2)                                              │
│ ┌────────┬────────┬─────────┬─────────┬─────────┬─────────┬────┐  │
│ │ a_ego  │ w_ego  │ a_ec0   │ w_ec0   │ a_ec1   │ w_ec1   │... │  │
│ └────────┴────────┴─────────┴─────────┴─────────┴─────────┴────┘  │
├────────────────────────────────────────────────────────────────────┤
│ STATES (nx = 5 + M*4)                                              │
│ ┌────┬────┬─────┬─────┬────────┬───────┬───────┬─────────┬───────┐│
│ │x_e │y_e │psi_e│v_e  │spline_e│ x_ec0 │ y_ec0 │ psi_ec0 │ v_ec0 ││
│ └────┴────┴─────┴─────┴────────┴───────┴───────┴─────────┴───────┘│
│ ┌───────┬───────┬─────────┬───────┬────┐                          │
│ │ x_ec1 │ y_ec1 │ psi_ec1 │ v_ec1 │... │                          │
│ └───────┴───────┴─────────┴───────┴────┘                          │
└────────────────────────────────────────────────────────────────────┘

Where M = max_ec_robots (typically 2)
```

### 2.3 Dynamics Equations

**Ego Robot** (ContouringSecondOrderUnicycleModel):
```
ẋ_ego = v_ego * cos(ψ_ego)
ẏ_ego = v_ego * sin(ψ_ego)  
ψ̇_ego = w_ego
v̇_ego = a_ego
ṡ_ego = v_ego                 ← Spline progress (arc length)
```

**EC Robot i** (SecondOrderUnicycleModel - NO spline):
```
ẋ_ec_i = v_ec_i * cos(ψ_ec_i)
ẏ_ec_i = v_ec_i * sin(ψ_ec_i)
ψ̇_ec_i = w_ec_i
v̇_ec_i = a_ec_i
```

**Key Point**: EC robots do NOT have a spline state because:
1. We don't have spline information for other robots
2. We only have their geometric predicted trajectory `[x, y]` over time
3. The spline is for MPCC path tracking, which only applies to ego

### 2.4 Complete Model Implementation

The complete implementation is in `solver_generator/solver_model.py`. Here is the full code:

```python
class ContouringSecondOrderUnicycleModelWithEC(DynamicsModel):
    """
    Joint planning model: Ego (with spline) + M EC robots (without spline).
    
    This model combines:
    - Ego: ContouringSecondOrderUnicycleModel (5 states: x, y, psi, v, spline)
    - EC robots: SecondOrderUnicycleModel (4 states each: x, y, psi, v)
    
    Design Rationale:
        EC robots use SecondOrderUnicycleModel (not ContouringSecondOrderUnicycleModel)
        because we only have geometric trajectory predictions for other robots,
        not spline/path information. The spline state is specifically for
        MPCC path tracking, which only applies to the ego robot.
    
    Total dimensions (for max_ec_robots = M):
        nx = 5 + M * 4
        nu = 2 + M * 2
    """

    def __init__(self, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        
        # Ego robot dimensions (ContouringSecondOrderUnicycleModel)
        self.nu_ego = 2  # a, w
        self.nx_ego = 5  # x, y, psi, v, spline
        
        # EC robot dimensions (SecondOrderUnicycleModel - NO spline)
        self.nu_ec = 2  # a_ec, w_ec
        self.nx_ec = 4  # x_ec, y_ec, psi_ec, v_ec (NO spline!)
        
        # Total dimensions
        self.nu = self.nu_ego + max_ec_robots * self.nu_ec
        self.nx = self.nx_ego + max_ec_robots * self.nx_ec
        
        # Build state and input lists
        self.states = ["x", "y", "psi", "v", "spline"]  # Ego states
        self.inputs = ["a", "w"]  # Ego inputs
        
        # EC robot states and inputs (no spline!)
        for ec_idx in range(max_ec_robots):
            prefix = f"ec{ec_idx}_"
            self.states.extend([prefix + "x", prefix + "y", prefix + "psi", prefix + "v"])
            self.inputs.extend([prefix + "a", prefix + "w"])
        
        # Setup bounds
        self._setup_bounds()

    def _setup_bounds(self):
        """Set up bounds for all variables."""
        # Position limits
        POS_LIMIT = 2000.0
        
        # Ego bounds (from ContouringSecondOrderUnicycleModel)
        EGO_MAX_ACCEL = 2.0
        EGO_MAX_ANGULAR_VEL = 0.8
        EGO_MAX_VEL = 3.0
        EGO_MIN_VEL = -0.01
        SPLINE_MIN = -1.0
        SPLINE_MAX = 10000.0
        
        # EC robot bounds
        EC_MAX_ACCEL = 1.5
        EC_MAX_ANGULAR_VEL = 1.0
        EC_MAX_VEL = 2.0
        EC_MIN_VEL = -0.1
        
        # Build bounds: [inputs..., states...]
        # Ego inputs
        self.lower_bound = [-EGO_MAX_ACCEL, -EGO_MAX_ANGULAR_VEL]
        self.upper_bound = [EGO_MAX_ACCEL, EGO_MAX_ANGULAR_VEL]
        
        # EC inputs
        for _ in range(self.max_ec_robots):
            self.lower_bound.extend([-EC_MAX_ACCEL, -EC_MAX_ANGULAR_VEL])
            self.upper_bound.extend([EC_MAX_ACCEL, EC_MAX_ANGULAR_VEL])
        
        # Ego states
        self.lower_bound.extend([-POS_LIMIT, -POS_LIMIT, -np.pi * 4, EGO_MIN_VEL, SPLINE_MIN])
        self.upper_bound.extend([POS_LIMIT, POS_LIMIT, np.pi * 4, EGO_MAX_VEL, SPLINE_MAX])
        
        # EC states (no spline!)
        for _ in range(self.max_ec_robots):
            self.lower_bound.extend([-POS_LIMIT, -POS_LIMIT, -np.pi * 4, EC_MIN_VEL])
            self.upper_bound.extend([POS_LIMIT, POS_LIMIT, np.pi * 4, EC_MAX_VEL])

    def continuous_model(self, x, u):
        """
        Continuous dynamics for ego and all EC robots.
        
        Ego: ContouringSecondOrderUnicycleModel dynamics (with spline)
        EC:  SecondOrderUnicycleModel dynamics (no spline)
        """
        # Ego dynamics
        a_ego = u[0]
        w_ego = u[1]
        psi_ego = x[2]
        v_ego = x[3]
        
        ego_dynamics = [
            v_ego * cd.cos(psi_ego),  # x_dot
            v_ego * cd.sin(psi_ego),  # y_dot
            w_ego,                     # psi_dot
            a_ego,                     # v_dot
            v_ego                      # spline_dot = velocity (arc length param)
        ]
        
        # EC robot dynamics (SecondOrderUnicycleModel - no spline!)
        ec_dynamics = []
        for ec_idx in range(self.max_ec_robots):
            u_offset = self.nu_ego + ec_idx * self.nu_ec
            x_offset = self.nx_ego + ec_idx * self.nx_ec
            
            a_ec = u[u_offset]
            w_ec = u[u_offset + 1]
            psi_ec = x[x_offset + 2]
            v_ec = x[x_offset + 3]
            
            ec_dynamics.extend([
                v_ec * cd.cos(psi_ec),  # x_ec_dot
                v_ec * cd.sin(psi_ec),  # y_ec_dot
                w_ec,                    # psi_ec_dot
                a_ec                     # v_ec_dot
            ])
        
        return np.array(ego_dynamics + ec_dynamics)

    def acados_symbolics(self):
        """Create CasADi symbolic variables for acados."""
        x = cd.SX.sym("x", self.nx)
        u = cd.SX.sym("u", self.nu)
        z = cd.vertcat(u, x)
        self.load(z)
        return z

    def get_acados_dynamics(self):
        """Return dynamics expressions for acados OCP."""
        self._x_dot = cd.SX.sym("x_dot", self.nx)
        f_expl = numpy_to_casadi(self.continuous_model(self._z[self.nu:], self._z[:self.nu]))
        f_impl = self._x_dot - f_expl
        return f_expl, f_impl

    def get_xinit(self):
        """
        Get indices for initial state constraint.
        
        Only ego robot states are constrained to initial state.
        EC robot initial states are set via parameters.
        """
        return range(self.nu, self.nu + self.nx_ego)

    def get_ec_state_indices(self, ec_idx):
        """Get state indices for a specific EC robot."""
        if ec_idx >= self.max_ec_robots:
            raise ValueError(f"EC robot index {ec_idx} out of range")
        start_idx = self.nx_ego + ec_idx * self.nx_ec
        return range(start_idx, start_idx + self.nx_ec)

    def get_ec_input_indices(self, ec_idx):
        """Get input indices for a specific EC robot."""
        if ec_idx >= self.max_ec_robots:
            raise ValueError(f"EC robot index {ec_idx} out of range")
        start_idx = self.nu_ego + ec_idx * self.nu_ec
        return range(start_idx, start_idx + self.nu_ec)
```

---

## 3. Module Architecture

### 3.1 Module Design Pattern

The codebase follows a strict separation:

```python
# control_modules.py

class Module:
    def __init__(self):
        self.module_name = "UNDEFINED"
        self.type = None  # Set by subclasses

class ConstraintModule(Module):
    def __init__(self):
        super().__init__()
        self.type = "constraint"      # Fixed type
        self.constraints = []

class ObjectiveModule(Module):
    def __init__(self):
        super().__init__()
        self.type = "objective"       # Fixed type
        self.objectives = []
```

### 3.2 Why Dual Inheritance Fails

```python
# BAD: This causes self.type conflict!
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

The correct approach is to split into two modules:

**JointECObjectiveModule** (pure ObjectiveModule):
- Location: `mpc_planner_modules/scripts/joint_ec_objective.py`
- Contains: `JointECDeviationObjective`, `JointECControlObjective`
- Purpose: EC robot deviation and control costs

**JointECConstraintModule** (pure ConstraintModule):
- Location: `mpc_planner_modules/scripts/joint_ec_constraints.py`
- Contains: `JointCollisionConstraint`
- Purpose: Coupled collision constraints between ego and EC robots

---

## 4. Complete Implementation

### 4.1 File Structure

```
oscar_mpc_planner_mr_modification/
├── solver_generator/
│   └── solver_model.py                 # ContouringSecondOrderUnicycleModelWithEC
│
├── mpc_planner_modules/
│   └── scripts/
│       ├── joint_ec_objective.py       # EC deviation + control costs
│       └── joint_ec_constraints.py     # Coupled collision constraints
│
├── mpc_planner_jackalsimulator/
│   ├── config/
│   │   └── settings.yaml               # joint_planning section
│   └── scripts/
│       └── generate_jackalsimulator_solver.py  # Joint planning configuration
│
└── docs/
    └── JOINT_PLANNING_DEBUG_AND_IMPLEMENTATION.md
```

### 4.2 Complete `joint_ec_objective.py`

```python
"""
Joint EC (Ego-Conditioned) Robot Objective Module

This module defines the objective (cost) terms for EC robots in joint optimization.
It implements the deviation cost (J_dev) and control effort cost (J_u) from the
Interactive Joint Planning paper (Equation 8):

    J_ec = η_o * [J_dev(x_ec, x_pred) + J_u(u_ec)]

Where:
    - J_dev: Penalty for EC robot deviation from unconditioned prediction
    - J_u: Penalty for EC robot control effort (acceleration and angular velocity)
    - η_o: Weight for EC robot costs (altruism parameter)

Design Decision:
    This is implemented as a PURE ObjectiveModule (not mixed with ConstraintModule)
    to respect the existing module architecture.
"""

import os
import sys
import casadi as cd
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ObjectiveModule, Objective


class JointECDeviationObjective(Objective):
    """
    Deviation cost penalizing EC robots for deviating from their predicted trajectories.
    
    Cost term:
        J_dev_i[k] = w_dev * ||pos_ec_i[k] - pos_pred_i[k]||²
                   = w_dev * ((x_ec - x_pred)² + (y_ec - y_pred)²)
    """

    def __init__(self, max_ec_robots, settings):
        super().__init__()
        self.max_ec_robots = max_ec_robots
        self.settings = settings

    def define_parameters(self, params):
        """Define EC robot prediction parameters."""
        params.add(
            "ec_deviation_weight", 
            add_to_rqt_reconfigure=True,
            rqt_config_name=lambda p: f'["weights"]["ec_deviation"]',
            rqt_min_value=0.0,
            rqt_max_value=50.0
        )
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            params.add(prefix + "pred_x", bundle_name="ec_pred_x")
            params.add(prefix + "pred_y", bundle_name="ec_pred_y")
            params.add(prefix + "active", bundle_name="ec_active")

    def get_value(self, model, params, settings, stage_idx):
        """Compute deviation cost for all EC robots at this stage."""
        w_dev = params.get("ec_deviation_weight")
        total_cost = 0.0
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            try:
                ec_x = model.get(prefix + "x")
                ec_y = model.get(prefix + "y")
            except (IOError, KeyError):
                continue
            
            pred_x = params.get(prefix + "pred_x")
            pred_y = params.get(prefix + "pred_y")
            active = params.get(prefix + "active")
            
            dev_x = ec_x - pred_x
            dev_y = ec_y - pred_y
            deviation_cost = w_dev * (dev_x**2 + dev_y**2)
            
            total_cost += active * deviation_cost
        
        return total_cost


class JointECControlObjective(Objective):
    """
    Control effort cost penalizing EC robot control inputs.
    
    Cost term:
        J_u_i[k] = w_ctrl * (a_ec_i² + w_ec_i²)
    """

    def __init__(self, max_ec_robots, settings):
        super().__init__()
        self.max_ec_robots = max_ec_robots
        self.settings = settings

    def define_parameters(self, params):
        """Define EC robot control weight parameter."""
        params.add(
            "ec_control_weight",
            add_to_rqt_reconfigure=True,
            rqt_config_name=lambda p: f'["weights"]["ec_control"]',
            rqt_min_value=0.0,
            rqt_max_value=10.0
        )

    def get_value(self, model, params, settings, stage_idx):
        """Compute control effort cost for all EC robots at this stage."""
        w_ctrl = params.get("ec_control_weight")
        total_cost = 0.0
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            try:
                ec_a = model.get(prefix + "a")
                ec_w = model.get(prefix + "w")
            except (IOError, KeyError):
                continue
            
            active = params.get(prefix + "active")
            control_cost = w_ctrl * (ec_a**2 + ec_w**2)
            total_cost += active * control_cost
        
        return total_cost


class JointECObjectiveModule(ObjectiveModule):
    """
    Objective module for EC (Ego-Conditioned) robot costs in joint optimization.
    
    Usage:
        if settings.get("joint_planning", {}).get("enabled", False):
            max_ec_robots = settings["joint_planning"]["max_ec_robots"]
            modules.add_module(JointECObjectiveModule(settings, max_ec_robots))
    """

    def __init__(self, settings, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        
        self.module_name = "JointECObjective"
        self.import_name = "joint_ec_objective.h"
        self.description = (
            f"EC robot deviation and control costs for joint optimization "
            f"(max {max_ec_robots} EC robots)"
        )
        
        self.objectives.append(JointECDeviationObjective(max_ec_robots, settings))
        self.objectives.append(JointECControlObjective(max_ec_robots, settings))
```

### 4.3 Complete `joint_ec_constraints.py`

```python
"""
Joint EC (Ego-Conditioned) Robot Constraint Module

This module defines the COUPLED collision constraints for joint optimization.
Unlike standard collision avoidance where obstacles have fixed trajectories,
these constraints couple the ego robot and EC robot positions as joint
decision variables.

Standard constraint (fixed obstacle):
    ||x_ego[k] - x_obs[k]||² ≥ (r_ego + r_obs)²   where x_obs is a parameter (fixed)

Coupled constraint (joint optimization):
    ||x_ego[k] - x_ec[k]||² ≥ (r_ego + r_ec)²    where x_ec is a decision variable
"""

import os
import sys
import casadi as cd
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from util.math import rotation_matrix
from control_modules import ConstraintModule


class JointCollisionConstraint:
    """
    Coupled collision constraint between ego robot and EC robots.
    
    Constraint formulation (per EC robot, per ego disc):
        h(x) = ||pos_ego_disc - pos_ec||² / (r_ego + r_ec + margin)²
        
    Bound: h(x) ≥ 1.0 (constraint satisfied when distance ≥ min_distance)
    """

    def __init__(self, n_discs, max_ec_robots):
        self.n_discs = n_discs
        self.max_ec_robots = max_ec_robots
        self.nh = max_ec_robots * n_discs

    def define_parameters(self, params):
        """Define EC robot radius and safety margin parameters."""
        if not params.has_parameter("joint_safety_margin"):
            params.add("joint_safety_margin", bundle_name="joint_safety_margin")
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            if not params.has_parameter(prefix + "r"):
                params.add(prefix + "r", bundle_name="ec_r")

    def get_lower_bound(self):
        """Lower bound for constraint: h(x) ≥ 1.0"""
        return [1.0] * (self.max_ec_robots * self.n_discs)

    def get_upper_bound(self):
        """Upper bound for constraint: h(x) ≤ inf"""
        return [np.inf] * (self.max_ec_robots * self.n_discs)

    def get_constraints(self, model, params, settings, stage_idx):
        """Build coupled collision constraints."""
        constraints = []
        
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])
        
        try:
            psi = model.get("psi")
        except Exception:
            psi = 0.0
        
        rotation_car = rotation_matrix(psi)
        r_ego = params.get("ego_disc_radius")
        safety_margin = params.get("joint_safety_margin")
        
        CONSTRAINT_EPSILON = 1e-6
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            try:
                ec_x = model.get(prefix + "x")
                ec_y = model.get(prefix + "y")
            except (IOError, KeyError):
                raise RuntimeError(
                    f"EC robot {ec_idx} variables not found in model. "
                    f"Ensure ContouringSecondOrderUnicycleModelWithEC is used."
                )
            
            ec_pos = np.array([ec_x, ec_y])
            ec_r = params.get(prefix + "r")
            
            min_dist = r_ego + ec_r + safety_margin
            min_dist_sq = min_dist**2
            
            for disc_it in range(self.n_discs):
                disc_offset = params.get(f"ego_disc_{disc_it}_offset")
                disc_relative_pos = np.array([disc_offset, 0])
                disc_pos = pos + rotation_car @ disc_relative_pos
                
                diff = disc_pos - ec_pos
                dist_sq = diff[0]**2 + diff[1]**2
                
                constraint = dist_sq / (min_dist_sq + CONSTRAINT_EPSILON)
                constraints.append(constraint)
        
        return constraints


class JointECConstraintModule(ConstraintModule):
    """
    Constraint module for coupled collision avoidance in joint optimization.
    
    Usage:
        if settings.get("joint_planning", {}).get("enabled", False):
            max_ec_robots = settings["joint_planning"]["max_ec_robots"]
            modules.add_module(JointECConstraintModule(settings, max_ec_robots))
    """

    def __init__(self, settings, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        self.n_discs = settings.get("n_discs", 1)
        
        self.module_name = "JointECConstraints"
        self.import_name = "joint_ec_constraints.h"
        self.description = (
            f"Coupled collision constraints for joint optimization "
            f"({max_ec_robots} EC robots × {self.n_discs} discs)"
        )
        
        self.constraints.append(JointCollisionConstraint(self.n_discs, max_ec_robots))
```

---

## 5. Solver Generation

### 5.1 Complete `generate_jackalsimulator_solver.py` Configuration

The solver generator script includes the joint planning configuration function:

```python
def configuration_tmpc_joint_planning(settings):
    """
    T-MPC++ with joint EC robot optimization (Variant B from IJP paper).
    
    This configuration:
    - Uses ContouringSecondOrderUnicycleModelWithEC for extended state/input space
    - EC robots use SecondOrderUnicycleModel dynamics (NO spline)
    - Adds JointECObjectiveModule for EC deviation and control costs
    - Adds JointECConstraintModule for coupled collision constraints
    """
    joint_planning_config = settings.get("joint_planning", {})
    joint_planning_enabled = joint_planning_config.get("enabled", False)
    max_ec_robots = joint_planning_config.get("max_ec_robots", 2)
    
    modules = ModuleManager()
    
    if joint_planning_enabled:
        model = ContouringSecondOrderUnicycleModelWithEC(max_ec_robots=max_ec_robots)
        print(f"[Joint Planning] Using model with {max_ec_robots} EC robots")
        print(f"[Joint Planning] Ego: 5 states (x,y,psi,v,spline), 2 inputs (a,w)")
        print(f"[Joint Planning] EC:  4 states each (x,y,psi,v - NO spline), 2 inputs each (a,w)")
        print(f"[Joint Planning] Total: nx={model.nx}, nu={model.nu}")
    else:
        model = ContouringSecondOrderUnicycleModel()
    
    # Base module
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    
    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(
            var_name="v",
            weight_names=["velocity", "reference_velocity"],
            cost_function=lambda x, w: w[0] * (x - w[1])**2
        )
    
    # Contouring module
    modules.add_module(ContouringModule(settings))
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings))
    
    # Consistency module if enabled
    if settings.get("JULES", {}).get("consistency_enabled", False):
        modules.add_module(ConsistencyModule(settings))
    
    # Joint EC modules (only when joint planning is enabled)
    if joint_planning_enabled:
        modules.add_module(JointECObjectiveModule(settings, max_ec_robots=max_ec_robots))
        modules.add_module(JointECConstraintModule(settings, max_ec_robots=max_ec_robots))
        print(f"[Joint Planning] Added JointECObjectiveModule and JointECConstraintModule")
    
    # Guidance constraints
    modules.add_module(GuidanceConstraintModule(
        settings,
        constraint_submodule=EllipsoidConstraintModule
    ))
    
    return model, modules
```

### 5.2 Configuration Selection

```python
def select_configuration(settings):
    """Select configuration based on settings."""
    joint_planning_enabled = settings.get("joint_planning", {}).get("enabled", False)
    
    if joint_planning_enabled:
        print("=" * 60)
        print("Joint Planning ENABLED")
        print("=" * 60)
        return configuration_tmpc_joint_planning(settings)
    else:
        print("=" * 60)
        print("Joint Planning DISABLED")
        print("=" * 60)
        return configuration_tmpc_consistency_cost(settings)

# Main execution
settings = load_settings()
model, modules = select_configuration(settings)
generate_solver(modules, model, settings)
```

---

## 6. Configuration

### 6.1 Settings YAML

The `joint_planning` section in `settings.yaml`:

```yaml
joint_planning:
  enabled: false                          # Toggle joint planning on/off
  max_ec_robots: 2                        # Max robots to jointly optimize (1-3)
  
  # EC robot selection
  ec_robot_selection_radius: 10.0         # [m] Select EC robots within this radius
  
  # Cost weights (Equation 8 from IJP paper)
  deviation_weight: 5.0                   # J_dev weight
  ec_control_effort_weight: 1.0           # J_u weight for EC robots
  ego_selfishness: 0.8                    # η_e: 0=altruistic, 1=selfish
  
  # SQP iterations for iterative coupling
  sqp_iterations: 2
  
  # Constraint parameters
  use_coupled_collision: true             # Use joint collision constraints
  safety_margin: 0.1                      # [m] Additional buffer
  
  # EC robot dynamics bounds
  ec_max_velocity: 2.0                    # [m/s]
  ec_max_acceleration: 1.5                # [m/s²]
  ec_max_angular_velocity: 1.0            # [rad/s]
```

### 6.2 Enabling Joint Planning

1. **Edit settings.yaml**: Set `joint_planning.enabled: true`

2. **Regenerate solver**:
   ```bash
   cd /path/to/oscar_mpc_planner_mr_modification
   poetry run python mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
   ```

3. **Rebuild package**:
   ```bash
   catkin build mpc_planner_jackalsimulator
   ```

---

## 7. Integration Checklist

### Phase 1: Solver Generation (Python) ✅
- [x] `ContouringSecondOrderUnicycleModelWithEC` in `solver_model.py`
  - EC robots use SecondOrderUnicycleModel (4 states, no spline)
  - Ego robot uses ContouringSecondOrderUnicycleModel (5 states, with spline)
- [x] `joint_ec_objective.py` - pure `ObjectiveModule`
- [x] `joint_ec_constraints.py` - pure `ConstraintModule`
- [x] `generate_jackalsimulator_solver.py` - configuration function
- [x] `settings.yaml` - joint_planning section

### Phase 2: C++ Runtime (Future Work)
- [ ] EC robot selection in `guidance_constraints.cpp`
- [ ] SQP outer loop for iterative coupling
- [ ] EC trajectory extraction from solver output
- [ ] `setJointPlanningParametersForPlanner()` implementation
- [ ] `module_data.h` updates for joint planning results

### Phase 3: Testing
- [ ] Verify solver generates with joint planning enabled
- [ ] Test with single EC robot
- [ ] Test with multiple EC robots
- [ ] Benchmark solve times
- [ ] Compare trajectory quality

---

## 8. FAQ & Design Rationale

### Q1: Why do EC robots NOT have a spline state?

**Answer**: EC robots use `SecondOrderUnicycleModel` (4 states: x, y, psi, v) instead of `ContouringSecondOrderUnicycleModel` (5 states: x, y, psi, v, spline) because:

1. **No spline information**: We only receive geometric trajectory predictions for other robots via communication, not path/spline information
2. **Spline is for path tracking**: The spline state is specifically for MPCC path tracking, which only applies to the ego robot
3. **Simpler dynamics**: EC robots just need position/velocity tracking, not path progress tracking

### Q2: Why is the model in `solver_model.py`?

**Answer**: This follows the established codebase pattern:
- All dynamics models are in `solver_model.py`
- Models inherit from `DynamicsModel` base class
- Solver generator imports from this central location

### Q3: Why separate Constraint and Objective modules?

**Answer**: The codebase architecture requires single-type modules:
- `solver_definition.py` iterates modules by type
- Dual inheritance causes `self.type` conflicts
- Separate modules ensure correct parameter registration

### Q4: How are inactive EC robots handled?

**Answer**: Using the `active` flag pattern:
- `ec{i}_active = 1.0` for robots within selection radius
- `ec{i}_active = 0.0` for unused slots
- Inactive robots are positioned at (1000, 1000)
- Costs multiply by active flag (zero contribution when inactive)

### Q5: What about FORCES Pro solver?

**Answer**: This implementation is **acados-specific** as stated in the requirements. The same concepts apply to FORCES Pro, but the solver generation would use `generate_forces_solver.py` instead.

---

## Appendix: Equation Reference (IJP Paper)

### Joint Cost Function (Equation 8)

```
min_{u_e, u_o, x_e, x_o}  η_e * [J_ref(x_e, x_ref) + J_u(u_e)]
                        + η_o * [J_dev(x_o, x_pred) + J_u(u_o)]

subject to:
    x_e[0] = x0_e                              (Ego initial state)
    x_o[0] = x0_o                              (EC initial states)
    x_e[k+1] = f(x_e[k], u_e[k])               (Ego dynamics)
    x_o[k+1] = f(x_o[k], u_o[k])               (EC dynamics)
    ||x_e[k] - x_o[k]|| ≥ d_safe               (Coupled collision)
```

Where:
- `J_ref`: Reference tracking cost (ego path following)
- `J_dev`: Deviation cost (EC from prediction)
- `J_u`: Control effort cost (acceleration, angular velocity)
- `η_e`, `η_o`: Selfishness/altruism balance weights

---

*Document version: 3.0*
*Last updated: 2025*
*Compatible with: T-MPC++ codebase (acados solver)*
