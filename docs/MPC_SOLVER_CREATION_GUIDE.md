# MPC Solver Creation Guide: From Zero to Hero

A Comprehensive Educational Guide for Creating Custom MPC Modules with Acados in ROS Noetic

---

## Table of Contents

1. [Introduction and Prerequisites](#1-introduction-and-prerequisites)
2. [Repository Architecture Overview](#2-repository-architecture-overview)
3. [The Solver Generation Pipeline (Deep Dive)](#3-the-solver-generation-pipeline-deep-dive)
4. [Integration into C++/ROS](#4-integration-into-cros)
5. [Tutorial: Creating a Stacked MPC Module](#5-tutorial-creating-a-stacked-mpc-module)
6. [Appendix: Quick Reference](#appendix-quick-reference)

---

## 1. Introduction and Prerequisites

### What This Guide Covers

This guide is designed to take you from having zero knowledge about acados-based MPC solver generation to being able to:

1. **Understand** how the existing MPC solver pipeline works
2. **Create** your own custom MPC modules (cost functions, constraints, or dynamic models)
3. **Integrate** these modules into a ROS Noetic-based motion planner

### What is Model Predictive Control (MPC)?

**Model Predictive Control (MPC)** is an optimization-based control strategy that:

1. **Predicts** the future behavior of a system using a dynamic model
2. **Optimizes** a sequence of control inputs over a finite horizon to minimize a cost function
3. **Applies** only the first control input, then repeats the process at the next timestep

In mathematical terms, MPC solves an **Optimal Control Problem (OCP)** at each timestep:

```
minimize:    Σ_{k=0}^{N-1} L(x_k, u_k)    (stage cost)
subject to:  x_{k+1} = f(x_k, u_k)        (dynamics)
             g(x_k, u_k) ≤ 0               (constraints)
             x_0 = x_current               (initial state)
```

Where:
- `x_k` = state vector at step k (e.g., position, velocity, orientation)
- `u_k` = control input at step k (e.g., acceleration, angular velocity)
- `N` = prediction horizon (number of steps)
- `L(·)` = stage cost function
- `f(·)` = dynamics model
- `g(·)` = constraint functions

### What is Acados?

**Acados** is an open-source software framework for solving Optimal Control Problems (OCPs). It:

1. Takes a **symbolic problem definition** (typically in Python using CasADi)
2. **Generates C code** that efficiently solves the OCP
3. Provides a high-performance solver that can run in real-time

**Key Acados Concepts:**
- **AcadosOcp**: Python object that defines the OCP formulation
- **AcadosModel**: Defines the system dynamics
- **AcadosOcpSolver**: The generated solver object

### Prerequisites

Before using this guide, you should:

1. **Have access to the `mpc_planner_ws` workspace** with the Docker environment set up
2. **Basic understanding of:**
   - Python programming
   - C++ programming
   - ROS 1 Noetic concepts (nodes, topics, messages)
   - Linear algebra and optimization basics

### Environment Setup Reference

The acados library is loaded via the higher-level `mpc_planner_ws` repository which creates a Docker environment. The `setup.sh` script:

1. Clones all required repositories
2. Installs acados from source with the specific commit
3. Sets up the Poetry virtual environment
4. Installs Python dependencies including `acados_template`

---

## 2. Repository Architecture Overview

### 2.1 High-Level Package Structure

```
oscar_mpc_planner_mr_modification/
├── solver_generator/              # Python code for OCP formulation and C code generation
├── mpc_planner_solver/            # C++ solver interface wrapper
├── mpc_planner_modules/           # Python (scripts/) + C++ modules
├── mpc_planner/                   # Core C++ planner logic
├── mpc_planner_types/             # Common data structures
├── mpc_planner_util/              # Utility functions
├── mpc_planner_msgs/              # ROS message definitions
├── mpc_planner_jackalsimulator/   # System-specific implementation (example)
├── mpc_planner_jackal/            # Another system implementation
├── mpc_planner_rosnavigation/     # ROS Navigation integration
└── docs/                          # Documentation
```

### 2.2 The Three-Layer Architecture

The codebase follows a **three-layer architecture** that bridges Python solver generation with C++ ROS integration:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: PYTHON PROBLEM DEFINITION (Offline)                                │
│ Location: solver_generator/ + mpc_planner_modules/scripts/                  │
│                                                                             │
│ Purpose: Define the mathematical optimization problem                       │
│ • Dynamic model (solver_model.py)                                          │
│ • Cost functions (goal_module.py, contouring.py, etc.)                     │
│ • Constraints (ellipsoid_constraints.py, etc.)                             │
│ • Parameters mapping                                                        │
│                                                                             │
│ Key Files:                                                                  │
│ • solver_generator/generate_solver.py        - Entry point                 │
│ • solver_generator/generate_acados_solver.py - Acados-specific generation  │
│ • solver_generator/solver_model.py           - Dynamic models              │
│ • solver_generator/solver_definition.py      - OCP assembly                │
│ • mpc_planner_modules/scripts/*.py           - Module definitions          │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↓
                        Code Generation (acados_template)
                                    ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: GENERATED C CODE + C++ INTERFACE                                   │
│ Location: mpc_planner_solver/acados/                                        │
│                                                                             │
│ Purpose: High-performance solver that runs in real-time                     │
│ • Generated C code from acados                                              │
│ • C++ wrapper (acados_solver_interface.cpp)                                │
│ • Parameter accessor functions (mpc_planner_parameters.cpp)                │
│                                                                             │
│ Key Files:                                                                  │
│ • mpc_planner_solver/acados/Solver/*.c       - Generated solver code       │
│ • mpc_planner_solver/src/acados_solver_interface.cpp - C++ wrapper         │
│ • mpc_planner_solver/include/.../solver_interface.h  - Interface header    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↓
                            C++ Module Layer
                                    ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3: C++ ROS INTEGRATION (Online)                                       │
│ Location: mpc_planner/ + mpc_planner_modules/src/ + mpc_planner_<system>/  │
│                                                                             │
│ Purpose: Real-time planning in ROS environment                              │
│ • Planner orchestration (planner.cpp)                                      │
│ • Module implementations (goal_module.cpp, etc.)                           │
│ • System-specific ROS nodes (ros1_jackalsimulator.cpp)                     │
│                                                                             │
│ Key Files:                                                                  │
│ • mpc_planner/src/planner.cpp                - Core planning loop          │
│ • mpc_planner_modules/src/*.cpp              - Module implementations      │
│ • mpc_planner_jackalsimulator/src/*.cpp      - ROS node                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.3 Key File Connections

Understanding how files connect is crucial. Here's the connection map:

```
Python Definition Files          Generated Files           C++ Runtime Files
─────────────────────────────    ────────────────────      ──────────────────────
                                                           
mpc_planner_modules/             mpc_planner_solver/       mpc_planner_modules/
scripts/goal_module.py    →→→    config/parameter_map.yaml →→→ src/goal_module.cpp
                                                                    ↑
                                 mpc_planner_modules/            Uses
                                 include/.../modules.h    →→→ generated headers
                                                           
solver_generator/                mpc_planner_solver/       
generate_acados_solver.py →→→    acados/Solver/*.c   →→→  src/acados_solver_interface.cpp
                                                           
                                 mpc_planner_solver/       
                                 src/mpc_planner_parameters.cpp ←←← Parameter setters
```

### 2.4 Module Duality: Python + C++

Each MPC module has **two implementations**:

| Aspect | Python (scripts/) | C++ (src/) |
|--------|-------------------|------------|
| **When** | Solver generation (offline) | Runtime execution (online) |
| **Purpose** | Define cost/constraint math | Set parameter values at runtime |
| **Location** | `mpc_planner_modules/scripts/` | `mpc_planner_modules/src/` |
| **Example** | `goal_module.py` | `goal_module.cpp` |

**Why two implementations?**
- **Python**: Uses CasADi symbolic math to define *what* the optimization problem looks like
- **C++**: Runs in the ROS control loop to set *parameter values* for each optimization call

---

## 3. The Solver Generation Pipeline (Deep Dive)

### 3.1 Conceptual Flow Overview

```
┌──────────────────────┐
│ 1. Settings YAML     │  config/settings.yaml
│    (N, dt, weights)  │
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 2. Python Script     │  scripts/generate_<system>_solver.py
│    (Module config)   │
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 3. Module Manager    │  solver_generator/control_modules.py
│    (Collect modules) │
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 4. Solver Definition │  solver_generator/solver_definition.py
│    (Build OCP)       │
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 5. Acados Generator  │  solver_generator/generate_acados_solver.py
│    (Create AcadosOcp)│
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 6. acados_template   │  External library
│    (C code export)   │
└─────────┬────────────┘
          ↓
┌──────────────────────┐
│ 7. Generated C Code  │  mpc_planner_solver/acados/Solver/
│    + C++ Integration │
└──────────────────────┘
```

### 3.2 Step-by-Step: Solver Generation Script

Let's walk through `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py`:

```python
#!/usr/bin/python3

import os
import sys

# 1. Add paths to find modules
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# 2. Import core utilities
from util.files import load_settings
from control_modules import ModuleManager
from generate_solver import generate_solver

# 3. Import module definitions
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from ellipsoid_constraints import EllipsoidConstraintModule
from guidance_constraints import GuidanceConstraintModule

# 4. Import dynamic models
from solver_model import ContouringSecondOrderUnicycleModel

# 5. Define a configuration function
def configuration_tmpc(settings):
    """Configure T-MPC: Topology-driven Model Predictive Control"""
    
    # Create module manager to collect all modules
    modules = ModuleManager()
    
    # Choose dynamic model
    model = ContouringSecondOrderUnicycleModel()
    
    # Add base module for input penalization
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    
    # Add contouring module for path following
    modules.add_module(ContouringModule(settings))
    
    # Add T-MPC guidance with ellipsoid obstacle avoidance
    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule
    ))
    
    return model, modules

# 6. Load settings from YAML
settings = load_settings()

# 7. Build the configuration
model, modules = configuration_tmpc(settings)

# 8. Generate the solver!
generate_solver(modules, model, settings)
```

**Key Observations:**
- **Line-by-line comments explain each step**
- Multiple configurations can be defined (see `configuration_basic`, `configuration_lmpcc`, etc.)
- The `ModuleManager` collects all modules
- `generate_solver()` does the actual code generation

### 3.3 Dynamic Model Definition

Located in `solver_generator/solver_model.py`:

```python
class ContouringSecondOrderUnicycleModel(DynamicsModel):
    """
    Second-order unicycle model for path-tracking applications.
    
    State vector x = [x, y, psi, v, spline]
        - x, y: Position in 2D
        - psi: Heading angle
        - v: Linear velocity
        - spline: Progress along the reference path (path parameter)
    
    Input vector u = [a, w]
        - a: Linear acceleration
        - w: Angular velocity (yaw rate)
    """

    def __init__(self):
        super().__init__()
        self.nu = 2  # Number of inputs
        self.nx = 5  # Number of states

        # Define state and input names (used in C++ to access values)
        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]

        # Define bounds: [u_min, u_max, x_min, x_max] flattened
        # Format: [a_min, w_min, x_min, y_min, psi_min, v_min, spline_min]
        self.lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0]
        self.upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0]

    def continuous_model(self, x, u):
        """
        Define the continuous-time dynamics: dx/dt = f(x, u)
        
        For a unicycle model:
            x_dot = v * cos(psi)
            y_dot = v * sin(psi)
            psi_dot = w
            v_dot = a
            spline_dot = v  (path progress increases with velocity)
        """
        a = u[0]    # Acceleration input
        w = u[1]    # Angular velocity input
        psi = x[2]  # Current heading
        v = x[3]    # Current velocity

        return np.array([
            v * cd.cos(psi),   # x_dot
            v * cd.sin(psi),   # y_dot
            w,                  # psi_dot
            a,                  # v_dot
            v                   # spline_dot (path progress)
        ])
```

**Why CasADi (`cd`)?**
- CasADi is a symbolic computation library
- Allows automatic differentiation for gradient computation
- Enables efficient code generation for the solver

### 3.4 Module Parameter Definition (Python Side)

Example from `mpc_planner_modules/scripts/goal_module.py`:

```python
class GoalObjective(Objective):
    """Objective to track a 2D goal position."""

    def define_parameters(self, params):
        """
        Define solver parameters needed by this objective.
        
        Parameters are values that change at runtime but are needed
        by the optimization (e.g., goal position, weights).
        """
        # Add a weight parameter (tunable via rqt_reconfigure)
        params.add("goal_weight", 
                   add_to_rqt_reconfigure=True, 
                   rqt_config_name=lambda p: f'["weights"]["goal"]')
        
        # Add goal position parameters (set from ROS topics)
        params.add("goal_x")
        params.add("goal_y")

    def get_value(self, model, params, settings, stage_idx):
        """
        Compute the cost value using symbolic CasADi expressions.
        
        This is called during solver generation to build the cost function.
        """
        cost = 0

        # Get robot position from model state
        pos_x = model.get("x")
        pos_y = model.get("y")

        # Get parameters
        goal_weight = params.get("goal_weight")
        goal_x = params.get("goal_x")
        goal_y = params.get("goal_y")

        # Compute weighted distance to goal (normalized)
        cost += goal_weight * ((pos_x - goal_x) ** 2 + (pos_y - goal_y) ** 2) \
                / (goal_x**2 + goal_y**2 + 0.01)

        return cost
```

**Key Concepts:**
- `define_parameters()`: Declares *what* parameters exist
- `get_value()`: Uses symbolic math to define the cost
- Parameters are indexed by name and accessed via generated C++ functions

### 3.5 Constraint Definition (Python Side)

Example from `mpc_planner_modules/scripts/ellipsoid_constraints.py`:

```python
class EllipsoidConstraint:
    """Collision avoidance constraint for ellipsoidal obstacles."""

    def __init__(self, n_discs, max_obstacles):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs
        # Total number of constraints: one per disc-obstacle pair
        self.nh = max_obstacles * n_discs

    def define_parameters(self, params):
        """Define parameters for obstacle positions, sizes, etc."""
        params.add("ego_disc_radius")
        
        for disc_id in range(self.n_discs):
            params.add(f"ego_disc_{disc_id}_offset", bundle_name="ego_disc_offset")
        
        for obs_id in range(self.max_obstacles):
            params.add(f"ellipsoid_obst_{obs_id}_x", bundle_name="ellipsoid_obst_x")
            params.add(f"ellipsoid_obst_{obs_id}_y", bundle_name="ellipsoid_obst_y")
            # ... more parameters for each obstacle

    def get_lower_bound(self):
        """Lower bound for constraint: h(x) >= 1 means safe"""
        return [1.0] * (self.max_obstacles * self.n_discs)

    def get_upper_bound(self):
        """Upper bound: no upper limit (inf)"""
        return [np.inf] * (self.max_obstacles * self.n_discs)

    def get_constraints(self, model, params, settings, stage_idx):
        """
        Build symbolic constraint expressions.
        
        Constraint form: dist_normalized >= 1 (outside ellipsoid)
        """
        constraints = []
        pos_x = model.get("x")
        pos_y = model.get("y")
        # ... build constraint expressions
        return constraints
```

### 3.6 The Core Generation Function

In `solver_generator/generate_solver.py`:

```python
def generate_solver(modules, model, settings=None):
    """
    Main entry point for solver generation.
    
    This function:
    1. Creates the acados OCP
    2. Generates C code
    3. Generates C++ wrapper files
    4. Creates CMake configuration
    """
    if settings is None:
        settings = load_settings()

    # Choose solver backend
    if settings["solver_settings"]["solver"] == "acados":
        from generate_acados_solver import generate_acados_solver
        solver, simulator = generate_acados_solver(modules, settings, model)
    
    # Save parameter and model maps
    settings["params"].save_map()
    model.save_map()
    
    # Generate C++ files
    generate_cpp_code(settings, model)           # Warmstart helpers
    generate_parameter_cpp_code(settings, model) # Parameter setters
    generate_module_header(modules)              # Module includes
    generate_module_definitions(modules)         # Preprocessor defines
    generate_module_cmake(modules)               # CMake config
    generate_rqtreconfigure(settings)            # Dynamic reconfigure
    generate_solver_cmake(settings)              # Solver CMake
```

### 3.7 Acados OCP Creation

In `solver_generator/generate_acados_solver.py`:

```python
def generate_acados_solver(modules, settings, model, skip_solver_generation):
    """
    Create and configure the Acados OCP, then generate the solver.
    """
    # 1. Create parameter manager
    params = AcadosParameters()
    define_parameters(modules, params, settings)
    settings["params"] = params
    
    # 2. Create Acados model
    model_acados = create_acados_model(settings, model, modules)
    
    # 3. Create Acados OCP
    ocp = AcadosOcp()
    ocp.model = model_acados
    ocp.dims.N = settings["N"]  # Horizon length
    
    # 4. Configure costs
    ocp.cost.cost_type = "EXTERNAL"  # Using custom cost expressions
    
    # 5. Configure constraints
    ocp.constraints.lh = constraint_lower_bounds(modules)
    ocp.constraints.uh = constraint_upper_bounds(modules)
    
    # 6. Configure solver options
    ocp.solver_options.tf = settings["N"] * settings["integrator_step"]
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Real-time iteration
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    
    # 7. Generate solver
    solver = AcadosOcpSolver(acados_ocp=ocp, json_file=json_file_name)
    
    return solver, simulator
```

---

## 4. Integration into C++/ROS

### 4.1 How Generated Code is Linked

The CMake configuration in `mpc_planner_solver/CMakeLists.txt` and `solver.cmake` links the generated code:

```cmake
# solver.cmake (generated)
add_definitions(-DACADOS_SOLVER)

# Find acados library
find_library(acados_LIBRARY libacados.so PATHS ${LIBRARY_DIRS})

# Set solver libraries and includes
set(solver_LIBRARIES
    ${PROJECT_SOURCE_DIR}/acados/Solver/libacados_ocp_solver_Solver.so
    ${acados_LIBRARY}
    ${blasfeo_LIBRARY}
    ${hpipm_LIBRARY}
)
set(solver_INCLUDE_DIRS
    acados/Solver
    ${acados_include_path}
)
```

### 4.2 C++ Solver Interface

The `mpc_planner_solver/src/acados_solver_interface.cpp` wraps the generated solver:

```cpp
// Key operations provided by the solver interface:

class Solver {
public:
    // Set initial state
    void setXinit(const State& state);
    
    // Set a parameter value for step k
    void setParameter(int k, const std::string& name, double value);
    
    // Solve the optimization problem
    int solve();
    
    // Get output value
    double getOutput(int k, const std::string& var_name);
    
    // Warmstart from previous solution
    void initializeWarmstart(const State& state, bool shift_forward);
};
```

### 4.3 Generated Parameter Setters

The generator creates type-safe parameter setter functions in `mpc_planner_solver/src/mpc_planner_parameters.cpp`:

```cpp
// Auto-generated functions for setting parameters
void setSolverParameterGoalX(int k, AcadosParameters& params, const double value, int index) {
    params.all_parameters[k * NPAR + GOAL_X_INDEX] = value;
}

void setSolverParameterGoalY(int k, AcadosParameters& params, const double value, int index) {
    params.all_parameters[k * NPAR + GOAL_Y_INDEX] = value;
}

void setSolverParameterGoalWeight(int k, AcadosParameters& params, const double value, int index) {
    params.all_parameters[k * NPAR + GOAL_WEIGHT_INDEX] = value;
}
```

### 4.4 C++ Module Implementation

Each Python module has a corresponding C++ implementation. Example from `mpc_planner_modules/src/goal_module.cpp`:

```cpp
#include <mpc_planner_modules/goal_module.h>
#include <mpc_planner_solver/mpc_planner_parameters.h>

namespace MPCPlanner {

GoalModule::GoalModule(std::shared_ptr<Solver> solver)
    : ControllerModule(ModuleType::OBJECTIVE, solver, "goal_module")
{
    LOG_INITIALIZE("Goal Tracking");
    LOG_INITIALIZED();
}

void GoalModule::setParameters(const RealTimeData &data, 
                               const ModuleData &module_data, int k)
{
    // Set goal position from ROS data
    setSolverParameterGoalX(k, _solver->_params, data.goal(0));
    setSolverParameterGoalY(k, _solver->_params, data.goal(1));
    
    // Set weight from configuration
    setSolverParameterGoalWeight(k, _solver->_params, 
                                  CONFIG["weights"]["goal"].as<double>());
}

bool GoalModule::isDataReady(const RealTimeData &data, std::string &missing_data)
{
    if (!data.goal_received) {
        missing_data += "Goal ";
        return false;
    }
    return true;
}

} // namespace MPCPlanner
```

### 4.5 The Control Loop Flow

In the ROS node (e.g., `ros1_jackalsimulator.cpp`):

```cpp
void JackalPlanner::loop(const ros::TimerEvent &event) {
    // 1. Record planning start time
    _data.planning_start_time = std::chrono::system_clock::now();
    
    // 2. Call the main MPC function
    auto output = _planner->solveMPC(_state, _data);
    
    // 3. Apply control or brake
    geometry_msgs::Twist cmd;
    if (output.success) {
        cmd.linear.x = _planner->getSolution(1, "v");   // Velocity at k=1
        cmd.angular.z = _planner->getSolution(0, "w");  // Angular vel at k=0
    } else {
        // Emergency braking
        cmd.linear.x = std::max(_state.get("v") - DECELERATION * dt, 0.0);
        cmd.angular.z = 0.0;
    }
    _cmd_pub.publish(cmd);
    
    // 4. Visualize
    _planner->visualize(_state, _data);
}
```

### 4.6 Data Flow from ROS to Solver

```
ROS Topics                    ROS Callbacks               RealTimeData           Solver Parameters
─────────────                 ──────────────              ────────────           ─────────────────
/input/state         →→→      stateCallback()      →→→    State                  setXinit()
/input/goal          →→→      goalCallback()       →→→    data.goal              setSolverParameterGoalX/Y()
/input/obstacles     →→→      obstacleCallback()   →→→    data.dynamic_obstacles setSolverParameterEllipsoidObst*()
/input/reference_path →→→     pathCallback()       →→→    data.reference_path    setSolverParameterSpline*()
```

---

## 5. Tutorial: Creating a Stacked MPC Module

This tutorial walks you through creating a new MPC module that can be "stacked" alongside existing modules. We'll create a **"Velocity Zone"** module that enforces different velocity limits in different regions.

### 5.1 Module Design

**Goal**: Create a constraint module that limits robot velocity based on its position (e.g., slow down near obstacles or in certain areas).

**Mathematical formulation**:
```
Constraint: v ≤ v_max(x, y)

Where v_max depends on position:
- If in "slow zone": v_max = v_slow
- Otherwise: v_max = v_normal
```

### 5.2 Step 1: Create Python Module Definition

Create `mpc_planner_modules/scripts/velocity_zone.py`:

```python
"""
Velocity Zone Module
====================
A constraint module that enforces position-dependent velocity limits.

This module demonstrates how to:
1. Define custom parameters
2. Create inequality constraints
3. Integrate with the module system
"""

import sys
import os

# Add path to find solver_generator modules
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ConstraintModule
import casadi as cd
import numpy as np


class VelocityZoneConstraint:
    """
    Defines the velocity zone constraint.
    
    The constraint is: v - v_max_zone <= 0
    which means velocity must be less than or equal to the zone limit.
    """
    
    def __init__(self, settings):
        # Number of constraint equations (one velocity limit)
        self.nh = 1
        
    def define_parameters(self, params):
        """
        Define parameters that will be set at runtime.
        
        We need:
        - zone_center_x, zone_center_y: Center of the slow zone
        - zone_radius: Size of the slow zone
        - v_slow: Velocity limit in the slow zone
        - v_normal: Normal velocity limit
        """
        params.add("zone_center_x")
        params.add("zone_center_y")
        params.add("zone_radius")
        params.add("v_slow", add_to_rqt_reconfigure=True,
                   rqt_config_name=lambda p: '["velocity_zone"]["v_slow"]')
        params.add("v_normal", add_to_rqt_reconfigure=True,
                   rqt_config_name=lambda p: '["velocity_zone"]["v_normal"]')
    
    def get_lower_bound(self):
        """
        Lower bound: -infinity (no lower limit on v - v_max)
        """
        return [-np.inf]
    
    def get_upper_bound(self):
        """
        Upper bound: v - v_max <= 0
        """
        return [0.0]
    
    def get_constraints(self, model, params, settings, stage_idx):
        """
        Build the constraint expression.
        
        We use a smooth transition between zones using a sigmoid-like function.
        """
        # Get robot position and velocity
        pos_x = model.get("x")
        pos_y = model.get("y")
        vel = model.get("v")
        
        # Get zone parameters
        zone_x = params.get("zone_center_x")
        zone_y = params.get("zone_center_y")
        zone_r = params.get("zone_radius")
        v_slow = params.get("v_slow")
        v_normal = params.get("v_normal")
        
        # Compute distance to zone center
        dist_to_zone = cd.sqrt((pos_x - zone_x)**2 + (pos_y - zone_y)**2)
        
        # Smooth transition: 0 inside zone, 1 outside
        # Using tanh for smooth transition
        transition = 0.5 * (1 + cd.tanh(5 * (dist_to_zone - zone_r)))
        
        # Interpolate velocity limit
        v_max = v_slow * (1 - transition) + v_normal * transition
        
        # Constraint: v - v_max <= 0
        constraints = [vel - v_max]
        
        return constraints


class VelocityZoneModule(ConstraintModule):
    """
    Module wrapper for the velocity zone constraint.
    
    This class connects the constraint definition to the module system.
    """
    
    def __init__(self, settings):
        super().__init__()
        
        # Module identification (must match C++ class name)
        self.module_name = "VelocityZoneModule"
        
        # Import name (C++ header file)
        self.import_name = "velocity_zone.h"
        
        # Description for documentation
        self.description = "Position-dependent velocity limits for safety zones"
        
        # Optional: Add dependencies on other ROS packages
        # self.dependencies.append("some_other_package")
        
        # Add the constraint
        self.constraints.append(VelocityZoneConstraint(settings))
```

### 5.3 Step 2: Create C++ Header

Create `mpc_planner_modules/include/mpc_planner_modules/velocity_zone.h`:

```cpp
#ifndef __MPC_VELOCITY_ZONE_H__
#define __MPC_VELOCITY_ZONE_H__

#include <mpc_planner_modules/controller_module.h>

namespace MPCPlanner
{
    /**
     * @brief Velocity Zone Module
     * 
     * Enforces position-dependent velocity limits.
     * Robots slow down when entering designated "slow zones".
     */
    class VelocityZoneModule : public ControllerModule
    {
    public:
        VelocityZoneModule(std::shared_ptr<Solver> solver);

        /**
         * @brief Update module state before optimization
         * @param state Current robot state
         * @param data Real-time data from ROS
         * @param module_data Shared data between modules
         */
        void update(State &state, const RealTimeData &data, 
                    ModuleData &module_data) override;

        /**
         * @brief Set solver parameters for each horizon step
         * @param data Real-time data
         * @param module_data Module shared data
         * @param k Horizon step index (0 to N-1)
         */
        void setParameters(const RealTimeData &data, 
                          const ModuleData &module_data, int k) override;

        /**
         * @brief Visualize the velocity zone in RViz
         */
        void visualize(const RealTimeData &data, 
                      const ModuleData &module_data) override;

        /**
         * @brief Check if required data is available
         */
        bool isDataReady(const RealTimeData &data, 
                        std::string &missing_data) override;

    private:
        // Zone configuration
        double _zone_center_x{0.0};
        double _zone_center_y{0.0};
        double _zone_radius{2.0};
        double _v_slow{0.5};
        double _v_normal{2.0};
    };
}

#endif // __MPC_VELOCITY_ZONE_H__
```

### 5.4 Step 3: Create C++ Implementation

Create `mpc_planner_modules/src/velocity_zone.cpp`:

```cpp
#include <mpc_planner_modules/velocity_zone.h>

#include <mpc_planner_solver/mpc_planner_parameters.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>

namespace MPCPlanner
{

VelocityZoneModule::VelocityZoneModule(std::shared_ptr<Solver> solver)
    : ControllerModule(ModuleType::CONSTRAINT, solver, "velocity_zone")
{
    LOG_INITIALIZE("Velocity Zone");
    
    // Load configuration from settings.yaml
    _zone_center_x = CONFIG["velocity_zone"]["center_x"].as<double>();
    _zone_center_y = CONFIG["velocity_zone"]["center_y"].as<double>();
    _zone_radius = CONFIG["velocity_zone"]["radius"].as<double>();
    _v_slow = CONFIG["velocity_zone"]["v_slow"].as<double>();
    _v_normal = CONFIG["velocity_zone"]["v_normal"].as<double>();
    
    LOG_VALUE("Zone Center", "(" << _zone_center_x << ", " << _zone_center_y << ")");
    LOG_VALUE("Zone Radius", _zone_radius);
    LOG_VALUE("V Slow", _v_slow);
    LOG_VALUE("V Normal", _v_normal);
    
    LOG_INITIALIZED();
}

void VelocityZoneModule::update(State &state, const RealTimeData &data, 
                                 ModuleData &module_data)
{
    // Optional: Update zone parameters dynamically
    // For example, move the zone based on detected areas
    (void)state;
    (void)data;
    (void)module_data;
}

void VelocityZoneModule::setParameters(const RealTimeData &data, 
                                        const ModuleData &module_data, int k)
{
    (void)data;
    (void)module_data;
    
    // Set zone parameters for each horizon step k
    // These functions are auto-generated from the Python parameter definitions
    setSolverParameterZoneCenterX(k, _solver->_params, _zone_center_x);
    setSolverParameterZoneCenterY(k, _solver->_params, _zone_center_y);
    setSolverParameterZoneRadius(k, _solver->_params, _zone_radius);
    setSolverParameterVSlow(k, _solver->_params, 
                            CONFIG["velocity_zone"]["v_slow"].as<double>());
    setSolverParameterVNormal(k, _solver->_params, 
                              CONFIG["velocity_zone"]["v_normal"].as<double>());
}

void VelocityZoneModule::visualize(const RealTimeData &data, 
                                    const ModuleData &module_data)
{
    (void)data;
    (void)module_data;
    
    // Visualize the slow zone as a circle in RViz
    auto &publisher = VISUALS.getPublisher(_name + "/zone");
    auto &circle = publisher.getNewLine();
    
    circle.setColor(1.0, 0.5, 0.0, 0.5);  // Orange, semi-transparent
    circle.setScale(0.1);
    
    // Draw circle
    const int num_points = 36;
    Eigen::Vector2d prev_point;
    for (int i = 0; i <= num_points; i++)
    {
        double angle = 2.0 * M_PI * i / num_points;
        Eigen::Vector2d point(
            _zone_center_x + _zone_radius * std::cos(angle),
            _zone_center_y + _zone_radius * std::sin(angle)
        );
        
        if (i > 0)
            circle.addLine(prev_point, point);
        
        prev_point = point;
    }
    
    publisher.publish();
}

bool VelocityZoneModule::isDataReady(const RealTimeData &data, 
                                      std::string &missing_data)
{
    (void)data;
    (void)missing_data;
    // No external data required - always ready
    return true;
}

} // namespace MPCPlanner
```

### 5.5 Step 4: Add Configuration to settings.yaml

Add to `mpc_planner_jackalsimulator/config/settings.yaml`:

```yaml
# Velocity Zone Configuration
velocity_zone:
  center_x: 10.0    # X coordinate of slow zone center
  center_y: 5.0     # Y coordinate of slow zone center
  radius: 3.0       # Radius of slow zone
  v_slow: 0.5       # Velocity limit inside zone (m/s)
  v_normal: 2.0     # Normal velocity limit (m/s)
```

### 5.6 Step 5: Register Module in Solver Generation

Modify `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py`:

```python
# Add import at the top
from velocity_zone import VelocityZoneModule

# Create a new configuration or modify existing one
def configuration_with_velocity_zone(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()
    
    # Base module
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    
    # Path following
    modules.add_module(ContouringModule(settings))
    
    # Obstacle avoidance (T-MPC)
    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule
    ))
    
    # ADD YOUR NEW MODULE HERE
    modules.add_module(VelocityZoneModule(settings))
    
    return model, modules

# Use the new configuration
model, modules = configuration_with_velocity_zone(settings)
generate_solver(modules, model, settings)
```

### 5.7 Step 6: Update CMakeLists.txt

The module CMake is auto-generated by `generate_module_cmake()`, but if you have special dependencies, you might need to modify `mpc_planner_modules/CMakeLists.txt`:

```cmake
# The source file will be automatically added if the module is used
# But if you have special includes or libraries:

# find_package(my_special_package REQUIRED)  # If needed

# The module source (velocity_zone.cpp) is added automatically by:
# generate_module_cmake() in generate_cpp_files.py
```

### 5.8 Step 7: Generate and Build

```bash
# 1. Generate the solver (this creates all the C++ interface code)
cd /workspace
poetry run python src/mpc_planner/mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py

# 2. Build the workspace
catkin build mpc_planner_jackalsimulator
```

### 5.9 Step 8: Test the Module

```bash
# 1. Launch the simulator
roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch

# 2. In RViz, you should see:
#    - The orange velocity zone circle
#    - Robot slowing down when entering the zone

# 3. Use rqt_reconfigure to tune parameters online:
rosrun rqt_reconfigure rqt_reconfigure
#    - Look for v_slow and v_normal parameters
```

### 5.10 Files You Created/Modified (Summary)

| File | Action | Purpose |
|------|--------|---------|
| `mpc_planner_modules/scripts/velocity_zone.py` | **Created** | Python module definition |
| `mpc_planner_modules/include/.../velocity_zone.h` | **Created** | C++ header |
| `mpc_planner_modules/src/velocity_zone.cpp` | **Created** | C++ implementation |
| `mpc_planner_jackalsimulator/config/settings.yaml` | **Modified** | Add configuration |
| `mpc_planner_jackalsimulator/scripts/generate_*.py` | **Modified** | Register module |

### 5.11 Common Pitfalls and Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| `setSolverParameter* not found` | Parameter function not generated | Re-run solver generation; check parameter name matches |
| Module not initialized | Module not added in Python script | Add `modules.add_module(...)` |
| Constraint always violated | Bounds incorrect | Check `get_lower_bound()` and `get_upper_bound()` |
| No visualization | Publisher not configured | Check VISUALS setup in C++ |
| Config not loaded | YAML key missing | Add to settings.yaml with correct nesting |

---

## Appendix: Quick Reference

### A.1 File Location Quick Reference

| What | Where |
|------|-------|
| Python module definitions | `mpc_planner_modules/scripts/` |
| C++ module headers | `mpc_planner_modules/include/mpc_planner_modules/` |
| C++ module implementations | `mpc_planner_modules/src/` |
| Dynamic models | `solver_generator/solver_model.py` |
| Solver generation script | `mpc_planner_<system>/scripts/generate_<system>_solver.py` |
| System settings | `mpc_planner_<system>/config/settings.yaml` |
| Generated solver code | `mpc_planner_solver/acados/Solver/` |
| Generated parameter map | `mpc_planner_solver/config/parameter_map.yaml` |

### A.2 Parameter Naming Convention

When you define a parameter in Python:
```python
params.add("my_param_name")
```

The generated C++ setter function is:
```cpp
setSolverParameterMyParamName(k, params, value);
```

**Naming transformation:**
- Underscores become word boundaries
- Each word is capitalized (PascalCase)
- Prefix: `setSolverParameter`

### A.3 Module Type Reference

| Type | Python Base Class | C++ ModuleType | Purpose |
|------|-------------------|----------------|---------|
| Objective | `ObjectiveModule` | `ModuleType::OBJECTIVE` | Add cost terms |
| Constraint | `ConstraintModule` | `ModuleType::CONSTRAINT` | Add inequality constraints |

### A.4 Key C++ Methods to Override

```cpp
class MyModule : public ControllerModule {
    // REQUIRED: Set parameters for each horizon step
    void setParameters(const RealTimeData &data, 
                      const ModuleData &module_data, int k) override;
    
    // OPTIONAL: Pre-computation before optimization
    void update(State &state, const RealTimeData &data, 
               ModuleData &module_data) override;
    
    // OPTIONAL: RViz visualization
    void visualize(const RealTimeData &data, 
                  const ModuleData &module_data) override;
    
    // OPTIONAL: Check data readiness
    bool isDataReady(const RealTimeData &data, 
                    std::string &missing_data) override;
    
    // OPTIONAL: React to new data
    void onDataReceived(RealTimeData &data, std::string &&data_name) override;
    
    // OPTIONAL: Custom optimization (e.g., T-MPC)
    int optimize(State &state, const RealTimeData &data, 
                ModuleData &module_data) override;
};
```

### A.5 Solver Generation Command

```bash
# Basic generation
poetry run python mpc_planner_<system>/scripts/generate_<system>_solver.py

# Skip actual solver generation (only regenerate C++ files)
poetry run python mpc_planner_<system>/scripts/generate_<system>_solver.py false
```

### A.6 Common CasADi Functions

```python
import casadi as cd

# Basic math
cd.sqrt(x)
cd.sin(x), cd.cos(x), cd.tan(x)
cd.exp(x), cd.log(x)
cd.fabs(x)  # Absolute value

# Smooth min/max (differentiable)
cd.fmin(a, b), cd.fmax(a, b)

# Conditional (smooth)
cd.if_else(condition, true_val, false_val)

# Hyperbolic (for smooth transitions)
cd.tanh(x), cd.sinh(x), cd.cosh(x)

# Power
cd.power(x, n)
```

---

## Conclusion

This guide has walked you through the complete process of understanding and extending the MPC planner:

1. **Architecture**: The three-layer design connecting Python definitions to generated C code to C++ ROS integration
2. **Solver Generation**: How acados converts symbolic math to efficient solvers
3. **C++ Integration**: How the generated code is wrapped and used in ROS
4. **Module Creation**: Step-by-step tutorial for creating a new stacked module

**Key Takeaways:**
- Every module has **two implementations**: Python (symbolic math) and C++ (runtime parameters)
- The **naming convention** maps Python parameters to C++ setter functions
- Modules can be **stacked** by adding them to the ModuleManager
- Always **regenerate the solver** after changing Python definitions
- Use **rqt_reconfigure** for online parameter tuning

For more advanced topics, see:
- `docs/guidance_constraints_documentation.md` - T-MPC deep dive
- `docs/mpc_pipeline_documentation.md` - Detailed pipeline documentation
- The source code comments throughout the repository

Happy MPC development! 🚀
