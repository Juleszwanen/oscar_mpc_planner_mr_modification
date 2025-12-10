# Acados Solver Generator Documentation

This document explains the role of the `solver_generator` directory and the acados-related files within it. The solver generator is responsible for creating Model Predictive Control (MPC) solvers using the acados framework.

## Directory Overview

The `solver_generator` directory contains Python scripts and utilities that generate C++ solver code for MPC motion planning. The generated solvers can use either [Acados](https://docs.acados.org/) (open-source) or Forces Pro (licensed). This documentation focuses specifically on the acados-related functionality.

## Acados-Related Files

### 1. `generate_acados_solver.py`

**Purpose:** The core file responsible for generating an acados-based MPC solver.

**Key Functions:**

- **`parse_constraint_bounds(bounds)`**: Converts infinite bounds to large finite values since acados does not support infinity (`np.inf`). Uses `1e15` as the large value substitute.

- **`create_acados_model(settings, model, modules)`**: Creates an `AcadosModel` object by:
  - Setting up the model name and dynamics (explicit and implicit forms)
  - Defining parameters using `AcadosParameters`
  - Formulating constraints from the registered modules
  - Setting up stage costs and terminal costs from objective modules
  - Configuring state variables (`x`), control inputs (`u`), and their derivatives

- **`generate_acados_solver(modules, settings, model, skip_solver_generation)`**: The main entry point that:
  1. Creates and configures `AcadosParameters`
  2. Builds the acados model via `create_acados_model()`
  3. Creates an `AcadosOcp` (Optimal Control Problem) object
  4. Sets up:
     - Horizon length (`N`)
     - External cost type
     - Initial state constraints
     - State bounds (`lbx`, `ubx`)
     - Control input bounds (`lbu`, `ubu`)
     - Path constraints (`lh`, `uh`)
     - Optional slack variables for constraint softening
  5. Configures solver options:
     - **Integrator:** ERK (Explicit Runge-Kutta) with 4 stages and 3 integration steps per shooting interval
     - **NLP Solver:** Configurable via settings (SQP or other)
     - **Hessian approximation:** EXACT
     - **Regularization:** MIRROR method
     - **QP Solver:** PARTIAL_CONDENSING_HPIPM with warm start
  6. Generates the solver and returns both the `AcadosOcpSolver` and `AcadosSimSolver`

**Dependencies:**
- `acados_template` (AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver)
- `casadi` (for symbolic math)
- `numpy` (for numerical operations)

---

### 2. `generate_solver.py`

**Purpose:** The main orchestrator that dispatches solver generation to either acados or Forces Pro based on configuration settings.

**Key Function:**

- **`generate_solver(modules, model, settings=None)`**: 
  1. Loads settings from configuration files if not provided
  2. Validates that the solver type is either "acados" or "forces"
  3. Dispatches to `generate_acados_solver()` or `generate_forces_solver()` accordingly
  4. Saves parameter maps and model maps
  5. Generates auxiliary C++ code files via helper functions:
     - `generate_cpp_code()` - Solver interface code
     - `generate_parameter_cpp_code()` - Parameter handling code
     - `generate_module_header()` - Module initialization headers
     - `generate_module_definitions()` - Module definitions
     - `generate_module_cmake()` - CMake configuration for modules
     - `generate_rqtreconfigure()` - ROS dynamic reconfigure files
     - `generate_ros2_rqtreconfigure()` - ROS2 reconfigure headers
     - `generate_solver_cmake()` - Solver-specific CMake configuration

**Acados-Specific Behavior:**
When `settings["solver_settings"]["solver"] == "acados"`, the function imports and calls `generate_acados_solver` from `generate_acados_solver.py`.

---

### 3. `solver_definition.py`

**Purpose:** Defines the mathematical structure of the MPC problem including parameters, objectives, and constraints that are used by both acados and Forces Pro solvers.

**Key Functions:**

- **`define_parameters(modules, params, settings)`**: Iterates through all registered modules (objectives first, then constraints) and calls their `define_parameters()` method to register all required parameters with the parameter manager.

- **`objective(modules, z, p, model, settings, stage_idx)`**: Computes the total cost at a given stage by:
  1. Loading the current parameters and model state
  2. Summing up the cost contributions from all objective modules

- **`constraints(modules, z, p, model, settings, stage_idx)`**: Collects all inequality constraints from constraint modules for a given stage. Returns a list of constraint expressions.

- **`constraint_upper_bounds(modules)`**: Collects upper bounds for all constraints across all modules.

- **`constraint_lower_bounds(modules)`**: Collects lower bounds for all constraints across all modules.

- **`constraint_number(modules)`**: Returns the total number of constraints by summing `nh` from all constraint modules.

**Usage in Acados:**
These functions are called during acados model creation to set up:
- Cost expressions (`cost_expr_ext_cost`, `cost_expr_ext_cost_e`)
- Constraint expressions (`con_h_expr`)
- Constraint bounds (`lh`, `uh`)

---

### 4. `solver_model.py`

**Purpose:** Defines the vehicle dynamics models used in the MPC. Contains both common model functionality and acados-specific methods.

**Key Classes:**

#### `DynamicsModel` (Base Class)
The base class for all dynamics models with acados-specific methods:

- **`acados_symbolics()`**: Creates CasADi symbolic variables for states (`x`) and controls (`u`), then combines them into `z = [u, x]` (acados convention).

- **`get_acados_dynamics()`**: Returns the explicit and implicit forms of the continuous dynamics:
  - `f_expl`: Explicit dynamics `f(x, u)`
  - `f_impl`: Implicit dynamics `x_dot - f_expl = 0`

- **`get_x()`**: Returns the state portion of `z`

- **`get_acados_u()`**: Returns the control input portion of `z`

- **`get_acados_x_dot()`**: Returns the state derivative symbolic variable

#### Implemented Models (with acados support):

1. **`SecondOrderUnicycleModel`**: Basic unicycle with states `[x, y, psi, v]` and inputs `[a, w]` (acceleration, angular velocity).

2. **`ContouringSecondOrderUnicycleModel`**: Extends the unicycle with a `spline` state for path progress tracking. States: `[x, y, psi, v, spline]`.

3. **`ContouringSecondOrderUnicycleModelCurvatureAware`**: Adds curvature-aware dynamics but includes a **warning** that it's not fully supported in acados. This is because it requires custom discrete dynamics for the spline state (in `model_discrete_dynamics()`), whereas acados expects purely continuous dynamics for ERK integration. For curvature-aware functionality with acados, users should consider using `BicycleModel2ndOrderCurvatureAware` which provides similar functionality with appropriate workarounds, or implementing the curvature-aware logic in the cost function rather than the dynamics.

4. **`ContouringSecondOrderUnicycleModelWithSlack`**: Adds a `slack` state variable for constraint relaxation. States: `[x, y, psi, v, spline, slack]`.

5. **`BicycleModel2ndOrder`**: Bicycle model with steering dynamics. States: `[x, y, psi, v, delta, spline]`, Inputs: `[a, w, slack]`.

6. **`BicycleModel2ndOrderCurvatureAware`**: Curvature-aware bicycle model with custom discrete dynamics for the spline state.

**Acados Integration:**
- Models define `continuous_model(x, u)` which returns the state derivative
- Acados uses `get_acados_dynamics()` to get both explicit and implicit formulations
- ERK integration is applied using the explicit dynamics

---

### 5. `util/parameters.py`

**Purpose:** Manages MPC parameters with specialized support for acados.

**Key Classes:**

#### `Parameters` (Base Class)
- Manages a dictionary of parameters with their indices
- Supports parameter bundles for C++ code generation
- Handles ROS rqt_reconfigure integration

#### `AcadosParameters` (Acados-Specific)
Extends `Parameters` with acados-specific functionality:

- **`load_acados_parameters()`**: Creates CasADi symbolic variables (`cd.SX.sym`) for each registered parameter. This is required because acados uses symbolic expressions.

- **`get_acados_parameters()`**: Returns a vertically concatenated CasADi expression of all parameters, which is assigned to `acados_model.p`.

- **`get_acados_p()`**: Returns the list of parameter symbolic variables for use in constraint and cost expressions.

**Usage:**
```python
params = AcadosParameters()
define_parameters(modules, params, settings)
params.load_acados_parameters()  # Creates CasADi symbols
# Then params.get_acados_parameters() is used in model creation
```

---

### 6. `spline.py`

**Purpose:** Provides spline interpolation utilities for path tracking in MPC models.

**Key Classes:**

- **`SplineSegment`**: Represents a single cubic polynomial segment with coefficients `a, b, c, d` and methods:
  - `at(s)`: Evaluate spline value at parameter `s`
  - `deriv(s)`: First derivative
  - `deriv2(s)`: Second derivative

- **`Spline`**: Manages multiple spline segments with smooth transitions using sigmoid blending.

- **`Spline2D`**: Combines two `Spline` objects for 2D path representation with methods:
  - `at(s)`: Get (x, y) position
  - `deriv(s)`: Get (dx, dy) derivatives
  - `deriv_normalized(s)`: Get normalized tangent vector
  - `get_curvature(s)`: Compute path curvature

**Acados Usage:**
Used in curvature-aware models (e.g., `BicycleModel2ndOrderCurvatureAware`) to compute path-related quantities within the CasADi symbolic framework for the acados solver.

---

## Solver Generation Workflow

The typical workflow for generating an acados solver is:

1. **Define Settings**: Load from `config/settings.yaml` with `solver_settings/solver: acados`

2. **Create Modules**: Use `ModuleManager` to add objective and constraint modules

3. **Select Model**: Choose an appropriate dynamics model (e.g., `ContouringSecondOrderUnicycleModel`)

4. **Generate Solver**: Call `generate_solver(modules, model, settings)`

5. **Output**: The generated solver is placed in `mpc_planner_solver/acados/` directory

## Example Usage

```python
from solver_generator.generate_solver import generate_solver
from solver_generator.solver_model import ContouringSecondOrderUnicycleModel
from solver_generator.control_modules import ModuleManager

# Load settings (solver_settings/solver should be "acados")
settings = load_settings()

# Create module manager and add modules
modules = ModuleManager()
model = ContouringSecondOrderUnicycleModel()

# Add your objective and constraint modules
# modules.add_module(YourModule(settings))

# Generate the acados solver
solver, simulator = generate_solver(modules, model, settings)
```

## Configuration Options

In `settings.yaml`, the following options affect acados solver generation:

```yaml
solver_settings:
  solver: acados  # Select acados as the solver
  acados:
    solver_type: SQP_RTI  # NLP solver type (SQP_RTI, SQP, etc.)
```

## Generated Files

After running the solver generator with acados, the following files are created:

- `mpc_planner_solver/acados/Solver/` - Generated acados C code
- `mpc_planner_solver/config/parameter_map.yaml` - Parameter index mapping
- `mpc_planner_solver/config/model_map.yaml` - State/input variable mapping
- `mpc_planner_solver/config/solver_settings.yaml` - Runtime solver settings
- Various C++ header and source files for ROS integration

## Key Differences from Forces Pro

| Aspect | Acados | Forces Pro |
|--------|--------|------------|
| Dynamics | Continuous (ERK integrated) | Discrete (user-provided) |
| Cost type | External (symbolic) | Function callback |
| Parameter handling | CasADi symbolic | Direct array |
| Constraint bounds | Must be finite (1e15 max) | Supports infinity |
| License | Open-source | Commercial |

## References

- [Acados Documentation](https://docs.acados.org/)
- [CasADi Documentation](https://web.casadi.org/)
- [Main MPC Planner README](../../README.md)
