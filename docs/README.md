# MPC Planner Documentation

This directory contains comprehensive documentation for the MPC (Model Predictive Control) planner framework for mobile robot navigation in dynamic environments.

## Documentation Structure

### Core Architecture

#### 📘 [MPC Pipeline Documentation](mpc_pipeline_documentation.md)
**Comprehensive guide to the MPC planner architecture and pipeline**

Complete documentation covering:
- **Overview** - System design principles and phases
- **Architecture** - Layer-by-layer breakdown (ROS interface → Planner core → Modules → Solver)
- **Pipeline Phases** - Offline solver generation and online real-time planning
- **Module System** - How to create and use modular cost/constraint components
- **Data Flow** - Input/output data structures and flow through the system
- **Key Components** - Planner core, solver interface, module manager, dynamic models
- **Execution Flow** - Complete step-by-step execution diagram
- **Solver Generation Process** - How Python code becomes C++ solver
- **Runtime Optimization Loop** - What happens at each control cycle
- **Customization Guide** - How to add custom modules and systems
- **Advanced Topics** - T-MPC, SH-MPC, static obstacle avoidance
- **Debugging and Troubleshooting** - Common issues and solutions

**Audience:** All developers working with the MPC planner  
**Length:** ~960 lines  
**Status:** Comprehensive and up-to-date

---

### T-MPC++ (Topology-Driven MPC)

#### 📗 [Guidance Constraints Documentation](guidance_constraints_documentation.md)
**In-depth guide to the T-MPC++ implementation**

Complete documentation of the GuidanceConstraints module:
- **Executive Summary** - What T-MPC++ does and why it's innovative
- **Overview** - Role in the planner pipeline
- **Key Concepts** - Homotopy/topology classes, T-MPC vs T-MPC++
- **Integration with Planner::solveMPC()** - Where it fits in the pipeline
- **Core Data Structures** - SolverResult, LocalPlanner
- **Important Functions** - Constructor, update(), optimize(), initializeSolverWithGuidance()
- **Parallel Computation Architecture** - OpenMP parallelization details
- **Module Integration and Data Flow** - How it interacts with other modules
- **Configuration Parameters** - Settings for T-MPC++
- **Performance Characteristics** - Timing and computational cost
- **Troubleshooting** - Common issues and solutions
- **Integration with jules_ros1_jackalplanner** - How robot controller uses it
- **Storing and Using Homology IDs** - Methods for accessing topology information

**Audience:** Developers implementing or debugging T-MPC++  
**Length:** ~2000 lines  
**Status:** Comprehensive implementation guide

#### 📊 [Guidance Constraints Diagrams](guidance_constraints_diagrams.md)
**Visual flowcharts and diagrams for guidance constraints**

Visual aids including:
- Main control flow diagram
- Parallel optimization flow
- State transition diagrams
- Data structure visualizations

**Audience:** Visual learners, presentation preparation  
**Status:** Companion to main guidance documentation

#### 📋 [Guidance Constraints Quick Reference](guidance_constraints_quick_reference.md)
**Quick lookup guide for guidance constraints**

Condensed reference including:
- Function signatures
- Key parameters
- Common patterns
- Quick troubleshooting

**Audience:** Experienced developers needing quick reference  
**Status:** Cheat sheet format

#### 📖 [README: Guidance Constraints](README_GUIDANCE_CONSTRAINTS.md)
**Index and navigation guide for guidance constraints documentation**

**Purpose:** Navigation hub for all guidance-related documentation  
**Contents:** Links to all guidance documents with descriptions

---

### New Features (October 2025)

#### ⭐ [Topology Metadata Feature](topology_metadata_feature.md) **NEW**
**Comprehensive guide to topology metadata in PlannerOutput**

Documentation of the topology metadata feature added in October 2025:
- **Overview** - What topology metadata is and why it's useful
- **Field Descriptions** - Detailed explanation of each metadata field:
  - `selected_topology_id` - Homology class identifier
  - `selected_planner_index` - Which parallel planner was chosen
  - `used_guidance` - Guided vs T-MPC++ non-guided
  - `trajectory_cost` - Objective function value
  - `solver_exit_code` - Solver status
  - `following_new_homology` - Topology switch detection
- **Integration with T-MPC++** - How metadata is populated
- **Configuration** - Enabling and configuring metadata
- **Usage Examples** - Logging, tracking switches, multi-robot coordination, cost-based decisions
- **Benefits** - Multi-robot coordination, learning, debugging
- **Topology Metadata in Module Data** - Internal data flow
- **ROS Message Integration** - Custom messages for topology communication
- **Limitations and Considerations** - When metadata is available
- **Testing and Validation** - How to test the feature
- **Future Enhancements** - Potential improvements

**Audience:** Developers using topology information for coordination or learning  
**Length:** ~800 lines  
**Status:** New feature documentation (October 2025)

#### 🌐 [Topology-Aware Communication](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems) **NEW**
**Implementation of bandwidth-optimized multi-robot communication**

As of October 2025, the planner includes **topology-aware communication** to reduce multi-robot network overhead:
- **60-80% bandwidth reduction** - Only publish when topology changes
- **Smart decision logic** - Communicates on switches, failures, and non-guided mode
- **Safety-first approach** - Always communicates when behavior is unpredictable
- **Configuration** - `communicate_on_topology_switch_only` setting
- **Full implementation details** - See [Guidance Constraints Documentation](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems)

**Audience:** Multi-robot system developers optimizing communication  
**Status:** Production-ready (October 2025)

---

## Quick Navigation

### I want to...

**Understand the overall MPC planner architecture:**
→ Start with [MPC Pipeline Documentation](mpc_pipeline_documentation.md)

**Learn how T-MPC++ works:**
→ Read [Guidance Constraints Documentation](guidance_constraints_documentation.md)  
→ Then check [Guidance Constraints Diagrams](guidance_constraints_diagrams.md) for visual aids

**Use topology information in my code:**
→ Read [Topology Metadata Feature](topology_metadata_feature.md)

**Quickly look up guidance functions:**
→ Use [Guidance Constraints Quick Reference](guidance_constraints_quick_reference.md)

**Navigate guidance documentation:**
→ See [README: Guidance Constraints](README_GUIDANCE_CONSTRAINTS.md)

**Customize the MPC problem:**
→ See "Customization Guide" in [MPC Pipeline Documentation](mpc_pipeline_documentation.md)

**Debug a planning issue:**
→ See "Debugging and Troubleshooting" in [MPC Pipeline Documentation](mpc_pipeline_documentation.md)  
→ Or "Troubleshooting" in [Guidance Constraints Documentation](guidance_constraints_documentation.md)

**Implement multi-robot coordination:**
→ See "Benefits" → "Multi-Robot Coordination" in [Topology Metadata Feature](topology_metadata_feature.md)  
→ Also check the JulesJackalPlanner documentation in `mpc_planner_jackalsimulator/docs/`

**Optimize multi-robot communication bandwidth:**
→ Read [Topology-Aware Communication section](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems) in Guidance Constraints Documentation  
→ See configuration in [Multi-Robot Configuration](../README.md#configuration) in main README

---

## System-Specific Documentation

In addition to this general documentation, each system package has its own documentation:

### JulesJackalPlanner (Multi-Robot Simulator)

Location: `../mpc_planner_jackalsimulator/docs/`

**Key Documents:**
- [JulesJackalPlanner Architecture](../mpc_planner_jackalsimulator/docs/JulesJackalPlanner_Architecture.md)
- [State Machine Implementation](../mpc_planner_jackalsimulator/docs/State_Machine_Implementation.md) **NEW**
- [Refactoring Strategy](../mpc_planner_jackalsimulator/docs/Refactoring_Strategy.md)
- [State Machine Diagram](../mpc_planner_jackalsimulator/docs/state_machine_diagram.md)

**What it covers:**
- Multi-robot coordination and synchronization
- State machine-based control flow (October 2025 refactoring)
- Robot-to-robot communication
- Obstacle tracking from other robots
- Experimental setup and reset coordination

---

## Key Concepts

### MPC Planner Framework

The MPC planner is a **modular**, **robot-agnostic** motion planning framework that:
- Uses Model Predictive Control for trajectory optimization
- Supports both Forces Pro and Acados solvers
- Operates in 2D dynamic environments
- Achieves 20-30 Hz real-time control
- Handles static and dynamic obstacle avoidance
- Supports ROS1 and ROS2

### T-MPC++ (Topology-Driven MPC)

T-MPC++ is an advanced MPC variant that:
- Evaluates multiple trajectory topologies in parallel (e.g., pass left vs right of obstacle)
- Uses guidance from fast graph search to initialize each topology
- Selects the optimal topology based on MPC cost
- Includes a non-guided "fallback" planner (T-MPC++)
- Leverages multi-core CPUs for parallel optimization

### Topology Metadata

Topology metadata exposes information about which trajectory topology was selected:
- Which homotopy class (topological path variant) was chosen
- Whether guidance constraints were used or T-MPC++ fallback
- Trajectory cost and solver status
- Enables multi-robot coordination and learning

### Module System

The module system allows stacking of functionality:
- **Objective modules**: Cost terms to minimize (e.g., path tracking, goal tracking)
- **Constraint modules**: Safety constraints (e.g., obstacle avoidance, corridor constraints)
- Modules are defined in Python for solver generation
- Modules are implemented in C++ for runtime execution
- Fully modular and extensible

---

## Development History

### October 2025 Updates

1. **State Machine Refactoring** (JulesJackalPlanner)
   - Replaced boolean flags with explicit finite state machine
   - 12 well-defined states with validated transitions
   - Improved robustness and debuggability
   - See: `mpc_planner_jackalsimulator/docs/State_Machine_Implementation.md`

2. **Topology Metadata Feature**
   - Added topology information to PlannerOutput
   - Enables topology-aware multi-robot coordination
   - Supports learning and debugging
   - See: `docs/topology_metadata_feature.md`

3. **Multi-Robot Utility Functions**
   - New helper functions for multi-robot coordination
   - Namespace parsing and robot identification
   - Data state logging utilities
   - Location: `mpc_planner_util/include/mpc_planner_util/multi_robot_utility_functions.h`

### Earlier Updates (2024)

4. **Guidance Constraints Documentation Suite**
   - Comprehensive documentation of T-MPC++ implementation
   - Visual diagrams and quick reference guides
   - Implementation guide for integration

5. **MPC Pipeline Documentation**
   - Complete architecture and customization guide
   - Solver generation process explanation
   - Module system detailed guide

---

## Repository Structure

```
mpc_planner/                        # Core planner logic
├── src/planner.cpp                 # Main planner orchestration
├── include/mpc_planner/planner.h   # PlannerOutput with topology metadata
└── ...

mpc_planner_modules/                # Objective and constraint modules
├── src/guidance_constraints.cpp    # T-MPC++ implementation
├── src/ellipsoid_constraints.cpp   # Obstacle avoidance
└── ...

mpc_planner_solver/                 # Solver interface
├── src/solver_interface.cpp        # Forces/Acados wrapper
└── ...

mpc_planner_types/                  # Data structures
├── include/mpc_planner_types/
│   ├── data_types.h                # Core types, PlannerState enum
│   ├── module_data.h               # Topology metadata storage
│   └── realtime_data.h             # Runtime data structures
└── ...

mpc_planner_util/                   # Utilities
├── include/mpc_planner_util/
│   └── multi_robot_utility_functions.h  # Multi-robot helpers (NEW)
└── ...

mpc_planner_jackalsimulator/        # JulesJackalPlanner system
├── src/jules_ros1_jackalplanner.cpp     # State machine implementation
├── docs/                                # JulesJackalPlanner docs
│   ├── State_Machine_Implementation.md  # NEW
│   └── ...
└── ...

solver_generator/                   # Python solver generation
├── control_modules.py              # Module manager
├── generate_forces_solver.py       # Forces Pro generation
├── generate_acados_solver.py       # Acados generation
└── ...

docs/                               # THIS DIRECTORY
├── README.md                       # This file
├── mpc_pipeline_documentation.md   # Core architecture
├── guidance_constraints_documentation.md  # T-MPC++ guide
├── topology_metadata_feature.md    # Topology metadata (NEW)
└── ...
```

---

## Related External Resources

### Papers

1. **T-MPC++**: O. de Groot et al., "Topology-Driven Parallel Trajectory Optimization in Dynamic Environments," IEEE T-RO, 2024. https://doi.org/10.1109/TRO.2024.3475047

2. **SH-MPC**: O. de Groot et al., "Scenario-Based Trajectory Optimization with Bounded Probability of Collision," IJRR, 2024. https://arxiv.org/pdf/2307.01070

3. **MPCC**: J. Ziegler et al., "Model Predictive Contouring Control," IEEE T-IV, 2019. https://ieeexplore.ieee.org/document/8768044

### External Dependencies

- **Forces Pro**: https://www.embotech.com/softwareproducts/forcespro/
- **Acados**: https://docs.acados.org/
- **Guidance Planner**: https://github.com/tud-amr/guidance_planner
- **Scenario Module**: https://github.com/oscardegroot/scenario_module (for SH-MPC)
- **DecompUtil**: https://github.com/oscardegroot/DecompUtil (for static obstacles)

### Repository

- **Main Repository**: https://github.com/tud-amr/mpc_planner
- **Fork with Multi-Robot**: https://github.com/Juleszwanen/oscar_mpc_planner_mr_modification

---

## Contributing to Documentation

When updating documentation:

1. **Keep it synchronized** with code changes
2. **Update all related documents** when making changes
3. **Add examples** for new features
4. **Include diagrams** where helpful
5. **Update this README** when adding new documentation files
6. **Date your changes** in commit messages
7. **Test code examples** before including them

### Documentation Standards

- Use Markdown format
- Include code examples with syntax highlighting
- Add visual diagrams where appropriate
- Link between related documents
- Keep table of contents updated
- Mark new features with **NEW** tag
- Include commit references for major changes
- Date documentation updates

---

## Questions or Issues?

- **Architecture questions**: See [MPC Pipeline Documentation](mpc_pipeline_documentation.md)
- **T-MPC++ questions**: See [Guidance Constraints Documentation](guidance_constraints_documentation.md)
- **Topology metadata**: See [Topology Metadata Feature](topology_metadata_feature.md)
- **Topology-aware communication**: See [Topology-Aware Communication section](guidance_constraints_documentation.md#topology-aware-communication-for-multi-robot-systems)
- **JulesJackalPlanner**: See `../mpc_planner_jackalsimulator/docs/README.md`
- **State machine**: See `../mpc_planner_jackalsimulator/docs/State_Machine_Implementation.md`

For issues not covered in documentation, contact the development team or create an issue in the repository.

---

## Document History

| Date       | Update                                  | Author  |
|------------|-----------------------------------------|---------|
| 2024-10-08 | Initial documentation suite created     | Copilot |
| 2024-10-08 | MPC pipeline documentation              | Copilot |
| 2024-10-08 | Guidance constraints documentation      | Copilot |
| 2025-10-10 | State machine refactoring implemented   | Jules   |
| 2025-10-17 | Topology metadata documentation added   | Copilot |
| 2025-10-17 | Main docs README created                | Copilot |
| 2025-10-17 | State machine documentation added       | Copilot |
| 2025-10-17 | Topology-aware communication docs enhanced | Copilot |

---

*Last updated: October 17, 2025*  
*Documentation status: Current and comprehensive*
