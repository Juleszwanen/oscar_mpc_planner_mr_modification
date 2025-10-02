# Guidance Constraints Module - Documentation Index

This directory contains comprehensive documentation for the `guidance_constraints.cpp` module, which implements T-MPC++ (Topology-aware Model Predictive Control) for autonomous navigation.

## Documentation Files

### 📘 [guidance_constraints_documentation.md](./guidance_constraints_documentation.md) - **Main Documentation**

**Target Audience**: Developers who need deep understanding of the module
**Length**: ~1400 lines / ~70 pages
**Content**:
- Executive summary and purpose
- Integration with `Planner::solveMPC()` pipeline
- Complete code walkthrough of `update()` function (10 detailed steps)
- Complete code walkthrough of `optimize()` function (4 phases, 7 sub-phases)
- OpenMP parallelization architecture
- Core data structures and relationships
- **Homology ID storage implementation guide** (3 methods with complete code)
- Integration with `jules_ros1_jackalplanner.cpp`
- Performance characteristics and timing analysis
- Troubleshooting guide with solutions
- Advanced topics and extensions

**Start here if**: You want to fully understand how the module works internally

---

### 📗 [guidance_constraints_quick_reference.md](./guidance_constraints_quick_reference.md) - **Quick Reference**

**Target Audience**: Developers actively working with the code
**Length**: ~250 lines / ~12 pages
**Content**:
- Quick overview and key concepts
- Function signatures and call sequences
- Data structure definitions
- Configuration parameters reference
- Performance metrics table
- Common issues & solutions
- Ready-to-use code snippets
- Related files index

**Use this when**: You need quick answers during development

---

### 📊 [guidance_constraints_diagrams.md](./guidance_constraints_diagrams.md) - **Visual Diagrams**

**Target Audience**: Anyone who learns visually
**Length**: ~550 lines / ~28 pages
**Content**:
- High-level system architecture (ASCII art)
- Detailed `optimize()` execution flow
- Parallel thread execution timeline
- Data structures relationship diagram
- Consistency bonus mechanism visualization
- Constraint architecture explanation
- Homology ID storage flow diagram
- Complete system timing breakdown

**Start here if**: You prefer visual learning and want conceptual understanding

---

## Quick Start Guide

### For Understanding the System:

1. **Begin with visuals**: Read [guidance_constraints_diagrams.md](./guidance_constraints_diagrams.md) sections 1-3
   - Get high-level architecture overview
   - Understand data flow
   - See execution sequence

2. **Deep dive**: Read [guidance_constraints_documentation.md](./guidance_constraints_documentation.md)
   - Focus on "Integration with Planner::solveMPC() Pipeline"
   - Read complete walkthrough of `update()` and `optimize()`
   - Understand why the design works

3. **Reference as needed**: Use [guidance_constraints_quick_reference.md](./guidance_constraints_quick_reference.md)
   - Look up configuration parameters
   - Find code snippets
   - Check troubleshooting tips

### For Implementing Homology ID Storage:

1. **Read the guide**: [guidance_constraints_documentation.md](./guidance_constraints_documentation.md) 
   - Jump to section "Storing and Using Homology IDs"
   - Choose implementation method (Method 1 recommended)

2. **Follow the code**: Copy-paste from the step-by-step examples
   - Modify `PlannerOutput` structure
   - Update `GuidanceConstraints::optimize()`
   - Update `Planner::solveMPC()`
   - Update `jules_ros1_jackalplanner.cpp`

3. **Test**: Use examples from [guidance_constraints_quick_reference.md](./guidance_constraints_quick_reference.md)
   - Track topology switches
   - Publish topology information
   - Log for analysis

### For Debugging Issues:

1. **Identify symptom**: Check "Common Issues & Solutions" in [guidance_constraints_quick_reference.md](./guidance_constraints_quick_reference.md)

2. **Understand timing**: Review "Complete System Timing Breakdown" in [guidance_constraints_diagrams.md](./guidance_constraints_diagrams.md)

3. **Deep investigation**: Use "Troubleshooting" section in [guidance_constraints_documentation.md](./guidance_constraints_documentation.md)

---

## Key Concepts (From the Documentation)

### What is Guidance Constraints?

The `GuidanceConstraints` module implements **parallel multi-topology MPC**:
- Runs fast graph search to find topologically distinct paths
- Executes multiple MPC optimizations in parallel (8 threads)
- Each optimization explores a different way to pass obstacles
- Selects the best feasible solution based on cost
- Achieves 6-7x speedup through parallelization

### Two Key Functions:

1. **`update()`**: Finds guidance trajectories (1-5ms)
   - Graph search in 2D/3D space
   - Identifies homotopy classes
   - Maps trajectories to planners

2. **`optimize()`**: Refines into feasible trajectories (20-40ms)
   - Parallel MPC optimization (8 solvers)
   - Each explores different topology
   - Selects best solution
   - Transfers to main solver

### Why It's Powerful:

**Traditional MPC**: One trajectory → If it fails, robot stops
**T-MPC with GuidanceConstraints**: 8 parallel trajectories → Best one succeeds

Result: More robust, optimal navigation in complex environments

---

## Implementation Roadmap

### Phase 1: Understanding (Estimated: 2-4 hours)
- [ ] Read visual diagrams (sections 1-3)
- [ ] Read main documentation introduction
- [ ] Understand `update()` function walkthrough
- [ ] Understand `optimize()` function walkthrough
- [ ] Watch the code execute (add LOG statements)

### Phase 2: Integration (Estimated: 4-8 hours)
- [ ] Choose homology ID storage method
- [ ] Modify data structures (`PlannerOutput`, `ModuleData`)
- [ ] Update `GuidanceConstraints::optimize()` to store IDs
- [ ] Update `Planner::solveMPC()` to transfer IDs
- [ ] Update `jules_ros1_jackalplanner.cpp` to use IDs
- [ ] Test with simple scenarios
- [ ] Add logging and visualization

### Phase 3: Advanced Usage (Estimated: variable)
- [ ] Implement topology switch tracking
- [ ] Publish topology information to ROS
- [ ] Integrate with higher-level planner
- [ ] Add multi-robot coordination with topology
- [ ] Implement learning/preferences for topologies
- [ ] Performance tuning and optimization

---

## Configuration Reference

### Key Parameters (from `config/settings.yaml`):

```yaml
t-mpc:
  use_t-mpc++: true                    # Enable non-guided fallback
  enable_constraints: true              # Use guidance constraints
  warmstart_with_mpc_solution: true    # Warm-start strategy
  selection_weight_consistency_: 0.8   # Hysteresis (0.7-0.9)

guidance:
  n_paths_: 3                          # Number of topologies (1-8)
  longitudinal_goals_: 5               # Goals along path
  vertical_goals_: 5                   # Goals perpendicular to path

control_frequency: 20.0                # Hz
N: 20                                  # MPC horizon
integrator_step: 0.1                   # Time step
```

See [guidance_constraints_quick_reference.md](./guidance_constraints_quick_reference.md) for complete configuration reference.

---

## Related Code Files

| File | Purpose | Location |
|------|---------|----------|
| Implementation | Core module logic | `mpc_planner_modules/src/guidance_constraints.cpp` |
| Header | Class definition | `mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h` |
| Planner orchestrator | Module pipeline | `mpc_planner/src/planner.cpp` |
| Planner header | PlannerOutput definition | `mpc_planner/include/mpc_planner/planner.h` |
| Robot controller | Usage example | `mpc_planner_jackalsimulator/src/jules_ros1_jackalplanner.cpp` |
| Module initialization | Module loading | Auto-generated `modules.h` |

---

## FAQ

### Q: Which document should I read first?
**A**: Start with the diagrams document for visual overview, then move to the main documentation for depth.

### Q: How do I store the homology ID that was selected?
**A**: See the "Storing and Using Homology IDs" section in the main documentation. Method 1 (extending PlannerOutput) is recommended.

### Q: Why does the robot switch between paths frequently?
**A**: Lower the `selection_weight_consistency_` parameter (e.g., from 0.8 to 0.7) to increase hysteresis. See troubleshooting section.

### Q: Can I run with fewer parallel optimizations?
**A**: Yes, reduce `n_paths_` in configuration. The module will scale accordingly.

### Q: How do I visualize what the module is doing?
**A**: Enable `debug_visuals: true` in settings.yaml and subscribe to `/guidance_constraints/optimized_trajectories` in RViz.

### Q: What if all optimizations fail?
**A**: Enable T-MPC++ (`use_t-mpc++: true`) to add a non-guided fallback planner.

---

## Contributing to Documentation

If you find errors or have suggestions for improvement:

1. **Open an issue**: Describe what's unclear or incorrect
2. **Submit a PR**: Propose improvements or additions
3. **Share examples**: Contribute your integration examples

### Documentation Standards:
- Use markdown formatting consistently
- Include code examples with comments
- Add diagrams where helpful (ASCII or image)
- Cross-reference between documents
- Keep quick reference concise (< 15 pages)
- Expand main documentation as needed

---

## Version History

### v1.0 (Current)
- Initial comprehensive documentation suite
- Complete code walkthroughs for `update()` and `optimize()`
- Three implementation methods for homology ID storage
- Visual diagrams and timing analysis
- Integration guide with `jules_ros1_jackalplanner.cpp`
- Quick reference for development
- Troubleshooting and FAQ sections

---

## License

This documentation is part of the oscar_mpc_planner_mr_modification repository and follows the same license (Apache 2.0).

---

## Contact

For questions about the guidance constraints module:
- Open an issue on GitHub
- Refer to the main repository README for contact information
- Check existing documentation first!

**Remember**: The documentation is comprehensive. Use search (Ctrl+F) to find specific topics quickly!
