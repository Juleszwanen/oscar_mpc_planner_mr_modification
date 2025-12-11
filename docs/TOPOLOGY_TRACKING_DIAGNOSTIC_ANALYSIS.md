# Topology Tracking & Planner Mapping Diagnostic Analysis

## Executive Summary

This document provides a comprehensive analysis of the topology tracking and planner mapping system in the T-MPC++ multi-robot motion planning pipeline. The analysis covers the complete data flow from guidance layer through MPC optimization, ROS interface, communication, and visualization.

**Key Components Analyzed:**
1. Guidance Layer (`GuidancePlanner::GlobalGuidance`)
2. MPC Planner Layer (`GuidanceConstraints`, `LocalPlanner`)
3. ROS Interface (`JulesRealJackalPlanner`)
4. Communication Logic (`CommunicationTriggers`)
5. Visualization Modules

---

## 1. Current Implementation Logic (Ground Truth)

### 1.1 Topology ID Generation and Assignment

#### Source: `GlobalGuidance` (External Library)
The `GlobalGuidance` class generates topology IDs through the `topology_class` field in guidance trajectories.

**Topology ID Range:**
- **Guided topologies**: `0` to `n_paths - 1` (typically 0-7 for n_paths=8)
- **Non-guided topology ID**: `2 * n_paths` (e.g., 16 when n_paths=8)
- **Topology matching failed**: Same as non-guided: `2 * n_paths`
- **Topology not selected**: `999` (TOPOLOGY_NOT_SELECTED)
- **Default/uninitialized**: `-1`

#### Key Code Locations:
```cpp
// guidance_constraints.cpp - Constructor (lines 42-44)
TOPOLOGY_NO_MATCH = 2 * global_guidance_->GetConfig()->n_paths_;
TOPOLOGY_NOT_SELECTED = 999;
_non_guided_topology_id = TOPOLOGY_NOT_SELECTED;
```

### 1.2 Planner Construction & ID Assignment

#### Planner Creation Logic (guidance_constraints.cpp, lines 58-76):
```cpp
int n_solvers = global_guidance_->GetConfig()->n_paths_;
int total_planners = n_solvers + (_use_tmpcpp ? 1 : 0);

// Create guided planners with IDs 0 to n_solvers-1
for (int i = 0; i < n_solvers; i++) {
    planners_.emplace_back(i);  // id = i, is_original_planner = false
}

// Add non-guided planner with ID = n_solvers (last position)
if (_use_tmpcpp) {
    planners_.emplace_back(n_solvers, true);  // id = n_solvers, is_original_planner = true
}
```

#### LocalPlanner Structure:
```cpp
struct LocalPlanner {
    int id;                              // Planner index (0 to n_solvers)
    bool is_original_planner;            // True for T-MPC++ non-guided planner
    std::shared_ptr<Solver> local_solver;
    SolverResult result;                 // Contains guidance_ID (topology class)
    bool has_consistency_enabled;        // For consistency cost tracking
};
```

#### Critical Distinction:
| Field | Meaning | Range |
|-------|---------|-------|
| `planner.id` | Index in planners_ vector | 0 to n_solvers |
| `planner.result.guidance_ID` | Topology class ID | 0 to 2*n_paths |

**⚠️ Potential Issue #1: `planner.id` ≠ `guidance_ID`**
- `planner.id` is used for array indexing and solver identification
- `guidance_ID` represents the actual topology class from guidance search
- These are NOT interchangeable and mixing them causes bugs

### 1.3 Topology Assignment to Planners

#### Mapping Logic (mapGuidanceTrajectoriesToPlanners, lines 215-273):

```cpp
void GuidanceConstraints::mapGuidanceTrajectoriesToPlanners() {
    // Phase 1: Reset all planners
    for (auto &planner : planners_) {
        planner.taken = false;
        planner.existing_guidance = false;
    }
    _map_homotopy_class_to_planner.clear();

    // Phase 2: Match guidance trajectories to planners with same previous guidance_ID
    for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); i++) {
        int homotopy_class = global_guidance_->GetGuidanceTrajectory(i).topology_class;
        
        for (size_t p = 0; p < planners_.size(); p++) {
            // Match by guidance_ID from previous iteration's result
            if (planners_[p].result.guidance_ID == homotopy_class && !planners_[p].taken) {
                _map_homotopy_class_to_planner[i] = p;
                planners_[p].taken = true;
                planners_[p].existing_guidance = true;
                break;
            }
        }
    }

    // Phase 3: Assign remaining trajectories to remaining planners
    for (int i : remaining_trajectories) {
        for (size_t p = 0; p < planners_.size(); p++) {
            if (!planners_[p].taken) {
                _map_homotopy_class_to_planner[i] = p;
                planners_[p].taken = true;
                planners_[p].existing_guidance = false;
            }
        }
    }
}
```

**⚠️ Potential Issue #2: Stale guidance_ID across iterations**
- The mapping uses `planners_[p].result.guidance_ID` from the previous iteration
- If guidance doesn't find the same topology classes, old IDs may persist
- This can cause incorrect warmstart initialization

### 1.4 Best Planner Selection

#### Selection Logic (optimize function, lines 594-612):

```cpp
int GuidanceConstraints::FindBestPlanner() {
    double best_solution = 1e10;
    int best_index = -1;
    
    for (size_t i = 0; i < planners_.size(); i++) {
        auto &planner = planners_[i];
        if (planner.disabled)
            continue;
        
        if (planner.result.success && planner.result.objective < best_solution) {
            best_solution = planner.result.objective;
            best_index = i;
        }
    }
    return best_index;
}
```

#### Result Assignment After Selection (lines 380-427):

```cpp
// For NON-GUIDED planner (is_original_planner = true):
if (planner.is_original_planner) {
    planner.result.guidance_ID = 2 * global_guidance_->GetConfig()->n_paths_;  // e.g., 16
    planner.result.color = -1;
    
    // Consistency cost subtraction for fair comparison
    if (planner.has_consistency_enabled) {
        double consistency_cost = calculateConsistencyCostForSolver(planner.local_solver);
        planner.result.objective -= consistency_cost;
    }
}
// For GUIDED planners:
else {
    auto &guidance_trajectory = global_guidance_->GetGuidanceTrajectory(planner.id);
    planner.result.guidance_ID = guidance_trajectory.topology_class;  // Actual topology ID
    planner.result.color = guidance_trajectory.color_;
    
    // Apply consistency bonus for previously selected trajectory
    if (guidance_trajectory.previously_selected_)
        planner.result.objective *= global_guidance_->GetConfig()->selection_weight_consistency_;
}
```

**⚠️ Potential Issue #3: Incorrect indexing for guided planners**
- Uses `planner.id` to index into guidance trajectories: `GetGuidanceTrajectory(planner.id)`
- `planner.id` ranges from 0 to n_solvers
- `NumberOfGuidanceTrajectories()` may return fewer than n_solvers
- Code assumes `planner.id < NumberOfGuidanceTrajectories()` but only checks in earlier loop

### 1.5 Non-Guided Topology Matching

#### Meaningful Topology Assignment (lines 462-510):

```cpp
if (best_planner.is_original_planner && _assign_meaningful_topology) {
    if (global_guidance_->NumberOfGuidanceTrajectories() > 0) {
        GuidancePlanner::GeometricPath mpc_path = convertMPCTrajectoryToGeometricPath(best_solver);
        int meaningful_topology_id = global_guidance_->FindTopologyClassForPath(mpc_path, _ego_robot_ns);
        
        if (meaningful_topology_id != TOPOLOGY_NO_MATCH) {
            best_planner.result.guidance_ID = meaningful_topology_id;
            _non_guided_topology_id = meaningful_topology_id;
            assignColorToNonGuidedPlanner(best_planner, meaningful_topology_id);
        } else {
            _non_guided_topology_id = TOPOLOGY_NO_MATCH;  // Falls back to 2*n_paths
        }
    }
}
```

**Key: FindTopologyClassForPath returns:**
- `topology_class` (0 to n_paths-1) if match found
- `2 * n_paths_` (fallback_id) if no match found

### 1.6 Module Data Transfer

#### From GuidanceConstraints to ModuleData (lines 533-555):

```cpp
if (CONFIG["JULES"]["use_extra_params_module_data"].as<bool>()) {
    module_data.selected_topology_id = best_planner.result.guidance_ID;
    module_data.selected_planner_index = best_planner_index_;
    module_data.used_guidance = !best_planner.is_original_planner;
    module_data.trajectory_cost = best_planner.result.objective;
    module_data.solver_exit_code = best_planner.result.exit_code;
    module_data.num_of_guidance_found = global_guidance_->NumberOfGuidanceTrajectories();
}
```

#### From ModuleData to PlannerOutput (planner.cpp, lines 213-226):

```cpp
if (CONFIG["JULES"]["use_extra_params_module_data"].as<bool>()) {
    _output.selected_topology_id = _module_data.selected_topology_id;
    _output.selected_planner_index = _module_data.selected_planner_index;
    _output.used_guidance = _module_data.used_guidance;
    _output.trajectory_cost = _module_data.trajectory_cost;
    _output.solver_exit_code = exit_flag;
    _output.following_new_topology = (prev_followed_topology == _module_data.selected_topology_id) ? false : true;
    _output.previous_topology_id = prev_followed_topology;
}
```

---

## 2. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          GUIDANCE LAYER                                          │
│                                                                                  │
│  global_guidance_->Update()                                                      │
│       │                                                                          │
│       ↓                                                                          │
│  ┌─────────────────────────────────────────┐                                    │
│  │ GuidanceTrajectory                      │                                    │
│  │   ├─ topology_class (0 to n_paths-1)    │  ← Actual homotopy class ID        │
│  │   ├─ color_ (visualization)             │                                    │
│  │   ├─ previously_selected_ (bool)        │  ← For consistency bonus           │
│  │   └─ spline (trajectory data)           │                                    │
│  └─────────────────────────────────────────┘                                    │
└──────────────────────────────┬──────────────────────────────────────────────────┘
                               │
                               ↓ mapGuidanceTrajectoriesToPlanners()
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          MPC PLANNER LAYER                                       │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ planners_[0..n_solvers-1] (GUIDED)                                       │   │
│  │   ├─ id: 0..n_solvers-1                                                  │   │
│  │   ├─ is_original_planner: false                                          │   │
│  │   ├─ result.guidance_ID: topology_class from guidance                    │   │
│  │   └─ result.color: from guidance trajectory                              │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ planners_[n_solvers] (NON-GUIDED / T-MPC++)                              │   │
│  │   ├─ id: n_solvers                                                       │   │
│  │   ├─ is_original_planner: true                                           │   │
│  │   ├─ result.guidance_ID: 2*n_paths (default) OR matched topology         │   │
│  │   └─ result.color: -1 (default) OR matched color                         │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
│  FindBestPlanner() → best_planner_index_                                        │
│       │                                                                          │
│       ↓                                                                          │
│  ┌─────────────────────────────────────────┐                                    │
│  │ module_data                             │                                    │
│  │   ├─ selected_topology_id              │  ← best_planner.result.guidance_ID │
│  │   ├─ selected_planner_index            │  ← best_planner_index_ (array idx) │
│  │   ├─ used_guidance                     │  ← !is_original_planner            │
│  │   └─ trajectory_cost                   │  ← objective after adjustments     │
│  └─────────────────────────────────────────┘                                    │
└──────────────────────────────┬──────────────────────────────────────────────────┘
                               │
                               ↓ Planner::solveMPC()
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          PLANNER OUTPUT LAYER                                    │
│                                                                                  │
│  ┌─────────────────────────────────────────┐                                    │
│  │ PlannerOutput                           │                                    │
│  │   ├─ selected_topology_id              │  ← module_data.selected_topology_id│
│  │   ├─ selected_planner_index            │  ← module_data.selected_planner_idx│
│  │   ├─ previous_topology_id              │  ← from previous iteration         │
│  │   ├─ following_new_topology            │  ← computed by comparison          │
│  │   ├─ used_guidance                     │  ← true if guided planner selected │
│  │   └─ trajectory (positions, orientations)                                   │
│  └─────────────────────────────────────────┘                                    │
└──────────────────────────────┬──────────────────────────────────────────────────┘
                               │
                               ↓ publishCmdAndVisualize()
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          ROS INTERFACE LAYER                                     │
│                                                                                  │
│  decideCommunication(output)                                                    │
│       │                                                                          │
│       ↓                                                                          │
│  shouldCommunicate(output, data)                                                │
│       │                                                                          │
│       ├─→ checkInfeasible(output)                                               │
│       ├─→ checkNonGuidedHomologyFail(output, n_paths)                           │
│       ├─→ checkTopologyChange(output, n_paths)                                  │
│       ├─→ checkGeometricDeviation(trajectory, last_trajectory, threshold)       │
│       └─→ checkTime(last_send_time, current_time, heartbeat)                    │
│                                                                                  │
│  publishDirectTrajectory(output)                                                │
│       │                                                                          │
│       ↓                                                                          │
│  ┌─────────────────────────────────────────┐                                    │
│  │ mpc_planner_msgs::ObstacleGMM           │                                    │
│  │   ├─ id (robot ID)                      │                                    │
│  │   ├─ pose (current pose)                │                                    │
│  │   └─ gaussians[].mean.poses[]           │  ← trajectory positions            │
│  └─────────────────────────────────────────┘                                    │
│                                                                                  │
│  ⚠️ NOTE: Topology ID is NOT included in the message!                           │
└──────────────────────────────┬──────────────────────────────────────────────────┘
                               │
                               ↓ Other robots receive
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          OTHER ROBOTS                                            │
│                                                                                  │
│  trajectoryCallback(msg, ns)                                                    │
│       │                                                                          │
│       ↓                                                                          │
│  Update trajectory_dynamic_obstacles[ns]                                        │
│  ⚠️ No topology information available - robots don't know each other's topology │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Identified Potential Problems

### 3.1 Incorrect Mapping Issues

#### Issue A: planner.id vs guidance_ID confusion

**Location**: `guidance_constraints.cpp`, lines 404-406
```cpp
auto &guidance_trajectory = global_guidance_->GetGuidanceTrajectory(planner.id);
planner.result.guidance_ID = guidance_trajectory.topology_class;
```

**Problem**: 
- `planner.id` is the index in the planners_ vector (0 to n_solvers)
- `NumberOfGuidanceTrajectories()` may return fewer trajectories
- When `planner.id >= NumberOfGuidanceTrajectories()`, this causes out-of-bounds access

**Evidence**: The guard at line 318 checks:
```cpp
if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories()) {
    if (!planner.is_original_planner) {
        planner.disabled = true;
        continue;
    }
}
```

**Guard Condition Analysis:**
- This guard runs at the START of the optimization loop
- It correctly disables planners when `planner.id >= NumberOfGuidanceTrajectories()`
- However, it ONLY disables non-original planners (`is_original_planner=false`)
- The original planner (T-MPC++) correctly bypasses this check and remains enabled

**Why this guard may be insufficient:**
1. The guard assumes `planner.id` == index into guidance trajectories, but after mapping, this may not hold
2. Later code at line 404-406 uses `planner.id` directly without re-checking bounds
3. If `NumberOfGuidanceTrajectories()` changes between mapping and optimization (unlikely but possible), indexing could fail

**Impact**: Potential crash or incorrect topology assignment in edge cases

#### Issue B: Stale guidance_ID in mapGuidanceTrajectoriesToPlanners()

**Location**: `guidance_constraints.cpp`, lines 238-247
```cpp
if (planners_[p].result.guidance_ID == homotopy_class && !planners_[p].taken) {
    _map_homotopy_class_to_planner[i] = p;
    // ...
}
```

**Problem**:
- Uses `planners_[p].result.guidance_ID` from the previous iteration
- If a topology class disappears (obstacle moves), old ID persists
- Planner may be mapped to wrong guidance trajectory

**Impact**: Incorrect warmstarting, poor optimization convergence

#### Issue C: Non-guided planner topology offset

**Location**: Various, but critical at line 386:
```cpp
planner.result.guidance_ID = 2 * global_guidance_->GetConfig()->n_paths_;
```

**Analysis**: This is consistent with `TOPOLOGY_NO_MATCH`:
```cpp
TOPOLOGY_NO_MATCH = 2 * global_guidance_->GetConfig()->n_paths_;
```

**Status**: ✅ This appears CORRECT. Non-guided default ID matches the fallback ID.

### 3.2 Communication Errors

#### Issue D: Topology ID not transmitted in messages

**Location**: `jules_ros1_real_jackalplanner.cpp`, `publishDirectTrajectory()` (lines 1471-1532)

**Problem**: The `ObstacleGMM` message does not include:
- `topology_id`
- `used_guidance`
- `following_new_topology`

**Impact**: 
- Other robots cannot know which topology the sender is following
- Makes topology-aware coordination impossible
- Limits effectiveness of the communication optimization

#### Issue E: Non-guided topology ID comparison in communication triggers

**Location**: `communication_triggers.cpp`, lines 73-83
```cpp
bool CommunicationTriggers::checkNonGuidedHomologyFail(const PlannerOutput& output, int n_paths) {
    if (!output.success) 
        return false;
    const int non_guided_topology_id = 2 * n_paths;
    return (output.selected_topology_id == non_guided_topology_id);
}
```

**Problem**: 
- When `_assign_meaningful_topology=true`, non-guided planner may get a DIFFERENT topology ID
- This check uses `2 * n_paths` but the actual `selected_topology_id` may be 0-7 if matched
- False negatives: matched non-guided won't trigger this, false positives: none

**Impact**: Inconsistent communication decisions when non-guided planner matches a topology

#### Issue F: Topology change detection includes non-guided transitions

**Location**: `communication_triggers.cpp`, lines 28-47
```cpp
bool CommunicationTriggers::checkTopologyChange(const PlannerOutput& output, int n_paths) {
    const int non_guided_topology_id = 2 * n_paths;
    bool is_to_guided = (output.selected_topology_id != non_guided_topology_id);
    return output.following_new_topology && is_to_guided;
}
```

**Analysis**: 
- This correctly excludes switches TO non-guided (handled by checkNonGuidedHomologyFail)
- But doesn't handle: Guided A → Non-Guided (matched to A) → Guided B
- In this case, the "matched non-guided" has same topology as Guided A, so `following_new_topology=false`
- Then switching to Guided B would be detected as topology change

**Status**: ⚠️ Partially correct, edge case may cause unexpected behavior

### 3.3 Visualization Errors

#### Issue G: Color assignment for matched non-guided planner

**Location**: `guidance_constraints.cpp`, lines 921-951
```cpp
void GuidanceConstraints::assignColorToNonGuidedPlanner(LocalPlanner& best_planner, int meaningful_topology_id) {
    for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); i++) {
        auto &guidance_traj = global_guidance_->GetGuidanceTrajectory(i);
        if (guidance_traj.topology_class == meaningful_topology_id) {
            best_planner.result.color = guidance_traj.color_;
            color_found = true;
            break;
        }
    }
    if (!color_found) {
        best_planner.result.color = -1;  // Fallback to dark red
    }
}
```

**Problem**: 
- Searches by `topology_class` not by trajectory index
- Multiple guidance trajectories could have the same topology class (though unlikely)
- First match is used, not necessarily the "best" one

**Impact**: Minor - color may not perfectly match expected visualization

#### Issue H: visualizeTrajectory color parameter usage

**Location**: `guidance_constraints.cpp`, lines 656-717 (visualize function)

The visualization uses different conventions:
- `color = -1`: Special handling (selected trajectory)
- `color = -2`: Black (non-guided not matched)
- `color >= 0`: Use color from guidance trajectory

**Analysis**: The logic at lines 687-689 is:
```cpp
int viz_color = (planner.result.color == -1) ? -2 : planner.result.color;
visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 1.0, viz_color, ...);
```

**Status**: ✅ This appears correct - converts -1 to -2 for non-selected non-guided

### 3.4 Consistency Tracking Issues

#### Issue I: Consistency cost not properly excluded from selection

**Location**: `guidance_constraints.cpp`, lines 389-427

**The current logic**:
```cpp
// For non-guided planner:
if (planner.has_consistency_enabled) {
    double consistency_cost = calculateConsistencyCostForSolver(planner.local_solver);
    planner.result.objective -= consistency_cost;  // Subtract for fair comparison
}

// For guided planner:
if (planner.has_consistency_enabled) {
    double consistency_cost = calculateConsistencyCostForSolver(planner.local_solver);
    planner.result.objective -= consistency_cost;
}

// THEN apply selection weight (only for guided):
if (guidance_trajectory.previously_selected_)
    planner.result.objective *= global_guidance_->GetConfig()->selection_weight_consistency_;
```

**Analysis**: 
- Consistency cost is subtracted BEFORE selection weight is applied
- This is the correct order for fair comparison
- Formula: `fair_cost = (raw_cost - consistency_cost) * selection_weight`

**Status**: ✅ This appears correct

#### Issue J: Consistency tracking reset timing

**Location**: `guidance_constraints.cpp`, lines 800-817
```cpp
void GuidanceConstraints::resetConsistencyParameters() {
    if (!_consistency_module_available) return;
    
    _prev_selected_topology_id = -1;
    _prev_was_original_planner = false;
    _has_previous_trajectory = false;
    // ...
}
```

**Problem**: Called from `reset()` which is triggered on goal reached. But also needs to be called when:
- All solvers fail
- Major discontinuity in planning

**Evidence**: Lines 443-447 handle all-solvers-fail:
```cpp
_has_previous_trajectory = false;
_prev_selected_topology_id = -1;
_prev_was_original_planner = false;
```

**Status**: ✅ Correctly handled in both places

---

## 4. Expected Logic vs. Current Logic Comparison

### 4.1 Topology ID Assignment

| Aspect | Expected | Current | Match? |
|--------|----------|---------|--------|
| Guided planner topology ID | From guidance trajectory's topology_class | From GetGuidanceTrajectory(planner.id).topology_class | ⚠️ Assumes planner.id == trajectory index |
| Non-guided default ID | 2 * n_paths | 2 * n_paths | ✅ |
| Non-guided matched ID | Matched topology class | Matched topology class (when enabled) | ✅ |
| Non-guided fallback when no match | 2 * n_paths | 2 * n_paths (TOPOLOGY_NO_MATCH) | ✅ |

### 4.2 Planner Selection

| Aspect | Expected | Current | Match? |
|--------|----------|---------|--------|
| Select lowest cost feasible | Yes | Yes | ✅ |
| Consistency bonus for same topology | Multiply by selection_weight | Multiply by selection_weight | ✅ |
| Exclude consistency from raw comparison | Subtract before comparison | Subtract before comparison | ✅ |
| Handle all solvers fail | Return failure, reset tracking | Return first exit code, reset | ✅ |

### 4.3 Communication Triggers

| Aspect | Expected | Current | Match? |
|--------|----------|---------|--------|
| Communicate on MPC failure | Always | Always | ✅ |
| Communicate on non-guided (no match) | Always | Always | ✅ |
| Communicate on topology switch | Yes | Yes | ✅ |
| Communicate on geometric deviation | Yes | Yes | ✅ |
| Communicate on time trigger | Yes | Yes | ✅ |
| Include topology ID in message | Yes | NO | ❌ |

### 4.4 Visualization

| Aspect | Expected | Current | Match? |
|--------|----------|---------|--------|
| Color by topology class | Yes | Yes (via guidance color) | ✅ |
| Highlight selected trajectory | Yes | Yes (color=-1 special handling) | ✅ |
| Non-guided standout markers | Optional (config) | Optional (config) | ✅ |
| Consistency reference visualization | Orange line | Orange line | ✅ |

---

## 5. Hypotheses for Root Causes

### Hypothesis 1: Planner Index vs Topology ID Confusion
**Likelihood**: HIGH

The code uses `planner.id` and `planner.result.guidance_ID` in different contexts, and confusion between these two values is likely causing bugs. Key evidence:
- `planner.id` is used to index into guidance trajectories
- `guidance_ID` stores the topology class
- These are different values that get mixed up

**Test**: Add assertions that verify `planner.id < NumberOfGuidanceTrajectories()` before accessing

### Hypothesis 2: Stale Topology Mappings
**Likelihood**: MEDIUM

The mapping from guidance trajectories to planners uses stale `guidance_ID` values from previous iterations. If topology classes change between iterations, the mapping becomes incorrect.

**Test**: Log the mapping before and after mapGuidanceTrajectoriesToPlanners()

### Hypothesis 3: Non-Guided Matched vs Unmatched Ambiguity
**Likelihood**: MEDIUM

When non-guided planner is selected and matches a topology, the communication logic may not correctly identify this as a "matched" case because it compares against `2*n_paths`.

**Test**: When non-guided is selected with `_assign_meaningful_topology=true`, verify:
- `selected_topology_id` is the matched ID (0-7), not 2*n_paths
- Communication triggers behave correctly

### Hypothesis 4: Missing Topology ID in Communication
**Likelihood**: HIGH (confirmed as missing)

The `ObstacleGMM` message doesn't include topology information. This prevents receiving robots from knowing which topology the sender is following.

**Test**: Add topology_id field to message and verify other robots receive it

### Hypothesis 5: Visualization Color Mismatch
**Likelihood**: LOW

Color assignment may not correctly match topology ID due to searching by topology_class vs index.

**Test**: Verify colors match topology classes in visualization

---

## 6. Debugging Instrumentation Suggestions

### 6.1 Add Detailed Logging

Add the following log statements to track topology tracking:

```cpp
// In mapGuidanceTrajectoriesToPlanners():
LOG_DEBUG(_ego_robot_ns + ": Mapping guidance trajectories to planners:");
for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); i++) {
    int topology_class = global_guidance_->GetGuidanceTrajectory(i).topology_class;
    int mapped_planner = _map_homotopy_class_to_planner.count(i) ? 
                         _map_homotopy_class_to_planner[i] : -1;
    LOG_DEBUG("  Guidance[" << i << "] topology=" << topology_class 
              << " -> Planner[" << mapped_planner << "]");
}

// After optimization in optimize():
LOG_INFO(_ego_robot_ns + ": Optimization results:");
for (size_t i = 0; i < planners_.size(); i++) {
    auto &p = planners_[i];
    LOG_DEBUG("  Planner[" << i << "] id=" << p.id 
              << " is_orig=" << p.is_original_planner
              << " disabled=" << p.disabled
              << " success=" << p.result.success
              << " guidance_ID=" << p.result.guidance_ID
              << " objective=" << p.result.objective
              << " color=" << p.result.color);
}

// Before best planner selection:
LOG_INFO(_ego_robot_ns + ": Best planner selection:");
LOG_DEBUG("  best_planner_index_=" << best_planner_index_);
LOG_DEBUG("  selected_topology_id=" << best_planner.result.guidance_ID);
LOG_DEBUG("  is_original_planner=" << best_planner.is_original_planner);
LOG_DEBUG("  used_guidance=" << !best_planner.is_original_planner);
```

### 6.2 Add Data Saver Fields

Extend `saveData()` to include:

```cpp
void GuidanceConstraints::saveData(RosTools::DataSaver &data_saver) {
    // Existing fields...
    
    // ADD: Track all planner topology IDs (guidance_ID represents the topology class)
    for (size_t i = 0; i < planners_.size(); i++) {
        data_saver.AddData("planner_" + std::to_string(i) + "_topology_id", 
                          planners_[i].result.guidance_ID);
        data_saver.AddData("planner_" + std::to_string(i) + "_is_original", 
                          planners_[i].is_original_planner ? 1.0 : 0.0);
    }
    
    // ADD: Track mapping validity
    data_saver.AddData("num_guidance_trajectories", 
                      global_guidance_->NumberOfGuidanceTrajectories());
    data_saver.AddData("num_enabled_planners", 
                      std::count_if(planners_.begin(), planners_.end(),
                                   [](const LocalPlanner& p) { return !p.disabled; }));
}
```

### 6.3 Add Runtime Assertions

```cpp
// In optimize(), before accessing guidance trajectories:
ROSTOOLS_ASSERT(planner.id < global_guidance_->NumberOfGuidanceTrajectories() || 
                planner.is_original_planner,
                "Planner ID " << planner.id << " >= NumberOfGuidanceTrajectories() " 
                << global_guidance_->NumberOfGuidanceTrajectories());

// In mapGuidanceTrajectoriesToPlanners():
for (auto& [guidance_idx, planner_idx] : _map_homotopy_class_to_planner) {
    ROSTOOLS_ASSERT(guidance_idx >= 0 && guidance_idx < global_guidance_->NumberOfGuidanceTrajectories(),
                    "Invalid guidance index in mapping: " << guidance_idx);
    ROSTOOLS_ASSERT(planner_idx >= 0 && planner_idx < planners_.size(),
                    "Invalid planner index in mapping: " << planner_idx);
}
```

### 6.4 Communication Trigger Logging

```cpp
// In shouldCommunicate():
LOG_INFO(_ego_robot_ns + ": Communication decision:");
LOG_DEBUG("  output.success=" << output.success);
LOG_DEBUG("  output.selected_topology_id=" << output.selected_topology_id);
LOG_DEBUG("  output.previous_topology_id=" << output.previous_topology_id);
LOG_DEBUG("  output.following_new_topology=" << output.following_new_topology);
LOG_DEBUG("  output.used_guidance=" << output.used_guidance);
LOG_DEBUG("  non_guided_topology_id=" << (2 * CONFIG["JULES"]["n_paths"].as<int>()));
LOG_DEBUG("  trigger_reason=" << MPCPlanner::toString(_communication_trigger_reason));
```

---

## 7. Validation Tests

### Test 1: Planner ID to Topology ID Consistency

**Objective**: Verify planner IDs correctly map to topology IDs

**Procedure**:
1. Set `n_paths = 4`
2. Run planner in static environment with known topology classes
3. Record for each planner: `planner.id`, `planner.result.guidance_ID`
4. Verify:
   - Planners 0-3: `guidance_ID` should be 0-3 (or whatever guidance returns)
   - Planner 4 (non-guided): `guidance_ID` should be 8 (2*4) or matched value

**Expected Result**: Each planner's `guidance_ID` matches the topology class from its assigned guidance trajectory

### Test 2: Non-Guided Topology Matching

**Objective**: Verify non-guided planner correctly matches topologies

**Procedure**:
1. Enable `_assign_meaningful_topology = true`
2. Create scenario where non-guided is selected
3. Verify `FindTopologyClassForPath` returns correct class
4. Verify `selected_topology_id` in output matches the returned class

**Expected Result**: Non-guided planner gets meaningful topology ID when it matches a guidance trajectory

### Test 3: Communication Trigger Correctness

**Objective**: Verify communication triggers fire correctly

**Test Cases**:
| Scenario | prev_topology | curr_topology | used_guidance | Expected Trigger |
|----------|---------------|---------------|---------------|------------------|
| Stay on guided A | 0 | 0 | true | NO_COMMUNICATION |
| Switch guided A→B | 0 | 1 | true | TOPOLOGY_CHANGE |
| Switch guided→non-guided | 0 | 8 | false | NON_GUIDED_HOMOLOGY_FAIL |
| Stay on non-guided | 8 | 8 | false | NON_GUIDED_HOMOLOGY_FAIL |
| Non-guided matched | 0 | 0 | false | ??? (verify) |
| MPC failure | any | any | any | INFEASIBLE |

### Test 4: Visualization Color Consistency

**Objective**: Verify visualization colors match topology IDs

**Procedure**:
1. Run with `debug_visuals = true`
2. For each trajectory visualized, record:
   - Planner index
   - `planner.result.color`
   - Actual rendered color
3. Verify colors are consistent across iterations for same topology

### Test 5: Multi-Robot Communication Integration

**Objective**: Verify topology information is correctly communicated

**Procedure**:
1. Run two robots in same environment
2. Robot A selects topology 0
3. Verify Robot B receives trajectory message
4. Check if Robot B can determine Robot A's topology (currently: NO)

**Expected Result**: Currently fails - topology ID not in message. Document as known limitation.

### Test 6: Stale Mapping Detection

**Objective**: Detect when mapping uses stale topology IDs

**Procedure**:
1. Create scenario where topology classes change between iterations:
   - t=0: guidance returns classes [0, 1, 2]
   - t=1: guidance returns classes [0, 2, 3] (1 disappears, 3 appears)
2. Verify mapping correctly handles the change
3. Check that planner previously assigned to class 1 doesn't keep old ID

---

## 8. Recommended Fixes

### Fix 1: Add Topology ID to Communication Message

**Priority**: HIGH

**Change**: Extend `ObstacleGMM.msg` to include:
```
int32 topology_id
bool used_guidance
```

**Files to modify**:
- `mpc_planner_msgs/msg/ObstacleGMM.msg`
- `jules_ros1_real_jackalplanner.cpp` (publishDirectTrajectory)
- `jules_ros1_real_jackalplanner.cpp` (trajectoryCallback)

### Fix 2: Validate Planner ID Before Guidance Access

**Priority**: HIGH

**Change**: Add bounds checking before accessing guidance trajectories

```cpp
// In optimize(), before guided planner processing:
if (!planner.is_original_planner) {
    if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories()) {
        LOG_ERROR("Planner ID " << planner.id << " out of range for guidance access");
        planner.disabled = true;
        continue;
    }
}
```

### Fix 3: Clear Stale Mappings

**Priority**: MEDIUM

**Change**: Reset guidance_ID when planner is unmapped

```cpp
void GuidanceConstraints::mapGuidanceTrajectoriesToPlanners() {
    // At start, clear old guidance IDs for planners that won't be mapped
    for (auto &planner : planners_) {
        if (!planner.is_original_planner) {
            planner.result.guidance_ID = -1;  // Mark as unmapped
        }
    }
    // ... rest of mapping logic
}
```

### Fix 4: Fix Non-Guided Communication Trigger

**Priority**: MEDIUM

**Change**: Update `checkNonGuidedHomologyFail` to handle matched non-guided

```cpp
bool CommunicationTriggers::checkNonGuidedHomologyFail(const PlannerOutput& output, int n_paths) {
    if (!output.success) return false;
    
    // If we're using non-guided planner (indicated by used_guidance=false)
    // AND the topology ID is the fallback (2*n_paths), trigger communication
    const int non_guided_topology_id = 2 * n_paths;
    return !output.used_guidance && (output.selected_topology_id == non_guided_topology_id);
}
```

---

## 9. Summary

This analysis identifies several potential issues in the topology tracking and planner mapping system:

1. **High Priority Issues**:
   - Topology ID not included in robot-to-robot communication
   - Potential planner ID vs topology ID confusion causing incorrect guidance access

2. **Medium Priority Issues**:
   - Stale topology mappings across iterations
   - Non-guided topology matching may confuse communication triggers

3. **Low Priority Issues**:
   - Minor visualization color assignment inconsistencies

The recommended debugging instrumentation will help confirm these hypotheses, and the validation tests provide a framework for verifying fixes.

---

## Appendix A: Key Constants and Their Values

| Constant | Definition | Typical Value |
|----------|------------|---------------|
| `n_paths` | Number of guided planners | 4-8 |
| `TOPOLOGY_NO_MATCH` | 2 * n_paths | 8-16 |
| `TOPOLOGY_NOT_SELECTED` | 999 | 999 |
| Default `guidance_ID` | -1 | -1 |
| Non-guided planner ID | n_paths (or n_solvers) | 4-8 |

## Appendix B: ID Ranges Quick Reference

| ID Type | Range | Notes |
|---------|-------|-------|
| planner.id (guided) | 0 to n_solvers-1 | Array index |
| planner.id (non-guided) | n_solvers | Last planner |
| topology_class (guidance) | 0 to ~7 | Actual homotopy class |
| guidance_ID (guided) | 0 to ~7 | Copied from topology_class |
| guidance_ID (non-guided default) | 2*n_paths | Fallback value |
| guidance_ID (non-guided matched) | 0 to ~7 | When matching succeeds |
| selected_topology_id | Any of above | In PlannerOutput |
| color (guidance) | 0 to n_paths-1 | Color palette index |
| color (non-guided) | -1 | Special handling |
| color (non-guided not matched viz) | -2 | Black in visualization |
