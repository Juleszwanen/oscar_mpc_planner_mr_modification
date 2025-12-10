# YAML Configuration System Documentation

This document explains how the `load_yaml.hpp` and `parameters.h` files work together to provide a centralized configuration management system for the MPC Planner codebase.

## Table of Contents

1. [Overview](#overview)
2. [File Descriptions](#file-descriptions)
3. [Architecture](#architecture)
4. [Step-by-Step Workflow](#step-by-step-workflow)
5. [Usage Patterns](#usage-patterns)
6. [API Reference](#api-reference)
7. [Configuration File Structure](#configuration-file-structure)
8. [Examples from the Codebase](#examples-from-the-codebase)

---

## Overview

The configuration system provides two key functionalities:

1. **File-relative YAML loading** (`load_yaml.hpp`) - Macros to locate config files relative to source files
2. **Global singleton configuration** (`parameters.h`) - A thread-safe singleton pattern to access configuration parameters globally via the `CONFIG` macro

This design allows:
- **Centralized configuration**: One `settings.yaml` file per robot/simulator variant
- **Type-safe access**: YAML values are accessed with explicit type casting via `yaml-cpp`
- **Global accessibility**: Any source file can access configuration via `CONFIG["key"]`
- **Compile-time path resolution**: Config paths are resolved at compile time using `__FILE__`

---

## File Descriptions

### `load_yaml.hpp`

```cpp
#ifndef LOAD_YAML_H
#define LOAD_YAML_H

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>

#define SYSTEM_CONFIG_PATH(x, filename) \
    std::filesystem::path(x).parent_path().string() + "/../config/" + filename + ".yaml"

#define SYSTEM_CONFIG_PATH_INCLUDE(x, filename) \
    std::filesystem::path(x).parent_path().parent_path().string() + "/../config/" + filename + ".yaml"

inline void loadConfigYaml(const std::string &file, YAML::Node &_yaml_out)
{
    _yaml_out = YAML::LoadFile(file);
}

#endif // LOAD_YAML_H
```

**Purpose**: Provides utility macros and functions for loading YAML configuration files.

| Component | Description |
|-----------|-------------|
| `SYSTEM_CONFIG_PATH(x, filename)` | Resolves to `<parent_of_x>/../config/<filename>.yaml` |
| `SYSTEM_CONFIG_PATH_INCLUDE(x, filename)` | Resolves to `<grandparent_of_x>/../config/<filename>.yaml` (for header files) |
| `loadConfigYaml()` | Wrapper around `YAML::LoadFile()` for loading YAML files |

### `parameters.h`

```cpp
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <mpc_planner_util/load_yaml.hpp>
#include <ros_tools/logging.h>

#define LOG_MARK(x)                        \
    if (CONFIG["debug_output"].as<bool>()) \
    LOG_HOOK_MSG(x)

#define CONFIG Configuration::getInstance().getYAMLNode()

class Configuration
{
public:
    static Configuration &getInstance()
    {
        static Configuration instance;
        return instance;
    }

    void initialize(const std::string &config_file)
    {
        loadConfigYaml(config_file, _config);
    }

    YAML::Node &getYAMLNode()
    {
        return _config;
    }

private:
    YAML::Node _config;
    Configuration() { }
    Configuration(const Configuration &) = delete;
    Configuration &operator=(const Configuration &) = delete;
};

#endif // PARAMETERS_H
```

**Purpose**: Provides a global singleton for application-wide configuration access.

| Component | Description |
|-----------|-------------|
| `CONFIG` macro | Shorthand for `Configuration::getInstance().getYAMLNode()` |
| `LOG_MARK(x)` | Conditional logging based on `debug_output` setting |
| `Configuration` class | Meyer's singleton holding the global YAML configuration |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Application Startup                          │
│                                                                     │
│  JackalPlanner::JackalPlanner(ros::NodeHandle &nh)                 │
│  {                                                                  │
│      Configuration::getInstance().initialize(                       │
│          SYSTEM_CONFIG_PATH(__FILE__, "settings")                   │
│      );                                                             │
│  }                                                                  │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    SYSTEM_CONFIG_PATH Macro                         │
│                                                                     │
│  Input:  __FILE__ = ".../mpc_planner_jackalsimulator/src/ros1_..."  │
│  Output: ".../mpc_planner_jackalsimulator/config/settings.yaml"     │
│                                                                     │
│  Path calculation:                                                  │
│    parent_path() → .../mpc_planner_jackalsimulator/src              │
│    + "/../config/" → .../mpc_planner_jackalsimulator/config/        │
│    + "settings.yaml"                                                │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    Configuration Singleton                          │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Configuration::getInstance()                                │   │
│  │  ┌─────────────────────────────────────────────────────────┐│   │
│  │  │  static Configuration instance;  ◄── Created once       ││   │
│  │  │  YAML::Node _config;             ◄── Holds all settings ││   │
│  │  └─────────────────────────────────────────────────────────┘│   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       Global Access via CONFIG                      │
│                                                                     │
│  // Anywhere in the codebase:                                       │
│  double dt = CONFIG["integrator_step"].as<double>();                │
│  int N = CONFIG["N"].as<int>();                                     │
│  bool debug = CONFIG["debug_output"].as<bool>();                    │
│  std::string solver = CONFIG["solver_settings"]["acados"]           │
│                            ["solver_type"].as<std::string>();       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Step-by-Step Workflow

### Step 1: Application Entry Point Initializes Configuration

When the ROS node starts, the planner constructor initializes the global configuration:

```cpp
// File: mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp

JackalPlanner::JackalPlanner(ros::NodeHandle &nh)
{
    LOG_INFO("Started Jackal Planner");

    // Initialize the global Configuration singleton
    Configuration::getInstance().initialize(
        SYSTEM_CONFIG_PATH(__FILE__, "settings")
    );
    
    // Now CONFIG macro works everywhere...
}
```

### Step 2: Path Resolution via SYSTEM_CONFIG_PATH

The `SYSTEM_CONFIG_PATH` macro computes the config file path at compile time:

```cpp
// Given:
//   __FILE__ = "/workspace/src/mpc_planner/mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp"
//   filename = "settings"

// SYSTEM_CONFIG_PATH expands to:
std::filesystem::path("/workspace/.../src/ros1_jackalsimulator.cpp")
    .parent_path()                    // → "/workspace/.../src"
    .string() + "/../config/"         // → "/workspace/.../config/"
    + "settings" + ".yaml"            // → "/workspace/.../config/settings.yaml"
```

**Result**: `/workspace/src/mpc_planner/mpc_planner_jackalsimulator/config/settings.yaml`

### Step 3: YAML File Loading

The `initialize()` method calls `loadConfigYaml()` which uses `yaml-cpp` to parse the YAML file:

```cpp
void initialize(const std::string &config_file)
{
    loadConfigYaml(config_file, _config);  // Calls YAML::LoadFile internally
}
```

### Step 4: Accessing Configuration Values

After initialization, any file that includes `parameters.h` can use the `CONFIG` macro:

```cpp
#include <mpc_planner_util/parameters.h>

void someFunction()
{
    // Simple value access
    double dt = CONFIG["integrator_step"].as<double>();  // → 0.2
    int N = CONFIG["N"].as<int>();                        // → 30
    
    // Nested value access
    int iterations = CONFIG["solver_settings"]["acados"]["iterations"].as<int>();  // → 10
    
    // Boolean check
    if (CONFIG["debug_output"].as<bool>()) {
        // Debug mode enabled
    }
}
```

### Step 5: Local Config Files (Alternative Pattern)

Some modules load their own config files instead of using the global `CONFIG`:

```cpp
// File: mpc_planner_solver/src/acados_solver_interface.cpp

Solver::Solver(int solver_id)
{
    // Load module-specific configs (NOT the global CONFIG)
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "parameter_map"), _parameter_map);
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
    
    // Access local config
    nu = _config["nu"].as<unsigned int>();  // From solver_settings.yaml
    
    // But can also access global CONFIG
    dt = CONFIG["integrator_step"].as<double>();  // From settings.yaml
}
```

---

## Usage Patterns

### Pattern 1: Global Configuration Access

```cpp
// Include the header
#include <mpc_planner_util/parameters.h>

// Access values anywhere
double freq = CONFIG["control_frequency"].as<double>();
```

### Pattern 2: Nested Configuration

```yaml
# settings.yaml
solver_settings:
  acados:
    iterations: 10
    solver_type: SQP_RTI
```

```cpp
int iters = CONFIG["solver_settings"]["acados"]["iterations"].as<int>();
std::string type = CONFIG["solver_settings"]["acados"]["solver_type"].as<std::string>();
```

### Pattern 3: Conditional Debug Logging

```cpp
// Using LOG_MARK macro (defined in parameters.h)
LOG_MARK("Processing obstacle " << i);

// Equivalent to:
if (CONFIG["debug_output"].as<bool>())
    LOG_HOOK_MSG("Processing obstacle " << i);
```

### Pattern 4: Safe Access with Try-Catch

```cpp
try
{
    _assign_meaningful_topology = CONFIG["JULES"]["assign_meaningful_topology_id_to_non_guided"].as<bool>();
}
catch (...)
{
    _assign_meaningful_topology = false;  // Default if key doesn't exist
    LOG_INFO("Config parameter not found, defaulting to false");
}
```

### Pattern 5: Module-Specific Config Files

```cpp
// Load separate config files for different purposes
loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "parameter_map"), _parameter_map);
loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
```

---

## API Reference

### Macros

| Macro | Expansion | Description |
|-------|-----------|-------------|
| `CONFIG` | `Configuration::getInstance().getYAMLNode()` | Access the global YAML node |
| `SYSTEM_CONFIG_PATH(x, filename)` | `<parent(x)>/../config/<filename>.yaml` | Build path from source file location |
| `SYSTEM_CONFIG_PATH_INCLUDE(x, filename)` | `<grandparent(x)>/../config/<filename>.yaml` | Build path from header file location |
| `LOG_MARK(x)` | Conditional `LOG_HOOK_MSG(x)` | Debug logging if `debug_output` is true |

### Functions

| Function | Signature | Description |
|----------|-----------|-------------|
| `loadConfigYaml` | `void loadConfigYaml(const std::string &file, YAML::Node &_yaml_out)` | Load a YAML file into a node |

### Configuration Class

| Method | Return Type | Description |
|--------|-------------|-------------|
| `getInstance()` | `Configuration&` | Get the singleton instance |
| `initialize(config_file)` | `void` | Load the main configuration file |
| `getYAMLNode()` | `YAML::Node&` | Get the root YAML node |

### YAML::Node Access (from yaml-cpp)

| Method | Example | Description |
|--------|---------|-------------|
| `operator[]` | `CONFIG["key"]` | Access child node |
| `.as<T>()` | `node.as<int>()` | Convert to type T |

Supported types for `.as<T>()`:
- `int`, `unsigned int`
- `double`, `float`
- `bool`
- `std::string`
- `std::vector<T>`

---

## Configuration File Structure

### Main Settings File (`settings.yaml`)

```yaml
# mpc_planner_jackalsimulator/config/settings.yaml

# System identification
name: "jackal"

# MPC parameters
N: 30                    # Time horizon steps
integrator_step: 0.2     # Integration step (seconds)
control_frequency: 20    # Control loop frequency (Hz)

# Debug settings
debug_output: false      # Enable verbose logging
debug_visuals: true      # Show visualization markers

# Solver configuration
solver_settings:
  solver: "acados"       # Solver backend
  acados:
    iterations: 10
    solver_type: SQP_RTI

# Robot parameters  
robot_radius: 0.325      # meters
robot:
  length: 0.65
  width: 0.65

# Module-specific settings
t-mpc:
  use_t-mpc++: true
  enable_constraints: true

weights:
  goal: 1.0
  velocity: 0.55
  acceleration: 0.34
```

### Solver-Specific Files

```yaml
# mpc_planner_solver/config/solver_settings.yaml
N: 30
npar: 98
nu: 2
nvar: 7
nx: 5
```

```yaml
# mpc_planner_solver/config/parameter_map.yaml
acceleration: 0
angular_velocity: 1
consistency_weight: 53
# ... parameter name → index mapping
```

```yaml
# mpc_planner_solver/config/model_map.yaml
a:
- u        # type: control input
- 0        # index
- -2.0     # min bound
- 2.0      # max bound
```

---

## Examples from the Codebase

### Example 1: Initializing Robot Geometry

```cpp
// ros1_jackalsimulator.cpp
_data.robot_area = defineRobotArea(
    CONFIG["robot"]["length"].as<double>(),    // 0.65
    CONFIG["robot"]["width"].as<double>(),     // 0.65
    CONFIG["n_discs"].as<int>()                // 1
);
```

### Example 2: Setting Up Control Loop Timer

```cpp
// ros1_jackalsimulator.cpp
_timer = nh.createTimer(
    ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),  // 1/20 = 0.05s
    &JackalPlanner::loop,
    this
);
```

### Example 3: Publishing to Pedestrian Simulator

```cpp
// ros1_jackalsimulator.cpp
std_msgs::Int32 horizon_msg;
horizon_msg.data = CONFIG["N"].as<int>();  // 30
_ped_horizon_pub.publish(horizon_msg);

std_msgs::Float32 integrator_step_msg;
integrator_step_msg.data = CONFIG["integrator_step"].as<double>();  // 0.2
_ped_integrator_step_pub.publish(integrator_step_msg);
```

### Example 4: Configuring Guidance Module

```cpp
// guidance_constraints.cpp
global_guidance_->SetPlanningFrequency(CONFIG["control_frequency"].as<double>());

_use_tmpcpp = CONFIG["t-mpc"]["use_t-mpc++"].as<bool>();
_enable_constraints = CONFIG["t-mpc"]["enable_constraints"].as<bool>();
_control_frequency = CONFIG["control_frequency"].as<double>();
```

### Example 5: Mixed Local and Global Config

```cpp
// acados_solver_interface.cpp

// Local configs (module-specific)
nu = _config["nu"].as<unsigned int>();         // From solver_settings.yaml
nx = _config["nx"].as<unsigned int>();         // From solver_settings.yaml

// Global CONFIG (from settings.yaml)
dt = CONFIG["integrator_step"].as<double>();   // From settings.yaml
_num_iterations = CONFIG["solver_settings"]["acados"]["iterations"].as<int>();
```

---

## Directory Layout

```
mpc_planner_jackalsimulator/
├── config/
│   ├── settings.yaml          ◄── Main configuration file
│   └── guidance_planner.yaml
├── src/
│   └── ros1_jackalsimulator.cpp  ◄── Initializes Configuration

mpc_planner_solver/
├── config/
│   ├── solver_settings.yaml   ◄── Solver-specific config
│   ├── parameter_map.yaml     ◄── Parameter name→index mapping
│   └── model_map.yaml         ◄── Model variable definitions
├── src/
│   └── acados_solver_interface.cpp  ◄── Loads its own configs

mpc_planner_util/
├── include/mpc_planner_util/
│   ├── load_yaml.hpp          ◄── Path macros & loadConfigYaml()
│   └── parameters.h           ◄── Configuration singleton & CONFIG macro
```

---

## Key Design Decisions

1. **Singleton Pattern**: Ensures only one global configuration exists, preventing inconsistencies

2. **Compile-Time Path Resolution**: Using `__FILE__` allows source files to locate their config directory without hardcoded paths

3. **yaml-cpp Integration**: Leverages the mature yaml-cpp library for robust YAML parsing and type conversion

4. **Macro Abstraction**: The `CONFIG` macro provides clean, readable syntax while hiding singleton access complexity

5. **Separation of Concerns**: Global settings in `settings.yaml`, module-specific settings in dedicated config files

---

## Troubleshooting

### Config File Not Found

```
terminate called after throwing an instance of 'YAML::BadFile'
```

**Solution**: Verify the config file exists at the path computed by `SYSTEM_CONFIG_PATH`. Print the path:
```cpp
std::cout << SYSTEM_CONFIG_PATH(__FILE__, "settings") << std::endl;
```

### Key Not Found

```
terminate called after throwing an instance of 'YAML::InvalidNode'
```

**Solution**: Wrap in try-catch or check if key exists:
```cpp
if (CONFIG["optional_key"]) {
    value = CONFIG["optional_key"].as<double>();
}
```

### Wrong Type Conversion

```
terminate called after throwing an instance of 'YAML::TypedBadConversion<int>'
```

**Solution**: Ensure the YAML value matches the requested type. Check for quoted strings vs numbers.

---

## Summary

The `load_yaml.hpp` and `parameters.h` files form the backbone of the MPC Planner's configuration system:

- **`load_yaml.hpp`**: Provides path resolution macros (`SYSTEM_CONFIG_PATH`) and the `loadConfigYaml()` function
- **`parameters.h`**: Provides the `Configuration` singleton and the `CONFIG` macro for global access

Together, they enable:
- ✅ Centralized configuration management
- ✅ Type-safe YAML value access
- ✅ Flexible file-relative path resolution
- ✅ Clean API via the `CONFIG` macro
