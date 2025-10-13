#include "mpc_planner_solver/state.h"

#include <ros_tools/logging.h>

using namespace MPCPlanner;

State::State()
{
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
    initialize();
}

void State::initialize()
{
    _state = std::vector<double>(_config["nx"].as<int>(), 0.0);
    _nu = _config["nu"].as<int>();
}

double State::get(std::string &&var_name) const
{
    return _state[_model_map[var_name][1].as<int>() - _nu]; // States come after the inputs
}

Eigen::Vector2d State::getPos() const
{
    return Eigen::Vector2d(get("x"), get("y"));
}

void State::set(std::string &&var_name, double value)
{
    _state[_model_map[var_name][1].as<int>() - _nu] = value;
}

void State::print() const
{
    for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it)
    {
        if (it->second[0].as<std::string>() == "x")
        {
            LOG_VALUE(it->first.as<std::string>(), get(it->first.as<std::string>()));
        }
    }
}

/** @note Jules: This function can be used to check if the localicazation mechanism has inserted valid data via the for instance statePoseCallback*/
bool State::validData() const
{
    // Get the state values using the existing get() method
    const double x = get("x");
    const double y = get("y");
    const double psi = get("psi");
    const double v = get("v");
    
    // Check for uninitialized values (default is 0.0 from initialize())
    // In robotics, it's extremely unlikely that x, y, and psi are all exactly 0.0
    // unless they haven't been set yet
    
    // Basic validity checks:
    // 1. Position shouldn't be exactly (0,0) unless robot actually started there
    // 2. All values should be finite (not NaN or infinity)
    // 3. Velocity should be reasonable (not negative for this robot model)
    
    // Check for NaN or infinity values
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(psi) || !std::isfinite(v)) {
        return false;
    }

    // The state is initialized with zeros
    if (x == 0.0 && y == 0.0 && psi == 0.0 && v == 0.0) {
        return false;
    }

    return true;
}