#include <mpc_planner_modules/consistency_module.h>

#include <mpc_planner_solver/mpc_planner_parameters.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals. h>
#include <ros_tools/logging.h>

namespace MPCPlanner
{

ConsistencyModule::ConsistencyModule(std::shared_ptr<Solver> solver)
    : ControllerModule(ModuleType::OBJECTIVE, solver, "consistency_module")
{
    LOG_INITIALIZE("Consistency Module");
    
    // Initialize previous trajectory storage - exactly N stages
    _prev_trajectory.resize(_solver->N);
    for (auto& pos : _prev_trajectory)
        pos = Eigen::Vector2d::Zero();
    
    _has_previous_trajectory = false;
    
    LOG_INITIALIZED();
}

void ConsistencyModule::update(State &state, const RealTimeData &data, 
                                ModuleData &module_data)
{
    (void)data;
    (void)module_data;
    
    if (_has_previous_trajectory)
    {
        // Shift the previous trajectory by 1 timestep
        // What was k=1 becomes k=0, etc.  (because time has advanced)
        for (int k = 0; k < _solver->N; k++)
        {
            if (k < _solver->N - 1)
            {
                _prev_trajectory[k](0) = _solver->getOutput(k + 1, "x");
                _prev_trajectory[k](1) = _solver->getOutput(k + 1, "y");
            }
            else
            {
                // For the last stage, repeat the last known position
                _prev_trajectory[k] = _prev_trajectory[k - 1];
            }
        }
    }
    else
    {
        // First iteration: fill with current state as dummy data
        // The weight will be set to 0, so these values won't matter
        for (int k = 0; k < _solver->N; k++)
        {
            _prev_trajectory[k](0) = state. get("x");
            _prev_trajectory[k](1) = state. get("y");
        }
    }
}

void ConsistencyModule::setParameters(const RealTimeData &data, 
                                       const ModuleData &module_data, int k)
{
    (void)data;
    (void)module_data;
    
    // Get weight from config
    double consistency_weight = CONFIG["weights"]["consistency_weight"].as<double>();
    
    // If no previous trajectory exists, set weight to 0 (disable the cost)
    if (! _has_previous_trajectory)
    {
        consistency_weight = 0.0;
    }
    
    setSolverParameterConsistencyWeight(k, _solver->_params, consistency_weight);
    
    // Set the previous trajectory position for stage k
    // (these values are ignored when weight is 0, but we set them for consistency)
    setSolverParameterPrevTrajX(k, _solver->_params, _prev_trajectory[k](0));
    setSolverParameterPrevTrajY(k, _solver->_params, _prev_trajectory[k](1));
}

void ConsistencyModule::onDataReceived(RealTimeData &data, std::string &&data_name)
{
    (void)data;
    (void)data_name;
    // Not used for this module
}

void ConsistencyModule::postSolve(const State &state, const RealTimeData &data,
                                   ModuleData &module_data)
{
    (void)state;
    (void)data;
    (void)module_data;
    
    // After first successful solve, we have a trajectory to use next iteration
    if (!_has_previous_trajectory)
    {
        _has_previous_trajectory = true;
        LOG_INFO("Consistency Module: First trajectory stored, will apply consistency cost next iteration");
    }
}

void ConsistencyModule::visualize(const RealTimeData &data, 
                                   const ModuleData &module_data)
{
    (void)data;
    (void)module_data;
    
    if (!_has_previous_trajectory)
        return;
    
    // Visualize the previous trajectory as reference
    auto &publisher = VISUALS. getPublisher(_name + "/prev_trajectory");
    auto &points = publisher.getNewPointMarker("SPHERE");
    
    points.setColor(0.5, 0.5, 1.0, 0.5);  // Light blue, semi-transparent
    points.setScale(0.15, 0.15, 0.15);
    
    for (int k = 0; k < _solver->N; k++)
    {
        points.addPointMarker(_prev_trajectory[k]);
    }
    
    publisher.publish();
}

bool ConsistencyModule::isDataReady(const RealTimeData &data, 
                                     std::string &missing_data)
{
    (void)data;
    (void)missing_data;
    // Always ready - we handle the "no previous trajectory" case with weight=0
    return true;
}

void ConsistencyModule::reset()
{
    _has_previous_trajectory = false;
    for (auto& pos : _prev_trajectory)
        pos = Eigen::Vector2d::Zero();
}

} // namespace MPCPlanner