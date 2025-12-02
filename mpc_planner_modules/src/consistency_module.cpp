#include <mpc_planner_modules/consistency_module.h>

#include <mpc_planner_solver/mpc_planner_parameters.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>

namespace MPCPlanner
{

ConsistencyModule::ConsistencyModule(std::shared_ptr<Solver> solver)
    : ControllerModule(ModuleType::OBJECTIVE, solver, "consistency_module")
{
    LOG_INITIALIZE("Consistency Module");
    LOG_INITIALIZED();
    
}

void ConsistencyModule::update(State &state, const RealTimeData &data, 
                                ModuleData &module_data)
{
    (void)data;
    (void)module_data;
    (void)module_data;
}

void ConsistencyModule::setParameters(const RealTimeData &data, 
                                       const ModuleData &module_data, int k)
{
    (void)data;
    (void)module_data;
    (void)k;
    LOG_DEBUG("NOT USING setParameters of module here, defined in guidance_constraints.cpp");
}



void ConsistencyModule::visualize(const RealTimeData &data, 
                                   const ModuleData &module_data)
{
    (void)data;
    (void)module_data;
    
    // if (!_has_previous_trajectory)
    //     return;
    
    // // Visualize the previous trajectory as reference
    // auto &publisher = VISUALS. getPublisher(_name + "/prev_trajectory");
    // auto &points = publisher.getNewPointMarker("SPHERE");
    
    // points.setColor(0.5, 0.5, 1.0, 0.5);  // Light blue, semi-transparent
    // points.setScale(0.15, 0.15, 0.15);
    
    // for (int k = 0; k < _solver->N; k++)
    // {
    //     points.addPointMarker(_prev_trajectory[k]);
    // }
    
    // publisher.publish();
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
    // _has_previous_trajectory = false;
    // for (auto& pos : _prev_trajectory)
    //     pos = Eigen::Vector2d::Zero();
}

} // namespace MPCPlanner