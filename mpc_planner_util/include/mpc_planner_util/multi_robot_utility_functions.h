#pragma once
#include <string>
#include <set>
#include <vector>
#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/realtime_data.h>

namespace MultiRobot
{
    // Returns a set of all the of other robots in the environment excluding the namespace of the _ego_robots
    std::set<std::string> identifyOtherRobotNamespaces(const std::vector<std::string> &all_namespaces, const std::string &_ego_robot_ns);

    // This function returns the id which corresponds to a robot, for instance /jackal1 results in 1
    int extractRobotIdFromNamespace(const std::string &ns);

    // This function returns a string of data based on the _data member variable of a planner instance, this can then be used for logging purposes.
    std::string dataToString(const MPCPlanner::RealTimeData& _data, const std::string& _ego_robot_ns);

}