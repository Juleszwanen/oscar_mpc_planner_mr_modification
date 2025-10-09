#pragma once
#include <string>
#include <set>
#include <vector>
#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/realtime_data.h>

namespace MultiRobot
{
    std::set<std::string> identifyOtherRobotNamespaces(const std::vector<std::string> &all_namespaces, const std::string &_ego_robot_ns);

    int extractRobotIdFromNamespace(const std::string &ns);

    std::string dataToString(const MPCPlanner::RealTimeData& _data, const std::string& _ego_robot_ns);

}