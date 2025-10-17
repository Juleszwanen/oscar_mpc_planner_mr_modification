#pragma once
#define _USE_MATH_DEFINES
#include <string>
#include <cmath>
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
    std::string dataToString(const MPCPlanner::RealTimeData &_data, const std::string &_ego_robot_ns);

    // Angle interpolation (shortest arc and in the correct direction)
    // Interpolate between two angles using shortest arc
    // psi_k: start angle (radians)
    // psi_kl: end angle (radians)  
    // alpha: interpolation factor [0, 1]
    // Returns: interpolated angle in [-π, π]
    double interpolateAngle(double psi_k, double psi_kl, double alpha);

    // Normalize angle to [-π, π]
    double wrapAngle(double angle);

    // Wrap angle difference to shortest arc [-π, π]
    double wrapAngleDifference(double angle);

}