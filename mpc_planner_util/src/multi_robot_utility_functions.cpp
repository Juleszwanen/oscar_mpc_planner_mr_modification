#include <mpc_planner_util/multi_robot_utility_functions.h>
#include <ros_tools/logging.h>
#include <numeric>
#include <algorithm>

namespace MultiRobot
{
    std::set<std::string> identifyOtherRobotNamespaces(const std::vector<std::string> &all_namespaces, const std::string &_ego_robot_ns)
    {
        std::set<std::string> other_robot_namespaces;
        for (const auto &robot_name : all_namespaces)
        {
            if (_ego_robot_ns != robot_name)
            {
                other_robot_namespaces.insert(robot_name);
                LOG_INFO("For ego_robot " + _ego_robot_ns + " found other robot " + robot_name);
            }
        }
        return other_robot_namespaces;
    }

    int extractRobotIdFromNamespace(const std::string &ns)
    {
        // Handle both "/jackalX" and "jackalX" formats
        if (ns.front() == '/')
            return std::stoi(ns.substr(7)); // skip "/jackal"
        else
            return std::stoi(ns.substr(6)); // skip "jackal"
    }

    std::string dataToString(const MPCPlanner::RealTimeData &_data, const std::string &_ego_robot_ns)
    {
        std::string result = _ego_robot_ns + " RealTimeData{\n";

        // Basic info
        result += "  robot_area: " + std::to_string(_data.robot_area.size()) + " discs\n";
        result += "  goal: [" + std::to_string(_data.goal.x()) + ", " + std::to_string(_data.goal.y()) + "]\n";
        result += "  goal_received: " + std::string(_data.goal_received ? "true" : "false") + "\n";
        result += "  past_trajectory_size: " + std::to_string(_data.past_trajectory.positions.size()) + "\n";
        result += "  reference_path_size: " + std::to_string(_data.reference_path.x.size()) + "\n";
        result += "  intrusion: " + std::to_string(_data.intrusion) + "\n";

        // Dynamic obstacles using accumulate pattern (limit to first 3 for readability)
        result += "  dynamic_obstacles: [" + std::to_string(_data.dynamic_obstacles.size()) + "] {\n";
        if (!_data.dynamic_obstacles.empty())
        {
            std::string obstacles_str = std::accumulate(
                _data.dynamic_obstacles.begin(),
                std::min(_data.dynamic_obstacles.end(), _data.dynamic_obstacles.begin() + 3), // Limit to first 3
                std::string(),
                [](const std::string &s, const MPCPlanner::DynamicObstacle &obs)
                {
                    return s + (s.empty() ? "" : ",\n") + "    " + obs.toString();
                });
            result += obstacles_str;
            if (_data.dynamic_obstacles.size() > 3)
            {
                result += "\n    ...(+" + std::to_string(_data.dynamic_obstacles.size() - 3) + " more obstacles)";
            }
            result += "\n";
        }
        result += "  }\n";

        // Trajectory dynamic obstacles (multi-robot specific)
        result += "  trajectory_dynamic_obstacles: [" + std::to_string(_data.trajectory_dynamic_obstacles.size()) + "] {\n";
        if (!_data.trajectory_dynamic_obstacles.empty())
        {
            std::string traj_obstacles_str = std::accumulate(
                _data.trajectory_dynamic_obstacles.begin(),
                _data.trajectory_dynamic_obstacles.end(),
                std::string(),
                [](const std::string &s, const std::pair<std::string, MPCPlanner::DynamicObstacle> &pair)
                {
                    return s + (s.empty() ? "" : ",\n") + "    [" + pair.first + "]: " + pair.second.toString();
                });
            result += traj_obstacles_str + "\n";
        }
        result += "  }\n";

        result += "}";
        return result;
    }

    double interpolateAngle(double psi_k, double psi_kl, double alpha)
    {
        // Compute angle difference
        double diff = psi_kl - psi_k; //

        // Wrap to shortest arc [-π, π]
        double wrapped_diff = wrapAngleDifference(diff);

        // Interpolate along shortest arc
        double psi_interp = psi_k + wrapped_diff * alpha;

        // Wrap result back to [-π, π]
        return wrapAngle(psi_interp);
    }

    // Both have the same implementation:
    double wrapAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    double wrapAngleDifference(double diff)
    {
        // Semantically the same as wrapAngle, but used for differences
        // Ensures shortest arc representation
        return wrapAngle(diff);
    }

}