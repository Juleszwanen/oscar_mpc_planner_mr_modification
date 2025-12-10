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
        // Returns 0-based index to match Vicon's 0-based object ID system
        // jackal1 -> 0, jackal2 -> 1, jackal3 -> 2, etc.
        if (ns.front() == '/')
            return std::stoi(ns.substr(7)) - 1; // skip "/jackal" and convert to 0-based
        else
            return std::stoi(ns.substr(6)) - 1; // skip "jackal" and convert to 0-based
    }

    /**
     * @brief Extract indices for non-communicating objects (e.g., humans, dynamic obstacles)
     *
     * This function generates ID indices for non-communicating entities based on the Vicon
     * bundle_obstacles.launch convention where IDs are assigned sequentially:
     * - Communicating robots: IDs [0, num_robots-1]
     * - Non-communicating objects: IDs [num_robots, num_robots+num_non_com_obj-1]
     *
     * Example 1: 2 robots (jackal1, jackal2), 1 non-communicating object
     *   - Vicon assigns: jackal1=0, jackal2=1, dynamic_object1=2
     *   - extractRobotIdFromNamespace returns: jackal1→0, jackal2→1 (0-based)
     *   - This function returns: [2] for non-comm objects
     *
     * Example 2: 3 robots, 2 non-communicating objects
     *   - Vicon assigns: jackal1=0, jackal2=1, jackal3=2, object1=3, object2=4
     *   - extractRobotIdFromNamespace returns: jackal1→0, jackal2→1, jackal3→2 (0-based)
     *   - This function returns: [3, 4] for non-comm objects
     *
     * @param all_namespaces Vector of robot namespaces (communicating entities)
     * @param num_non_com_obj Number of non-communicating objects in the scene
     * @return Vector of indices for non-communicating objects, empty if num_non_com_obj == 0
     */
    std::vector<int> extractIdentifierIndicesNonComObj(const std::vector<std::string> &all_robot_namespaces, const unsigned int &num_non_com_obj)
    {
        std::vector<int> identifier_indices_non_com_obj;

        // Non-communicating objects start after all robot IDs
        const int start_index = all_robot_namespaces.size();
        const int end_index = start_index + num_non_com_obj;

        identifier_indices_non_com_obj.reserve(num_non_com_obj); // Pre-allocate for efficiency

        // Generate sequential indices: [num_robots, num_robots + num_non_com_obj - 1]
        for (int i = start_index; i < end_index; ++i)
        {
            identifier_indices_non_com_obj.push_back(i);
        }

        return identifier_indices_non_com_obj; // Returns empty vector if num_non_com_obj == 0
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

    void transitionTo(MPCPlanner::PlannerState &_current_state, MPCPlanner::PlannerState &_previous_state, const MPCPlanner::PlannerState &new_state, const std::string &ego_robot_ns)
    {
        if (!canTransitionTo(_current_state, new_state, ego_robot_ns))
        {
            LOG_ERROR(ego_robot_ns + ": Cannot Transitioning from state: " + MPCPlanner::stateToString(_current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            MPCPlanner::PlannerState error_state = MPCPlanner::PlannerState::ERROR_STATE;
            onStateEnter(_current_state, error_state, ego_robot_ns);
            return;
        }

        onStateEnter(_current_state, new_state, ego_robot_ns);
        _previous_state = _current_state; // Track previous state before changing
        _current_state = new_state;
    }

    bool canTransitionTo(const MPCPlanner::PlannerState &_current_state, const MPCPlanner::PlannerState &new_state, const std::string &_ego_robot_ns)
    {
        // Define valid state transitions based on your robot's workflow
        switch (_current_state)
        {
        case MPCPlanner::PlannerState::UNINITIALIZED:
            // From UNINITIALIZED, can only go to TIMER_STARTUP or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::TIMER_STARTUP ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::TIMER_STARTUP:
            // From TIMER_STARTUP, can go to WAITING_FOR_FIRST_EGO_POSE or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
            // From WAITING_FOR_FIRST_EGO_POSE, can go to INITIALIZING_OBSTACLES or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::INITIALIZING_OBSTACLES ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::INITIALIZING_OBSTACLES:
            // From INITIALIZING_OBSTACLES, can go to WAITING_FOR_TRAJECTORY_DATA or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::WAITING_FOR_OTHER_ROBOTS_FIRST_POSES:
            // From WAITING_FOR_OTHER_ROBOTS_FIRST_POSES, can go to WAITING_FOR_SYNC or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::WAITING_FOR_SYNC ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::WAITING_FOR_SYNC:
            // From WAITING_FOR_SYNC, can go to WAITING_FOR_TRAJECTORY_DATA or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA:
            // From WAITING_FOR_TRAJECTORY_DATA, can go to PLANNING_ACTIVE or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::PLANNING_ACTIVE ||
                    new_state == MPCPlanner::PlannerState::GOAL_REACHED ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::PLANNING_ACTIVE:
            // From PLANNING_ACTIVE, can go to JUST_REACHED_GOAL or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::JUST_REACHED_GOAL ||
                    new_state == MPCPlanner::PlannerState::GOAL_REACHED ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::JUST_REACHED_GOAL:
            // From JUST_REACHED_GOAL, can go to GOAL_REACHED or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::GOAL_REACHED ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::GOAL_REACHED:
            // From GOAL_REACHED, can go to RESETTING or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::RESETTING ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::RESETTING:
            // From RESETTING, can go back to TIMER_STARTUP for new cycle or ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::TIMER_STARTUP ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        case MPCPlanner::PlannerState::ERROR_STATE:
            // From ERROR_STATE, can only go to RESETTING (recovery) or stay in ERROR_STATE
            return (new_state == MPCPlanner::PlannerState::RESETTING ||
                    new_state == MPCPlanner::PlannerState::ERROR_STATE);

        default:
            // Unknown current state - reject all transitions
            LOG_ERROR(_ego_robot_ns + ": Unknown current state in canTransitionTo: " +
                      std::to_string(static_cast<int>(_current_state)));
            return false;
        }
    }

    void onStateEnter(const MPCPlanner::PlannerState &current_state, const MPCPlanner::PlannerState &new_state, const std::string &ego_robot_ns)
    {
        switch (new_state)
        {

        case MPCPlanner::PlannerState::UNINITIALIZED:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::TIMER_STARTUP:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::INITIALIZING_OBSTACLES:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::WAITING_FOR_OTHER_ROBOTS_FIRST_POSES:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::WAITING_FOR_SYNC:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::PLANNING_ACTIVE:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::JUST_REACHED_GOAL:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::GOAL_REACHED:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::RESETTING:
            LOG_INFO(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        case MPCPlanner::PlannerState::ERROR_STATE:
            LOG_ERROR(ego_robot_ns + ": Transitioning from state: " + MPCPlanner::stateToString(current_state) + " to state: " + MPCPlanner::stateToString(new_state));
            break;
        default:
            LOG_WARN(ego_robot_ns + ": Unknown state transition from: " + MPCPlanner::stateToString(current_state) + " to: " + std::to_string(static_cast<int>(new_state)));
            break;
        }
    }

}