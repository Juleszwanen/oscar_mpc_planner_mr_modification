#include <mpc_planner_jackal/common/jackal_planner_initializer.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>
#include <mpc_planner_types/realtime_data.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>
#include <mpc_planner_types/multi_robot_utility_functions.h>

#include <ros_tools/logging.h>
#include <ros_tools/profiling.h>  // For RosTools::Timer and other types used in Planner

namespace JackalPlanner {

InitializationConfig JackalPlannerInitializer::loadCommonConfiguration(
    ros::NodeHandle& nh,
    const std::string& config_path)
{
    InitializationConfig config;
    
    // Initialize YAML configuration
    Configuration::getInstance().initialize(config_path);
    
    // ===== Robot Identity ===== // For the real robot this is empty
    config.ego_robot_ns = ros::this_node::getNamespace();
    
    // Allow override from parameter server // for the real robot this will be thus overwritten by the parameter set in the launch file
    nh.param("/ego_robot_ns", config.ego_robot_ns, config.ego_robot_ns);
    
    config.ego_robot_id = MultiRobot::extractRobotIdFromNamespace(config.ego_robot_ns);
    
    // ===== Multi-Robot Configuration =====
    if (!nh.getParam("/robot_ns_list", config.robot_ns_list))
    {
        LOG_ERROR(config.ego_robot_ns + ": No robot_ns_list param found");
        throw std::runtime_error("Missing required parameter: /robot_ns_list");
    }
    
    config.other_robot_nss = MultiRobot::identifyOtherRobotNamespaces(
        config.robot_ns_list, config.ego_robot_ns);
    
    // ===== Configuration Flags =====
    config.communicate_on_topology_switch_only = 
        CONFIG["JULES"]["communicate_on_topology_switch_only"].as<bool>();
    
    config.save_extra_data = CONFIG["JULES"]["safe_extra_data"].as<bool>();
    
    // Optional: enable_output (may not exist in all configs)
    if (CONFIG["enable_output"])
    {
        config.enable_output = CONFIG["enable_output"].as<bool>();
    }
    else
    {
        config.enable_output = true; // Default value
    }
    
    // ===== Physical Parameters =====
    config.robot_radius = CONFIG["robot_radius"].as<double>();
    config.control_frequency = CONFIG["control_frequency"].as<double>();
    
    // Optional: robot dimensions for detailed footprint (simulator uses this)
    if (CONFIG["robot"]["length"] && CONFIG["robot"]["width"])
    {
        config.robot_length = CONFIG["robot"]["length"].as<double>();
        config.robot_width = CONFIG["robot"]["width"].as<double>();
        config.n_discs = CONFIG["n_discs"].as<int>();
    }
    else
    {
        config.robot_length = 0.0;
        config.robot_width = 0.0;
        config.n_discs = 1; // Single disc representation
    }
    
    // Optional: infeasible deceleration (simulator uses this)
    if (CONFIG["deceleration_at_infeasible"])
    {
        config.infeasible_deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
    }
    else
    {
        config.infeasible_deceleration = 0.5; // Default value
    }
    
    // ===== Frame IDs and Tolerances =====
    nh.param("frames/global", config.global_frame, config.global_frame);
    nh.param("goal_tolerance", config.goal_tolerance, config.goal_tolerance);
    
    // ===== Multi-Robot Parameters =====
    nh.param("/num_non_com_obj", config.num_non_com_obj, config.num_non_com_obj);
    
    return config;
}

void JackalPlannerInitializer::validateConfiguration(InitializationConfig& config)
{
    // Validate deadman switch configuration
    validateDeadmanSwitchConfig(config);
    
    // Validate multi-robot setup
    validateMultiRobotConfig(config);
    
    // Validate physical parameters
    if (config.robot_radius <= 0.0)
    {
        LOG_ERROR("Invalid robot_radius: " + std::to_string(config.robot_radius));
        throw std::runtime_error("Robot radius must be positive");
    }
    
    if (config.control_frequency <= 0.0)
    {
        LOG_ERROR("Invalid control_frequency: " + std::to_string(config.control_frequency));
        throw std::runtime_error("Control frequency must be positive");
    }
    
    if (config.goal_tolerance < 0.0)
    {
        LOG_WARN("Goal tolerance is negative: " + std::to_string(config.goal_tolerance) + 
                 ", using absolute value");
        config.goal_tolerance = std::abs(config.goal_tolerance);
    }
    
    // Mark as valid
    config.is_valid = true;
}

bool JackalPlannerInitializer::initializeOtherRobotsAsObstacles(
    const std::set<std::string>& other_robot_namespaces,
    MPCPlanner::RealTimeData& data,
    double robot_radius)
{
    // Early validation
    if (other_robot_namespaces.empty())
    {
        LOG_WARN("No other robots to initialize as obstacles");
        return false;
    }

    // Constants for readability
    const Eigen::Vector2d FAR_AWAY_POSITION(100.0, 100.0);
    const Eigen::Vector2d ZERO_VELOCITY(0.0, 0.0);

    std::string summary = "Initialized obstacles for: ";

    // Initialize communicating robots
    for (const auto& robot_ns : other_robot_namespaces)
    {
        summary += robot_ns + " ";

        // Create trajectory obstacle for this robot
        data.trajectory_dynamic_obstacles.emplace(
            robot_ns,
            MPCPlanner::DynamicObstacle(
                MultiRobot::extractRobotIdFromNamespace(robot_ns),
                FAR_AWAY_POSITION,
                0.0,
                robot_radius));

        auto& traj_obs = data.trajectory_dynamic_obstacles.at(robot_ns);
        traj_obs.last_trajectory_update_time = ros::Time::now();
        traj_obs.trajectory_needs_interpolation = false;

        // Create corresponding dynamic obstacle
        MPCPlanner::DynamicObstacle dummy_obstacle = data.trajectory_dynamic_obstacles.at(robot_ns);
        data.dynamic_obstacles.push_back(dummy_obstacle);

        // Initialize with zero velocity prediction (stationary dummy obstacle)
        auto& obstacle = data.dynamic_obstacles.back();
        obstacle.prediction = MPCPlanner::getConstantVelocityPrediction(
            obstacle.position,
            ZERO_VELOCITY,
            CONFIG["integrator_step"].as<double>(),
            CONFIG["N"].as<int>());

        LOG_INFO("Created obstacle for robot " + robot_ns +
                 " with index " + std::to_string(obstacle.index));
    }

    LOG_INFO(summary);

    // Validate that initialization was successful
    const bool initialization_successful =
        !data.trajectory_dynamic_obstacles.empty() &&
        !data.dynamic_obstacles.empty() &&
        data.trajectory_dynamic_obstacles.size() == other_robot_namespaces.size();

    if (!initialization_successful)
    {
        LOG_ERROR("Failed to initialize robot obstacles properly");
    }

    return initialization_successful;
}

void JackalPlannerInitializer::logInitializationSummary(
    const InitializationConfig& config,
    const std::string& robot_ns)
{
    LOG_INFO("========== Initialization Summary ==========");
    LOG_INFO("Robot namespace: " + robot_ns);
    LOG_INFO("Robot ID: " + std::to_string(config.ego_robot_id));
    LOG_INFO("Other robots (" + std::to_string(config.other_robot_nss.size()) + "): " + 
             [&config]() {
                 std::string list;
                 for (const auto& ns : config.other_robot_nss) {
                     list += ns + " ";
                 }
                 return list.empty() ? "none" : list;
             }());
    LOG_INFO("Enable output: " + std::to_string(config.enable_output) ? "TRUE" : "FALSE");
    LOG_INFO("Control frequency: " + std::to_string(config.control_frequency) + " Hz");
    LOG_INFO("Robot radius: " + std::to_string(config.robot_radius) + " m");
    LOG_INFO("Goal tolerance: " + std::to_string(config.goal_tolerance) + " m");
    LOG_INFO("Communicate on topology switch only: " + 
             std::string(config.communicate_on_topology_switch_only ? "TRUE" : "FALSE"));
    LOG_INFO("Non-communicating objects: " + std::to_string(config.num_non_com_obj));
    LOG_INFO("Save extra data: " + std::string(config.save_extra_data ? "TRUE" : "FALSE"));
    LOG_INFO("============================================");
}

// Private validation helpers

void JackalPlannerInitializer::validateDeadmanSwitchConfig(InitializationConfig& config)
{
    // Check for conflicting deadman switches
    if (config.jules_controller_deadman_switch && config.rqt_dead_man_switch)
    {
        LOG_ERROR("FATAL: Both RQT and Jules controller deadman switches enabled! "
                  "Defaulting to Jules controller only.");
        config.rqt_dead_man_switch = false; // Force RQT to be disabled
        LOG_INFO(config.ego_robot_ns + ": Jules controller deadman switch: ENABLED, "
                 "RQT deadman switch: DISABLED (forced)");
    }
    else if (config.jules_controller_deadman_switch)
    {
        LOG_INFO(config.ego_robot_ns + ": Jules controller deadman switch: ENABLED, "
                 "RQT deadman switch: DISABLED");
    }
    else if (config.rqt_dead_man_switch)
    {
        LOG_INFO(config.ego_robot_ns + ": Jules controller deadman switch: DISABLED, "
                 "RQT deadman switch: ENABLED");
    }
    else
    {
        LOG_INFO(config.ego_robot_ns + ": Jules controller deadman switch: DISABLED, "
                 "RQT deadman switch: DISABLED (using Bluetooth/PS3 controller)");
    }
}

void JackalPlannerInitializer::validateMultiRobotConfig(const InitializationConfig& config)
{
    if (config.robot_ns_list.empty())
    {
        LOG_ERROR("robot_ns_list is empty!");
        throw std::runtime_error("Multi-robot configuration invalid: empty robot list");
    }
    
    // Check that ego robot is in the list
    bool ego_found = false;
    for (const auto& ns : config.robot_ns_list)
    {
        if (ns == config.ego_robot_ns)
        {
            ego_found = true;
            break;
        }
    }
    
    if (!ego_found)
    {
        LOG_WARN("Ego robot " + config.ego_robot_ns + " not found in robot_ns_list");
    }
    
    // Verify other_robot_nss makes sense
    if (config.robot_ns_list.size() > 1 && config.other_robot_nss.empty())
    {
        LOG_WARN("Multiple robots in list but no other robots identified");
    }
}

} // namespace JackalPlanner
