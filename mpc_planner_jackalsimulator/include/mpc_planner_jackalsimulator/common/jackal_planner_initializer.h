#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <set>

namespace MPCPlanner {
    class Planner;
    struct RealTimeData;
}

namespace JackalPlanner {

/**
 * @brief Configuration structure for Jackal planner initialization
 * 
 * Holds all configuration parameters needed to initialize both real and simulated
 * Jackal planners. Platform-specific parameters are optional and populated by
 * the respective platform classes.
 */
struct InitializationConfig
{
    // ===== Robot Identity =====
    std::string ego_robot_ns;
    int ego_robot_id;
    std::vector<std::string> robot_ns_list;
    std::set<std::string> other_robot_nss;
    
    // ===== Configuration Flags =====
    bool communicate_on_topology_switch_only;
    bool enable_output;
    bool save_extra_data;
    
    // ===== Physical Parameters =====
    double robot_radius;
    double robot_length;
    double robot_width;
    int n_discs;
    double control_frequency;
    double goal_tolerance;
    double infeasible_deceleration;
    
    // ===== Multi-Robot =====
    int num_non_com_obj;
    
    // ===== Frame IDs =====
    std::string global_frame;
    
    // ===== Platform-Specific (Real Hardware) =====
    bool forward_x_experiment = false;
    bool rqt_dead_man_switch = false;
    bool jules_controller_deadman_switch = false;
    
    // ===== Validation State =====
    bool is_valid = false;
};

/**
 * @brief Utility class for Jackal planner initialization
 * 
 * Provides static methods to load, validate, and initialize common components
 * shared between real and simulated Jackal planners. This promotes code reuse
 * and maintainability by centralizing initialization logic.
 * 
 * Design Decision: Static methods are used because:
 * - No internal state needed between calls
 * - Purely utility/helper functionality
 * - Acts as organized namespace with encapsulation
 * - Thread-safe by default (no shared state)
 * - No object lifecycle management needed
 */
class JackalPlannerInitializer
{
public:
    JackalPlannerInitializer() = delete; // Prevent instantiation
    
    /**
     * @brief Load common configuration from ROS parameters and YAML
     * 
     * Loads configuration shared between real and simulation planners including:
     * - Robot namespace and multi-robot setup
     * - Physical parameters (radius, frequency, etc.)
     * - Communication settings
     * - Frame IDs and tolerances
     * 
     * @param nh ROS node handle for parameter access
     * @param config_path Path to YAML config file (use SYSTEM_CONFIG_PATH macro)
     * @return InitializationConfig populated with common parameters
     * @throws std::runtime_error if critical parameters are missing
     */
    static InitializationConfig loadCommonConfiguration(
        ros::NodeHandle& nh,
        const std::string& config_path);
    
    /**
     * @brief Validate configuration and check for conflicts
     * 
     * Performs validation including:
     * - Deadman switch configuration conflicts
     * - Multi-robot setup consistency
     * - Physical parameter sanity checks
     * 
     * Sets config.is_valid = true if all checks pass.
     * 
     * @param config Configuration to validate (modified in-place)
     * @throws std::runtime_error if validation fails critically
     */
    static void validateConfiguration(InitializationConfig& config);
    
    /**
     * @brief Initialize other robots as obstacles in RealTimeData
     * 
     * Sets up obstacle tracking for all other robots in the multi-robot system.
     * Each robot is represented as a circular obstacle with the given radius.
     * 
     * @param other_robot_nss Set of other robot namespaces
     * @param data RealTimeData to populate with obstacle information
     * @param robot_radius Radius for obstacle representation
     * @return true if successful, false otherwise
     */
    static bool initializeOtherRobotsAsObstacles(
        const std::set<std::string>& other_robot_nss,
        MPCPlanner::RealTimeData& data,
        double robot_radius);
    
    /**
     * @brief Log initialization summary for debugging
     * 
     * Outputs a comprehensive summary of the initialization configuration
     * to help with debugging and verification.
     * 
     * @param config Configuration to log
     * @param robot_ns Robot namespace for logging context
     */
    static void logInitializationSummary(
        const InitializationConfig& config,
        const std::string& robot_ns);

private:
    /**
     * @brief Validate deadman switch configuration for conflicts
     * 
     * Ensures only one deadman switch mechanism is enabled at a time.
     * If multiple are enabled, logs error and disables conflicting switches.
     * 
     * @param config Configuration to validate (modified in-place)
     */
    static void validateDeadmanSwitchConfig(InitializationConfig& config);
    
    /**
     * @brief Validate multi-robot configuration
     * 
     * Checks that robot namespace list is properly configured and
     * other robot namespaces are correctly identified.
     * 
     * @param config Configuration to validate
     * @throws std::runtime_error if multi-robot setup is invalid
     */
    static void validateMultiRobotConfig(const InitializationConfig& config);
};

} // namespace JackalPlanner
