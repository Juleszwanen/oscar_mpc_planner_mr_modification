#include <mpc_planner_jackalsimulator/jules_ros1_jackalplanner.h>
#include <mpc_planner_jackalsimulator/common/jackal_planner_initializer.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/load_yaml.hpp> // must come first
#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/multi_robot_utility_functions.h>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>

#include <numeric>
#include <algorithm>

// 1. CORE LIFECYCLE FUNCTIONS
//    - Constructor, destructor, main entry point
// Constructor: Initialize MPC planner with multi-robot coordination
JulesJackalPlanner::JulesJackalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    LOG_HEADER("JULES: Jackal planner " + ros::this_node::getName() + " starting");

    // Load common configuration using initializer
    std::string config_path = SYSTEM_CONFIG_PATH(__FILE__, "settings");
    auto config = JackalPlanner::JackalPlannerInitializer::loadCommonConfiguration(
        nh, config_path);
    
    // Validate configuration
    JackalPlanner::JackalPlannerInitializer::validateConfiguration(config);
    
    // Apply configuration to member variables
    applyConfiguration(config);
    
    // Define robot footprint from CONFIG (simulator uses detailed footprint)
    _data.robot_area = MPCPlanner::defineRobotArea(
        config.robot_length,
        config.robot_width,
        config.n_discs);
    
    // Initialize common components using shared initializer
    bool initialization_successful = JackalPlanner::JackalPlannerInitializer::initializeOtherRobotsAsObstacles(
        config.other_robot_nss, _data, config.robot_radius);
    
    LOG_INFO("DEBUG: About to create MPCPlanner::Planner...");
    _planner = std::make_unique<MPCPlanner::Planner>(
        config.ego_robot_ns,
        config.save_extra_data);
    LOG_INFO("DEBUG: MPCPlanner::Planner created successfully!");
    
    // Initialize platform-specific components
    LOG_INFO("DEBUG: About to initialize subscribers and publishers...");
    initializeSubscribersAndPublishers(nh, pnh);
    LOG_INFO("DEBUG: Subscribers and publishers initialized!");
    initializeSimulatorComponents(nh, config);
    initializeTimersAndStateMachine(nh, config);
    
    // Log summary
    JackalPlanner::JackalPlannerInitializer::logInitializationSummary(config, _ego_robot_ns);
    LOG_DIVIDER();
}

JulesJackalPlanner::~JulesJackalPlanner()
{
    LOG_INFO(_ego_robot_ns + ": Stopped JulesJackalPlanner");
}

// ===== INITIALIZATION HELPER METHODS =====

void JulesJackalPlanner::applyConfiguration(
    const JackalPlanner::InitializationConfig& config)
{
    _ego_robot_ns = config.ego_robot_ns;
    _ego_robot_id = config.ego_robot_id;
    _robot_ns_list = config.robot_ns_list;
    _other_robot_nss = config.other_robot_nss;
    _control_frequency = config.control_frequency;
    _enable_output = config.enable_output;
    _infeasible_deceleration = config.infeasible_deceleration;
    _communicate_on_topology_switch_only = config.communicate_on_topology_switch_only;
    _goal_tolerance = config.goal_tolerance;
    _global_frame = config.global_frame;
}

void JulesJackalPlanner::initializeSimulatorComponents(
    ros::NodeHandle& nh,
    const JackalPlanner::InitializationConfig& config)
{
    _reconfigure = std::make_unique<JackalsimulatorReconfigure>();
    
}

void JulesJackalPlanner::initializeTimersAndStateMachine(
    ros::NodeHandle& nh,
    const JackalPlanner::InitializationConfig& config)
{
    _startup_timer = std::make_unique<RosTools::Timer>();
    _startup_timer->setDuration(10.0);
    _startup_timer->start();
    
    MultiRobot::transitionTo(_current_state, _previous_state, 
        MPCPlanner::PlannerState::TIMER_STARTUP, _ego_robot_ns);
    
    _timer = nh.createTimer(
        ros::Duration(1.0 / config.control_frequency),
        &JulesJackalPlanner::loopDirectTrajectoryStateMachine,
        this);
}

// ===== END INITIALIZATION HELPER METHODS =====

// 2. ROS COMMUNICATION SETUP
//    - Functions that initialize publishers, subscribers, services
// Initialize ROS communication (subscribers and publishers)
void JulesJackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh, ros::NodeHandle & /*pnh*/)
{
    LOG_INFO(_ego_robot_ns + ": initializeSubscribersAndPublishers");

    /** @note Some Topics are mapped in the launch file! */
    // Input subscribers
    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "input/state", 5,
        boost::bind(&JulesJackalPlanner::stateCallback, this, _1));

    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "input/state_pose", 1,
        boost::bind(&JulesJackalPlanner::statePoseCallback, this, _1));

    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "input/goal", 1,
        boost::bind(&JulesJackalPlanner::goalCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>(
        "input/reference_path", 1,
        boost::bind(&JulesJackalPlanner::pathCallback, this, _1));

    _obstacles_sub = nh.subscribe<mpc_planner_msgs::ObstacleArray>(
        "input/obstacles", 1,
        boost::bind(&JulesJackalPlanner::obstacleCallback, this, _1));

    _all_robots_reached_objective_sub = nh.subscribe<std_msgs::Bool>(
        "/all_robots_reached_objective", 1,
        boost::bind(&JulesJackalPlanner::allRobotsReachedObjectiveCallback, this, _1));

    // Subscribe to other robots
    this->subscribeToOtherRobotTopics(nh, _other_robot_nss);

    // Service client for trajectory requests
    _trajectory_client = nh.serviceClient<mpc_planner_msgs::GetOtherTrajectories>("/get_other_robot_obstacles_srv");

    // Output publishers
    _cmd_pub = nh.advertise<geometry_msgs::Twist>("output/command", 1);
    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("output/pose", 1);
    _trajectory_pub = nh.advertise<nav_msgs::Path>("output/current_trajectory", 1);
    _direct_trajectory_pub = nh.advertise<mpc_planner_msgs::ObstacleGMM>("robot_to_robot/output/current_trajectory", 1);
    _objective_pub = nh.advertise<std_msgs::Bool>("events/objective_reached", 1);

    // Environment resets
    _reset_simulation_pub = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);
    _reset_simulation_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    // Roadmap reverse
    _reverse_roadmap_pub = nh.advertise<std_msgs::Empty>("roadmap/reverse", 1);
}
void JulesJackalPlanner::subscribeToOtherRobotTopics(ros::NodeHandle &nh, const std::set<std::string> &other_robot_namespaces)
{
    // Clear existing subscribers for safety
    this->_other_robot_pose_sub_list.clear();
    this->_other_robot_trajectory_sub_list.clear();

    // Reserve space for efficiency
    this->_other_robot_pose_sub_list.reserve(other_robot_namespaces.size());
    this->_other_robot_trajectory_sub_list.reserve(other_robot_namespaces.size());

    // Create subscribers for each robot's pose and trajectory topics
    for (const auto &ns : other_robot_namespaces)
    {
        // Subscribe to robot pose updates
        const std::string topic_pose = ns + "/robot_to_robot/output/pose";
        LOG_INFO(_ego_robot_ns + " is subscribing to: " + topic_pose);
        auto sub_pose_i = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose, 1,
                                                                   boost::bind(&JulesJackalPlanner::poseOtherRobotCallback, this, _1, ns));
        this->_other_robot_pose_sub_list.push_back(sub_pose_i);

        // Subscribe to robot trajectory predictions
        const std::string topic_trajectory = ns + "/robot_to_robot/output/current_trajectory";
        LOG_INFO(_ego_robot_ns + " is subscribing to: " + topic_trajectory);
        auto sub_traject_i = nh.subscribe<mpc_planner_msgs::ObstacleGMM>(topic_trajectory, 1,
                                                                         boost::bind(&JulesJackalPlanner::trajectoryCallback, this, _1, ns));
        this->_other_robot_trajectory_sub_list.push_back(sub_traject_i);
    }
}

// 3. MAIN CONTROL LOOP FUNCTIONS
void JulesJackalPlanner::loopDirectTrajectoryStateMachine(const ros::TimerEvent & /*event*/)
{
    LOG_DEBUG(_ego_robot_ns + "============= Loop Start =============");
    LOG_DEBUG(_ego_robot_ns + ": Obstacles: dynamic=" + std::to_string(_data.dynamic_obstacles.size()) +
              ", trajectory=" + std::to_string(_data.trajectory_dynamic_obstacles.size()));

    _data.planning_start_time = std::chrono::system_clock::now();

    switch (_current_state)
    {
    case MPCPlanner::PlannerState::UNINITIALIZED:
    {
        LOG_WARN(_ego_robot_ns + ": In UNINITIALIZED state - this should not happen during planning loop");
        break;
    }
    case MPCPlanner::PlannerState::TIMER_STARTUP:
    {
        if (!_startup_timer->hasFinished())
        {
            LOG_INFO_THROTTLE(2000, _ego_robot_ns + ": In startup period, skipping planning");
            return;
        }
        else
        {
            MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE, _ego_robot_ns);
        }

        break;
    }

    // We need to know if the poseCallback has gone of, it has gone of if the _state has valid data
    case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
    {
        if (_state.validData())
        {
            MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::INITIALIZING_OBSTACLES, _ego_robot_ns);
        }
        else
        {
            LOG_ERROR(_ego_robot_ns + ": INVALID _state PLANNING (waiting for correct pose callback update) - State: [" +
                      "x: " + std::to_string(_state.get("x")) + ", " +
                      "y: " + std::to_string(_state.get("y")) + ", " +
                      "psi: " + std::to_string(_state.get("psi")) + ", " +
                      "v: " + std::to_string(_state.get("v")) + "]");
        }
        break;
    }
    case MPCPlanner::PlannerState::INITIALIZING_OBSTACLES:
    {
        bool initialization_successful = initializeOtherRobotsAsObstacles(_other_robot_nss, _data, CONFIG["robot_radius"].as<double>());
        MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA, _ego_robot_ns);
        break;
    }
    case MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA:
    {
        // While we are waiting for the first trajectory data what we can do is plan with the dummy obstacles which we initialized in initializeOtherRobotsAsObstacles
        LOG_INFO(_ego_robot_ns + ": INITIAL PLANNING (waiting for trajectory data) - State: [" +
                 std::to_string(_state.get("x")) + ", " +
                 std::to_string(_state.get("y")) + ", " +
                 std::to_string(_state.get("psi")) + "]");

        prepareObstacleData();
        auto [cmd, output] = generatePlanningCommand(_current_state);

        publishCmdAndVisualize(cmd, output);
        _data.past_trajectory.replaceTrajectory(output.trajectory);
        // The state transition is be triggered by a trajectory callback function
        break;
    }
    case MPCPlanner::PlannerState::PLANNING_ACTIVE: // This state is transitioned to by the trajectory callback function
    {
        // Use braces to create a new scope for variable declarations
        if (this->objectiveReached(_state, _data))
        {
            MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::JUST_REACHED_GOAL, _ego_robot_ns);
        }

        prepareObstacleData();
        auto [cmd, output] = generatePlanningCommand(_current_state);
        // LOG_INFO(_ego_robot_ns + output.logOutput());
        publishCmdAndVisualize(cmd, output);
        _data.past_trajectory.replaceTrajectory(output.trajectory);
        break;
    }
    case MPCPlanner::PlannerState::JUST_REACHED_GOAL:
    {
        // Use braces to create a new scope for variable declarations
        prepareObstacleData();
        auto [cmd, output] = generatePlanningCommand(_current_state);
        _data.past_trajectory.replaceTrajectory(output.trajectory);
        publishCmdAndVisualize(cmd, output);
        MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::GOAL_REACHED, _ego_robot_ns);
        break;
    }
    case MPCPlanner::PlannerState::GOAL_REACHED:
    {
        // Use braces to create a new scope for variable declarations
        prepareObstacleData();
        auto [cmd, output] = generatePlanningCommand(_current_state);
        _data.past_trajectory.replaceTrajectory(output.trajectory);
        publishCmdAndVisualize(cmd, output);
        break;
    }
    case MPCPlanner::PlannerState::RESETTING:
    {
        LOG_INFO_THROTTLE(2000, _ego_robot_ns + ": In resetting state, waiting for allRobotsReachedObjecitveCallback... to change _ego state to TIMER_STARTUP");
        break;
    }
    case MPCPlanner::PlannerState::ERROR_STATE:
    {
        LOG_ERROR(_ego_robot_ns + ": In error state - stopping planning");
        return;
    }
    default:
    {
        LOG_ERROR(_ego_robot_ns + ": UNKNOWN STATE: " + std::to_string(static_cast<int>(_current_state)));
        break;
    }
    }
    if (CONFIG["recording"]["enable"].as<bool>())
        _planner->saveData(_state, _data);
    LOG_DEBUG(_ego_robot_ns + ": ============= Loop End =============");
}
void JulesJackalPlanner::applyBrakingCommand(geometry_msgs::Twist &cmd)
{
    double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
    double dt = 1. / CONFIG["control_frequency"].as<double>();
    double velocity = _state.get("v");
    double velocity_after_braking = velocity - deceleration * dt;

    cmd.linear.x = std::max(velocity_after_braking, 0.0); // Don't drive backwards
    cmd.angular.z = 0.0;
}

// 4. ROS CALLBACK FUNCTIONS
//    - All subscriber callbacks grouped by data type
void JulesJackalPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    _state.set("v", std::hypot(vx, vy));

    // Flip detection
    if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) ||
        std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
    {
        LOG_WARN("Detected flipped robot (odom).");
    }
}
void JulesJackalPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Write planar pose directly from the encoded message fields.
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z); // encoded yaw (NOT a quaternion)

    // Linear speed v is encoded in position.z by upstream publisher.
    _state.set("v", msg->pose.position.z);

    // Flip detection: large roll/pitch implies the robot may have flipped.
    // NOTE: This only makes sense if `orientation` is a real quaternion.
    // With the encoded format, x/y are usually 0 and this check won’t trigger.
    if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) ||
        std::abs(msg->pose.orientation.y) > (M_PI / 8.))
    {
        LOG_ERROR("Detected flipped robot. Resetting.");

        // In the original single-robot setup this hard-resets the sim.
        // In a multi-robot setup, prefer publishing an event and let a
        // Supervisor decide what to reset.
        // reset(false); // Reset without success
    }

    // switch (_current_state)
    // {
    // case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
    //     MultiRobot::transitionTo(MPCPlanner::PlannerState::WAITING_FOR_SYNC);
    //     publishEgoPose();
    //     break;
    // default:
    //     break;
    // }
}
void JulesJackalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    LOG_DEBUG("Goal callback");

    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _goal_xy = Eigen::Vector2d(_data.goal(0), _data.goal(1));

    _data.goal_received = true;
    _goal_received = true;
}
void JulesJackalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    if (isPathTheSame(msg))
        return;

    _data.reference_path.clear();

    for (const auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
        _data.reference_path.psi.push_back(_state.get("psi"));
    }

    /** @note Oscar had instead of pushback "psi" oscar had 0.0, why not clear? */
    // _data.reference_path.psi.push_back(0.0);
    _planner->onDataReceived(_data, "reference_path");
}
void JulesJackalPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
{
    /** @todo This does not work right now because  _have_received_meaningful_trajectory_data does not represent the timercallback in centralAggregator, perhaps think of a fix solution*/
    // Start fresh each message; the solver expects the latest snapshot.
    // In obstacleCallback at the very start:
    if (!_have_received_meaningful_trajectory_data)
    {
        LOG_WARN(_ego_robot_ns + ": ObstacleCallback came too early Abording this call");
        return;
    }

    if (_received_obstacle_callback_first_time)
    {
        LOG_HEADER(_ego_robot_ns + ": ObstacleCallback running correctly for the first time");
        _received_obstacle_callback_first_time = false;
    }

    LOG_DEBUG("OBSTACLE_CALLBACK: received " + std::to_string(msg->obstacles.size()) + " obstacles");

    const std::string profiling_name = _ego_robot_ns + "_" + "ObstacleCallback";
    PROFILE_SCOPE(profiling_name.c_str());
    // LOG_DEBUG(profiling_name);

    _data.dynamic_obstacles.clear();

    for (const auto &obstacle : msg->obstacles)
    {
        // Create a new dynamic obstacle with:
        //  - id:      stable identifier from upstream (aggregator/ped sim)
        //  - pos:     current xy position
        //  - yaw:     orientation (used by some footprint models)
        //  - radius:  planner-wide default unless you encode per-obstacle sizes
        _data.dynamic_obstacles.emplace_back(
            obstacle.id,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            RosTools::quaternionToAngle(obstacle.pose),
            CONFIG["obstacle_radius"].as<double>());

        auto &dyn = _data.dynamic_obstacles.back(); // ret

        // Single-mode Gaussian predictions if provided; otherwise deterministic.
        // We treat "has probabilities AND exactly one gaussian" as "one-mode"
        // and fill per-step means + ellipse semiaxes. The actual probability
        // values are not used here; they are interpreted downstream.
        if (!obstacle.probabilities.empty() && obstacle.gaussians.size() == 1)
        {
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::GAUSSIAN);

            const auto &mode = obstacle.gaussians[0];
            dyn.prediction.modes[0].reserve(mode.mean.poses.size());
            for (size_t k = 0; k < mode.mean.poses.size(); ++k)
            {
                dyn.prediction.modes[0].emplace_back(
                    Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                    RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                    mode.major_semiaxis[k],
                    mode.minor_semiaxis[k]);
            }

            // If uncertainty is effectively zero or probabilistic handling is
            // disabled in config, fall back to deterministic to simplify constraints.
            if (mode.major_semiaxis.back() == 0.0 ||
                !CONFIG["probabilistic"]["enable"].as<bool>())
            {
                dyn.prediction.type = MPCPlanner::PredictionType::DETERMINISTIC;
            }
        }
        else
        {
            // No prediction provided (or multi-mode not supported here):
            // treat obstacle as deterministic (current pose only).
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::DETERMINISTIC);
        }
    }

    // Pad or trim to the configured max_obstacles so solver parameter sizes
    // remain constant (adds “dummy” obstacles if needed).
    MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);

    // Optionally inflate/propagate uncertainty over time if enabled.
    if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
        MPCPlanner::propagatePredictionUncertainty(_data.dynamic_obstacles);

    // Notify planner modules (e.g., collision avoidance) that new obstacle
    // data is available so they can refresh solver params before the next solve.
    _planner->onDataReceived(_data, "dynamic obstacles");
}
void JulesJackalPlanner::obstacleServiceCallback(const mpc_planner_msgs::ObstacleArray &msg)
{
    // Start fresh each message; the solver expects the latest snapshot.
    // In obstacleCallback at the very start:
    LOG_DEBUG("OBSTACLE_CALLBACK: received " + std::to_string(msg.obstacles.size()) + " obstacles");
    const auto ns = ros::this_node::getNamespace();
    const std::string profiling_name = ns + "_" + "ObstacleCallback";
    PROFILE_SCOPE(profiling_name.c_str());
    LOG_DEBUG(profiling_name);

    _data.dynamic_obstacles.clear();

    for (const auto &obstacle : msg.obstacles)
    {
        // Create a new dynamic obstacle with:
        //  - id:      stable identifier from upstream (aggregator/ped sim)
        //  - pos:     current xy position
        //  - yaw:     orientation (used by some footprint models)
        //  - radius:  planner-wide default unless you encode per-obstacle sizes
        _data.dynamic_obstacles.emplace_back(
            obstacle.id,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            RosTools::quaternionToAngle(obstacle.pose),
            CONFIG["obstacle_radius"].as<double>());

        auto &dyn = _data.dynamic_obstacles.back(); // ret

        // Single-mode Gaussian predictions if provided; otherwise deterministic.
        // We treat "has probabilities AND exactly one gaussian" as "one-mode"
        // and fill per-step means + ellipse semiaxes. The actual probability
        // values are not used here; they are interpreted downstream.
        if (!obstacle.probabilities.empty() && obstacle.gaussians.size() == 1)
        {
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::GAUSSIAN);

            const auto &mode = obstacle.gaussians[0];
            dyn.prediction.modes[0].reserve(mode.mean.poses.size());
            for (size_t k = 0; k < mode.mean.poses.size(); ++k)
            {
                dyn.prediction.modes[0].emplace_back(
                    Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                    RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                    mode.major_semiaxis[k],
                    mode.minor_semiaxis[k]);
            }

            // If uncertainty is effectively zero or probabilistic handling is
            // disabled in config, fall back to deterministic to simplify constraints.
            if (mode.major_semiaxis.back() == 0.0 ||
                !CONFIG["probabilistic"]["enable"].as<bool>())
            {
                dyn.prediction.type = MPCPlanner::PredictionType::DETERMINISTIC;
            }
        }
        else
        {
            // No prediction provided (or multi-mode not supported here):
            // treat obstacle as deterministic (current pose only).
            dyn.prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::DETERMINISTIC);
        }
    }

    // Pad or trim to the configured max_obstacles so solver parameter sizes
    // remain constant (adds “dummy” obstacles if needed).
    MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);

    // Optionally inflate/propagate uncertainty over time if enabled.
    if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
        MPCPlanner::propagatePredictionUncertainty(_data.dynamic_obstacles);

    // Notify planner modules (e.g., collision avoidance) that new obstacle
    // data is available so they can refresh solver params before the next solve.
    _planner->onDataReceived(_data, "dynamic obstacles");
}
void JulesJackalPlanner::poseOtherRobotCallback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                                                const std::string ns)
{
    // Check if trajectory obstacle exists for this namespace (prevents race condition crash)
    auto it = _data.trajectory_dynamic_obstacles.find(ns);
    if (it == _data.trajectory_dynamic_obstacles.end())
    {
        LOG_WARN(_ego_robot_ns + ": Received pose from " + ns + " but obstacle not initialized yet. Ignoring.");
        return;
    }

    // Additional safety check
    if (!msg)
    {
        LOG_WARN(_ego_robot_ns + ": Received null pose message from " + ns + ". Ignoring.");
        return;
    }

    auto &robot_trajectory_obstacle = it->second;
    // Decode state from your encoded PoseStamped
    const double &psi = msg->pose.orientation.z; // encoded yaw
    const double &v = msg->pose.position.z;      // encoded speed

    // const double &vx = std::cos(psi) * v;
    // const double &vy = std::sin(psi) * v;

    robot_trajectory_obstacle.angle = psi;
    robot_trajectory_obstacle.position = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
    robot_trajectory_obstacle.current_speed = v;
}
void JulesJackalPlanner::trajectoryCallback(const mpc_planner_msgs::ObstacleGMM::ConstPtr &msg, const std::string ns)
{
    switch (_current_state)
    {
    case MPCPlanner::PlannerState::UNINITIALIZED:
    case MPCPlanner::PlannerState::TIMER_STARTUP:
    case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
    case MPCPlanner::PlannerState::INITIALIZING_OBSTACLES:
        // In these states, we're not ready to process trajectory data yet
        LOG_WARN_THROTTLE(2000, _ego_robot_ns + ": Received trajectory from " + ns + " but not ready to process (state: " +
                                    MPCPlanner::stateToString(_current_state) + "). Ignoring...");
        return;

    case MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA:
    case MPCPlanner::PlannerState::PLANNING_ACTIVE:
    case MPCPlanner::PlannerState::JUST_REACHED_GOAL:
    case MPCPlanner::PlannerState::GOAL_REACHED:
    case MPCPlanner::PlannerState::RESETTING:
    case MPCPlanner::PlannerState::ERROR_STATE:
    {
        // Process trajectory data in these states
        LOG_DEBUG(_ego_robot_ns + ": Received trajectory callback from " + ns + " in state: " +
                  MPCPlanner::stateToString(_current_state));

        // Check if trajectory obstacle exists for this namespace (prevents race condition crash)
        auto it = _data.trajectory_dynamic_obstacles.find(ns);
        if (it == _data.trajectory_dynamic_obstacles.end())
        {
            LOG_WARN_THROTTLE(1000, _ego_robot_ns + ": Received trajectory from " + ns +
                                        " but obstacle not initialized yet. Ignoring...");
            return;
        }

        // Additional safety checks
        if (!msg || msg->gaussians.empty())
        {
            LOG_WARN(_ego_robot_ns + ": Received invalid trajectory message from " + ns + ". Ignoring...");
            return;
        }

        // Get the trajectory-based obstacle for this robot namespace
        auto &robot_trajectory_obstacle = it->second;

        // Check if the obstacle id in msg corresponds with retrieved trajectory obstacle
        if (robot_trajectory_obstacle.index != msg->id)
        {
            LOG_WARN(_ego_robot_ns + ": Trajectory obstacle ID mismatch for robot " + ns +
                     " - expected ID: " + std::to_string(robot_trajectory_obstacle.index) +
                     ", received ID: " + std::to_string(msg->id) + ". Ignoring...");
            return;
        }

        // Update current pose of the trajectory obstacle
        const auto &position = msg->pose.position;
        const auto &orientation = msg->pose.orientation;
        robot_trajectory_obstacle.position = Eigen::Vector2d(position.x, position.y);
        robot_trajectory_obstacle.angle = RosTools::quaternionToAngle(orientation);

        // Get trajectory predictions
        const auto &list_of_gaussians_trajectories = msg->gaussians;
        // Remove unused variable: const auto &list_of_traject_costs = msg->probabilities;

        if (!list_of_gaussians_trajectories.empty())
        {
            // We expect only 1 gaussian from the direct trajectory publisher
            if (list_of_gaussians_trajectories.size() != 1)
            {
                LOG_WARN(_ego_robot_ns + ": Expected 1 trajectory, got " +
                         std::to_string(list_of_gaussians_trajectories.size()) + " from " + ns);
            }

            // Process the first (and expected only) Gaussian trajectory
            const auto &gaussian_trajectory = list_of_gaussians_trajectories[0];
            const auto &mean_trajectory = gaussian_trajectory.mean;
            const size_t trajectory_size = mean_trajectory.poses.size();

            // Create new prediction - DETERMINISTIC constructor automatically creates one empty mode
            MPCPlanner::Prediction new_prediction(MPCPlanner::PredictionType::DETERMINISTIC);

            // Create a single mode for the trajectory
            MPCPlanner::Mode new_mode;
            new_mode.reserve(trajectory_size);

            // Fill the mode with prediction steps from the mean trajectory
            for (size_t k = 0; k < trajectory_size; ++k)
            {
                const auto &pose_obj = mean_trajectory.poses[k];
                new_mode.emplace_back(
                    Eigen::Vector2d(pose_obj.pose.position.x, pose_obj.pose.position.y),
                    RosTools::quaternionToAngle(pose_obj.pose.orientation),
                    -1.0, // uncertainty ellipse major axis
                    -1.0  // uncertainty ellipse minor axis
                );
            }

            // Check if this is meaningful trajectory data BEFORE the move
            const bool has_meaningful_data = !new_mode.empty();

            // Replace the auto-created empty mode with our trajectory data
            new_prediction.modes[0] = std::move(new_mode);

            // Update the obstacle with the new prediction
            robot_trajectory_obstacle.prediction = std::move(new_prediction);

            /** @note Jules this is needed for the interpolation functionality */
            // ========== USE ros::Time ==========
            robot_trajectory_obstacle.last_trajectory_update_time = ros::Time::now();
            robot_trajectory_obstacle.trajectory_needs_interpolation = false;
            // ===================================

            LOG_DEBUG(_ego_robot_ns + ": Updated trajectory prediction for " + ns +
                      " with " + std::to_string(trajectory_size) + " trajectory points");

            // State-specific handling
            if (has_meaningful_data && _current_state == MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA)
            {
                MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::PLANNING_ACTIVE, _ego_robot_ns);
                LOG_INFO(_ego_robot_ns + ": First meaningful trajectory data received from " + ns +
                         " (" + std::to_string(trajectory_size) + " trajectory points) - transitioning to active planning");
            }

            // Track validated robots
            if (_validated_trajectory_robots.count(ns) == 0)
            {
                _validated_trajectory_robots.insert(ns);
                LOG_INFO(_ego_robot_ns + ": Added " + ns + " to validated trajectory robots and will now be considered in the MPC");
            }
        }
        else
        {
            LOG_ERROR(_ego_robot_ns + ": No trajectory prediction for " + ns +
                      ", but received callback. This should not happen. Ignoring...");
            return;
        }

        if (CONFIG["recording"]["enable"].as<bool>())
        {
            RosTools::DataSaver& ds = _planner->getDataSaver();
            ds.AddData("rx_from_" + ns + "_trajectory", 1.0);
            
            ros::Duration message_delay = ros::Time::now() - msg->gaussians.back().mean.header.stamp;
            ds.AddData("rx_from_" + ns + "_delay_sec", message_delay.toSec());
        }   
        break;
    }

    default:
        LOG_WARN(_ego_robot_ns + ": Received trajectory callback in unknown state: " +
                 std::to_string(static_cast<int>(_current_state)));
        break;
    }
}

void JulesJackalPlanner::allRobotsReachedObjectiveCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (!(msg->data))
    {
        LOG_ERROR(_ego_robot_ns + ": CentralAggregator send signal that all robots reached objective but data is false: " + std::to_string(msg->data));
        return;
    }

    LOG_INFO(_ego_robot_ns + ": Received message that all robots have reached their objective - resetting for new planning cycle");

    // Reset trajectory data readiness to ensure fresh start
    // _have_received_meaningful_trajectory_data = false;

    // Start startup timer to allow roadmap reversal and new path generation
    std_msgs::Empty empty_msg;
    _reverse_roadmap_pub.publish(empty_msg);

    _planner->reset(_state, _data, true); // reset the complete planner: solver, modules, state and data
    _validated_trajectory_robots.clear(); // clear the set which records which robots have send a correct trajectory
    _data.trajectory_dynamic_obstacles.clear();

    LOG_DIVIDER();
    LOG_INFO(_ego_robot_ns + ": Cleared all data structures - " +
             "validated_trajectory_robots size: " + std::to_string(_validated_trajectory_robots.size()) + ", " +
             "trajectory_dynamic_obstacles size: " + std::to_string(_data.trajectory_dynamic_obstacles.size()) + ", " +
             "dynamic_obstacles size: " + std::to_string(_data.dynamic_obstacles.size()) + ", " +
             "current state: [x=" + std::to_string(_state.get("x")) +
             ", y=" + std::to_string(_state.get("y")) +
             ", psi=" + std::to_string(_state.get("psi")) +
             ", v=" + std::to_string(_state.get("v")) + "]");

    _startup_timer->setDuration(4.0); // Give time for roadmap to reverse and publish new path
    _startup_timer->start();

    MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::TIMER_STARTUP, _ego_robot_ns);
    LOG_INFO(_ego_robot_ns + ": Ready to resume planning with new objectives in 2 seconds");
}

// 5. PUBLISHER FUNCTIONS
//    - All functions that publish ROS messages
void JulesJackalPlanner::publishEgoPose()
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _global_frame;

    pose.pose.position.x = _state.get("x");
    pose.pose.position.y = _state.get("y");
    pose.pose.orientation.z = _state.get("psi"); // Encoded yaw
    pose.pose.position.z = _state.get("v");      // Encoded velocity

    _pose_pub.publish(pose);
}
void JulesJackalPlanner::publishCurrentTrajectory(const MPCPlanner::PlannerOutput &output)
{
    // Safety checks to prevent crashes
    if (output.trajectory.positions.empty() || output.trajectory.orientations.empty())
    {
        LOG_WARN(_ego_robot_ns + ": Empty trajectory data. Skipping trajectory publication.");
        return;
    }

    if (output.trajectory.positions.size() != output.trajectory.orientations.size())
    {
        LOG_WARN(_ego_robot_ns + ": Trajectory positions and orientations size mismatch! Skipping trajectory publication.");
        return;
    }

    nav_msgs::Path ros_trajectory_msg;
    auto ros_time = ros::Time::now();
    ros_trajectory_msg.header.stamp = ros_time;
    ros_trajectory_msg.header.frame_id = _global_frame;

    int k = 0;
    for (const auto &position : output.trajectory.positions)
    {
        ros_trajectory_msg.poses.emplace_back();
        ros_trajectory_msg.poses.back().pose.position.x = position(0);
        ros_trajectory_msg.poses.back().pose.position.y = position(1);
        ros_trajectory_msg.poses.back().pose.position.z = k * output.trajectory.dt;
        ros_trajectory_msg.poses.back().pose.orientation = RosTools::angleToQuaternion(output.trajectory.orientations.at(k));
        ros_trajectory_msg.poses.back().header.stamp = ros_time + ros::Duration(k * output.trajectory.dt);
        k++;
    }

    _trajectory_pub.publish(ros_trajectory_msg);
}
void JulesJackalPlanner::publishObjectiveReachedEvent()
{
    std_msgs::Bool event;
    event.data = true;
    _objective_pub.publish(event);
    LOG_INFO_THROTTLE(1000, _ego_robot_ns + ": Objective reached - published event");
}

void JulesJackalPlanner::publishDirectTrajectory(const MPCPlanner::PlannerOutput &output)
{
    // build from output a
    mpc_planner_msgs::ObstacleGMM ego_robot_trajectory_as_obstacle;
    ego_robot_trajectory_as_obstacle.id = _ego_robot_id;
    ego_robot_trajectory_as_obstacle.pose.position.x = _state.get("x");
    ego_robot_trajectory_as_obstacle.pose.position.y = _state.get("y");

    ego_robot_trajectory_as_obstacle.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));
    ego_robot_trajectory_as_obstacle.gaussians.emplace_back();

    // CRITICAL FIX: Use reference instead of copy!
    auto &gaussian = ego_robot_trajectory_as_obstacle.gaussians.back();
    auto ros_time = ros::Time::now();
    gaussian.mean.header.stamp = ros_time;
    gaussian.mean.header.frame_id = _global_frame;

    int k = 0;
    // FIXED: Changed condition from > 1 to > 0 to include single-point trajectories
    if (output.trajectory.positions.size() > 0)
    {
        gaussian.mean.poses.reserve(output.trajectory.positions.size());
        gaussian.major_semiaxis.reserve(output.trajectory.positions.size());
        gaussian.minor_semiaxis.reserve(output.trajectory.positions.size());

        for (const auto &position : output.trajectory.positions)
        {
            gaussian.mean.poses.emplace_back();
            auto &pose = gaussian.mean.poses.back();
            pose.pose.position.x = position(0);
            pose.pose.position.y = position(1);
            pose.pose.position.z = k * output.trajectory.dt; // Store the the timing of each element in the z - component for visualization in rviz
            pose.pose.orientation = RosTools::angleToQuaternion(output.trajectory.orientations.at(k));
            pose.header.stamp = ros_time + ros::Duration(k * output.trajectory.dt);

            // Add dummy semiaxis values for now (can be made configurable later)
            gaussian.major_semiaxis.push_back(-1);
            gaussian.minor_semiaxis.push_back(-1);

            k++;
        }

        LOG_DEBUG(_ego_robot_ns + ": Publishing trajectory with " + std::to_string(gaussian.mean.poses.size()) + " poses");
    }
    else
    {
        LOG_WARN(_ego_robot_ns + ": SENDING EMPTY TRAJECTORY - no positions in MPC output!");
    }

    // Publish the trajectory directly to other robots
    _direct_trajectory_pub.publish(ego_robot_trajectory_as_obstacle);

    if (CONFIG["recording"]["enable"].as<bool>())
    {
        auto& ds = _planner->getDataSaver();
        ds.AddData("tx_trajectory", 1.0);
    }

}
// 6. VISUALIZATION FUNCTIONS
//    - Functions for RViz/debugging visualization
void JulesJackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(
        Eigen::Vector2d(_state.get("x"), _state.get("y")),
        Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")),
                        _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
}

// 7. MULTI-ROBOT COORDINATION FUNCTIONS
//    - Functions specific to multi-robot scenarios
// Multi-robot synchronization: wait for all robots to be ready
// Identify all other robot namespaces (excluding ego robot) for multi-robot coordination

bool JulesJackalPlanner::initializeOtherRobotsAsObstacles(const std::set<std::string> &other_robot_namespaces,
                                                          MPCPlanner::RealTimeData &data,
                                                          const double radius)
{
    // Early validation
    if (other_robot_namespaces.empty())
    {
        LOG_WARN(_ego_robot_ns + ": No other robots to initialize as obstacles");
        return false;
    }

    // Constants for readability
    const Eigen::Vector2d FAR_AWAY_POSITION(100.0, 100.0);
    const Eigen::Vector2d ZERO_VELOCITY(0.0, 0.0);

    std::string summary = _ego_robot_ns + " created obstacles for: ";

    for (const auto &robot_ns : other_robot_namespaces)
    {
        summary += robot_ns + " ";

        // Create trajectory obstacle for this robot
        data.trajectory_dynamic_obstacles.emplace(
            robot_ns,
            MPCPlanner::DynamicObstacle(
                MultiRobot::extractRobotIdFromNamespace(robot_ns),
                FAR_AWAY_POSITION,
                0.0,
                radius));

        auto &traj_obs = data.trajectory_dynamic_obstacles.at((robot_ns));
        traj_obs.last_trajectory_update_time = ros::Time::now();
        traj_obs.trajectory_needs_interpolation = false;

        // Create corresponding dynamic obstacle
        MPCPlanner::DynamicObstacle dummy_obstacle = data.trajectory_dynamic_obstacles.at(robot_ns);
        data.dynamic_obstacles.push_back(dummy_obstacle);

        // Initialize with zero velocity prediction (stationary dummy obstacle)
        auto &obstacle = data.dynamic_obstacles.back();
        obstacle.prediction = MPCPlanner::getConstantVelocityPrediction(
            obstacle.position,
            ZERO_VELOCITY,
            CONFIG["integrator_step"].as<double>(),
            CONFIG["N"].as<int>());

        LOG_INFO(_ego_robot_ns + ": Created obstacle for robot " + robot_ns +
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
        LOG_ERROR(_ego_robot_ns + ": Failed to initialize robot obstacles properly");
    }

    return initialization_successful;
}

// 8. HELPER/UTILITY FUNCTIONS
//    - Static functions and utility methods

void JulesJackalPlanner::logDataState(const std::string &context) const
{
    std::string ctx = context.empty() ? "DATA_STATE" : context;
    LOG_INFO(_ego_robot_ns + ": ========== " + ctx + " ==========");
    LOG_INFO(MultiRobot::dataToString(_data, _ego_robot_ns));
    LOG_INFO(_ego_robot_ns + ": ========================" + std::string(ctx.length(), '=') + "==========");
}

bool JulesJackalPlanner::objectiveReached(MPCPlanner::State _state, MPCPlanner::RealTimeData _data) const
{
    // check if the objective for a robot is reached if it is reached then we return 0.
    bool objective_reached = _planner->isObjectiveReached(_state, _data);
    return objective_reached;
}

bool JulesJackalPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const
{
    // Quick reject: if lengths differ, treat as different.
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Compare only the first few points for speed/stability.
    const int num_points = std::min(2, static_cast<int>(_data.reference_path.x.size()));
    for (int i = 0; i < num_points; ++i)
    {
        // `pointInPath(i, x, y)` should apply the planner’s internal tolerance.
        if (!_data.reference_path.pointInPath(
                i,
                msg->poses[i].pose.position.x,
                msg->poses[i].pose.position.y))
        {
            return false; // diverges early → treat as a new path
        }
    }

    // Same length and first points match within tolerance → consider unchanged.
    return true;
}
void JulesJackalPlanner::buildOutputFromBrakingCommand(MPCPlanner::PlannerOutput &output, const geometry_msgs::Twist &cmd)
{
    // Build output when the MPC could not solve
    if (output.success)
    {
        LOG_WARN_THROTTLE(1, "Creating a braking command trajectory while the MPC solve was signaled as successful");
        return;
    }

    const auto &new_speed = cmd.linear.x;
    const auto &orientation = _state.get("psi");

    const auto &new_vx = std::cos(orientation) * new_speed;
    const auto &new_vy = std::sin(orientation) * new_speed;

    const Eigen::Vector2d new_velocity(new_vx, new_vy);
    const Eigen::Vector2d current_position(_state.get("x"), _state.get("y"));

    MPCPlanner::Prediction const_vel_prediction = MPCPlanner::getConstantVelocityPrediction(current_position,
                                                                                            new_velocity,
                                                                                            CONFIG["integrator_step"].as<double>(),
                                                                                            CONFIG["N"].as<int>());

    const int N = CONFIG["N"].as<int>();
    output.trajectory.positions.clear();
    output.trajectory.orientations.clear();
    output.trajectory.positions.reserve(N);
    output.trajectory.orientations.reserve(N);
    output.trajectory.dt = CONFIG["integrator_step"].as<double>();

    for (const auto &predstep : const_vel_prediction.modes[0])
    {
        output.trajectory.positions.push_back(predstep.position);
        output.trajectory.orientations.push_back(orientation);
    }

    LOG_WARN_THROTTLE(2000, _ego_robot_ns << " Creating a braking command trajectory");
}

// 9 functions structuring planning phase:
// Add this function to jules_ros1_jackalplanner.cpp in the "3. MAIN CONTROL LOOP FUNCTIONS" section

// Add this function after handleInitialPlanningPhase()
void JulesJackalPlanner::prepareObstacleData()
{
    if (static_cast<int>(_data.dynamic_obstacles.size()) > CONFIG["max_obstacles"].as<int>())
    {
        LOG_ERROR(_ego_robot_ns << "Received " << _data.dynamic_obstacles.size() << "That is too much removing most distant obstacles......");
    }

    interpolateTrajectoryPredictionsByTime(); // NEW: Interpolate stale trajectories
    MPCPlanner::MultiRobot::updateRobotObstaclesFromTrajectories(_data, _validated_trajectory_robots, _ego_robot_ns);

    // LOG_ERROR(_ego_robot_ns + " Received " << _data.dynamic_obstacles.size() << " < " << max_obstacles << " obstacles. Adding dummies.");
    MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state); // Be aware what happens when you have more obstacles than allowed

    _planner->onDataReceived(_data, "dynamic obstacles");

    if (CONFIG["debug_output"].as<bool>())
    {
        _state.print();
        this->logDataState(_ego_robot_ns + ": Obstacle data prepared");
    }
}

void JulesJackalPlanner::interpolateTrajectoryPredictionsByTime()
{
    const double dt = CONFIG["integrator_step"].as<double>();
    const int N = CONFIG["N"].as<int>();
    const bool enable_interpolation = CONFIG["JULES"]["enable_trajectory_interpolation"].as<bool>(true);

    if (!enable_interpolation)
    {
        return;
    }

    ros::Time now = ros::Time::now();
    for (auto &[ns, traj_obs] : _data.trajectory_dynamic_obstacles)
    {
        // ============================================================
        // VALIDATION CHECKS
        // ============================================================

        if (_validated_trajectory_robots.find(ns) == _validated_trajectory_robots.end())
        {
            continue;
        }

        if (traj_obs.prediction.modes.empty() || traj_obs.prediction.modes[0].empty())
        {
            continue;
        }

        auto &mode = traj_obs.prediction.modes[0];
        int n_measured = mode.size();

        if (n_measured != N)
        {
            LOG_WARN(_ego_robot_ns + ": Trajectory for " + ns + " has " +
                     std::to_string(n_measured) + " points, expected " + std::to_string(N) +
                     ". Skipping interpolation.");
            continue;
        }

        // ========== CALCULATE ELAPSED TIME WITH ros::Time ==========
        ros::Duration elapsed_time = now - traj_obs.last_trajectory_update_time;
        double dt_interp = elapsed_time.toSec();
        // ===========================================================

        double min_age_threshold = 1.0 / _control_frequency;
        if (dt_interp < min_age_threshold)
        {
            traj_obs.trajectory_needs_interpolation = false;
            continue; // Skip interpolation entirely for fresh trajectories
        }

        //
        int k = static_cast<int>(std::floor(dt_interp / dt));
        double tau = dt_interp - k * dt;
        double alpha = tau / dt; // Fractional remainder [0, 1)

        // Cap k to prevent going beyond trajectory
        if (k >= N)
        {
            LOG_ERROR(_ego_robot_ns + ": Trajectory for " + ns + " is critically stale! " +
                      "dt_interp=" + std::to_string(dt_interp) + "s exceeds horizon " +
                      std::to_string(N * dt) + "s. Regenerating trajectory.");

            // regenerateTrajectoryFromCurrentPose(traj_obs, N, dt);
            traj_obs.trajectory_needs_interpolation = false; // Mark as not using interpolated data
            continue;
        }

        // Skip if no meaningful shift
        if (k == 0 && alpha < 0.01)
        {
            traj_obs.trajectory_needs_interpolation = false; // No interpolation needed
            continue;
        }

        LOG_DEBUG(_ego_robot_ns + ": Interpolating trajectory for " + ns +
                  " (dt_interp=" + std::to_string(dt_interp) + "s, k=" + std::to_string(k) +
                  ", tau=" + std::to_string(tau) + "s, alpha=" + std::to_string(alpha) + ")");

        // This vector will filled with pionts we extrapolate from the back of the original prediction vector
        std::vector<MPCPlanner::PredictionStep> extrapolated_points;
        int num_extrap_points = k + 1; // Need k + 1 for properinterpolation, the new trajectory will still contain N points but we will interpolate between N + 1 points to get N points back again
        if (n_measured >= 2)
        {
            const auto &x_last = mode.back();
            const auto &x_second_last = mode[n_measured - 2];

            Eigen::Vector2d v = (x_last.position - x_second_last.position) / dt;
            double psi_dot = MultiRobot::wrapAngleDifference(x_last.angle - x_second_last.angle) / dt;

            // clamp velocities to physical limits
            double v_mag = v.norm();
            double v_max = CONFIG["JULES"]["robot_max_velocity"].as<double>(2.0);
            if (v_mag > v_max)
            {
                LOG_WARN_THROTTLE(2000, _ego_robot_ns + ": Clamping extrapolated velocity for " + ns +
                                            " from " + std::to_string(v_mag) + " to " + std::to_string(v_max) + " m/s");
                v = v.normalized() * v_max;
            }

            double psi_dot_max = CONFIG["JULES"]["robot_max_angular_velocity"].as<double>(2.0);
            if (std::abs(psi_dot) > psi_dot_max)
            {
                LOG_WARN_THROTTLE(2000, _ego_robot_ns + ": Clamping angular velocity for " + ns);
                psi_dot = std::clamp(psi_dot, -psi_dot_max, psi_dot_max);
            }

            // Generate k + 1 extrapolated points

            for (int i = 1; i <= num_extrap_points; ++i)
            {
                double t_extrap = i * dt;

                Eigen::Vector2d pos_extrap = x_last.position + v * t_extrap;
                double psi_extrap = MultiRobot::wrapAngle(x_last.angle + psi_dot * t_extrap);

                extrapolated_points.emplace_back(
                    pos_extrap,
                    psi_extrap,
                    -1.0,
                    -1.0);
            }
        }

        else
        {
            LOG_ERROR(_ego_robot_ns + ": Cannot extrapolate for " + ns +
                      " - insufficient points (need at least 2)");
        }

        // ============================================================
        // STEP 2: REMOVE FIRST k POINTS -> we have moved beyond them in time
        // ============================================================
        if (k > 0)
        {
            mode.erase(mode.begin(), mode.begin() + k);
            // e.g given that we have 8 points and k = 5 we should have a new mode with size = 3
            ROSTOOLS_ASSERT(static_cast<int>(mode.size()) == (n_measured - k), "The size of mode should be equal to N_original - k");
            LOG_DEBUG(_ego_robot_ns + ": Removed first " + std::to_string(k) +
                      " points from trajectory for " + ns);
        }

        // ============================================================
        // STEP 3: APPEND EXTRAPOLATED POINTS
        // ============================================================
        mode.insert(mode.end(), extrapolated_points.begin(), extrapolated_points.end());
        ROSTOOLS_ASSERT(static_cast<int>(mode.size()) == (n_measured + 1), "The size of the mode should be equal to N_original + 1");

        // ============================================================
        // STEP 4: INTERPOLATE ALL POINTS BY alpha
        // ============================================================
        if (alpha > 0.001)
        {
            std::vector<MPCPlanner::PredictionStep> interpolated_mode;
            interpolated_mode.reserve(N);
            // Interpolate between consecutive pairs
            // We have (N+1) points, so we get N interpolated points
            for (size_t i = 0; i < mode.size() - 1; ++i)
            {
                const auto x_curr = mode[i];
                const auto x_next = mode[i + 1];
                Eigen::Vector2d pos_interp = (1 - alpha) * x_curr.position + alpha * x_next.position;
                double psi_interp = MultiRobot::interpolateAngle(x_curr.angle, x_next.angle, alpha);

                interpolated_mode.emplace_back(
                    pos_interp,
                    psi_interp,
                    -1,
                    -1);
            }

            mode = std::move(interpolated_mode);
        }

        else
        {
            // No fractional shift needed, check if we have an extra point
            if (mode.size() > static_cast<size_t>(N))
            {
                mode.pop_back();
            }
        }

        // ============================================================
        // STEP 5: VERIFY EXACTLY N POINTS
        // ============================================================
        if (mode.size() != static_cast<size_t>(N))
        {
            LOG_ERROR(_ego_robot_ns + ": Trajectory length mismatch for " + ns +
                      " after interpolation! Expected " + std::to_string(N) +
                      ", got " + std::to_string(mode.size()) + ". Applying fix.");

            // Emergency fix: pad or trim
            while (mode.size() < static_cast<size_t>(N))
            {
                LOG_WARN(_ego_robot_ns + ": Padding trajectory for " + ns);
                mode.push_back(mode.back());
            }
            while (mode.size() > static_cast<size_t>(N))
            {
                LOG_WARN(_ego_robot_ns + ": Trimming trajectory for " + ns);
                mode.pop_back();
            }
        }

        if (!mode.empty())
        {
            traj_obs.position = mode[0].position;
            traj_obs.angle = mode[0].angle;
        }

        traj_obs.trajectory_needs_interpolation = true;

        // ============================================================
        // CRITICAL: Update timestamp to prevent compounding drift
        // ============================================================
        // We've successfully interpolated/extrapolated the trajectory forward by dt_interp seconds.
        // Update the timestamp to reflect that we've "consumed" this time shift.
        // This prevents the next interpolation from using the ORIGINAL timestamp and
        // creating cumulative errors (e.g., shifting by 0.2s, then 0.4s, then 0.6s...)
        traj_obs.last_trajectory_update_time = now;

        LOG_DEBUG(_ego_robot_ns + ": Successfully interpolated trajectory for " + ns +
                  " (shifted by " + std::to_string(dt_interp) + "s)" +
                  " - new start pose: (" + std::to_string(mode[0].position.x()) + ", " +
                  std::to_string(mode[0].position.y()) + ", " +
                  std::to_string(mode[0].angle * 180.0 / M_PI) + "°)");

    } // loop end for each _data.trajectory_dynamic_obstacles
}
// Replace the existing generatePlanningCommand function with this updated version:
std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> JulesJackalPlanner::generatePlanningCommand(const MPCPlanner::PlannerState &current_state)
{
    geometry_msgs::Twist cmd;
    MPCPlanner::PlannerOutput output; // Initialize with default values

    // Lambda to handle MPC solving and command extraction
    // Captures: cmd & output by reference (to modify), this for class members
    auto solveMPCAndExtractCommand = [&cmd, &output, this]()
    {
        output = _planner->solveMPC(_state, _data);

        if (_enable_output && output.success)
        {
            cmd.linear.x = _planner->getSolution(1, "v");
            cmd.angular.z = _planner->getSolution(0, "w");
            LOG_VALUE_DEBUG("Commanded", "v=" + std::to_string(cmd.linear.x) + ", w=" + std::to_string(cmd.angular.z));
        }
        else
        {
            applyBrakingCommand(cmd);
            buildOutputFromBrakingCommand(output, cmd);
            LOG_WARN(_ego_robot_ns + ": Applying braking command - solver failed or output disabled");
        }
    };

    // Lambda to handle braking scenarios
    // Captures: cmd & output by reference (to modify), this for class member functions
    auto applyBrakingScenario = [&cmd, &output, this]()
    {
        applyBrakingCommand(cmd);
        buildOutputFromBrakingCommand(output, cmd);
    };

    switch (current_state)
    {
    case MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA:
    {
        LOG_WARN(_ego_robot_ns + ": Waiting for first real trajectory data, planning with Dummy Obstacles");
        solveMPCAndExtractCommand();
        break;
    }
    case MPCPlanner::PlannerState::JUST_REACHED_GOAL:
    {
        applyBrakingScenario();
        LOG_INFO(_ego_robot_ns + ": JUST REACHED GOAL - Applying braking command");
        break;
    }
    case MPCPlanner::PlannerState::PLANNING_ACTIVE:
    {
        LOG_DEBUG(_ego_robot_ns + ": Calling MPC solver with " + std::to_string(_data.dynamic_obstacles.size()) + " obstacles");
        solveMPCAndExtractCommand();
        break;
    }
    case MPCPlanner::PlannerState::GOAL_REACHED:
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        LOG_DEBUG(_ego_robot_ns + ": Maintaining stopped state at goal");

        if (_stop_when_reached_goal)
        {
            buildOutputFromBrakingCommand(output, cmd);
            publishObjectiveReachedEvent();
        }
        else
        {
            rotatePiRadiansCw(cmd);
            buildOutputFromBrakingCommand(output, cmd);
        }
        break;
    }
    default:
        break;
    }

    return {cmd, output};
}

// Add this function after generatePlanningCommand()
void JulesJackalPlanner::publishCmdAndVisualize(const geometry_msgs::Twist &cmd, const MPCPlanner::PlannerOutput &output)
{
    bool should_communicate = true; // Default: always communicate (safer)
    if (_communicate_on_topology_switch_only)
    {
        const int n_paths = CONFIG["JULES"]["n_paths"].as<int>();
        const int non_guided_topology_id = 2 * n_paths;

        // Case 1: MPC solver failed
        if (!output.success)
        {
            should_communicate = true;
            LOG_DEBUG(_ego_robot_ns + ": Communicating - MPC failed");
        }

        // Case 2a: Switched TO non-guided (special case of topology switch)
        else if (output.following_new_topology &&
                 output.selected_topology_id == non_guided_topology_id)
        {
            should_communicate = true;
            LOG_DEBUG(_ego_robot_ns + ": Communicating - Switched to non-guided from topology " +
                      std::to_string(output.previous_topology_id));
        }

        // Case 2b: Topology switch (between guided topologies or from non-guided to guided)
        else if (output.following_new_topology)
        {
            should_communicate = true;
            LOG_DEBUG(_ego_robot_ns + ": Communicating - Topology switch from " +
                      std::to_string(output.previous_topology_id) + " to " +
                      std::to_string(output.selected_topology_id));
        }

        // Case 3: Staying in non-guided topology (continuous communication)
        else if (output.selected_topology_id == non_guided_topology_id)
        {
            should_communicate = true;
            LOG_DEBUG(_ego_robot_ns + ": Communicating - Staying in non-guided (unidentified topology)");
        }

        // Case 4: Same guided topology
        else
        {
            should_communicate = false;
            LOG_DEBUG(_ego_robot_ns + ": NOT communicating - Same guided topology (" +
                      std::to_string(output.selected_topology_id) + ")");
        }
    }
    // else: _communicate_on_topology_switch_only is false, so should_communicate stays true

    // Always publish velocity command
    _cmd_pub.publish(cmd);

    // Conditional trajectory communication
    if (should_communicate)
    {
        this->publishDirectTrajectory(output);
        this->publishCurrentTrajectory(output);
        LOG_VALUE_DEBUG("Communication", "Published trajectory (Topology: " +
                                             std::to_string(output.selected_topology_id) + ")");
    }
    else
    {
        LOG_VALUE_DEBUG("Communication", "Skipped (Same topology: " +
                                             std::to_string(output.selected_topology_id) + ")");
    }

    // Record inside data if we have communicated or not
    _data.communicated_trajectory = should_communicate;
    
    // Always visualize
    _planner->visualizeObstaclePredictionsPlanner(_state, _data, false);
    _planner->visualize(_state, _data);
    // visualize();
}

void JulesJackalPlanner::rotatePiRadiansCw(geometry_msgs::Twist &cmd)
{
    LOG_INFO_THROTTLE(1500, _ego_robot_ns + ": Rotating pi radians clockwise");

    // Safety check
    if (_data.reference_path.psi.empty())
    {
        LOG_WARN(_ego_robot_ns + ": No reference path available for rotation");
        MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::ERROR_STATE, _ego_robot_ns);

        return;
    }

    // Get current heading from reference path (last available point)
    const double current_ref_heading = _data.reference_path.psi.back();
    const double goal_angle = current_ref_heading + M_PI; // Rotate 180 degrees
    double angle_diff = goal_angle - _state.get("psi");

    // Normalize angle difference
    while (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;

    LOG_VALUE_DEBUG(_ego_robot_ns + " psi:", _state.get("psi"));
    LOG_VALUE_DEBUG(_ego_robot_ns + " goal_angle:", current_ref_heading);
    LOG_VALUE_DEBUG(_ego_robot_ns + " angle_diff", angle_diff);

    if (std::abs(angle_diff) > 0.1) // 0.1 rad ≈ 5.7 degrees tolerance, if the angle is too large than we want to keep on rotating otherwise stop
    {
        cmd.linear.x = 0.0;
        if (_enable_output)
            cmd.angular.z = 1.1 * RosTools::sgn(angle_diff);
        else
            cmd.angular.z = 0.0;
    }
    else
    {
        // We publish that we reached our objective, it does not matter that we do this multiple times
        publishObjectiveReachedEvent();
        MultiRobot::transitionTo(_current_state, _previous_state, MPCPlanner::PlannerState::RESETTING, _ego_robot_ns);
        LOG_INFO(_ego_robot_ns + " Is waiting for the rest of the robots to reach their objective before starting to follow reverserd path");
    }
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "JULES_jackalplanner");
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        VISUALS.init(&nh);

        JulesJackalPlanner planner(nh, pnh);

        ros::spin();

        return 0;
    }
    catch (const YAML::Exception& e)
    {
        ROS_FATAL_STREAM("JULES_jackalplanner YAML error: " << e.what());
        ROS_FATAL_STREAM("This usually means a configuration file is missing or malformed.");
        return 1;
    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("JULES_jackalplanner failed with exception: " << e.what());
        ROS_FATAL_STREAM("Stack trace or details may be above this message.");
        return 1;
    }
    catch (...)
    {
        ROS_FATAL_STREAM("JULES_jackalplanner failed with unknown exception!");
        ROS_FATAL_STREAM("This is a critical error - check all dependencies are loaded.");
        return 1;
    }
}
