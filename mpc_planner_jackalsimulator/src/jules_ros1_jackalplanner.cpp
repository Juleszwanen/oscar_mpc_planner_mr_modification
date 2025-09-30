#include <mpc_planner_jackalsimulator/jules_ros1_jackalplanner.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/load_yaml.hpp> // must come first
#include <mpc_planner_util/parameters.h>

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
    LOG_HEADER("JULES PlannerNode: " + ros::this_node::getName() + " starting");
    _ego_robot_ns = ros::this_node::getNamespace();
    _ego_robot_id = extractRobotIdFromNamespace(_ego_robot_ns);

    if (!nh.getParam("/robot_ns_list", _robot_ns_list))
    {
        LOG_ERROR(_ego_robot_ns + ": No robot_ns_list param found");
    }

    _other_robot_nss = this->identifyOtherRobotNamespaces(_robot_ns_list);

    // Load configuration before constructing Planner
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    // Define robot footprint from CONFIG
    this->_data.robot_area = MPCPlanner::defineRobotArea(
        CONFIG["robot"]["length"].as<double>(),
        CONFIG["robot"]["width"].as<double>(),
        CONFIG["n_discs"].as<int>());

    this->_control_frequency = CONFIG["control_frequency"].as<double>();
    this->_enable_output = CONFIG["enable_output"].as<bool>();
    this->_infeasible_deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
    this->initializeOtherRobotsAsObstacles(_other_robot_nss, _data, CONFIG["robot_radius"].as<double>());

    // Read parameters
    bool wait_for_sync;
    pnh.param("frames/global", this->_global_frame, this->_global_frame);
    pnh.param("goal_tolerance", this->_goal_tolerance, this->_goal_tolerance);
    nh.param<bool>("/wait_for_sync", wait_for_sync, false);

    // Construct Planner after configuration is ready
    this->_planner = std::make_unique<MPCPlanner::Planner>();

    // Setup ROS I/O
    this->initializeSubscribersAndPublishers(nh, pnh);

    // Robot synchronization
    // _reconfigure = std::make_unique<JackalsimulatorReconfigure>(); // Initialize RQT reconfigure

    ros::Duration(5).sleep(); // Rember that this blocks all callbacks of this node, but this should give the whole system and the other nodes some startup timeq
    // Give system startup time
    if (wait_for_sync)
    {
        this->waitForAllRobotsReady(nh);
    }

    // publish the current pose of the ego robot, which the other robots can consume

    // this->publishEgoPose();

    // Start control loop timer
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &JulesJackalPlanner::loopDirectTrajectory,
        this);

    // LOG_INFO(_ego_robot_ns + ": CONSTRUCTOR: dynamic_obstacles size = " + std::to_string(_data.dynamic_obstacles.size()));
    LOG_DIVIDER();
}
JulesJackalPlanner::~JulesJackalPlanner()
{
    LOG_INFO("Stopped JulesJackalPlanner");
}

// 2. ROS COMMUNICATION SETUP
//    - Functions that initialize publishers, subscribers, services
// Initialize ROS communication (subscribers and publishers)
void JulesJackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh, ros::NodeHandle & /*pnh*/)
{
    LOG_INFO(_ego_robot_ns + ": initializeSubscribersAndPublishers");

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
void JulesJackalPlanner::loop(const ros::TimerEvent & /*event*/)
{
    LOG_DEBUG(_ego_robot_ns + "============= Loop =============");
    LOG_DEBUG(_ego_robot_ns + ": LOOP: dynamic_obstacles size = " + std::to_string(_data.dynamic_obstacles.size()));
    LOG_DEBUG(_ego_robot_ns + ": LOOP: trajectory_dynamic_obstacle size = " + std::to_string(_data.trajectory_dynamic_obstacles.size()));

    if (_planning_for_the_frist_time)
    {
        // LOG_HEADER(_ego_robot_ns + ": Planning for the first time");
        LOG_INFO(_ego_robot_ns + ": State x: " + std::to_string(_state.get("x")));
        LOG_INFO(_ego_robot_ns + ": State y: " + std::to_string(_state.get("y")));
        LOG_INFO(_ego_robot_ns + ": State psi[radians]: " + std::to_string(_state.get("psi")));
        applyDummyObstacleCommand();
        _planning_for_the_frist_time = false;
        this->logDataState(_ego_robot_ns + ": Planning for the first time");
        return;
    }

    _data.planning_start_time = std::chrono::system_clock::now();

    _planner->visualizeObstaclePredictionsPlanner(_state, _data, true);

    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    geometry_msgs::Twist cmd;

    // Goal reached - apply braking
    if (!_goal_reached && this->objectiveReached(_state, _data))
    {
        applyBrakingCommand(cmd);
        _goal_reached = true;
        publishObjectiveReachedEvent();
        const auto message = ros::this_node::getNamespace().substr(1) + " Goal reached - applying braking";
        LOG_INFO(message);
    }

    // Maintain stopped state at goal
    else if (_goal_reached)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        const auto message = ros::this_node::getNamespace().substr(1) + " Maintaining stopped state at goal";
        LOG_DEBUG(message);
    }
    // Normal MPC operation
    else
    {
        if (!_data.dynamic_obstacles.size())
        {
            LOG_WARN(_ego_robot_ns + ": PLANNING WITH EMPTY OBSTACLEs");
        }

        auto output = _planner->solveMPC(_state, _data);
        LOG_MARK("Success: " << output.success);

        if (_enable_output && output.success)
        {
            cmd.linear.x = _planner->getSolution(1, "v");
            cmd.angular.z = _planner->getSolution(0, "w");
            LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
        }
        else
        {
            applyBrakingCommand(cmd);
            buildOutputFromBrakingCommand(output, cmd); // Fills in the output object with a braking trajectory
            LOG_WARN(_ego_robot_ns + ": Solver failed or output disabled - applying braking");
        }

        this->publishCurrentTrajectory(output);
        this->publishDirectTrajectory(output);
    }

    // Publish the velocity command to the robot’s differential drive.
    _cmd_pub.publish(cmd);

    // Keep downstream consumers updated with our current pose (for RViz/aggregator).
    // this->publishPose();

    // Built-in planner visuals (predicted trajectory, footprints, etc.), if available.
    // Jules Important this always visualizes the trajectory of the other robot in the previous time step.
    // Could consider visualizing obstacles when the obstacle callback is triggered.
    // Check if the dynamicObstaclepredictions visualization in the function is turned on or off
    _planner->visualize(_state, _data);

    // Quick heading ray for sanity checking.
    visualize();

    LOG_DEBUG(_ego_robot_ns + ": ============= End Loop =============");
}
void JulesJackalPlanner::loopDirectTrajectory(const ros::TimerEvent & /*event*/)
{
    LOG_DEBUG(_ego_robot_ns + "============= Loop Start =============");
    LOG_DEBUG(_ego_robot_ns + ": Obstacles: dynamic=" + std::to_string(_data.dynamic_obstacles.size()) +
              ", trajectory=" + std::to_string(_data.trajectory_dynamic_obstacles.size()));

    if (_planning_for_the_frist_time)
    {
        LOG_INFO(_ego_robot_ns + ": INITIAL PLANNING - State: [" +
                 std::to_string(_state.get("x")) + ", " +
                 std::to_string(_state.get("y")) + ", " +
                 std::to_string(_state.get("psi")) + "]");
        LOG_INFO(_ego_robot_ns + ": Waiting for trajectory callbacks, applying dummy command");

        applyDummyObstacleCommand();
        _planning_for_the_frist_time = false;
        this->logDataState(_ego_robot_ns + ": Initial planning complete");
        LOG_INFO(_ego_robot_ns + ": ============= End INITIAL Loop =============");
        return;
    }

    _data.planning_start_time = std::chrono::system_clock::now();

    // Update existing robot obstacles in place - more efficient than clearing and rebuilding
    LOG_DEBUG(_ego_robot_ns + ": Updating robot obstacles from trajectories...");
    updateRobotObstaclesFromTrajectories();

    LOG_DEBUG(_ego_robot_ns + ": Ensuring obstacle size and notifying planner...");
    MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);
    this->logDataState(_ego_robot_ns + ": Initial planning complete");
    _planner->onDataReceived(_data, "dynamic obstacles");

    // _planner->visualizeObstaclePredictionsPlanner(_state, _data, true);

    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    geometry_msgs::Twist cmd;

    // Goal reached - apply braking
    if (!_goal_reached && this->objectiveReached(_state, _data))
    {
        applyBrakingCommand(cmd);
        _goal_reached = true;
        publishObjectiveReachedEvent();
        LOG_INFO(_ego_robot_ns + ": GOAL REACHED - Applying braking command");
    }

    // Maintain stopped state at goal
    else if (_goal_reached)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        LOG_DEBUG(_ego_robot_ns + ": Maintaining stopped state at goal");
    }
    // Normal MPC operation
    else
    {
        if (_data.dynamic_obstacles.empty())
        {
            LOG_WARN(_ego_robot_ns + ": No dynamic obstacles - planning in free space");
        }

        LOG_DEBUG(_ego_robot_ns + ": Calling MPC solver with " + std::to_string(_data.dynamic_obstacles.size()) + " obstacles");
        auto output = _planner->solveMPC(_state, _data);

        if (output.success)
        {
            LOG_DEBUG(_ego_robot_ns + ": MPC solver SUCCESS");
        }
        else
        {
            LOG_WARN(_ego_robot_ns + ": MPC solver FAILED");
        }

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

        // Publish trajectory for robot-to-robot communication
        this->publishDirectTrajectory(output);
    }

    // Publish the velocity command to the robot’s differential drive.
    _cmd_pub.publish(cmd);

    // Keep downstream consumers updated with our current pose (for RViz/aggregator).
    // this->publishPose();

    // Built-in planner visuals (predicted trajectory, footprints, etc.), if available.
    _planner->visualize(_state, _data);

    // Quick heading ray for sanity checking.
    visualize();

    LOG_DEBUG(_ego_robot_ns + ": ============= Loop End =============");
}
void JulesJackalPlanner::loopWithService(const ros::TimerEvent & /*event*/)
{
    LOG_DEBUG("loopWithService: dynamic_obstacles size = " + std::to_string(_data.dynamic_obstacles.size()));

    const std::string profiling_name = _ego_robot_ns + "_" + "planningLoop";
    PROFILE_SCOPE(profiling_name.c_str());

    // Timestamp the beginning of this planning cycle (used by internal profiling).
    // _planner->visualizeObstaclePredictionsPlanner(_state, _data);
    _data.planning_start_time = std::chrono::system_clock::now();

    mpc_planner_msgs::GetOtherTrajectories srv;
    // Call the client and ask for all the trajectories of the other robots except the one of the ego robot.
    srv.request.request_header = std_msgs::Header();
    srv.request.requesting_robot_id = _ego_robot_ns;
    srv.request.requesting_robot_pose = geometry_msgs::Pose();
    if (_trajectory_client.call(srv))
    {
        // fills in the correct member data
        this->obstacleServiceCallback(srv.response.obstacle_trajectories);
    }

    else
    {
        LOG_ERROR(_ego_robot_ns + "Failed to call the GetOtherTrajectoriesSerivice");
    }

    LOG_DEBUG(_ego_robot_ns + "============= loopWithService =============");

    if (_data.dynamic_obstacles.empty())
    {
        LOG_DEBUG("EMPTY OBSTACLE");
    }

    // Optional verbose state print for debugging if enabled in config.
    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    geometry_msgs::Twist cmd;

    // Check if we just reached the goal (transition)
    if (!_goal_reached && this->objectiveReached(_state, _data))
    {
        applyBrakingCommand(cmd);
        _goal_reached = true;
        publishObjectiveReachedEvent();
        const auto message = ros::this_node::getNamespace().substr(1) + " Goal reached - applying braking";
        LOG_INFO(message);
    }
    // Already at goal - maintain stopped state
    else if (_goal_reached)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        const auto message = ros::this_node::getNamespace().substr(1) + " Maintaining stopped state at goal";
        LOG_DEBUG(message);
    }
    // Normal operation - run solver
    else
    {
        // Run the MPC using the latest state & data assembled by the callbacks.
        auto output = _planner->solveMPC(_state, _data);
        LOG_MARK("Success: " << output.success);

        if (_enable_output && output.success)
        {
            // Extract the first applicable controls from the solution
            cmd.linear.x = _planner->getSolution(1, "v");
            cmd.angular.z = _planner->getSolution(0, "w");
            LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
        }
        else
        {
            // Fail-safe braking if solver failed or output disabled
            applyBrakingCommand(cmd);
            LOG_DEBUG("Solver failed or output disabled - applying braking");
        }

        // Publish tra_outputjectory only during normal operation
        this->publishCurrentTrajectory(output);
    }

    // Publish the velocity command to the robot’s differential drive.
    _cmd_pub.publish(cmd);

    // Keep downstream consumers updated with our current pose (for RViz/aggregator).
    // this->publishPose();

    // Built-in planner visuals (predicted trajectory, footprints, etc.), if available.
    // Jules Important this always visualizes the trajectory of the other robot in the previous time step.
    // Could consider visualizing obstacles when the obstacle callback is triggered.
    _planner->visualize(_state, _data);

    // Quick heading ray for sanity checking.
    // visualize();

    LOG_DEBUG(_ego_robot_ns + "============= End loopWithService =============");
}
void JulesJackalPlanner::applyDummyObstacleCommand()
{

    geometry_msgs::Twist cmd;

    // Normal operation - run solver

    if (!_data.dynamic_obstacles.size())
    {
        LOG_WARN(_ego_robot_ns + ": PLANNING WITH EMPTY OBSTACLEs, INSERTING DUMMY ...");
        MPCPlanner::ensureObstacleSize(_data.dynamic_obstacles, _state);
        LOG_WARN(_ego_robot_ns + ": PLANNING WITH Dummy obstacles");
    }

    // Run the MPC using the latest state & data assembled by the callbacks.
    auto output = _planner->solveMPC(_state, _data);
    LOG_MARK("Success: " << output.success);

    if (_enable_output && output.success)
    {
        // Extract the first applicable controls from the solution
        cmd.linear.x = _planner->getSolution(1, "v");
        cmd.angular.z = _planner->getSolution(0, "w");
        LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
    }
    else
    {
        // Fail-safe braking if solver failed or output disabled
        applyBrakingCommand(cmd);
        buildOutputFromBrakingCommand(output, cmd);
        LOG_WARN(_ego_robot_ns + ": Solver failed or output disabled - applying braking");
    }

    // Publish tra_outputjectory only during normal operation
    // this->publishCurrentTrajectory(output);
    this->publishDirectTrajectory(output);
    // Publish the velocity command to the robot’s differential drive.
    _cmd_pub.publish(cmd);

    // Keep downstream consumers updated with our current pose (for RViz/aggregator).
    // this->publishPose();

    // Built-in planner visuals (predicted trajectory, footprints, etc.), if available.
    // Jules Important this always visualizes the trajectory of the other robot in the previous time step.
    // Could consider visualizing obstacles when the obstacle callback is triggered.
    _planner->visualize(_state, _data);

    // Quick heading ray for sanity checking.
    visualize();
    _data.dynamic_obstacles.clear();
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
    }

    _data.reference_path.psi.push_back(0.0);
    _planner->onDataReceived(_data, "reference_path");
}
void JulesJackalPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
{
    // Start fresh each message; the solver expects the latest snapshot.
    // In obstacleCallback at the very start:
    if (_planning_for_the_frist_time)
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

    // Check if trajectory obstacle exists for this namespace (prevents race condition crash)
    LOG_DEBUG(_ego_robot_ns + " Received a callback triggered by a trajectory send by: " + ns);
    auto it = _data.trajectory_dynamic_obstacles.find(ns);
    if (it == _data.trajectory_dynamic_obstacles.end())
    {
        LOG_WARN(_ego_robot_ns + ": Received trajectory from " + ns + " but obstacle not initialized yet. Ignoring.");
        return;
    }

    // Additional safety checks
    if (!msg || msg->gaussians.empty())
    {
        LOG_WARN(_ego_robot_ns + ": Received invalid trajectory message from " + ns + ". Ignoring.");
        return;
    }

    // Get the trajectory-based obstacle for this robot namespace
    auto &robot_trajectory_obstacle = it->second;

    // Check if the obstacle id in msg corresponds with retrieved trajectory obstacle
    if (robot_trajectory_obstacle.index != msg->id)
    {
        LOG_WARN(_ego_robot_ns + ": Trajectory obstacle ID mismatch for robot " + ns +
                 " - expected ID: " + std::to_string(robot_trajectory_obstacle.index) +
                 ", received ID: " + std::to_string(msg->id));
        return;
    }

    // Update current pose of the trajectory obstacle
    const auto &position = msg->pose.position;
    const auto &orientation = msg->pose.orientation;
    robot_trajectory_obstacle.position = Eigen::Vector2d(position.x, position.y);
    robot_trajectory_obstacle.angle = RosTools::quaternionToAngle(orientation);

    // Get trajectory predictions
    const auto &list_of_gaussians_trajectories = msg->gaussians;
    const auto &list_of_traject_costs = msg->probabilities;

    if (!list_of_gaussians_trajectories.empty())
    {
        // FIXED: We expect only 1 gaussian from the direct trajectory publisher
        if (list_of_gaussians_trajectories.size() != 1)
        {
            LOG_WARN(_ego_robot_ns + ": Expected 1 trajectory, got " + std::to_string(list_of_gaussians_trajectories.size()) + " from " + ns);
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
            const auto &pose = mean_trajectory.poses[k];
            new_mode.emplace_back(
                Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y),
                RosTools::quaternionToAngle(pose.pose.orientation),
                -1.0, // uncertainty ellipse major axis
                -1.0  // uncertainty ellipse minor axis
            );
        }

        // Replace the auto-created empty mode with our trajectory data
        new_prediction.modes[0] = std::move(new_mode);

        // Update the obstacle with the new prediction
        robot_trajectory_obstacle.prediction = std::move(new_prediction);

        LOG_DEBUG(_ego_robot_ns + " Updated trajectory prediction for " + ns +
                  " with " + std::to_string(trajectory_size) + " trajectory points");
    }
    else
    {
        // No trajectory prediction available - use deterministic (current pose only)
        LOG_ERROR(_ego_robot_ns + " No trajectory prediction for " + ns + ", but we received a callback");
    }

    _first_direct_trajectory_cb_received = true;
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
void JulesJackalPlanner::publishCurrentTrajectory(MPCPlanner::PlannerOutput output)
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
    LOG_INFO("Objective reached - published event");
}
void JulesJackalPlanner::publishDirectTrajectory(MPCPlanner::PlannerOutput output)
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
            pose.pose.position.z = k * output.trajectory.dt;
            pose.pose.orientation = RosTools::angleToQuaternion(output.trajectory.orientations.at(k));
            pose.header.stamp = ros_time + ros::Duration(k * output.trajectory.dt);

            // Add dummy semiaxis values for now (can be made configurable later)
            gaussian.major_semiaxis.push_back(0.1);
            gaussian.minor_semiaxis.push_back(0.1);

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
std::set<std::string> JulesJackalPlanner::identifyOtherRobotNamespaces(const std::vector<std::string> &all_namespaces)
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
void JulesJackalPlanner::waitForAllRobotsReady(ros::NodeHandle &nh)
{
    std::string my_namespace = ros::this_node::getNamespace();

    std::vector<std::string> robot_list;
    if (!nh.getParam("/robot_ns_list", robot_list))
    {
        LOG_WARN(_ego_robot_ns + ": No robot_ns_list found, starting immediately");
        return;
    }

    std::string my_ready_param = my_namespace + "/ready_to_start";
    nh.setParam(my_ready_param, true);
    LOG_INFO("Robot " + my_namespace + " marked as ready, waiting for others...");
    LOG_DIVIDER();

    ros::Rate check_rate(50);
    while (ros::ok())
    {
        bool all_ready = true;
        for (const auto &robot_ns : robot_list)
        {
            bool robot_ready = false;
            std::string robot_param = robot_ns + "/ready_to_start";
            if (!nh.getParam(robot_param, robot_ready) || !robot_ready)
            {
                all_ready = false;
                break;
            }
        }

        if (all_ready)
        {
            LOG_INFO(_ego_robot_ns + ": All robots ready! Starting control loop...");
            LOG_DIVIDER();
            break;
        }

        check_rate.sleep();
    }
}
void JulesJackalPlanner::initializeOtherRobotsAsObstacles(const std::set<std::string> &other_robot_namespaces, MPCPlanner::RealTimeData &data, const double radius)
{
    for (const auto &robot_ns : other_robot_namespaces)
    {
        data.trajectory_dynamic_obstacles.emplace(robot_ns, MPCPlanner::DynamicObstacle(extractRobotIdFromNamespace(robot_ns), radius, MPCPlanner::ObstacleType::DYNAMIC));
    }
}

// 8. HELPER/UTILITY FUNCTIONS
//    - Static functions and utility methods
void JulesJackalPlanner::updateRobotObstaclesFromTrajectories()
{
    LOG_INFO(_ego_robot_ns + ": Updating robot obstacles from " + std::to_string(_data.trajectory_dynamic_obstacles.size()) + " trajectory sources");

    int updated_count = 0;
    int added_count = 0;

    for (auto [ns, trajectory_obs] : _data.trajectory_dynamic_obstacles)
    {
        // Find and update existing robot obstacle or add new one
        auto it = std::find_if(_data.dynamic_obstacles.begin(), _data.dynamic_obstacles.end(),
                               [&](const auto &obs)
                               { return obs.index == trajectory_obs.index; });

        if (it != _data.dynamic_obstacles.end())
        {
            LOG_INFO(_ego_robot_ns + ": Updating existing obstacle for robot " + ns + " (ID: " + std::to_string(trajectory_obs.index) + ")");
            *it = trajectory_obs;
            updated_count++;
        }
        else
        {
            LOG_INFO(_ego_robot_ns + ": Adding new obstacle for robot " + ns + " (ID: " + std::to_string(trajectory_obs.index) + ")");
            _data.dynamic_obstacles.push_back(trajectory_obs);
            added_count++;
        }
    }

    LOG_INFO(_ego_robot_ns + ": Robot obstacle update complete - Updated: " + std::to_string(updated_count) + ", Added: " + std::to_string(added_count) + ", Total obstacles: " + std::to_string(_data.dynamic_obstacles.size()));
}

std::string JulesJackalPlanner::dataToString() const
{
    std::string result = "RealTimeData{\n";

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

void JulesJackalPlanner::logDataState(const std::string &context) const
{
    std::string ctx = context.empty() ? "DATA_STATE" : context;
    LOG_INFO(_ego_robot_ns + ": ========== " + ctx + " ==========");
    LOG_INFO(dataToString());
    LOG_INFO(_ego_robot_ns + ": ========================" + std::string(ctx.length(), '=') + "==========");
}

bool JulesJackalPlanner::objectiveReached(MPCPlanner::State _state, MPCPlanner::RealTimeData _data) const
{
    // check if the objective for each robot is reached if it is reached then we return 0.
    bool objective_reached = _planner->isObjectiveReached(_state, _data);
    return objective_reached;
}
int JulesJackalPlanner::extractRobotIdFromNamespace(const std::string &ns)
{
    // Handle both "/jackalX" and "jackalX" formats
    if (ns.front() == '/')
        return std::stoi(ns.substr(7)); // skip "/jackal"
    else
        return std::stoi(ns.substr(6)); // skip "jackal"
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
        LOG_WARN("Creating a braking command trajectory while the MPC solve was signaled as successful");
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

    LOG_WARN(_ego_robot_ns << " Creating a braking command trajectory");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "JULES_jackalplanner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Optional: initialize the shared visualization helper used by the repo.
    // If you’re not linking ros_tools/visuals, you can remove this line, and
    // also remove calls to visualize() or planner_->visualize(...).
    VISUALS.init(&nh);

    // Construct your per-robot planner wrapper.
    // Why here: constructing AFTER ros::init + NodeHandles ensures ROS is ready.
    // The constructor will:
    //  - load config,
    //  - create _planner (unique_ptr),
    //  - wire up pubs/subs (relative topics),
    //  - start the control loop timer.
    JulesJackalPlanner planner(nh, pnh);

    // Hand control to ROS: this pumps callbacks (subs, timers) in this thread.
    // Single-threaded spinner keeps your planner loop and callbacks serialized,
    // which is simpler. If you ever need concurrency, switch to MultiThreadedSpinner.
    ros::spin();

    // On shutdown (Ctrl+C or node kill), ros::spin() returns, planner's destructor
    // runs (clean up timers/pubs/subs), then we exit main.
    return 0;
}
