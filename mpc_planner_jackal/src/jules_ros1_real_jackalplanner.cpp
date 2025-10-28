#include <mpc_planner_jackal/jules_ros1_real_jackalplanner.h>
#include <mpc_planner/data_preparation.h>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>
#include <mpc_planner_util/multi_robot_utility_functions.h>

#include <ros_tools/profiling.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>

#include <std_msgs/Empty.h>

JulesRealJackalPlanner::JulesRealJackalPlanner(ros::NodeHandle &nh)
{
    LOG_HEADER("JULES: Jackal planner " + ros::this_node::getName() + " starting");

    _ego_robot_ns = ros::this_node::getNamespace();
    _ego_robot_id = MultiRobot::extractRobotIdFromNamespace(_ego_robot_ns);

    if (!nh.getParam("/robot_ns_list", _robot_ns_list))
    {
        LOG_ERROR(_ego_robot_ns + ": No robot_ns_list param found");
    }

    _other_robot_nss = MultiRobot::identifyOtherRobotNamespaces(_robot_ns_list, _ego_robot_ns);
    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    _data.robot_area = {MPCPlanner::Disc(0., CONFIG["robot_radius"].as<double>())};

    this->_communicate_on_topology_switch_only = CONFIG["JULES"]["communicate_on_topology_switch_only"];
    bool initialization_successful = this->initializeOtherRobotsAsObstacles(_other_robot_nss, _data, CONFIG["robot_radius"].as<double>());

    nh.param("frames/global", this->_global_frame, this->_global_frame);
    nh.param("goal_tolerance", this->_goal_tolerance, this->_goal_tolerance);

    // Initialize the planner
    _planner = std::make_unique<MPCPlanner::Planner>(_ego_robot_ns);

    // Initialize the ROS interface
    initializeSubscribersAndPublishers(nh);

    _reconfigure = std::make_unique<JackalReconfigure>();

    _startup_timer = std::make_unique<RosTools::Timer>();
    _startup_timer->setDuration(15.0);
    _startup_timer->start();

    MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::TIMER_STARTUP, _ego_robot_ns);
    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    RosTools::Instrumentor::Get().BeginSession("mpc_planner_jackal");

    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &JulesRealJackalPlanner::loop,
        this);

    LOG_DIVIDER();
}

JulesRealJackalPlanner::~JulesRealJackalPlanner()
{
    LOG_INFO(_ego_robot_ns + ": Stopped Jackal Planner");
    RosTools::Instrumentor::Get().EndSession();
}

void JulesRealJackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    LOG_INFO("initializeSubscriberAndPublisher");

    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "input/state", 5,
        boost::bind(&JulesRealJackalPlanner::stateCallback, this, _1));

    /** @note Jules: The roadmap publishes on this whenever the planner node has send a signal to the roadmap node that is want to reverse*/
    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/goal", 1,
        boost::bind(&JulesRealJackalPlanner::goalCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>(
        "input/reference_path", 1,
        boost::bind(&JulesRealJackalPlanner::pathCallback, this, _1));

    _obstacle_sub = nh.subscribe<derived_object_msgs::ObjectArray>(
        "input/obstacles", 1,
        boost::bind(&JulesRealJackalPlanner::obstacleCallback, this, _1));

    _bluetooth_sub = nh.subscribe<sensor_msgs::Joy>(
        "input/bluetooth", 1,
        boost::bind(&JulesRealJackalPlanner::bluetoothCallback, this, _1)); // Deadman switch

    _all_robots_reached_objective_sub = nh.subscribe<std_msgs::Bool>(
        "/all_robots_reached_objective", 1,
        boost::bind(&JulesRealJackalPlanner::allRobotsReachedObjectiveCallback, this, _1));

    this->subscribeToOtherRobotTopics(nh, _other_robot_nss);

    _cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "output/command", 1);

    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("output/pose", 1);

    _direct_trajectory_pub = nh.advertise<mpc_planner_msgs::ObstacleGMM>("robot_to_robot/output/current_trajectory", 1);

    _objective_pub = nh.advertise<std_msgs::Bool>("events/objective_reached", 1);

    // Roadmap reverse
    _reverse_roadmap_pub = nh.advertise<std_msgs::Empty>("roadmap/reverse", 1);
}

void JulesRealJackalPlanner::loop(const ros::TimerEvent &event)
{
    (void)event; // Jules event contains alot of member variables which you could use
    LOG_MARK("============= Loop =============");

    // _data.planning_start_time = std::chrono::system_clock::now();

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
            MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE, _ego_robot_ns);
        }

        break;
    }

    // We need to know if the poseCallback has gone of, it has gone of if the _state has valid data
    case MPCPlanner::PlannerState::WAITING_FOR_FIRST_EGO_POSE:
    {
        if (_state.validData())
        {
            MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::INITIALIZING_OBSTACLES, _ego_robot_ns);
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
        MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::WAITING_FOR_TRAJECTORY_DATA, _ego_robot_ns);
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
        if (this->objectiveReached())
        {
            MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::JUST_REACHED_GOAL, _ego_robot_ns);
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
        MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::GOAL_REACHED, _ego_robot_ns);
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

    LOG_DEBUG("============= End Loop =============");
}

void JulesRealJackalPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _measured_velocity = std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.));
}

void JulesRealJackalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // This callback is triggered by the roadmap module on reverse roadmap call
    LOG_WARN("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;

    _planner->onDataReceived(_data, "goal");
}

void JulesRealJackalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    if (isPathTheSame(msg))
        return;

    _data.reference_path.clear();

    for (auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
    }
    _data.reference_path.psi.push_back(0.0);
    _planner->onDataReceived(_data, "reference_path");
}

void JulesRealJackalPlanner::obstacleCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg)
{
}

void JulesRealJackalPlanner::bluetoothCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->axes[2] < -0.9 && !_enable_output)
        LOG_INFO(_ego_robot_ns + ": Planning enabled (deadman switch pressed)");
    else if (msg->axes[2] > -0.9 && _enable_output)
        LOG_INFO(_ego_robot_ns + ": Deadmanswitch enabled (deadman switch released)");

    _enable_output = msg->axes[2] < -0.9;
    CONFIG["enable_output"] = _enable_output;
}

void JulesRealJackalPlanner::poseOtherRobotCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string ns)
{
}

void JulesRealJackalPlanner::trajectoryCallback(const mpc_planner_msgs::ObstacleGMM::ConstPtr &msg, const std::string ns)
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
                MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::PLANNING_ACTIVE, _ego_robot_ns);
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
        break;
    }

    default:
        LOG_WARN(_ego_robot_ns + ": Received trajectory callback in unknown state: " +
                 std::to_string(static_cast<int>(_current_state)));
        break;
    }
}

void JulesRealJackalPlanner::allRobotsReachedObjectiveCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (!(msg->data))
    {
        LOG_ERROR(_ego_robot_ns + ": CentralAggregator send signal that all robots reached objective but data is false: " + std::to_string(msg->data));
        return;
    }

    LOG_INFO(_ego_robot_ns + ": Received message that all robots have reached their objective - resetting for new planning cycle");

    // Reset trajectory data readiness to ensure fresh start
    // _have_received_meaningful_trajectory_data = false;

    // In the simulation one, we do the roadmap reverse here but that is not neccessary
}

bool JulesRealJackalPlanner::objectiveReached()
{
    bool reset_condition_forward_x = (_forward_x_experiment) && (_state.get("x") > x_max || _state.get("y") > y_max);
    bool reset_condition_backward_x = (!_forward_x_experiment) && (_state.get("x") < x_min || _state.get("y") < y_min);
    bool reset_condition = reset_condition_forward_x || reset_condition_backward_x;
    if (reset_condition)
    {
        _forward_x_experiment = !_forward_x_experiment;
    }
    return reset_condition;
}

void JulesRealJackalPlanner::rotateToGoal(geometry_msgs::Twist &cmd)
{
    LOG_INFO_THROTTLE(1500, _ego_robot_ns + ": Rotating to the goal");
    if (!_data.goal_received)
    {
        LOG_INFO(_ego_robot_ns + ": Waiting for the goal");
        return;
    }

    double goal_angle = std::atan2(_data.goal(1) - _state.get("y"), _data.goal(0) - _state.get("x"));
    double angle_diff = goal_angle - _state.get("psi");

    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;

    if (std::abs(angle_diff) > 0.1)
    {
        cmd.linear.x = 0.0;
        if (_enable_output)
            cmd.angular.z = 1.5 * RosTools::sgn(angle_diff);
        else
            cmd.angular.z = 0.;
    }
    else
    {
        // We publish that we reached our objective, it does not matter that we do this multiple times
        publishObjectiveReachedEvent();
        MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::RESETTING, _ego_robot_ns);
        LOG_INFO(_ego_robot_ns + " Is waiting for the rest of the robots to reach their objective before starting to follow reverserd path");
    }
}

void JulesRealJackalPlanner::reset()
{
    //  This is here for legacy reasons, the resetting of the robot is done
    // LOG_INFO(_ego_robot_ns + ": Resetting");

    // std_msgs::Empty empty_msg;
    // _reverse_roadmap_pub.publish(empty_msg);

    // _planner->reset(_state, _data);
    // _rotate_to_goal = true;

    _planner->reset(_state, _data, true); // reset the complete planner: solver, modules, state and data

    _validated_trajectory_robots.clear();       // clear the set which records which robots have send a correct trajectory
    _data.trajectory_dynamic_obstacles.clear(); //

    LOG_DIVIDER();
    LOG_INFO(_ego_robot_ns + ": Cleared all data structures - " +
             "validated_trajectory_robots size: " + std::to_string(_validated_trajectory_robots.size()) + ", " +
             "trajectory_dynamic_obstacles size: " + std::to_string(_data.trajectory_dynamic_obstacles.size()) + ", " +
             "dynamic_obstacles size: " + std::to_string(_data.dynamic_obstacles.size()) + ", " +
             "current state: [x=" + std::to_string(_state.get("x")) +
             ", y=" + std::to_string(_state.get("y")) +
             ", psi=" + std::to_string(_state.get("psi")) +
             ", v=" + std::to_string(_state.get("v")) + "]");

    _startup_timer->setDuration(4.0); // Give time for roadmap to reversqe and publish new path
    _startup_timer->start();

    MultiRobot::transitionTo(_current_state, MPCPlanner::PlannerState::TIMER_STARTUP, _ego_robot_ns);
    LOG_INFO(_ego_robot_ns + ": Ready to resume planning with new objectives in 2 seconds");
}

void JulesRealJackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("limits");
    auto &line = publisher.getNewLine();
    line.setColor(0., 0., 0.);
    line.setScale(0.1, 0.1);

    // Publish lab limits
    line.addLine(Eigen::Vector2d(x_min, y_min), Eigen::Vector2d(x_max, y_min));
    line.addLine(Eigen::Vector2d(x_max, y_min), Eigen::Vector2d(x_max, y_max));
    line.addLine(Eigen::Vector2d(x_max, y_max), Eigen::Vector2d(x_min, y_max));
    line.addLine(Eigen::Vector2d(x_min, y_max), Eigen::Vector2d(x_min, y_min));

    publisher.publish();

    auto &goal_publisher = VISUALS.getPublisher("goal");
    auto &cube = goal_publisher.getNewPointMarker("CUBE");

    cube.setScale(0.05, 0.05, 0.2);
    cube.setColorInt(4, 5);

    cube.addPointMarker(Eigen::Vector3d(_data.goal(0), _data.goal(1), 0.0));
    goal_publisher.publish();
}

bool JulesRealJackalPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
{
    // Check if the path is the same
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Check up to the first two points
    int num_points = std::min(2, (int)_data.reference_path.x.size());
    for (int i = 0; i < num_points; i++)
    {
        if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
            return false;
    }
    return true;
}

bool JulesRealJackalPlanner::initializeOtherRobotsAsObstacles(const std::set<std::string> &other_robot_namespaces,
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

void JulesRealJackalPlanner::subscribeToOtherRobotTopics(ros::NodeHandle &nh, const std::set<std::string> &other_robot_namespaces)
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
                                                                   boost::bind(&JulesRealJackalPlanner::poseOtherRobotCallback, this, _1, ns));
        this->_other_robot_pose_sub_list.push_back(sub_pose_i);

        // Subscribe to robot trajectory predictions
        const std::string topic_trajectory = ns + "/robot_to_robot/output/current_trajectory";
        LOG_INFO(_ego_robot_ns + " is subscribing to: " + topic_trajectory);
        auto sub_traject_i = nh.subscribe<mpc_planner_msgs::ObstacleGMM>(topic_trajectory, 1,
                                                                         boost::bind(&JulesRealJackalPlanner::trajectoryCallback, this, _1, ns));
        this->_other_robot_trajectory_sub_list.push_back(sub_traject_i);
    }
}

void JulesRealJackalPlanner::prepareObstacleData()
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
        // this->logDataState(_ego_robot_ns + ": Obstacle data prepared");
    }
}

void JulesRealJackalPlanner::interpolateTrajectoryPredictionsByTime()
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

        double min_age_threshold = 1.0 / CONFIG["control_frequency"].as<double>();
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
            ROSTOOLS_ASSERT(mode.size() == (n_measured - k), "The size of mode should be equal to N_original - k");
            LOG_DEBUG(_ego_robot_ns + ": Removed first " + std::to_string(k) +
                      " points from trajectory for " + ns);
        }

        // ============================================================
        // STEP 3: APPEND EXTRAPOLATED POINTS
        // ============================================================
        mode.insert(mode.end(), extrapolated_points.begin(), extrapolated_points.end());
        ROSTOOLS_ASSERT(mode.size() == (n_measured + 1), "The size of the mode should be equal to N_original + 1");

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

std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> JulesRealJackalPlanner::generatePlanningCommand(const MPCPlanner::PlannerState &current_state)
{
    geometry_msgs::Twist cmd;
    MPCPlanner::PlannerOutput output; // Initialize with default values

    // Remember that the _enable_output variable is triggered by the playstation controller
    // Lambda to handle MPC solving and command extraction
    // Captures: cmd & output by reference (to modify), this for class members
    auto solveMPCAndExtractCommand = [&cmd, &output, this]()
    {
        output = _planner->solveMPC(_state, _data);

        if (_enable_output && output.success)
        {
            cmd.linear.x = _planner->getSolution(1, "v");
            cmd.angular.z = _planner->getSolution(0, "w");
            _state.set("v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded", "v=" + std::to_string(cmd.linear.x) + ", w=" + std::to_string(cmd.angular.z));
            CONFIG["enable_output"] = true;
        }

        else if ((!_enable_output))
        {
            _state.set("v", _measured_velocity);
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
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
        std_msgs::Empty empty_msg;
        _reverse_roadmap_pub.publish(empty_msg); // this does not only reverse the reference path but also sends a goal to rotate pi radians to
        // Ask for the roadmap to be reve
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
            rotateToGoal(cmd);
            buildOutputFromBrakingCommand(output, cmd);
        }
        break;
    }
    default:
        break;
    }

    return {cmd, output};
}

void JulesRealJackalPlanner::applyBrakingCommand(geometry_msgs::Twist &cmd)
{
    _state.set("v", _measured_velocity);

    double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
    double velocity_after_braking;
    double velocity;
    double dt = 1. / CONFIG["control_frequency"].as<double>();

    velocity = _state.get("v");
    velocity_after_braking = velocity - deceleration * dt;

    cmd.linear.x = std::max(velocity_after_braking, 0.0); // Don't drive backwards
    cmd.angular.z = 0.0;
}

void JulesRealJackalPlanner::buildOutputFromBrakingCommand(MPCPlanner::PlannerOutput &output, const geometry_msgs::Twist &cmd)
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

void JulesRealJackalPlanner::publishCmdAndVisualize(const geometry_msgs::Twist &cmd, const MPCPlanner::PlannerOutput &output)
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

        LOG_VALUE_DEBUG("Communication", "Published trajectory (Topology: " +
                                             std::to_string(output.selected_topology_id) + ")");
    }
    else
    {
        LOG_VALUE_DEBUG("Communication", "Skipped (Same topology: " +
                                             std::to_string(output.selected_topology_id) + ")");
    }

    // Always visualize
    _planner->visualizeObstaclePredictionsPlanner(_state, _data, false);
    _planner->visualize(_state, _data);
    // visualize();
}

void JulesRealJackalPlanner::publishDirectTrajectory(const MPCPlanner::PlannerOutput &output)
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
}
void JulesRealJackalPlanner::publishObjectiveReachedEvent()
{
    std_msgs::Bool event;
    event.data = true;
    _objective_pub.publish(event);
    LOG_INFO_THROTTLE(1000, _ego_robot_ns + ": Objective reached - published event");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;
    auto jules_real_jackal_planner = std::make_shared<JulesRealJackalPlanner>(nh);
    VISUALS.init(&nh);
    ros::spin();
    return 0;
}