#pragma once

#include <mpc_planner_jackalsimulator/jackalsimulator_reconfigure.h>
#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_types/realtime_data.h>

#include <mpc_planner_msgs/ObstacleArray.h>
#include <mpc_planner_msgs/GetOtherTrajectories.h>
#include <ros_tools/profiling.h>

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/Joy.h>

#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

#include <tf2_ros/transform_broadcaster.h>
#include <set>
#include <memory>
#include <ros/console.h>

namespace MPCPlanner
{
    class Planner;
    struct PlannerOutput;
}

class JulesJackalPlanner
{
public:
    explicit JulesJackalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~JulesJackalPlanner();

    // Wiring
    void initializeSubscribersAndPublishers(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void subscribeToOtherRobotTopics(ros::NodeHandle &nh, const std::set<std::string> &other_robot_namespaces);

    // Planning loop functions
    void loop(const ros::TimerEvent &event);
    void loopDirectTrajectory(const ros::TimerEvent &event);
    void loopWithService(const ros::TimerEvent &event);
    void loopDirectTrajectoryStateMachine(const ros::TimerEvent &event);

    // Callbacks
    void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg);
    void obstacleServiceCallback(const mpc_planner_msgs::ObstacleArray &msg);

    void poseOtherRobotCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string ns);
    void trajectoryCallback(const mpc_planner_msgs::ObstacleGMM::ConstPtr &msg, const std::string ns);
    void allRobotsReachedObjectiveCallback(const std_msgs::Bool::ConstPtr &msg);

    // publish functions
    void publishCurrentTrajectory(const MPCPlanner::PlannerOutput &output);
    void publishObjectiveReachedEvent();
    void publishEgoPose();
    void publishDirectTrajectory(const MPCPlanner::PlannerOutput &output);

    // Apply commands
    void applyBrakingCommand(geometry_msgs::Twist &cmd);

    // Utils
    void visualize();

    bool initializeOtherRobotsAsObstacles(const std::set<std::string> &_other_robot_nss, MPCPlanner::RealTimeData &_data, const double);
    bool objectiveReached(MPCPlanner::State _state, MPCPlanner::RealTimeData _data) const;
    void buildOutputFromBrakingCommand(MPCPlanner::PlannerOutput &output, const geometry_msgs::Twist &cmd);
    void logDataState(const std::string &context = "") const;

private:
    bool isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const;
    double estimateYaw(const geometry_msgs::Quaternion &q) const;

    // Functions structuring planning phase:
    void prepareObstacleData();
    void interpolateTrajectoryPredictionsByTime();

    std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> generatePlanningCommand(const MPCPlanner::PlannerState &current_state);
    void publishCmdAndVisualize(const geometry_msgs::Twist &cmd, const MPCPlanner::PlannerOutput &output);
    void rotatePiRadiansCw(geometry_msgs::Twist &cmd);

private:
    // Core MPC types
    std::unique_ptr<MPCPlanner::Planner> _planner;

    // Planner data that is updated via callbacks

    MPCPlanner::State _state;
    MPCPlanner::RealTimeData _data;

    // dynamic reconfigure instance
    std::unique_ptr<JackalsimulatorReconfigure> _reconfigure;

    // ROS Subscribers
    ros::Subscriber _state_sub;
    ros::Subscriber _state_pose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacles_sub;
    ros::Subscriber _all_robots_reached_objective_sub;             // Subscriber for central aggregator signal
    std::vector<ros::Subscriber> _other_robot_pose_sub_list;       // List of otherrobot pose subcribers
    std::vector<ros::Subscriber> _other_robot_trajectory_sub_list; // List of otherRobot trajectory subscribers

    // ROS publishers
    ros::Publisher _cmd_pub;
    ros::Publisher _pose_pub;
    ros::Publisher _objective_pub;         // events/objective_reached
    ros::Publisher _trajectory_pub;        // publish the trajectory the robots is about to follow, this one publishes first to the central aggregator
    ros::Publisher _direct_trajectory_pub; // this publishes to a robot immediately so no central aggregator in between
    ros::Publisher _reverse_roadmap_pub;
    ros::Timer _timer;

    std::unique_ptr<RosTools::Timer> _startup_timer;
    ros::ServiceClient _trajectory_client;

    // Simulation reset:
    std_srvs::Empty _reset_msg;
    ros::Publisher _reset_simulation_pub;
    ros::ServiceClient _reset_simulation_client;

    // Config
    std::string _global_frame{"map"};
    std::string _ego_robot_ns;
    int _ego_robot_id{-1};
    std::set<std::string> _other_robot_nss;          // List of namespaces of all other robots in the area excluding ego robot ns
    std::vector<std::string> _robot_ns_list;         // List of namespaces of all robots in the area
    bool _immediate_robot_robot_communication{true}; // This boolean can turn on and off immideate robot to robot communication, which means there is no central aggregator in between

    bool _enable_output{true};
    double _control_frequency{20.0};
    double _infeasible_deceleration{1.0};
    double _goal_tolerance{0.8};
    bool _received_obstacle_callback_first_time{true};
    bool _have_received_meaningful_trajectory_data{false}; // Track trajectory data readiness
    bool _stop_when_reached_goal{false};                   // This is a configuration parameter that determines if we stop at out goal or that we will rotate pi radians.
    bool _communicate_on_topology_switch_only{false};       // This is a configuration parameter that determiens if an _ego_robot only communicates its trajectory when a topology switch is detected.

    std::set<std::string>
        _validated_trajectory_robots; // this set will record whihc robot has send a correct trajectory, this is important during the initialization phase.

    // Goal cache
    bool _goal_received{false};

    Eigen::Vector2d _goal_xy{0.0, 0.0};

private:
    // StateMachine logic
    MPCPlanner::PlannerState _current_state{MPCPlanner::PlannerState::UNINITIALIZED};
    MPCPlanner::PlannerState _previous_state{MPCPlanner::PlannerState::UNINITIALIZED};
    // void transitionTo(MPCPlanner::PlannerState &_current_state, const MPCPlanner::PlannerState &new_state, const std::string &ego_robot_ns);
    // bool canTransitionTo(const MPCPlanner::PlannerState &_current_state, const MPCPlanner::PlannerState &new_state, const std::string &ego_robot_ns);
    // void onStateEnter(const MPCPlanner::PlannerState &current_state, const MPCPlanner::PlannerState &new_state, const std::string &ego_robot_ns);
};
