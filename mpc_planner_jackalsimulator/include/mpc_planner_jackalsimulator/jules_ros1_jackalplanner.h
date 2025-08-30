#pragma once

#include <mpc_planner_jackalsimulator/jackalsimulator_reconfigure.h>
#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_types/realtime_data.h>

#include <mpc_planner_msgs/ObstacleArray.h>

#include <ros_tools/profiling.h>

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/Joy.h>

#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>

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
    void loop(const ros::TimerEvent &event);

    // Callbacks
    void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg);

    // Utils
    void publishPose();
    void publishCurrentTrajectory(MPCPlanner::PlannerOutput output);
    void visualize();
    bool objectiveReached() const;

private:
    bool isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const;
    double estimateYaw(const geometry_msgs::Quaternion &q) const;

private:
    // Core MPC types
    std::unique_ptr<MPCPlanner::Planner> _planner;

    MPCPlanner::State _state;
    MPCPlanner::RealTimeData _data;

    // ROS I/O
    ros::Subscriber _state_sub;
    ros::Subscriber _state_pose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacles_sub;

    ros::Publisher _cmd_pub;
    ros::Publisher _pose_pub;
    ros::Publisher _objective_pub;  // events/objective_reached
    ros::Publisher _trajectory_pub; // publish the trajectory the robots is about to follow

    ros::Timer _timer;

    // Config
    std::string _global_frame{"map"};
    bool _enable_output{true};
    double _control_frequency{20.0};
    double _infeasible_deceleration{1.0};
    double _goal_tolerance{0.5};

    // Goal cache
    bool _goal_received{false};
    Eigen::Vector2d _goal_xy{0.0, 0.0};
};
