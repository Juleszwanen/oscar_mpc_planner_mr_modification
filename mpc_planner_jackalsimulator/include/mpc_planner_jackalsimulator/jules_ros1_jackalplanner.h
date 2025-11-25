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
#include <mpc_planner/experiment_util.h>
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

// Forward declarations
namespace JackalPlanner {
    struct InitializationConfig;
}

namespace MPCPlanner
{
    class Planner;
    struct PlannerOutput;
}

class JulesJackalPlanner
{
public:
    explicit JulesJackalPlanner(ros::NodeHandle &nh);
    ~JulesJackalPlanner();

    // Wiring
    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);
    void subscribeToOtherRobotTopics(ros::NodeHandle &nh, const std::set<std::string> &other_robot_namespaces);

    bool objectiveReached(MPCPlanner::State& _state, MPCPlanner::RealTimeData& _data) const;
    void rotateToGoal(geometry_msgs::Twist &cmd);
    void reset();

    bool isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const;
    void visualize();
private:
    // ===== Initialization helper methods =====
    void loadSimulatorPlatformParameters(ros::NodeHandle& nh, JackalPlanner::InitializationConfig& config);
    void applyConfiguration(const JackalPlanner::InitializationConfig& config);
    // bool initializeOtherRobotsAsObstaclesWithNonCom(const std::set<std::string>& other_robot_namespaces, MPCPlanner::RealTimeData& data, double robot_radius);
    void initializeSimulatorComponents(ros::NodeHandle& nh, const JackalPlanner::InitializationConfig& config);
    void initializeTimersAndStateMachine(ros::NodeHandle& nh, const JackalPlanner::InitializationConfig& config);

public:
    bool initializeOtherRobotsAsObstacles(const std::set<std::string> &other_robot_namespaces, MPCPlanner::RealTimeData &data, const double radius);
    // Planning loop functions


    void prepareObstacleData();
    void interpolateTrajectoryPredictionsByTime();
    std::pair<geometry_msgs::Twist, MPCPlanner::PlannerOutput> generatePlanningCommand(const MPCPlanner::PlannerState &current_state);
    void applyBrakingCommand(geometry_msgs::Twist &cmd);
    void buildOutputFromBrakingCommand(MPCPlanner::PlannerOutput &output, const geometry_msgs::Twist &cmd);
    void publishCmdAndVisualize(const geometry_msgs::Twist &cmd, const MPCPlanner::PlannerOutput &output);
    void publishDirectTrajectory(const MPCPlanner::PlannerOutput &output);
    void publishObjectiveReachedEvent();

public:
    void loop(const ros::TimerEvent &event);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg);
    

public:
    void poseOtherRobotCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string ns);
    void trajectoryCallback(const mpc_planner_msgs::ObstacleGMM::ConstPtr &msg, const std::string ns);
    void allRobotsReachedObjectiveCallback(const std_msgs::Bool::ConstPtr &msg);
    // void rqtDeadManSwitchCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void julesControllerCallback(const sensor_msgs::Joy::ConstPtr &msg);
    
    void saveDataStateBased();
   
    // Communication helper functions
    bool decideCommunication(const MPCPlanner::PlannerOutput &output);
    void recordCommunicationDecision(bool communicated);
    void logCommunicationDecision(bool communicated, const MPCPlanner::PlannerOutput &output);

    // Communication triggers (orchestration - delegates to CommunicationTriggers utility)
    bool shouldCommunicate(const MPCPlanner::PlannerOutput &output, const MPCPlanner::RealTimeData &data);

private:
    // Core MPC types
    std::unique_ptr<MPCPlanner::Planner> _planner;
    std::unique_ptr<JackalsimulatorReconfigure> _reconfigure;
    // Planner data that is updated via callbacks

    MPCPlanner::RealTimeData _data;
    MPCPlanner::State _state;
    
    ros::Timer _timer;

    bool _enable_output{false};
    bool _rotate_to_goal{false};
    // bool _forward_x_experiment{true};

    // double _measured_velocity{0.};

    // double y_max{2.4}; // 2.6 when the blocks are not at the wall
    // double y_min{-2.0};
    // double x_max{3.6};
    // double x_min{-3.6};

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;
    
    // ROS Subscribers
    ros::Subscriber _state_pose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacle_sub;
    // ros::Subscriber _bluetooth_sub;
    // ros::Subscriber _rqt_dead_man_sub;
    ros::Subscriber _jules_controller_sub;

    ros::Subscriber _all_robots_reached_objective_sub;             // Subscriber for central aggregator signal
    std::vector<ros::Subscriber> _other_robot_pose_sub_list;       // List of otherrobot pose subcribers
    std::vector<ros::Subscriber> _other_robot_trajectory_sub_list; // List of otherRobot trajectory subscribers

    ros::Publisher _reverse_roadmap_pub;
    ros::Publisher _cmd_pub;


    // ROS publishers
    ros::Publisher _pose_pub;              // Publish your own pose in the system
    ros::Publisher _direct_trajectory_pub; // this publishes to a robot immediately so no central aggregator in between
    ros::Publisher _objective_pub;         // events/objective_reached
    
    std::unique_ptr<RosTools::Timer> _startup_timer;

   
private:
    std::string _global_frame{"map"};
    std::string _ego_robot_ns{"/jackalX"};
    int _ego_robot_id{-1};
    int _num_non_com_obj{0};
    std::set<std::string> _other_robot_nss;
    std::vector<std::string> _robot_ns_list;

    double _goal_tolerance{0.8};
    bool _stop_when_reached_goal{false};

    double _control_frequency{20.0};
    bool _communicate_on_topology_switch_only{false};
    bool _rqt_dead_man_switch{false};
    bool _jules_controller_deadman_switch{false};
    std::set<std::string> _validated_trajectory_robots;

    MPCPlanner::PlannerState _current_state{MPCPlanner::PlannerState::UNINITIALIZED};
    MPCPlanner::PlannerState _previous_state{MPCPlanner::PlannerState::UNINITIALIZED};
};

    

    


