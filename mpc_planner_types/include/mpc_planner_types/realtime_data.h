#ifndef MPC_REALTIME_DATA_TYPES_H
#define MPC_REALTIME_DATA_TYPES_H

#include <mpc_planner_types/data_types.h>
#include <map>
#include <chrono>
#include <ros/ros.h>
namespace costmap_2d
{
    class Costmap2D;
}

namespace MPCPlanner
{

    struct RealTimeData
    {

        std::vector<Disc> robot_area;
        FixedSizeTrajectory past_trajectory;
        

        ros::Time last_send_trajectory_time{ros::Time(0)};
        std::vector<DynamicObstacle> dynamic_obstacles;
        std::map<std::string, DynamicObstacle> trajectory_dynamic_obstacles;

        costmap_2d::Costmap2D *costmap{nullptr}; // Costmap for static obstacles

        ReferencePath reference_path;
        Boundary left_bound, right_bound;

        Eigen::Vector2d goal;
        bool goal_received{false};

        // Variable which resembles if the ego_robot communicated its trajectory
        bool communicated_trajectory{false};

        // Feedback data
        double intrusion;

        std::chrono::system_clock::time_point planning_start_time;

        ~RealTimeData() = default;
        RealTimeData() = default;

        void reset()
        {
            // Copy data that should remain at reset
            std::vector<Disc> robot_area_copy = robot_area;

            *this = RealTimeData();

            robot_area = robot_area_copy;
            goal_received = false;
        }
    };

    // struct JulesRealTimeData : RealTimeData
    // {
    //     std::map<std::string, DynamicObstacle> trajectory_dynamic_obstacles;

    //     JulesRealTimeData() = default;

    //     void reset() override
    //     {
    //         // Copy data that should remain at reset
    //         std::vector<Disc> robot_area_copy = robot_area;

    //         *this = JulesRealTimeData();

    //         robot_area = robot_area_copy;
    //         goal_received = false;
    //     }

    // };
}
#endif