#pragma once

#include <mpc_planner/planner.h>
#include <mpc_planner_types/data_types.h>
#include <ros/time.h>
#include <string>

namespace MPCPlanner {

/**
 * @brief Pure strategy functions for evaluating multi-robot communication triggers
 * 
 * These stateless functions implement the core logic for deciding when robots 
 * should communicate trajectory updates to neighbors. They operate on MPC data 
 * types and can be used by any planner implementation (real hardware, simulation, etc.).
 * 
 * Design rationale:
 * - Stateless static methods enable independent unit testing
 * - Platform-agnostic logic promotes code reuse across planners
 * - Co-location with data types they operate on improves cohesion
 */
class CommunicationTriggers
{
public:
    /**
     * @brief Check if elapsed time since last communication warrants new transmission
     * 
     * This implements a heartbeat mechanism to ensure periodic updates even when
     * trajectory hasn't changed significantly.
     * 
     * @param last_send_time Time of last transmission (ros::Time(0) means never sent)
     * @param current_time Current timestamp
     * @param heartbeat_period_sec Minimum time between heartbeat transmissions
     * @return true if heartbeat interval reached or first transmission
     */
    static bool elapsedTimeTrigger(
        const ros::Time& last_send_time,
        const ros::Time& current_time,
        double heartbeat_period_sec);

    /**
     * @brief Check if topology changes warrant communication
     * 
     * Topology changes indicate significant planning mode shifts (e.g., switching
     * between guided paths, entering non-guided mode, or solver recovery from failure).
     * 
     * @param output MPC solver output containing topology information
     * @param n_paths Number of guided paths (non-guided topology = 2*n_paths)
     * @param reason Output parameter describing why trigger fired or didn't
     * @return true if topology switch, solver failure, or non-guided mode
     */
    static bool topologyTrigger(
        const PlannerOutput& output,
        int n_paths,
        std::string& reason);

    /**
     * @brief Check if geometric deviation between trajectories exceeds threshold
     * 
     * Compares newly planned trajectory against previously communicated trajectory
     * to detect significant deviations that warrant updating neighbors' beliefs.
     * 
     * @param current_trajectory Newly planned trajectory
     * @param last_communicated_trajectory Previously communicated trajectory (should be interpolated)
     * @param max_deviation_threshold Maximum allowed deviation in meters
     * @param reason Output parameter describing result
     * @return true if deviation exceeds threshold
     */
    static bool geometricDeviationTrigger(
        const Trajectory& current_trajectory,
        const Trajectory& last_communicated_trajectory,
        double max_deviation_threshold,
        std::string& reason);
};

} // namespace MPCPlanner
