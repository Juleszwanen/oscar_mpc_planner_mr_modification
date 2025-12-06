#pragma once

#include <mpc_planner/planner.h>
#include <mpc_planner_types/data_types.h>
#include <ros/time.h>
#include <string>

namespace MPCPlanner {

    /**
     * @brief Reasons for triggering a communication broadcast
     */
    enum class CommunicationTriggerReason {
        NO_COMMUNICATION = 0,
        INFEASIBLE = 1,
        INFEASIBLE_TO_FEASIBLE = 2,
        TOPOLOGY_CHANGE = 3,
        GEOMETRIC = 4,
        TIME = 5,
        NON_GUIDED_HOMOLOGY_FAIL = 6
    };

    /**
     * @brief Convert trigger reason to string for logging
     */
    std::string toString(CommunicationTriggerReason reason);
    

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
    // =========================================================================
    // 1. Infeasible Trigger (Enum 1)
    // =========================================================================
    /**
     * @brief Check if the MPC solver failed to find a solution.
     */
    static bool checkInfeasible(const PlannerOutput& output);

    // =========================================================================
    // 3. Real Topology Change Trigger (Enum 3)
    // =========================================================================
    /**
     * @brief Check if we switched between valid guided topologies.
     * Excludes switches to/from non-guided mode (handled by Enum 6).
     */
    static bool checkTopologyChange(const PlannerOutput& output, int n_paths);

    // =========================================================================
    // 4. Geometric Trigger (Enum 4)
    // =========================================================================
    /**
     * @brief Check if the trajectory deviates geometrically from the last sent one.
     */
    static bool checkGeometricDeviation(
        const Trajectory& current_trajectory,
        const Trajectory& last_communicated_trajectory,
        double max_deviation_threshold);

    // =========================================================================
    // 5. Time Trigger (Enum 5)
    // =========================================================================
    /**
     * @brief Check if enough time has elapsed (heartbeat).
     */
    static bool checkTime(const ros::Time& last_send_time,const ros::Time& current_time,double heartbeat_period_sec);

    // =========================================================================
    // 6. Non-Guided / Homology Fail Trigger (Enum 6)
    // =========================================================================
    /**
     * @brief Check if the solver selected the non-guided topology (fallback).
     * This happens when no matching homology was found or it was the best option.
     */
    static bool checkNonGuidedHomologyFail(const PlannerOutput& output, int n_paths);
};

} // namespace MPCPlanner




