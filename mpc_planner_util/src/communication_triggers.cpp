#include <mpc_planner_util/communication_triggers.h>
#include <ros_tools/logging.h>

namespace MPCPlanner {

bool CommunicationTriggers::elapsedTimeTrigger(
    const ros::Time& last_send_time,
    const ros::Time& current_time,
    double heartbeat_period_sec)
{
    // First time: last_send_time is uninitialized (0), so we SHOULD communicate
    if (last_send_time == ros::Time(0))
    {
        return true;  // Must communicate on first iteration
    }

    // Subsequent times: check if enough time has elapsed
    const ros::Duration heartbeat_period(heartbeat_period_sec);
    ros::Duration time_elapsed = current_time - last_send_time;
    return (time_elapsed >= heartbeat_period);
}

bool CommunicationTriggers::topologyTrigger(
    const PlannerOutput& output,
    int n_paths,
    std::string& reason)
{
    const int non_guided_topology_id = 2 * n_paths;

    // Always communicate on solver failure
    if (!output.success)
    {
        reason = "Communicating - MPC failed";
        return true;
    }

    // Communicate when switching to non-guided mode (indicates obstacle avoidance)
    if (output.following_new_topology && output.selected_topology_id == non_guided_topology_id)
    {
        reason = "Communicating - Switched to non-guided from topology " +
                 std::to_string(output.previous_topology_id);
        return true;
    }

    // Communicate on any topology switch (path change)
    if (output.following_new_topology)
    {
        reason = "Communicating - Topology switch from " +
                 std::to_string(output.previous_topology_id) + " to " +
                 std::to_string(output.selected_topology_id);
        return true;
    }

    // Communicate every iteration while in non-guided mode (dynamic obstacle avoidance)
    if (output.selected_topology_id == non_guided_topology_id)
    {
        reason = "Communicating - Staying in non-guided (unidentified topology)";
        return true;
    }

    // No communication needed - staying on same guided path
    reason = "NOT communicating yet - Same guided topology (" +
             std::to_string(output.selected_topology_id) + ")";
    return false;
}

bool CommunicationTriggers::geometricDeviationTrigger(
    const Trajectory& current_trajectory,
    const Trajectory& last_communicated_trajectory,
    double max_deviation_threshold,
    std::string& reason)
{
    // Early return if either trajectory lacks data - nothing to compare yet
    if (current_trajectory.positions.empty() || last_communicated_trajectory.positions.empty())
    {
        reason = "No comparison - trajectories not yet initialized";
        return false;  // Don't trigger communication
    }

    // Use trajectory's built-in deviation check
    const bool geometric_trigger = current_trajectory.geomtricDeviationTrigger(
        last_communicated_trajectory, 
        max_deviation_threshold);
    
    if (geometric_trigger)
    {
        reason = "Geometric deviation exceeded threshold";
    }
    else
    {
        reason = "No geometric deviation";
    }
    
    return geometric_trigger;
}

} // namespace MPCPlanner
