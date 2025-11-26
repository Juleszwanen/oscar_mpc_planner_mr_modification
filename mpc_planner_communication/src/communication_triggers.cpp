#include <mpc_planner_communication/communication_triggers.h>
#include <ros_tools/logging.h>

namespace MPCPlanner {

    std::string toString(CommunicationTriggerReason reason)
    {
        switch (reason)
        {
            case CommunicationTriggerReason::NO_COMMUNICATION: return "NO_COMMUNICATION";
            case CommunicationTriggerReason::INFEASIBLE: return "INFEASIBLE";
            case CommunicationTriggerReason::INFEASIBLE_TO_FEASIBLE: return "INFEASIBLE_TO_FEASIBLE";
            case CommunicationTriggerReason::TOPOLOGY_CHANGE: return "TOPOLOGY_CHANGE";
            case CommunicationTriggerReason::GEOMETRIC: return "GEOMETRIC";
            case CommunicationTriggerReason::TIME: return "TIME";
            case CommunicationTriggerReason::NON_GUIDED_HOMOLOGY_FAIL: return "NON_GUIDED_HOMOLOGY_FAIL";
            default: return "UNKNOWN";
        }
    }

    // 1. Infeasible Trigger
    bool CommunicationTriggers::checkInfeasible(const PlannerOutput& output)
    {
        return !output.success;
    }

    // 3. Real Topology Change Trigger
    bool CommunicationTriggers::checkTopologyChange(const PlannerOutput& output, int n_paths)
    {
        if (!output.success) 
            return false;

        const int non_guided_topology_id = 2 * n_paths;

        // It is a topology change if:
        // 1. The flag is set
        // 2. We are NOT switching TO non-guided (that is Enum 6)
        // 3. We are NOT switching FROM non-guided (that is also effectively a recovery or change handled elsewhere, 
        //    but strictly speaking, switching Guided A -> Guided B is the core of this trigger)
        
        // Note: If we want "Real topological change" to include Non-Guided -> Guided, we can allow it here.
        // But Guided -> Non-Guided is definitely Enum 6.
        
        bool is_to_guided = (output.selected_topology_id != non_guided_topology_id);
        
        return output.following_new_topology && is_to_guided;
    }

    // 4. Geometric Trigger
    bool CommunicationTriggers::checkGeometricDeviation(const Trajectory& current_trajectory, const Trajectory& last_communicated_trajectory, double max_deviation_threshold)
    {
        if (current_trajectory.positions.empty() || last_communicated_trajectory.positions.empty())
        {
            return false;
        }

        return current_trajectory.geomtricDeviationTrigger(last_communicated_trajectory, max_deviation_threshold);
    }

    // 5. Time Trigger
    bool CommunicationTriggers::checkTime(const ros::Time& last_send_time, const ros::Time& current_time, double heartbeat_period_sec)
    {
        if (last_send_time == ros::Time(0))
        {
            return true;
        }

        const ros::Duration heartbeat_period(heartbeat_period_sec);
        ros::Duration time_elapsed = current_time - last_send_time;
        return (time_elapsed >= heartbeat_period);
    }

    // 6. Non-Guided / Homology Fail Trigger
    bool CommunicationTriggers::checkNonGuidedHomologyFail(const PlannerOutput& output, int n_paths)
    {
        if (!output.success) 
            return false;

        const int non_guided_topology_id = 2 * n_paths;

        // Trigger if we are currently using the non-guided topology and we could not map it to an homology
        return (output.selected_topology_id == non_guided_topology_id);
    }

} // namespace MPCPlanner
