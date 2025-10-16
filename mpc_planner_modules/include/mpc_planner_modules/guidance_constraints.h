/**
 * @file guidance_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Controller Module for computing guidance trajectories in the state-space and using them to construct
 * constraints (@see python_forces_code/modules.py)
 * @date 2022-09-23 (documented)
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GUIDANCE_CONSTRAINTS_H__
#define __GUIDANCE_CONSTRAINTS_H__

#include <mpc_planner_modules/definitions.h>

#include <mpc_planner_modules/linearized_constraints.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_modules/controller_module.h>
#include <mpc_planner_solver/solver_interface.h>

#include <unordered_map>

namespace GuidancePlanner
{
    class GlobalGuidance;
    struct GeometricPath;
    enum class NodeType;
    struct Node;
}

namespace MPCPlanner
{
    /** @brief Save all the relevant results for a parallel solver in one place */
    struct SolverResult
    {
        int exit_code;
        double objective;
        bool success;

        int guidance_ID;
        int color;

        void Reset()
        {
            success = false;
            objective = 1e10;
            exit_code = -1;

            guidance_ID = -1;
            color = -1;
        }
    };

    /**
     * @brief Homotopy Guidance controller module extends from the reference path ControlModule to implement MPCC over the
     * trajectory but starting from the robot
     */
    class GuidanceConstraints : public ControllerModule
    {
    public:
        GuidanceConstraints(std::shared_ptr<Solver> solver);

    public:
        void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
        void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

        bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

        void visualize(const RealTimeData &data, const ModuleData &module_data) override;

        /**
         * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
         *
         * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this
         * as a custom optimization
         */
        int optimize(State &state, const RealTimeData &data, ModuleData &module_data) override; // Default: no custom optimization

        /** @brief Load obstacles into the Homotopy module */
        void onDataReceived(RealTimeData &data, std::string &&data_name) override;

        void reset() override;
        void saveData(RosTools::DataSaver &data_saver) override;
        // void GetMethodName(std::string &name) override;

    private: // Private functions
        struct LocalPlanner
        {
            int id;
            std::unique_ptr<LinearizedConstraints> guidance_constraints;   // Keep the solver in the topology
            std::unique_ptr<GUIDANCE_CONSTRAINTS_TYPE> safety_constraints; // Avoid collisions

            std::shared_ptr<Solver> local_solver; // Distinct solver for each planner
            SolverResult result;

            bool is_original_planner = false;
            bool disabled = true;

            bool taken = false;
            bool existing_guidance = false;

            LocalPlanner(int _id, bool _is_original_planner = false);
        };

        void setGoals(State &state, const ModuleData &module_data);
        void mapGuidanceTrajectoriesToPlanners();
        void initializeSolverWithGuidance(LocalPlanner &planner);

        int FindBestPlanner();

    private: // Member variables
        std::vector<LocalPlanner> planners_;

        std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_; // This one is very important, this is where the guidance trajectories are created

        std::unordered_map<int, int> _map_homotopy_class_to_planner;

        // Configuration parameters
        bool _use_tmpcpp{true}, _enable_constraints{true};
        double _control_frequency{20.};
        double _planning_time;
        bool _assign_meaningful_topology{false};

        int TOPOLOGY_NO_MATCH{8}; // Indicates top
        RealTimeData empty_data_;

        int best_planner_index_ = -1;

    public:
        std::string _ego_robot_ns{"jackalX"};

        /** @note Jules: The functions under this public keyword are set by you */
    private:
        /**
         * @brief Convert MPC solver trajectory to GeometricPath for homotopy comparison
         *
         * Creates a sequence of nodes from the MPC optimization output, converting
         * the continuous trajectory into a discrete geometric path compatible with
         * the PRM's homotopy equivalence checking.
         *
         * @param solver The MPC solver containing the optimized trajectory
         * @param node_type The type of nodes to create (default: CONNECTOR)
         * @return GeometricPath object that can be compared with guidance trajectories
         *
         * @note The function creates nodes with discrete time indices k (not k*dt)
         *       to match the PRM's space-time representation
         * @note Nodes are stored in mpc_trajectory_nodes_ for memory management
         */
        GuidancePlanner::GeometricPath convertMPCTrajectoryToGeometricPath(
            std::shared_ptr<Solver> solver,
            GuidancePlanner::NodeType node_type = GuidancePlanner::NodeType::CONNECTOR);

        /**
         * @brief Storage for temporary nodes created during MPC trajectory conversion
         *
         * These nodes must persist while the GeometricPath is being used for
         * homotopy comparison, as GeometricPath stores raw pointers to them.
         */
        std::vector<std::unique_ptr<GuidancePlanner::Node>> mpc_trajectory_nodes_;

        /**
         * @brief Base ID for MPC trajectory nodes to avoid conflicts with PRM nodes
         *
         * PRM uses:
         * - Negative IDs for special nodes (START=-1, GOAL=-2, etc.)
         * - Non-negative IDs for regular graph nodes (0, 1, 2, ...)
         *
         * We use 1000000+ to ensure no conflicts
         */
        static constexpr int MPC_NODE_BASE_ID = 1000000;
    };
} // namespace MPCPlanner
#endif // __GUIDANCE_CONSTRAINTS_H__