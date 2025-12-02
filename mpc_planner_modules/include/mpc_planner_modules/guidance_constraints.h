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
        void resetConsistencyParameters();
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

            // ========== JULES: Added for consistency module integration ==========
            bool has_consistency_enabled = false;  // Track if consistency was enabled for this planner
            // =====================================================================

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
        bool    _use_tmpcpp{true}, _enable_constraints{true};
        double  _control_frequency{20.};
        double  _planning_time;
        bool    _assign_meaningful_topology{false};

        int TOPOLOGY_NO_MATCH{8}; // Indicates top
        RealTimeData empty_data_;

        int best_planner_index_ = -1;

        // ==================== JULES: Consistency Module Integration ====================
        // These members track the previous trajectory for fair cost comparison
        // when using consistency costs in the MPC formulation
        bool _consistency_module_available{false};   // True if solver has consistency_weight parameter
        bool _consistency_on_non_guided{false};      // Config: enable consistency tracking for non-guided planner
        
        int  _prev_selected_topology_id{-1};          // Topology ID from previous iteration
        bool _prev_was_original_planner{false};      // Was non-guided planner selected last time?
        bool _has_previous_trajectory{false};        // Do we have valid previous trajectory data?
        std::vector<Eigen::Vector2d> _prev_trajectory;  // Previous trajectory (shifted by 1 step)
        // ================================================================================

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

        // ==================== JULES: Consistency Module Integration Functions ====================
        /**
         * @brief Initialize consistency tracking at construction time
         * 
         * Checks if the solver has consistency parameters and initializes storage.
         * Called from constructor after planners are created.
         */
        void initializeConsistencyTracking();

        /**
         * @brief Determine if consistency cost should be enabled for a specific planner
         * 
         * Logic:
         * - Non-guided planner: Only if _consistency_on_non_guided AND previous was non-guided
         * - Guided planner: Only if previous was guided AND topology matches
         * 
         * @param planner The local planner to check
         * @return true if consistency cost should be applied to this planner
         */
        bool shouldEnableConsistencyForPlanner(const LocalPlanner& planner);

        /**
         * @brief Set consistency parameters for a specific planner at stage k
         * 
         * Sets consistency_weight, prev_traj_x, prev_traj_y in the solver parameters.
         * At k=0, also determines and stores has_consistency_enabled for the planner.
         * 
         * @param planner The local planner (modified to store has_consistency_enabled)
         * @param k The stage index (0 to N-1)
         */
        void setConsistencyParametersForPlanner(LocalPlanner& planner, int k);

        /**
         * @brief Calculate the consistency cost contribution from a solved trajectory
         * 
         * Computes: sum over k of { weight * ((x_k - prev_x_k)^2 + (y_k - prev_y_k)^2) }
         * This is used to subtract the cost BEFORE selection_weight_consistency_ is applied.
         * 
         * @param solver The solver with the optimized trajectory
         * @return The total consistency cost contribution
         */
        double calculateConsistencyCostForSolver(std::shared_ptr<Solver> solver);

        /**
         * @brief Store the selected trajectory for next iteration's consistency tracking
         * 
         * Shifts the trajectory by 1 step (k -> k+1 mapping) to account for time progression.
         * Called after best planner selection in the optimize() function.
         * 
         * @param selected_solver The solver from the selected planner
         */
        void storePreviousTrajectoryFromSolver(std::shared_ptr<Solver> selected_solver);

        /**
         * @brief Visualize the previous trajectory used for consistency tracking
         * 
         * Displays the trajectory from the previous planning cycle as an orange line
         * with small spheres at each waypoint. This shows "what we're trying to stay
         * consistent with" during the current optimization.
         * 
         * Called at the start of optimize() before the parallel optimization loop.
         * If no previous trajectory exists, clears any existing visualization.
         */
        void visualizePreviousTrajectory();
        // =========================================================================================
    };
} // namespace MPCPlanner
#endif // __GUIDANCE_CONSTRAINTS_H__