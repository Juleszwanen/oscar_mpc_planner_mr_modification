#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/module_data.h>

#include <memory>
#include <vector>

namespace RosTools
{
    class DataSaver;
    class Timer;
}

namespace MPCPlanner
{
    class RealTimeData;
    class State;
    class ControllerModule;
    class Solver;
    class ExperimentUtil;

    struct PlannerOutput
    {
        Trajectory trajectory;
        bool success{false};

        // ADD THESE FIELDS:
        int selected_topology_id{-9};       // Homology class ID (from guidance_ID)
        int selected_planner_index{-9};     // Which planner was chosen (0 to n_paths)
        bool used_guidance{true};           // false if T-MPC++ (non-guided) was chosen
        double trajectory_cost{0.0};        // Objective value of selected solution
        int solver_exit_code{-1};           // Exit code (1=success, 0=max_iter, -1=infeasible)
        bool following_new_homology{true};  // Check if we are following a new homology or compared to the previous iteration

        PlannerOutput(double dt, int N) : trajectory(dt, N) {}

        PlannerOutput() = default;

        std::string logOutput() const;
    };

    class Planner
    {
    public:
        Planner();

    public:
        PlannerOutput solveMPC(State &state, RealTimeData &data);
        double getSolution(int k, std::string &&var_name) const;

        void onDataReceived(RealTimeData &data, std::string &&data_name);

        void saveData(State &state, RealTimeData &data);
        void visualize(const State &state, const RealTimeData &data);

        void visualizeObstaclePredictionsPlanner(const State &state, const RealTimeData &data, bool include_time = false);

        void reset(State &state, RealTimeData &data, bool success = true);

        bool isObjectiveReached(const State &state, const RealTimeData &data) const;

        RosTools::DataSaver &getDataSaver() const;

    private:
        bool _is_data_ready{false}, _was_reset{true};

        std::shared_ptr<Solver> _solver; //
        std::shared_ptr<ExperimentUtil> _experiment_util;
        PlannerOutput _output;

        Trajectory _warmstart;

        ModuleData _module_data;

        std::unique_ptr<RosTools::Timer> _startup_timer;

        std::vector<std::shared_ptr<ControllerModule>> _modules; // Will contain all modules used in the mpc formulation and the _modules are filled by the function initializeModules() in the file modules.h
    };

}

#endif