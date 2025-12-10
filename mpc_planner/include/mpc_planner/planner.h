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

        /** @note Jules: new vriables to record data about if we used the guidance and other stuff*/
        int previous_topology_id{-1};      // Previous topology for logging purposes
        int selected_topology_id{-1};      // Homology class ID (from guidance_ID)
        int selected_planner_index{-1};    // Which planner was chosen (0 to n_paths)
        bool used_guidance{true};          // false if T-MPC++ (non-guided) was chosen
        double trajectory_cost{0.0};       // Objective value of selected solution
        int solver_exit_code{-1};          // Exit code (1=success, 0=max_iter, -1=infeasible)
        bool following_new_topology{true}; // Check if we are following a new homology or compared to the previous iteration
        int num_of_guidance_found{-1};      // Check how many guidance trajectories are found.
        PlannerOutput(double dt, int N) : trajectory(dt, N) {}

        PlannerOutput() = default;

        std::string logOutput() const;
    };

    class Planner
    {
    public:
        Planner();
        Planner(std::string ego_robot_ns, bool safe_extra_data);

    public:
        PlannerOutput solveMPC(State &state, RealTimeData &data);
        double getSolution(int k, std::string &&var_name) const;

        void onDataReceived(RealTimeData &data, std::string &&data_name);

        void saveData(State &state, RealTimeData &data);
        void saveData(State &state, RealTimeData &data, const double &current_state, const double &previous_state);
        void visualize(const State &state, const RealTimeData &data);

        void visualizeObstaclePredictionsPlanner(const State &state, const RealTimeData &data, bool include_time = false);

        void reset(State &state, RealTimeData &data, bool success = true);

        bool isObjectiveReached(const State &state, const RealTimeData &data) const;

        RosTools::DataSaver &getDataSaver() const;
        
        const int&  getControlIteration() const;
        const int& getExperimentCounter() const;

        bool setEgoNameSpaceGuidanceModule(const std::string &ego_robot_ns);

    public:
        /** @note Jules: Created this variable for debugging purposes, so log messages of different robots can be distinguished*/
        std::string _ego_robot_ns{"jackalX"};
        bool _safe_extra_data{false};

    private:
        bool _is_data_ready{false}, _was_reset{true};

        std::shared_ptr<Solver> _solver; //
        std::shared_ptr<ExperimentUtil> _experiment_util;
        PlannerOutput _output;

        Trajectory _warmstart;

        ModuleData _module_data;

        std::unique_ptr<RosTools::Timer> _startup_timer;

        std::vector<std::shared_ptr<ControllerModule>> _modules; // Will contain all modules used in the mpc formulation and the _modules are filled by the function initializeModules() in the file modules.h
    
        void saveSquaredDistanceOtherRobots(const State &state, const RealTimeData &data, RosTools::DataSaver &data_saver) const;
    };

}

#endif