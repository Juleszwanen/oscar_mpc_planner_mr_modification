#ifndef EXPERIMENT_UTIL_H
#define EXPERIMENT_UTIL_H

#include <ros_tools/data_saver.h>

#include <string>
#include <memory>

namespace RosTools
{
    class DataSaver;
}
namespace MPCPlanner
{
    class Solver;
    struct RealTimeData;
    struct State;

    class ExperimentUtil
    {
    public:
        ExperimentUtil();
        ExperimentUtil(const std::string& robot_ns);

    public:
        void update(const State &state, std::shared_ptr<Solver> solver, const RealTimeData &data);

        void onTaskComplete(bool objective_reached);

        void exportData();

        void setStartExperiment();

        void safeExtraData(const State &state, const RealTimeData &data);

        RosTools::DataSaver &getDataSaver() const { return *_data_saver; };
        const int                 &getControlIteration() const  {return _control_iteration;};
        const int                 &getExperimentCounter() const  {return _experiment_counter;}

    private:
        // Data is saved in this object
        std::shared_ptr<RosTools::DataSaver> _data_saver;

        std::string _save_folder, _save_file;

        int _experiment_counter = 0;
        int _control_iteration = 0;
        int _iteration_at_last_reset = 0;
        bool _save_obstacle_data = true;
        bool _save_ego_trajectory_plans=true;
    };
} // namespace MPCPlanner
#endif // EXPERIMENT_UTIL_H
