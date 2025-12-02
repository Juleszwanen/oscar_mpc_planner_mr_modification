#ifndef __MPC_CONSISTENCY_MODULE_H__
#define __MPC_CONSISTENCY_MODULE_H__

#include <mpc_planner_modules/controller_module.h>
#include <Eigen/Dense>
#include <vector>

namespace MPCPlanner
{
    class ConsistencyModule : public ControllerModule
    {
    public:
        ConsistencyModule(std::shared_ptr<Solver> solver);

        void update(State &state, const RealTimeData &data, 
                    ModuleData &module_data) override;

        void setParameters(const RealTimeData &data, 
                          const ModuleData &module_data, int k) override;

        void onDataReceived(RealTimeData &data, std::string &&data_name) override;

        void postSolve(const State &state, const RealTimeData &data,
                       ModuleData &module_data);

        bool isDataReady(const RealTimeData &data, 
                        std::string &missing_data) override;

        void visualize(const RealTimeData &data, 
                      const ModuleData &module_data) override;

        void reset();

    private:
        std::vector<Eigen::Vector2d> _prev_trajectory;
        bool _has_previous_trajectory{false};
    };
}

#endif // __MPC_CONSISTENCY_MODULE_H__