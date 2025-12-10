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

        virtual void update(State &state, const RealTimeData &data, ModuleData &module_data) override;

        virtual void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;


        bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

        void visualize(const RealTimeData &data,  const ModuleData &module_data) override;

        void reset();

    private:
    };
}

#endif // __MPC_CONSISTENCY_MODULE_H__