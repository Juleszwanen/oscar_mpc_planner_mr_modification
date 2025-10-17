#include <mpc_planner_types/module_data.h>

namespace MPCPlanner
{
        void ModuleData::reset()
        {
                path.reset();
                path_width_left.reset();
                path_width_right.reset();
                path_velocity.reset();
                current_path_segment = -1;

                /** @note Jules: These reset statements are created by you on OCt 13 2025; to also reset self defined states */
                selected_topology_id = -1;
                selected_planner_index = -1;
                used_guidance = true;
                trajectory_cost = 0.0;
                solver_exit_code = -1;
        }
}