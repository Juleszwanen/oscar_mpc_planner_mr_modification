import sys, os

sys.path.append(os. path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ObjectiveModule, Objective

"""
Stick close to a previously calculated trajectory to improve temporal consistency
"""


class ConsistencyObjective(Objective):

    def __init__(self, settings):
        self.N = settings["N"]  # Horizon length

    def define_parameters(self, params):
        # Weight for the consistency cost
        params.add("consistency_weight", add_to_rqt_reconfigure=True, 
                   rqt_config_name=lambda p: f'["weights"]["consistency_weight"]')
        
        # Previous trajectory positions (one for each stage)
        # These will be set in C++ from the previous solution
        params.add("prev_traj_x")  # Previous x position for this stage
        params.add("prev_traj_y")  # Previous y position for this stage
        #params.add(prev_traj_psi)
    def get_value(self, model, params, settings, stage_idx):
        cost = 0

        # Current predicted position at this stage
        pos_x = model.get("x")
        pos_y = model.get("y")

        # Get the weight
        consistency_weight = params.get("consistency_weight")

        # Get the previous trajectory position for THIS stage
        # (set as a parameter from C++ at runtime)
        prev_x = params.get("prev_traj_x")
        prev_y = params.get("prev_traj_y")

        # Penalize deviation from previous trajectory
        # Note: No normalization needed here - the weight handles scaling
        # You might want to skip k=0 since that's the current state (always matches)
        # if stage_idx > 0:
        cost += consistency_weight * ((pos_x - prev_x) ** 2 + (pos_y - prev_y) ** 2)

        return cost


class ConsistencyModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()
        self.module_name = "ConsistencyModule"  # Must match C++ class name
        self.import_name = "consistency_module.h"  # C++ header file
        self.description = "Penalizes deviation from the previous planned trajectory for temporal consistency"

        self.objectives. append(ConsistencyObjective(settings))