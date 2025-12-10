"""
Joint EC (Ego-Conditioned) Robot Objective Module

This module defines the objective (cost) terms for EC robots in joint optimization.
It implements the deviation cost (J_dev) and control effort cost (J_u) from the
Interactive Joint Planning paper (Equation 8):

    J_ec = η_o * [J_dev(x_ec, x_pred) + J_u(u_ec)]

Where:
    - J_dev: Penalty for EC robot deviation from unconditioned prediction
    - J_u: Penalty for EC robot control effort (acceleration and angular velocity)
    - η_o: Weight for EC robot costs (altruism parameter)

Design Decision:
    This is implemented as a PURE ObjectiveModule (not mixed with ConstraintModule)
    to respect the existing module architecture where:
    - ObjectiveModule has self.type = "objective" and self.objectives list
    - ConstraintModule has self.type = "constraint" and self.constraints list
    
    Multiple inheritance from both would cause self.type conflicts.
    Instead, the constraint and objective aspects are separated into:
    - JointECObjectiveModule (this file): EC robot costs
    - JointECConstraintModule (joint_ec_constraints.py): Coupled collision constraints
"""

import os
import sys

import casadi as cd
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ObjectiveModule, Objective


class JointECDeviationObjective(Objective):
    """
    Deviation cost penalizing EC robots for deviating from their predicted trajectories.
    
    The deviation cost encourages EC robots to stay close to their unconditioned 
    predictions (i.e., trajectories received via communication). This acts as a 
    "stiffness" - EC robots won't deviate much unless necessary to avoid collision.
    
    Cost term:
        J_dev_i[k] = w_dev * ||pos_ec_i[k] - pos_pred_i[k]||²
                   = w_dev * ((x_ec - x_pred)² + (y_ec - y_pred)²)
    
    The deviation weight w_dev controls how "sticky" EC robots are to their predictions:
        - High w_dev: EC robots resist deviation (ego must yield)
        - Low w_dev: EC robots easily adjust (ego can be selfish)
    """

    def __init__(self, max_ec_robots, settings):
        super().__init__()
        self.max_ec_robots = max_ec_robots
        self.settings = settings

    def define_parameters(self, params):
        """
        Define EC robot prediction parameters.
        
        For each EC robot and each stage, we need:
        - ec{i}_pred_x: Predicted x position
        - ec{i}_pred_y: Predicted y position
        - ec{i}_active: Active flag (1.0 = active, 0.0 = inactive)
        """
        # EC deviation weight
        params.add(
            "ec_deviation_weight", 
            add_to_rqt_reconfigure=True,
            rqt_config_name=lambda p: f'["weights"]["ec_deviation"]',
            rqt_min_value=0.0,
            rqt_max_value=50.0
        )
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # EC robot predicted trajectory (reference for deviation cost)
            params.add(prefix + "pred_x", bundle_name="ec_pred_x")
            params.add(prefix + "pred_y", bundle_name="ec_pred_y")
            
            # Active flag (1.0 = active, 0.0 = inactive)
            # When inactive, cost contribution is zero (multiplied by active flag)
            params.add(prefix + "active", bundle_name="ec_active")

    def get_value(self, model, params, settings, stage_idx):
        """
        Compute deviation cost for all EC robots at this stage.
        
        Cost = Σ_i [active_i * w_dev * ||pos_ec_i - pos_pred_i||²]
        
        Args:
            model: Dynamics model with EC robot variables
            params: Parameter container
            settings: Solver settings
            stage_idx: Current stage index (0 to N-1)
        
        Returns:
            CasADi expression for total deviation cost
        """
        w_dev = params.get("ec_deviation_weight")
        total_cost = 0.0
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # Get EC robot position (decision variable)
            # If EC robot variables are not in the model, skip this robot
            # This allows the module to gracefully handle non-joint-planning models
            try:
                ec_x = model.get(prefix + "x")
                ec_y = model.get(prefix + "y")
            except (IOError, KeyError):
                # IOError is raised by model.get() when variable not found
                # KeyError could be raised if model uses dict-based storage
                continue
            
            # Get EC robot prediction (parameter)
            pred_x = params.get(prefix + "pred_x")
            pred_y = params.get(prefix + "pred_y")
            
            # Get active flag
            active = params.get(prefix + "active")
            
            # Deviation cost: ||position - prediction||²
            dev_x = ec_x - pred_x
            dev_y = ec_y - pred_y
            deviation_cost = w_dev * (dev_x**2 + dev_y**2)
            
            # Only add cost if EC robot is active
            # When inactive, active = 0 and cost contribution is 0
            total_cost += active * deviation_cost
        
        return total_cost


class JointECControlObjective(Objective):
    """
    Control effort cost penalizing EC robot control inputs.
    
    This cost encourages smooth trajectories for EC robots by penalizing
    acceleration and angular velocity inputs.
    
    Cost term:
        J_u_i[k] = w_ctrl * (a_ec_i² + w_ec_i²)
    
    The control weight affects trajectory smoothness:
        - High w_ctrl: Smooth EC trajectories (less responsive)
        - Low w_ctrl: EC robots can make quick adjustments
    """

    def __init__(self, max_ec_robots, settings):
        super().__init__()
        self.max_ec_robots = max_ec_robots
        self.settings = settings

    def define_parameters(self, params):
        """
        Define EC robot control weight parameter.
        """
        params.add(
            "ec_control_weight",
            add_to_rqt_reconfigure=True,
            rqt_config_name=lambda p: f'["weights"]["ec_control"]',
            rqt_min_value=0.0,
            rqt_max_value=10.0
        )

    def get_value(self, model, params, settings, stage_idx):
        """
        Compute control effort cost for all EC robots at this stage.
        
        Cost = Σ_i [active_i * w_ctrl * (a_ec_i² + w_ec_i²)]
        
        Args:
            model: Dynamics model with EC robot variables
            params: Parameter container
            settings: Solver settings
            stage_idx: Current stage index (0 to N-1)
        
        Returns:
            CasADi expression for total control effort cost
        """
        w_ctrl = params.get("ec_control_weight")
        total_cost = 0.0
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # Get EC robot control inputs (decision variables)
            # If EC robot variables are not in the model, skip this robot
            try:
                ec_a = model.get(prefix + "a")
                ec_w = model.get(prefix + "w")
            except (IOError, KeyError):
                # IOError is raised by model.get() when variable not found
                continue
            
            # Get active flag
            active = params.get(prefix + "active")
            
            # Control effort cost: ||u||²
            control_cost = w_ctrl * (ec_a**2 + ec_w**2)
            
            # Only add cost if EC robot is active
            total_cost += active * control_cost
        
        return total_cost


class JointECObjectiveModule(ObjectiveModule):
    """
    Objective module for EC (Ego-Conditioned) robot costs in joint optimization.
    
    This module implements the EC robot cost terms from the Interactive Joint Planning
    paper (Equation 8):
    
        J_ec = η_o * [J_dev(x_ec, x_pred) + J_u(u_ec)]
    
    Where:
        - J_dev: Deviation cost penalizing EC robots for moving from predictions
        - J_u: Control effort cost for smooth EC trajectories
    
    The module is implemented as a PURE ObjectiveModule (single inheritance) to
    avoid conflicts with the existing module architecture. Coupled collision 
    constraints are handled separately in JointECConstraintModule.
    
    Usage:
        In generate_jackalsimulator_solver.py:
        
        if settings.get("joint_planning", {}).get("enabled", False):
            max_ec_robots = settings["joint_planning"]["max_ec_robots"]
            modules.add_module(JointECObjectiveModule(settings, max_ec_robots))
    
    Args:
        settings: Solver settings dictionary
        max_ec_robots: Maximum number of EC robots (default: 2)
    """

    def __init__(self, settings, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        
        self.module_name = "JointECObjective"
        self.import_name = "joint_ec_objective.h"  # C++ header (placeholder)
        self.description = (
            f"EC robot deviation and control costs for joint optimization "
            f"(max {max_ec_robots} EC robots)"
        )
        
        # Add objectives for EC robots
        self.objectives.append(JointECDeviationObjective(max_ec_robots, settings))
        self.objectives.append(JointECControlObjective(max_ec_robots, settings))
