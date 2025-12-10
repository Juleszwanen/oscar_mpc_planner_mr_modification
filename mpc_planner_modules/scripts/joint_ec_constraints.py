"""
Joint EC (Ego-Conditioned) Robot Constraint Module

This module defines the COUPLED collision constraints for joint optimization.
Unlike standard collision avoidance where obstacles have fixed trajectories,
these constraints couple the ego robot and EC robot positions as joint
decision variables.

Standard constraint (fixed obstacle):
    ||x_ego[k] - x_obs[k]||² ≥ (r_ego + r_obs)²   where x_obs is a parameter (fixed)

Coupled constraint (joint optimization):
    ||x_ego[k] - x_ec[k]||² ≥ (r_ego + r_ec)²    where x_ec is a decision variable

This coupling is what makes the optimization "joint" - both ego and EC positions
can adjust to satisfy the constraint, rather than only the ego adjusting.

Design Decision:
    This is implemented as a PURE ConstraintModule (not mixed with ObjectiveModule)
    to respect the existing module architecture where:
    - ConstraintModule has self.type = "constraint" and self.constraints list
    - ObjectiveModule has self.type = "objective" and self.objectives list
    
    The EC costs (deviation, control) are in joint_ec_objective.py.
"""

import os
import sys

import casadi as cd
import numpy as np

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from util.math import rotation_matrix
from control_modules import ConstraintModule


class JointCollisionConstraint:
    """
    Coupled collision constraint between ego robot and EC robots.
    
    This constraint ensures minimum separation between ego and each EC robot.
    Unlike fixed-obstacle constraints, both positions are decision variables,
    creating a coupling that allows both robots to adjust.
    
    Constraint formulation (per EC robot, per ego disc):
        h(x) = ||pos_ego_disc - pos_ec||² / (r_ego + r_ec + margin)²
        
    Bound: h(x) ≥ 1.0 (constraint satisfied when distance ≥ min_distance)
    
    When EC robot is inactive (active = 0), the constraint is automatically
    satisfied because the EC position is set far away (1000, 1000).
    
    Args:
        n_discs: Number of discs modeling the ego robot
        max_ec_robots: Maximum number of EC robots
    """

    def __init__(self, n_discs, max_ec_robots):
        self.n_discs = n_discs
        self.max_ec_robots = max_ec_robots
        
        # One constraint per EC robot per ego disc
        self.nh = max_ec_robots * n_discs

    def define_parameters(self, params):
        """
        Define EC robot radius and safety margin parameters.
        
        Note: EC prediction coordinates and active flags are defined in
        JointECObjectiveModule. Here we only define radius parameters.
        """
        # Safety margin for collision avoidance
        if not params.has_parameter("joint_safety_margin"):
            params.add("joint_safety_margin", bundle_name="joint_safety_margin")
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # EC robot radius
            if not params.has_parameter(prefix + "r"):
                params.add(prefix + "r", bundle_name="ec_r")

    def get_lower_bound(self):
        """
        Lower bound for constraint: h(x) ≥ 1.0
        
        The constraint h(x) = dist² / min_dist² should be ≥ 1.0 for collision-free.
        
        Returns:
            list: Lower bounds (1.0 for each constraint)
        """
        return [1.0] * (self.max_ec_robots * self.n_discs)

    def get_upper_bound(self):
        """
        Upper bound for constraint: h(x) ≤ inf
        
        No upper limit on distance (robots can be arbitrarily far apart).
        
        Returns:
            list: Upper bounds (infinity for each constraint)
        """
        return [np.inf] * (self.max_ec_robots * self.n_discs)

    def get_constraints(self, model, params, settings, stage_idx):
        """
        Build coupled collision constraints.
        
        For each EC robot i and ego disc d:
            h = ||pos_ego_d - pos_ec_i||² / (r_ego + r_ec_i + margin)²
        
        The constraint h ≥ 1 ensures minimum separation.
        
        When EC robot is inactive, its position is far away (1000, 1000),
        so the constraint is automatically satisfied.
        
        Args:
            model: Dynamics model with EC robot variables
            params: Parameter container
            settings: Solver settings
            stage_idx: Current stage index
        
        Returns:
            list: CasADi expressions for all constraints
        """
        constraints = []
        
        # Ego position
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])
        
        # Ego heading (for disc rotation)
        try:
            psi = model.get("psi")
        except Exception:
            psi = 0.0
        
        rotation_car = rotation_matrix(psi)
        
        # Ego disc radius
        r_ego = params.get("ego_disc_radius")
        
        # Safety margin
        safety_margin = params.get("joint_safety_margin")
        
        # Numerical epsilon for division stability
        EPSILON = 1e-6
        
        for ec_idx in range(self.max_ec_robots):
            prefix = f"ec{ec_idx}_"
            
            # Get EC robot position (DECISION VARIABLE - this is the key difference!)
            try:
                ec_x = model.get(prefix + "x")
                ec_y = model.get(prefix + "y")
            except Exception:
                # EC robot variables not in model, use fallback
                # This shouldn't happen if model is properly configured
                ec_x = params.get(prefix + "pred_x")
                ec_y = params.get(prefix + "pred_y")
            
            ec_pos = np.array([ec_x, ec_y])
            
            # Get EC robot radius (parameter)
            ec_r = params.get(prefix + "r")
            
            # Total safety distance (squared for normalized constraint)
            min_dist = r_ego + ec_r + safety_margin
            min_dist_sq = min_dist**2
            
            for disc_it in range(self.n_discs):
                # Ego disc position
                disc_offset = params.get(f"ego_disc_{disc_it}_offset")
                disc_relative_pos = np.array([disc_offset, 0])
                disc_pos = pos + rotation_car @ disc_relative_pos
                
                # Distance squared between ego disc and EC robot
                diff = disc_pos - ec_pos
                dist_sq = diff[0]**2 + diff[1]**2
                
                # Normalized constraint: dist² / min_dist² >= 1
                # This formulation is better conditioned than dist² >= min_dist²
                constraint = dist_sq / (min_dist_sq + EPSILON)
                constraints.append(constraint)
        
        return constraints


class JointECConstraintModule(ConstraintModule):
    """
    Constraint module for coupled collision avoidance in joint optimization.
    
    This module implements collision constraints where both the ego robot and
    EC robot positions are decision variables (coupled constraints).
    
    Key difference from standard collision avoidance:
        Standard: ||x_ego - x_obs|| ≥ d  (x_obs is fixed parameter)
        Joint:    ||x_ego - x_ec|| ≥ d   (x_ec is decision variable)
    
    This coupling allows the optimization to adjust both trajectories to find
    the best collision-free solution, rather than forcing only the ego to yield.
    
    The module is implemented as a PURE ConstraintModule (single inheritance)
    to avoid conflicts with the existing module architecture.
    
    Usage:
        In generate_jackalsimulator_solver.py:
        
        if settings.get("joint_planning", {}).get("enabled", False):
            max_ec_robots = settings["joint_planning"]["max_ec_robots"]
            modules.add_module(JointECConstraintModule(settings, max_ec_robots))
    
    Args:
        settings: Solver settings dictionary
        max_ec_robots: Maximum number of EC robots (default: 2)
    """

    def __init__(self, settings, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        self.n_discs = settings.get("n_discs", 1)
        
        self.module_name = "JointECConstraints"
        self.import_name = "joint_ec_constraints.h"  # C++ header (placeholder)
        self.description = (
            f"Coupled collision constraints for joint optimization "
            f"({max_ec_robots} EC robots × {self.n_discs} discs)"
        )
        
        # Add coupled collision constraint
        self.constraints.append(JointCollisionConstraint(self.n_discs, max_ec_robots))
