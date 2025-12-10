#!/usr/bin/python3

import os
import sys
import numpy as np

# Add the solver_generator and mpc_planner_modules directories to the system path so that we can find the scripts
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

################################################################
# This comes from the solver_generator package
from util.files import load_settings, get_current_package
from control_modules import ModuleManager
from generate_solver import generate_solver
################################################################
# Import modules here from mpc_planner_modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from curvature_aware_contouring import CurvatureAwareContouringModule
from goal_module import GoalModule
from consistency_module import ConsistencyModule
from path_reference_velocity import PathReferenceVelocityModule

# Import modules here from mpc_planner_modules
from ellipsoid_constraints import EllipsoidConstraintModule
from gaussian_constraints import GaussianConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule
from scenario_constraints import ScenarioConstraintModule

########################################################################################################
# Import solver models that you want to use, these imports are coming from the solver_generator package
from solver_model import ContouringSecondOrderUnicycleModel, ContouringSecondOrderUnicycleModelWithSlack
from solver_model import ContouringSecondOrderUnicycleModelCurvatureAware
from solver_model import ContouringSecondOrderUnicycleModelWithEC
#########################################################################################################

# Import joint planning modules (for Variant B full joint optimization)
from joint_ec_objective import JointECObjectiveModule
from joint_ec_constraints import JointECConstraintModule

def configuration_no_obstacles(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    # You can manually set state/input bounds
    # lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 2, -1.0, -1.0]
    # upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 2, 3.0, 10000.0]
    # model.set_bounds(lower_bound, upper_bound)

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration") # w_a * ||a||_2^2
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity") # w_w * ||w||_2^2

    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2) # w_v * ||v - v_ref||_2^2

    modules.add_module(ContouringModule(settings)) # Contouring costs
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings)) # Possibly adaptive v_ref

    return model, modules


def configuration_basic(settings):
    model, modules = configuration_no_obstacles(settings)

    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


def configuration_safe_horizon(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModelWithSlack()

    # Module that allows for penalization of variables
    base_module = modules.add_module(MPCBaseModule(settings))

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="slack", weight_names="slack", rqt_max_value=10000.0)

    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2) # w_v * ||v - v_ref||_2^2

    modules.add_module(ContouringModule(settings)) # Contouring costs
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings)) # Possibly adaptive v_ref

    modules.add_module(ScenarioConstraintModule(settings))
    return model, modules


def configuration_tmpc(settings):
    model, modules = configuration_no_obstacles(settings)

    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule # This configures the obstacle avoidance used in each planner
    ))

    # modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=GaussianConstraintModule)) <- when this is commented we dont even use the GuassianConstraintModule

    return model, modules

def configuration_tmpc_consistency_cost(settings):
    model, modules = configuration_no_obstacles(settings)
    if(settings["JULES"]["consistency_enabled"]):
        modules.add_module(ConsistencyModule(settings))
    
    modules.add_module(GuidanceConstraintModule(
        settings, 
        constraint_submodule=EllipsoidConstraintModule # This configures the obstacle avoidance used in each planner
    ))
    return model, modules


def configuration_tmpc_joint_planning(settings):
    """
    T-MPC++ with joint EC robot optimization (Variant B from IJP paper).
    
    This configuration extends T-MPC++ with joint optimization capabilities:
    - Uses ContouringSecondOrderUnicycleModelWithEC for extended state/input space
    - Adds JointECObjectiveModule for EC deviation and control costs
    - Adds JointECConstraintModule for coupled collision constraints
    
    The joint planning modules add decision variables for EC (Ego-Conditioned) robots,
    allowing the optimizer to adjust both ego and EC trajectories to find
    collision-free solutions.
    
    Configuration in settings.yaml:
        joint_planning:
            enabled: true
            max_ec_robots: 2
            ...
    """
    # Get joint planning configuration
    joint_planning_config = settings.get("joint_planning", {})
    joint_planning_enabled = joint_planning_config.get("enabled", False)
    max_ec_robots = joint_planning_config.get("max_ec_robots", 2)
    
    modules = ModuleManager()
    
    if joint_planning_enabled:
        # Use extended model with EC robot variables
        model = ContouringSecondOrderUnicycleModelWithEC(max_ec_robots=max_ec_robots)
        print(f"[Joint Planning] Using ContouringSecondOrderUnicycleModelWithEC with {max_ec_robots} EC robots")
        print(f"[Joint Planning] Model dimensions: nx={model.nx}, nu={model.nu}")
    else:
        # Use standard model
        model = ContouringSecondOrderUnicycleModel()
        print("[Joint Planning] Joint planning disabled, using standard model")
    
    # Base module for input penalties
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    
    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(
            var_name="v",
            weight_names=["velocity", "reference_velocity"],
            cost_function=lambda x, w: w[0] * (x - w[1])**2
        )
    
    # Contouring module for path tracking
    modules.add_module(ContouringModule(settings))
    if settings["contouring"]["dynamic_velocity_reference"]:
        modules.add_module(PathReferenceVelocityModule(settings))
    
    # Consistency module if enabled
    if settings.get("JULES", {}).get("consistency_enabled", False):
        modules.add_module(ConsistencyModule(settings))
    
    # Joint EC modules (only when joint planning is enabled)
    if joint_planning_enabled:
        # EC robot objective costs (deviation + control effort)
        modules.add_module(JointECObjectiveModule(settings, max_ec_robots=max_ec_robots))
        
        # EC robot coupled collision constraints
        modules.add_module(JointECConstraintModule(settings, max_ec_robots=max_ec_robots))
        
        print(f"[Joint Planning] Added JointECObjectiveModule and JointECConstraintModule")
    
    # Guidance constraints with underlying obstacle avoidance
    # Note: Non-EC obstacles are still handled by EllipsoidConstraintModule
    modules.add_module(GuidanceConstraintModule(
        settings,
        constraint_submodule=EllipsoidConstraintModule
    ))
    
    return model, modules

def configuration_lmpcc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    # modules.add_module(ContouringModule(settings))
    modules.add_module(GoalModule(settings))

    modules.add_module(PathReferenceVelocityModule(settings))

    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules

# JULES: load the settings described in mpc_planner_jackalsimulator/config/settings.yaml
settings = load_settings()

# model, modules = configuration_basic(settings)
# model, modules = configuration_no_obstacles(settings)

# NOTE: LMPCC - basic MPC with deterministic obstacle avoidance
# model, modules = configuration_lmpcc(settings)

# NOTE: T-MPC - Parallelized MPC optimizing trajectories with several distinct passing behaviors.
# model, modules = configuration_tmpc(settings)

# NOTE: T-MPC++ with consistency cost (current default)
# model, modules = configuration_tmpc_consistency_cost(settings)

# NOTE: T-MPC++ with joint EC robot optimization (Variant B from IJP paper)
# Use this when joint_planning.enabled = true in settings.yaml
# model, modules = configuration_tmpc_joint_planning(settings)

# NOTE: SH-MPC - MPC incorporating non Gaussian uncertainty in obstacle motion. 
# More configuration parameters in `scenario_module/config/params.yaml`
# model, modules = configuration_safe_horizon(settings)


def select_configuration(settings):
    """
    Select the appropriate MPC configuration based on settings.
    
    This function chooses between different MPC configurations:
    - Joint planning (when joint_planning.enabled = true)
    - T-MPC++ with consistency cost (default)
    
    Args:
        settings: Dictionary of solver settings from settings.yaml
    
    Returns:
        tuple: (model, modules) for the selected configuration
    """
    joint_planning_enabled = settings.get("joint_planning", {}).get("enabled", False)
    
    if joint_planning_enabled:
        print("=" * 60)
        print("Joint Planning ENABLED - using configuration_tmpc_joint_planning")
        print("=" * 60)
        return configuration_tmpc_joint_planning(settings)
    else:
        print("=" * 60)
        print("Joint Planning DISABLED - using configuration_tmpc_consistency_cost")
        print("=" * 60)
        return configuration_tmpc_consistency_cost(settings)


# Select configuration and generate solver
model, modules = select_configuration(settings)
generate_solver(modules, model, settings)
exit(0)
