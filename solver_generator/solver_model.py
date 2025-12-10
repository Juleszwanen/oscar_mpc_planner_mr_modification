import casadi as cd
import numpy as np

from util.files import model_map_path, write_to_yaml
from util.logging import print_warning

from spline import Spline2D


# Returns discretized dynamics of a given model (see below)
def forces_discrete_dynamics(z, p, model, settings, nx=None, integration_step=None):
    import forcespro.nlp

    """
    @param z: state vector (u, x)
    @param p: parameters
    @param model: Model of the system
    @param settings: Integrator stepsize in seconds
    @param nx: Number of states (can be used to modify which states are integrated)
    @return:
    """
    # We use an explicit RK4 integrator here to discretize continuous dynamics

    if nx is None:
        nx = model.nx

    if integration_step is None:
        integration_step = settings["integrator_step"]

    return forcespro.nlp.integrate(
        model.continuous_model,
        z[model.nu : model.nu + nx],
        z[0 : model.nu],
        integrator=forcespro.nlp.integrators.RK4,
        stepsize=integration_step,
    )


def numpy_to_casadi(x: np.array) -> cd.SX:
    result = None
    for param in x:
        if result is None:
            result = param
        else:
            result = cd.vertcat(result, param)
    return result


class DynamicsModel:

    def __init__(self):
        self.nu = 0  # number of control variables
        self.nx = 0  # number of states

        self.states = []
        self.inputs = []

        self.lower_bound = []
        self.upper_bound = []

        self.params = None
        self.nx_integrate = None

    def discrete_dynamics(self, z, p, settings, **kwargs):
        params = settings["params"]
        params.load(p)
        self.load(z)
        self.load_settings(settings)

        nx_integrate = self.nx if self.nx_integrate is None else self.nx_integrate
        integrated_states = forces_discrete_dynamics(z, p, self, settings, nx=nx_integrate, **kwargs)

        integrated_states = self.model_discrete_dynamics(z, integrated_states, **kwargs)
        return integrated_states

    def model_discrete_dynamics(self, z, integrated_states, **kwargs):
        return integrated_states

    def get_nvar(self):
        return self.nu + self.nx

    def get_xinit(self):
        return range(self.nu, self.get_nvar())

    def acados_symbolics(self):
        x = cd.SX.sym("x", self.nx)  # [px, py, vx, vy]
        u = cd.SX.sym("u", self.nu)  # [ax, ay]
        z = cd.vertcat(u, x)
        self.load(z)
        return z

    def get_acados_dynamics(self):
        self._x_dot = cd.SX.sym("x_dot", self.nx)

        f_expl = numpy_to_casadi(self.continuous_model(self._z[self.nu :], self._z[: self.nu]))
        f_impl = self._x_dot - f_expl
        return f_expl, f_impl

    def get_x(self):
        return self._z[self.nu :]

    def get_u(self):
        return self._z[: self.nu]

    def get_acados_x_dot(self):
        return self._x_dot

    def get_acados_u(self):
        return self._z[: self.nu]

    def load(self, z):
        self._z = z

    def load_settings(self, settings):
        self.params = settings["params"]
        self.settings = settings

    def save_map(self):
        file_path = model_map_path()

        map = dict()
        for idx, state in enumerate(self.states):
            map[state] = ["x", idx + self.nu, self.get_bounds(state)[0], self.get_bounds(state)[1]]

        for idx, input in enumerate(self.inputs):
            map[input] = ["u", idx, self.get_bounds(input)[0], self.get_bounds(input)[1]]

        write_to_yaml(file_path, map)

    def integrate(self, z, settings, integration_step):
        return self.discrete_dynamics(z, settings["params"].get_p(), settings, integration_step=integration_step)

    def do_not_use_integration_for_last_n_states(self, n):
        self.nx_integrate = self.nx - n

    def get(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return self._z[self.nu + i]
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return self._z[i]
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")

    def set_bounds(self, lower_bound, upper_bound):
        assert len(lower_bound) == len(upper_bound) == len(self.lower_bound)
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_bounds(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return (
                self.lower_bound[self.nu + i],
                self.upper_bound[self.nu + i],
                self.upper_bound[self.nu + i] - self.lower_bound[self.nu + i],
            )
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return (
                self.lower_bound[i],
                self.upper_bound[i],
                self.upper_bound[i] - self.lower_bound[i],
            )
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")


class SecondOrderUnicycleModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 4  # number of states

        self.states = ["x", "y", "psi", "v"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-2.0, -2.0, -200.0, -200.0, -np.pi * 4, -2.0]
        self.upper_bound = [2.0, 2.0, 200.0, 200.0, np.pi * 4, 3.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a])


class ContouringSecondOrderUnicycleModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 5  # number of states

        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]

        # w = 0.8
        self.lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0]
        self.upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a, v])


class ContouringSecondOrderUnicycleModelCurvatureAware(DynamicsModel):  # NOT TESTED!

    def __init__(self):
        super().__init__()
        print_warning("ContouringSecondOrderUnicycleModelCurvatureAware is not supported in Acados as discrete dynamics are necessary for the spline state")
        self.nu = 2  # number of control variables
        self.nx = 5  # number of states

        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]

        self.do_not_use_integration_for_last_n_states(n=1)

        self.lower_bound = [-4.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0]
        self.upper_bound = [4.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a])

    def model_discrete_dynamics(self, z, integrated_states, **kwargs):

        x = self.get_x()

        pos_x = x[0]
        pos_y = x[1]
        s = x[-1]

        # CA-MPC
        path = Spline2D(self.params, self.settings["contouring"]["num_segments"], s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)
        path_ddx, path_ddy = path.deriv2(s)

        # Contour = n_vec
        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)

        dp = np.array([integrated_states[0] - pos_x, integrated_states[1] - pos_y])
        t_vec = np.array([path_dx_normalized, path_dy_normalized])
        n_vec = np.array([path_dy_normalized, -path_dx_normalized])

        vt_t = dp.dot(t_vec)
        vn_t = dp.dot(n_vec)

        R = 1.0 / path.get_curvature(s)  # max(R) = 1 / 0.0001
        R = cd.fmax(R, 1e5)

        theta = cd.atan2(vt_t, R - contour_error - vn_t)

        return cd.vertcat(integrated_states, s + R * theta)


class ContouringSecondOrderUnicycleModelWithSlack(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 6  # number of states

        self.states = ["x", "y", "psi", "v", "spline", "slack"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0, 0.0]  # v -0.01
        self.upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0, 5000.0]  # w 0.8

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a, v, 0.0])

    # NOTE: No initialization for slack variable
    def get_xinit(self):
        return range(self.nu, self.get_nvar() - 1)


# Bicycle model with dynamic steering
class BicycleModel2ndOrder(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 3
        self.nx = 6

        self.states = ["x", "y", "psi", "v", "delta", "spline"]
        self.inputs = ["a", "w", "slack"]

        # Prius limits: https://github.com/oscardegroot/lmpcc/blob/prius/lmpcc_solver/scripts/systems.py
        # w [-0.2, 0.2] | a [-1.0 1.0]
        # w was 0.5
        # delta was 0.45

        # NOTE: the angle of the vehicle should not be limited to -pi, pi, as the solution will not shift when it is at the border!
        # a was 3.0
        self.lower_bound = [-3.0, -1.5, 0.0, -1.0e6, -1.0e6, -np.pi * 4, -0.01, -0.55, -1.0]
        self.upper_bound = [3.0, 1.5, 1.0e2, 1.0e6, 1.0e6, np.pi * 4, 5.0, 0.55, 5000.0]

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]

        wheel_base = 2.79  # between front wheel center and rear wheel center
        wheel_tread = 1.64  # between left wheel center and right wheel center
        front_overhang = 1.0  # between front wheel center and vehicle front
        rear_overhang = 1.1  # between rear wheel center and vehicle rear
        left_overhang = 0.128  # between left wheel center and vehicle left
        right_overhang = 0.128  # between right wheel center and vehicle right

        # self.length = front_overhang + rear_overhang + wheel_base #4.54
        # self.width = wheel_tread + left_overhang + right_overhang #2.25

        # NOTE: Is at the rear wheel center.
        # This defines where it is w.r.t. the back
        # self.com_to_back = self.length/2.

        # NOTE: Mass is equally distributed according to the parameters
        lr = wheel_base / 2.0
        lf = wheel_base / 2.0
        ratio = lr / (lr + lf)
        self.width = 2.25

        beta = cd.arctan(ratio * cd.tan(delta))

        return np.array([v * cd.cos(psi + beta), v * cd.sin(psi + beta), (v / lr) * cd.sin(beta), a, w, v])


# Bicycle model with dynamic steering
class BicycleModel2ndOrderCurvatureAware(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 3
        self.nx = 6

        self.states = ["x", "y", "psi", "v", "delta", "spline"]
        self.inputs = ["a", "w", "slack"]

        self.do_not_use_integration_for_last_n_states(n=1)

        # Prius limits: https://github.com/oscardegroot/lmpcc/blob/prius/lmpcc_solver/scripts/systems.py
        # w [-0.2, 0.2] | a [-1.0 1.0]
        # w was 0.5
        # delta was 0.45

        # NOTE: the angle of the vehicle should not be limited to -pi, pi, as the solution will not shift when it is at the border!
        # a was 3.0
        # delta was -0.45, 0.45
        self.lower_bound = [-3.0, -1.5, 0.0, -1.0e6, -1.0e6, -np.pi * 4, -0.01, -0.55, -1.0]
        self.upper_bound = [3.0, 1.5, 1.0e2, 1.0e6, 1.0e6, np.pi * 4, 8.0, 0.55, 5000.0]

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]

        wheel_base = 2.79  # between front wheel center and rear wheel center

        # NOTE: Mass is equally distributed according to the parameters
        self.lr = wheel_base / 2.0
        self.lf = wheel_base / 2.0
        ratio = self.lr / (self.lr + self.lf)

        self.width = 2.25

        beta = cd.arctan(ratio * cd.tan(delta))

        return np.array([v * cd.cos(psi + beta), v * cd.sin(psi + beta), (v / self.lr) * cd.sin(beta), a, w])

    def model_discrete_dynamics(self, z, integrated_states, **kwargs):

        x = self.get_x()

        pos_x = x[0]
        pos_y = x[1]
        psi = x[2]
        vel = x[3]
        s = x[-1]

        # v = np.array([vel * cd.cos(psi), vel * cd.sin(psi)])

        # CA-MPC
        path = Spline2D(self.params, self.settings["contouring"]["num_segments"], s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)
        path_ddx, path_ddy = path.deriv2(s)

        # Contour = n_vec
        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)

        dp = np.array([integrated_states[0] - pos_x, integrated_states[1] - pos_y])
        t_vec = np.array([path_dx_normalized, path_dy_normalized])
        n_vec = np.array([path_dy_normalized, -path_dx_normalized])

        vt_t = dp.dot(t_vec)
        vn_t = dp.dot(n_vec)

        R = 1.0 / path.get_curvature(s)  # max(R) = 1 / 0.0001
        R = cd.fmax(R, 1e5)

        theta = cd.atan2(vt_t, R - contour_error - vn_t)

        # Lorenzo's equations
        # R = 1. / path.get_curvature(s) # max(R) = 1 / 0.0001
        # b = (integrated_states[0] - pos_x) * path_dx_normalized + (integrated_states[1] - pos_y) * path_dy_normalized
        # l = R * (1 - ((integrated_states[0] - path_x) * path_ddx + (integrated_states[1] - path_y) * path_ddy))
        # DS = R * cd.atan2(b, l)

        return cd.vertcat(integrated_states, s + R * theta)


# Bicycle model with dynamic steering
# class BicycleModel2ndOrderWithDelay(DynamicModel):

#     def __init__(self, system):
#         self.nu = 2
#         self.nx = 7
#         super(BicycleModel2ndOrderWithDelay, self).__init__(system)

#         self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in', 'spline']  # , 'ax', 'ay'
#         self.states_from_sensor = [True, True, True, True, False, True, False]  # , True, True
#         self.states_from_sensor_at_infeasible = [True, True, True, True, False, True, False]

#         self.inputs = ['a', 'w']
#         self.control_inputs['steering'] = 'delta_in'
#         self.control_inputs['velocity'] = 'v'
#         self.control_inputs['acceleration'] = 'a'
#         self.control_inputs['rot_velocity'] = 'w'

#     def continuous_model(self, x, u):
#         a = u[0]
#         w = u[1]
#         psi = x[2]
#         v = x[3]
#         delta = x[4] # affects the model
#         delta_in = x[5] # = w (i.e., is updated with the input)

#         ratio = self.system.ratio
#         lr = self.system.lr

#         beta = casadi.arctan(ratio * casadi.tan(delta))

#         return np.array([v * casadi.cos(psi + beta),
#                          v * casadi.sin(psi + beta),
#                          (v/lr) * casadi.sin(beta),
#                          a,
#                          0., # set in the discrete dynamics
#                          w,
#                          v])

# # Bicycle model with dynamic steering
# class BicycleModel2ndOrderWith2Delay(DynamicModel):

#     def __init__(self, system):
#         self.nu = 2
#         self.nx = 8
#         super(BicycleModel2ndOrderWith2Delay, self).__init__(system)

#         self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in2', 'delta_in', 'spline']
#         self.states_from_sensor = [True, True, True, True, False, False, True, False]
#         self.states_from_sensor_at_infeasible = [True, True, True, True, False, False, True, False]

#         self.inputs = ['a', 'w']
#         self.control_inputs['steering'] = 'delta_in'
#         self.control_inputs['velocity'] = 'v'
#         self.control_inputs['acceleration'] = 'a'
#         self.control_inputs['rot_velocity'] = 'w'

#     def continuous_model(self, x, u):
#         a = u[0]
#         w = u[1]
#         psi = x[2]
#         v = x[3]
#         delta = x[4] # affects the model ( = delta_in2)
#         delta_in2 = x[5] # = delta_in
#         delta_in = x[6] # = w (i.e., is updated with the input)

#         ratio = self.system.ratio
#         lr = self.system.lr

#         beta = casadi.arctan(ratio * casadi.tan(delta))

#         return np.array([v * casadi.cos(psi + beta),
#                          v * casadi.sin(psi + beta),
#                          (v/lr) * casadi.sin(beta),
#                          a,
#                          0., # set in the discrete dynamics
#                          0., # set in the discrete dynamics
#                          w,
#                          v])


class ContouringSecondOrderUnicycleModelWithEC(DynamicsModel):
    """
    Extended unicycle model that includes EC (Ego-Conditioned) robot state variables.
    
    This model is used for joint optimization (Variant B from Interactive Joint Planning paper).
    It extends the base ContouringSecondOrderUnicycleModel by adding decision variables
    for M EC robots that are jointly optimized with the ego robot.
    
    Each EC robot has:
    - States: x_ec, y_ec, psi_ec, v_ec (4 states per EC robot)
    - Inputs: a_ec, w_ec (2 inputs per EC robot)
    
    The ego robot uses the same dynamics as ContouringSecondOrderUnicycleModel:
    - States: x, y, psi, v, spline (5 states)
    - Inputs: a, w (2 inputs)
    
    Total dimensions:
    - nx = 5 (ego) + max_ec_robots * 4 (EC states)
    - nu = 2 (ego) + max_ec_robots * 2 (EC inputs)
    
    Args:
        max_ec_robots: Maximum number of EC robots to jointly optimize (default: 2)
    
    Design Rationale:
        This model is defined in solver_model.py to follow the same architecture as other
        dynamics models in the codebase. It inherits from DynamicsModel and implements
        all required methods (continuous_model, acados_symbolics, get_acados_dynamics, etc.)
        to integrate seamlessly with the solver generator pipeline.
    """

    def __init__(self, max_ec_robots=2):
        super().__init__()
        
        self.max_ec_robots = max_ec_robots
        
        # Ego robot dimensions (same as ContouringSecondOrderUnicycleModel)
        self.nu_ego = 2  # a, w
        self.nx_ego = 5  # x, y, psi, v, spline
        
        # EC robot dimensions (per robot)
        self.nu_ec = 2  # a_ec, w_ec
        self.nx_ec = 4  # x_ec, y_ec, psi_ec, v_ec
        
        # Total dimensions
        self.nu = self.nu_ego + max_ec_robots * self.nu_ec
        self.nx = self.nx_ego + max_ec_robots * self.nx_ec
        
        # Build state and input lists
        # Ego states and inputs
        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]
        
        # EC robot states and inputs
        for ec_idx in range(max_ec_robots):
            prefix = f"ec{ec_idx}_"
            self.states.extend([prefix + "x", prefix + "y", prefix + "psi", prefix + "v"])
            self.inputs.extend([prefix + "a", prefix + "w"])
        
        # Build bounds
        self._setup_bounds()
    
    def _setup_bounds(self):
        """
        Set up bounds for all variables (inputs first, then states).
        
        Bound order: [u0, u1, ..., x0, x1, ...]
        
        Ego bounds are taken from ContouringSecondOrderUnicycleModel.
        EC robot bounds are configurable but use reasonable defaults for unicycle robots.
        """
        # Constants for bounds (should match settings.yaml in production)
        POS_LIMIT = 2000.0
        EGO_MAX_ACCEL = 2.0
        EGO_MAX_ANGULAR_VEL = 0.8
        EGO_MAX_VEL = 3.0
        EGO_MIN_VEL = -0.01
        SPLINE_MIN = -1.0
        SPLINE_MAX = 10000.0
        
        # EC robot bounds (slightly more conservative than ego)
        EC_MAX_ACCEL = 1.5
        EC_MAX_ANGULAR_VEL = 1.0
        EC_MAX_VEL = 2.0
        EC_MIN_VEL = -0.1  # Allow small negative for numerical stability
        
        # Ego input bounds: [a, w]
        ego_input_lower = [-EGO_MAX_ACCEL, -EGO_MAX_ANGULAR_VEL]
        ego_input_upper = [EGO_MAX_ACCEL, EGO_MAX_ANGULAR_VEL]
        
        # Ego state bounds: [x, y, psi, v, spline]
        ego_state_lower = [-POS_LIMIT, -POS_LIMIT, -np.pi * 4, EGO_MIN_VEL, SPLINE_MIN]
        ego_state_upper = [POS_LIMIT, POS_LIMIT, np.pi * 4, EGO_MAX_VEL, SPLINE_MAX]
        
        # EC input bounds (per robot): [a_ec, w_ec]
        ec_input_lower = [-EC_MAX_ACCEL, -EC_MAX_ANGULAR_VEL]
        ec_input_upper = [EC_MAX_ACCEL, EC_MAX_ANGULAR_VEL]
        
        # EC state bounds (per robot): [x_ec, y_ec, psi_ec, v_ec]
        ec_state_lower = [-POS_LIMIT, -POS_LIMIT, -np.pi * 4, EC_MIN_VEL]
        ec_state_upper = [POS_LIMIT, POS_LIMIT, np.pi * 4, EC_MAX_VEL]
        
        # Build complete bound vectors (order: all inputs, then all states)
        self.lower_bound = ego_input_lower.copy()
        self.upper_bound = ego_input_upper.copy()
        
        # Add EC robot inputs
        for _ in range(self.max_ec_robots):
            self.lower_bound.extend(ec_input_lower)
            self.upper_bound.extend(ec_input_upper)
        
        # Add ego states
        self.lower_bound.extend(ego_state_lower)
        self.upper_bound.extend(ego_state_upper)
        
        # Add EC robot states
        for _ in range(self.max_ec_robots):
            self.lower_bound.extend(ec_state_lower)
            self.upper_bound.extend(ec_state_upper)

    def continuous_model(self, x, u):
        """
        Continuous dynamics for ego and all EC robots.
        
        Ego dynamics (unicycle):
            x_dot = v * cos(psi)
            y_dot = v * sin(psi)
            psi_dot = w
            v_dot = a
            spline_dot = v
        
        EC robot dynamics (same unicycle model for each):
            x_ec_dot = v_ec * cos(psi_ec)
            y_ec_dot = v_ec * sin(psi_ec)
            psi_ec_dot = w_ec
            v_ec_dot = a_ec
        
        Args:
            x: State vector [x_ego, x_ec_0, x_ec_1, ...]
            u: Input vector [u_ego, u_ec_0, u_ec_1, ...]
        
        Returns:
            np.array: State derivatives [x_dot_ego, x_dot_ec_0, x_dot_ec_1, ...]
        """
        # Ego dynamics
        a_ego = u[0]
        w_ego = u[1]
        psi_ego = x[2]
        v_ego = x[3]
        
        ego_dynamics = [
            v_ego * cd.cos(psi_ego),  # x_dot
            v_ego * cd.sin(psi_ego),  # y_dot
            w_ego,                     # psi_dot
            a_ego,                     # v_dot
            v_ego                      # spline_dot
        ]
        
        # EC robot dynamics
        ec_dynamics = []
        for ec_idx in range(self.max_ec_robots):
            # Input offset for this EC robot
            u_offset = self.nu_ego + ec_idx * self.nu_ec
            # State offset for this EC robot
            x_offset = self.nx_ego + ec_idx * self.nx_ec
            
            a_ec = u[u_offset]
            w_ec = u[u_offset + 1]
            psi_ec = x[x_offset + 2]
            v_ec = x[x_offset + 3]
            
            ec_dynamics.extend([
                v_ec * cd.cos(psi_ec),  # x_ec_dot
                v_ec * cd.sin(psi_ec),  # y_ec_dot
                w_ec,                    # psi_ec_dot
                a_ec                     # v_ec_dot
            ])
        
        return np.array(ego_dynamics + ec_dynamics)

    def acados_symbolics(self):
        """
        Create CasADi symbolic variables for acados OCP formulation.
        
        Returns:
            cd.SX: Concatenated symbolic vector z = [u; x]
        """
        x = cd.SX.sym("x", self.nx)
        u = cd.SX.sym("u", self.nu)
        z = cd.vertcat(u, x)
        self.load(z)
        return z

    def get_acados_dynamics(self):
        """
        Return dynamics expressions for acados OCP.
        
        Returns:
            tuple: (f_expl, f_impl) where:
                - f_expl: Explicit dynamics x_dot = f(x, u)
                - f_impl: Implicit dynamics x_dot - f(x, u) = 0
        """
        self._x_dot = cd.SX.sym("x_dot", self.nx)
        
        f_expl = numpy_to_casadi(self.continuous_model(self._z[self.nu:], self._z[:self.nu]))
        f_impl = self._x_dot - f_expl
        
        return f_expl, f_impl

    def get_xinit(self):
        """
        Get indices for initial state constraint.
        
        For the joint model, only ego robot states are constrained to initial state.
        EC robot initial states are set via parameters (their current observed state).
        
        Returns:
            range: Indices of ego states in the decision variable vector
        """
        # Only constrain ego states to initial state (indices nu to nu + nx_ego)
        # EC robot states are initialized via parameters
        return range(self.nu, self.nu + self.nx_ego)

    def get_ec_state_indices(self, ec_idx):
        """
        Get state indices for a specific EC robot.
        
        Args:
            ec_idx: Index of EC robot (0 to max_ec_robots-1)
        
        Returns:
            range: Indices of EC robot states in the state vector
        """
        if ec_idx >= self.max_ec_robots:
            raise ValueError(f"EC robot index {ec_idx} out of range (max: {self.max_ec_robots - 1})")
        
        start_idx = self.nx_ego + ec_idx * self.nx_ec
        return range(start_idx, start_idx + self.nx_ec)

    def get_ec_input_indices(self, ec_idx):
        """
        Get input indices for a specific EC robot.
        
        Args:
            ec_idx: Index of EC robot (0 to max_ec_robots-1)
        
        Returns:
            range: Indices of EC robot inputs in the input vector
        """
        if ec_idx >= self.max_ec_robots:
            raise ValueError(f"EC robot index {ec_idx} out of range (max: {self.max_ec_robots - 1})")
        
        start_idx = self.nu_ego + ec_idx * self.nu_ec
        return range(start_idx, start_idx + self.nu_ec)
