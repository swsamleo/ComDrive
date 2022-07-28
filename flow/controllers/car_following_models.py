"""
Contains several custom car-following control models.

These controllers can be used to modify the acceleration behavior of vehicles
in Flow to match various prominent car-following models that can be calibrated.

Each controller includes the function ``get_accel(self, env) -> acc`` which,
using the current state of the world and existing parameters, uses the control
model to return a vehicle acceleration.
"""
import math
import numpy as np
import copy

from flow.controllers.base_controller import BaseController
from flow.controllers.hit_history import Hit_History
from flow.utils.hyper_paras import collision_distance_offline
collision_distance_offline = 2
class CFMController(BaseController):
    """CFM controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : SumoCarFollowingParams
        see parent class
    k_d : float
        headway gain (default: 1)
    k_v : float
        gain on difference between lead velocity and current (default: 1)
    k_c : float
        gain on difference from desired velocity to current (default: 1)
    d_des : float
        desired headway (default: 1)
    v_des : float
        desired velocity (default: 8)
    time_delay : float, optional
        time delay (default: 0.0)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_d=1,
                 k_v=1,
                 k_c=1,
                 d_des=1,
                 v_des=8,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a CFM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.veh_id = veh_id
        self.k_d = k_d
        self.k_v = k_v
        self.k_c = k_c
        self.d_des = d_des
        self.v_des = v_des

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        d_l = env.k.vehicle.get_headway(self.veh_id)


        return self.k_d*(d_l - self.d_des) + self.k_v*(lead_vel - this_vel) + \
            self.k_c*(self.v_des - this_vel)


class CFMController_with_noise(BaseController):
    """CFM controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : SumoCarFollowingParams
        see parent class
    k_d : float
        headway gain (default: 1)
    k_v : float
        gain on difference between lead velocity and current (default: 1)
    k_c : float
        gain on difference from desired velocity to current (default: 1)
    d_des : float
        desired headway (default: 1)
    v_des : float
        desired velocity (default: 8)
    time_delay : float, optional
        time delay (default: 0.0)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_d=2,
                 k_v=1,
                 k_c=1,
                 d_des=2,
                 v_des=8,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a CFM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.veh_id = veh_id
        self.k_d = k_d
        self.k_v = k_v
        self.k_c = k_c
        self.d_des = d_des
        self.v_des = v_des

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        # d_l = env.k.vehicle.get_headway(self.veh_id)
        d_l = env.perception_system.get_data_with_noise("distance", self.veh_id)

        if self.k_d*(d_l - self.d_des) + self.k_v*(lead_vel - this_vel) + \
            self.k_c*(self.v_des - this_vel) < -self.max_deaccel:
            return -self.max_deaccel

        return self.k_d*(d_l - self.d_des) + self.k_v*(lead_vel - this_vel) + \
            self.k_c*(self.v_des - this_vel)


class BCMController(BaseController):
    """Bilateral car-following model controller.

    This model looks ahead and behind when computing its acceleration.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    k_d : float
        gain on distances to lead/following cars (default: 1)
    k_v : float
        gain on vehicle velocity differences (default: 1)
    k_c : float
        gain on difference from desired velocity to current (default: 1)
    d_des : float
        desired headway (default: 1)
    v_des : float
        desired velocity (default: 8)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_d=1,
                 k_v=1,
                 k_c=1,
                 d_des=1,
                 v_des=8,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a Bilateral car-following model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.veh_id = veh_id
        self.k_d = k_d
        self.k_v = k_v
        self.k_c = k_c
        self.d_des = d_des
        self.v_des = v_des

    def get_accel(self, env):
        """See parent class.

        From the paper:
        There would also be additional control rules that take
        into account minimum safe separation, relative speeds,
        speed limits, weather and lighting conditions, traffic density
        and traffic advisories
        """
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        trail_id = env.k.vehicle.get_follower(self.veh_id)
        trail_vel = env.k.vehicle.get_speed(trail_id)

        headway = env.k.vehicle.get_headway(self.veh_id)
        footway = env.k.vehicle.get_headway(trail_id)

        return self.k_d * (headway - footway) + \
            self.k_v * ((lead_vel - this_vel) - (this_vel - trail_vel)) + \
            self.k_c * (self.v_des - this_vel)


class LACController(BaseController):
    """Linear Adaptive Cruise Control.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    k_1 : float
        design parameter (default: 0.8)
    k_2 : float
        design parameter (default: 0.9)
    h : float
        desired time gap  (default: 1.0)
    tau : float
        lag time between control input u and real acceleration a (default:0.1)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_1=0.3,
                 k_2=0.4,
                 h=1,
                 tau=0.1,
                 a=0,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a Linear Adaptive Cruise controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.veh_id = veh_id
        self.k_1 = k_1
        self.k_2 = k_2
        self.h = h
        self.tau = tau
        self.a = a

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        headway = env.k.vehicle.get_headway(self.veh_id)
        L = env.k.vehicle.get_length(self.veh_id)
        ex = headway - L - self.h * this_vel
        ev = lead_vel - this_vel
        u = self.k_1*ex + self.k_2*ev
        a_dot = -(self.a/self.tau) + (u/self.tau)
        self.a = a_dot*env.sim_step + self.a

        if self.a > self.max_accel:
            return self.max_accel

        if self.a < -self.max_deaccel:
            return self.max_deaccel

        return self.a


class LACController_with_noise(BaseController):
    """Linear Adaptive Cruise Control.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    k_1 : float
        design parameter (default: 0.8)
    k_2 : float
        design parameter (default: 0.9)
    h : float
        desired time gap  (default: 1.0)
    tau : float
        lag time between control input u and real acceleration a (default:0.1)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_1=0.3,
                 k_2=0.4,
                 h=1,
                 tau=0.1,
                 a=0,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a Linear Adaptive Cruise controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.veh_id = veh_id
        self.k_1 = k_1
        self.k_2 = k_2
        self.h = h
        self.tau = tau
        self.a = a

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        # headway = env.k.vehicle.get_headway(self.veh_id)
        headway = env.perception_system.get_data_with_noise("distance", self.veh_id)
        L = env.k.vehicle.get_length(self.veh_id)
        ex = headway - L - self.h * this_vel
        ev = lead_vel - this_vel
        u = self.k_1*ex + self.k_2*ev
        a_dot = -(self.a/self.tau) + (u/self.tau)
        self.a = a_dot*env.sim_step + self.a

        if self.a > self.max_accel:
            return self.max_accel

        if self.a < -self.max_deaccel:
            return self.max_deaccel

        return self.a


class OVMController(BaseController):
    """Optimal Vehicle Model controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=1,
                 beta=1,
                 h_st=2,
                 h_go=15,
                 v_max=30,
                 time_delay=0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate an Optimal Vehicle Model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        h_dot = lead_vel - this_vel

        # V function here - input: h, output : Vh
        if h <= self.h_st:
            v_h = 0
        elif self.h_st < h < self.h_go:
            v_h = self.v_max / 2 * (1 - math.cos(math.pi * (h - self.h_st) /
                                                 (self.h_go - self.h_st)))
        else:
            v_h = self.v_max

        return self.alpha * (v_h - this_vel) + self.beta * h_dot


class LinearOVM(BaseController):
    """Linear OVM controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    v_max : float
        max velocity (default: 30)
    adaptation : float
        adaptation constant (default: 0.65)
    h_st : float
        headway for stopping (default: 5)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 adaptation=0.65,
                 h_st=5,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a Linear OVM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        # 4.8*1.85 for case I, 3.8*1.85 for case II, per Nakayama
        self.v_max = v_max
        # TAU in Traffic Flow Dynamics textbook
        self.adaptation = adaptation
        self.h_st = h_st

    def get_accel(self, env):
        """See parent class."""
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)

        # V function here - input: h, output : Vh
        alpha = 1.689  # the average value from Nakayama paper
        if h < self.h_st:
            v_h = 0
        elif self.h_st <= h <= self.h_st + self.v_max / alpha:
            v_h = alpha * (h - self.h_st)
        else:
            v_h = self.v_max

        return (v_h - this_vel) / self.adaptation


class IDMController(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        v = env.k.vehicle.get_speed(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)

        # in order to deal with ZeroDivisionError
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))

        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)


class SimCarFollowingController(BaseController):
    """Controller whose actions are purely defined by the simulator.

    Note that methods for implementing noise and failsafes through
    BaseController, are not available here. However, similar methods are
    available through sumo when initializing the parameters of the vehicle.

    Usage: See BaseController for usage example.
    """

    def get_accel(self, env):
        """See parent class."""
        return None


class GippsController(BaseController):
    """Gipps' Model controller.

    For more information on this controller, see:
    Traffic Flow Dynamics written by M.Treiber and A.Kesting
    By courtesy of Springer publisher, http://www.springer.com

    http://www.traffic-flow-dynamics.org/res/SampleChapter11.pdf

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    acc : float
        max acceleration, in m/s2 (default: 1.5)
    b : float
        comfortable deceleration, in m/s2 (default: -1)
    b_l : float
        comfortable deceleration for leading vehicle , in m/s2 (default: -1)
    s0 : float
        linear jam distance for saftey, in m (default: 2)
    tau : float
        reaction time in s (default: 1)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params=None,
                 v0=30,
                 acc=1.5,
                 b=-1,
                 b_l=-1,
                 s0=2,
                 tau=1,
                 delay=0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate a Gipps' controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )

        self.v_desired = v0
        self.acc = acc
        self.b = b
        self.b_l = b_l
        self.s0 = s0
        self.tau = tau

    def get_accel(self, env):
        """See parent class."""
        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        v_l = env.k.vehicle.get_speed(
            env.k.vehicle.get_leader(self.veh_id))

        # get velocity dynamics
        v_acc = v + (2.5 * self.acc * self.tau * (
                1 - (v / self.v_desired)) * np.sqrt(0.025 + (v / self.v_desired)))
        v_safe = (self.tau * self.b) + np.sqrt(((self.tau**2) * (self.b**2)) - (
                self.b * ((2 * (h-self.s0)) - (self.tau * v) - ((v_l**2) / self.b_l))))

        v_next = min(v_acc, v_safe, self.v_desired)

        return (v_next-v)/env.sim_step


class BandoFTLController(BaseController):
    """Bando follow-the-leader controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=.5,
                 beta=20,
                 h_st=2,
                 h_go=10,
                 v_max=32,
                 want_max_accel=False,
                 time_delay=0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate an Bando controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go
        self.want_max_accel = want_max_accel

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            if self.want_max_accel:
                return self.max_accel

        v_l = env.k.vehicle.get_speed(lead_id)
        v = env.k.vehicle.get_speed(self.veh_id)
        s = env.k.vehicle.get_headway(self.veh_id)
        return self.accel_func(v, v_l, s)

    def accel_func(self, v, v_l, s):
        """Compute the acceleration function."""
        v_h = self.v_max * ((np.tanh(s/self.h_st-2)+np.tanh(2))/(1+np.tanh(2)))
        s_dot = v_l - v
        u = self.alpha * (v_h - v) + self.beta * s_dot/(s**2)
        return u



class BandoFTLController_with_noise(BaseController):
    """Bando follow-the-leader controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=.5,
                 beta=20,
                 h_st=2,
                 h_go=100,
                 v_max=10,
                 want_max_accel=False,
                 time_delay=0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate an Bando controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go
        self.want_max_accel = want_max_accel
        self.max_accel  = 2
        self.max_deaccel = 5

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        # if not lead_id:  # no car ahead
        #     if self.want_max_accel:
        #         return self.max_accel

        if lead_id:
            v_l = env.k.vehicle.get_speed(lead_id)
        else:
            v_l = 0

        v = env.k.vehicle.get_speed(self.veh_id)
        # s = env.k.vehicle.get_headway(self.veh_id)
        # s = env.perception_system.get_data_with_noise("distance",self.veh_id)
        s = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        if s <= 0 :
             s = 1e-3
        return self.accel_func(v, v_l, s)

    def accel_func(self, v, v_l, s):
        """Compute the acceleration function."""
        v_h = self.v_max * ((np.tanh(s/self.h_st-2)+np.tanh(2))/(1+np.tanh(2)))
        s_dot = v_l - v
        u = self.alpha * (v_h - v) + self.beta * s_dot/(s**2)
        if u > self.max_accel:
            return self.max_accel
        if u < -1*self.max_deaccel:
            return -self.max_deaccel
        return u

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
                if self.veh_id in hit_histroies:
                    hit_histroies[self.veh_id] += 1
                else:
                    hit_histroies[self.veh_id] = 1
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action


class Hit_controller(IDMController):
    def __init__(self,
                 veh_id,
                 car_following_params=None,
                 noise=0,headway_noise = 0,s0=0,**kwargs):
        IDMController.__init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=s0,
                 time_delay=0.0,
                 noise=noise,
                 fail_safe='hit_test_action',
                 display_warnings=True,
                 car_following_params=car_following_params)

        '''hit_procedure records hit process. key is the vehicle_id and the value is the speed difference
        which is needed to simulate the process of hit 
        '''
        self.hit_procedure = {}
        print(self.s0)
        if hasattr(self,"headway_noise") == False:
            self.headway_noise = headway_noise

    def get_accel(self, env):
        # print(env.k.vehicle.type_parameters)
        v = env.k.vehicle.get_speed(self.veh_id)
        veh_type = env.k.vehicle.get_type(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        perception_layer = env.k.vehicle.type_parameters[veh_type]["perception"]
        distance_perception_obj = perception_layer.get_distance_perception_obj()
        h = distance_perception_obj.get_data(env, self.veh_id)
        # in order to deal with ZeroDivisionError
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)


    def hit_test_action(self,env,action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        sim_step = env.sim_step
        time = env.k.simulation.time

        # if self.veh_id in flow.controllers.hit_history.current_hit:
        #     acc_value = hit_histroies[current_hit[self.veh_id]].delta_v/sim_step
        #     del current_hit[self.veh_id]
        #     return acc_value

        v = env.k.vehicle.get_speed(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # print(self.headway_noise)
        h = env.k.vehicle.get_headway(self.veh_id)
        lead_v = env.k.vehicle.get_speed(lead_id)

        delta_v = lead_v - v
        if delta_v * sim_step + h <= 0:
            history = Hit_History(active_car=self.veh_id,
                                  passive_car=lead_id,
                                  active_car_speed=v,
                                  passive_car_speed=lead_v,
                                  step_time=time)

            hit_histroies[flow.controllers.hit_history.hit_id] = copy.copy(history)
            current_hit[lead_id] = flow.controllers.hit_history.hit_id
            flow.controllers.hit_history.hit_id += 1
            return delta_v / sim_step

        if action < -self.max_deaccel:
            return -self.max_deaccel
        return action





class IDMController_predict_margin_with_noise(BaseController):
    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        import torch
        import torch.optim as optim
        from torch.autograd import Variable
        import torch.nn.functional as F
        import torch.nn as nn
        import numpy as np

        self.margin_list = np.linspace(-5, 5, 101)
        class Net(nn.Module):
            def __init__(self):
                super(Net, self).__init__()
                self.fc1 = nn.Linear(8, 256)
                self.fc1.weight.data.normal_(0, 0.1)
                self.fc2 = nn.Linear(256, 128)
                self.fc2.weight.data.normal_(0, 0.1)
                self.out = nn.Linear(128, 101)
                self.out.weight.data.normal_(0, 0.1)


            def forward(self, x):
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                actions_value = self.out(x)
                return actions_value

            def choose_action(self, x):
                x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
                action_value = self.forward(x)
                action = torch.max(action_value, 1)[1].data.numpy()
                action = action[0]
                return action

        self.margin_net = Net()
        self.load_path = "flow/controllers/trained_network/margin_net_fairness_d1.pth"
        self.margin_net.load_state_dict(torch.load(self.load_path))
        self.preference = [0.5, 0.5]
        self.lead_velocity_threshold = 50
        self.backway_threshold = 1000
        self.margin_list = np.linspace(-5, 5,101)

    def get_state(self, veh_id,env):
        if not veh_id:
            return None
        headway = env.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = env.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = env.k.vehicle.get_leader(veh_id)
        follower_veh_id = env.k.vehicle.get_follower(veh_id)
        veh_ids = env.k.vehicle.get_ids()
        cur_traffic_throughput_with_noise = env.perception_system.get_traffic_throughput_with_noise(veh_ids)
        self_traffic_throughput_ratio = self_speed / (headway + 1e-5) / (cur_traffic_throughput_with_noise + 1e-5)
        if lead_veh_id and follower_veh_id:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = self.lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = self.lead_velocity_threshold
            backway = self.backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed, self_traffic_throughput_ratio, ]





    def predict_margin(self, veh_id, env):
        veh_state = self.get_state(veh_id, env)
        action = self.margin_net.choose_action(self.preference + veh_state)
        margin = self.margin_list[action]
        env.perception_system.set_margin(veh_id, margin)

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        self.predict_margin(self.veh_id, env)
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            import flow.controllers.hit_history
            from flow.controllers.hit_history import current_hit, hit_histroies
            flow.controllers.hit_history.hit_id += 1
            if self.display_warnings:
                print(
                    "=====================================\n"
                    "Vehicle {} is about to crash. Instantaneous acceleration "
                    "clipping applied.\n"
                    "=====================================".format(self.veh_id))

            return -this_vel / sim_step
        return action

        # if next_vel > 0:
        #     # the second and third terms cover (conservatively) the extra
        #     # distance the vehicle will cover before it fully decelerates
        #     if h < sim_step * next_vel + this_vel * 1e-3 + \
        #             0.5 * this_vel * sim_step:
        #         # if the vehicle will crash into the vehicle ahead of it in the
        #         # next time step (assuming the vehicle ahead of it is not
        #         # moving), then stop immediately
        #         if self.display_warnings:
        #             flow.controllers.hit_history.hit_id +=1
        #             print(
        #                 "=====================================\n"
        #                 "Vehicle {} is about to crash. Instantaneous acceleration "
        #                 "clipping applied.\n"
        #                 "=====================================".format(self.veh_id))
        #
        #         return -this_vel / sim_step
        #     else:
        #         # if the vehicle is not in danger of crashing, continue with
        #         # the requested action
        #         return action
        # else:
        #     return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel




class IDMController_with_noise_trainning(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)

        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

        # if next_vel > 0:
        #     # the second and third terms cover (conservatively) the extra
        #     # distance the vehicle will cover before it fully decelerates
        #     if h < sim_step * next_vel + this_vel * 1e-3 + \
        #             0.5 * this_vel * sim_step:
        #         # if the vehicle will crash into the vehicle ahead of it in the
        #         # next time step (assuming the vehicle ahead of it is not
        #         # moving), then stop immediately
        #         if self.display_warnings:
        #             flow.controllers.hit_history.hit_id +=1
        #             print(
        #                 "=====================================\n"
        #                 "Vehicle {} is about to crash. Instantaneous acceleration "
        #                 "clipping applied.\n"
        #                 "=====================================".format(self.veh_id))
        #
        #         return -this_vel / sim_step
        #     else:
        #         # if the vehicle is not in danger of crashing, continue with
        #         # the requested action
        #         return action
        # else:
        #     return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel



class IDMController_with_noise(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        h = env.perception_system.get_data_with_noise("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

        # if next_vel > 0:
        #     # the second and third terms cover (conservatively) the extra
        #     # distance the vehicle will cover before it fully decelerates
        #     if h < sim_step * next_vel + this_vel * 1e-3 + \
        #             0.5 * this_vel * sim_step:
        #         # if the vehicle will crash into the vehicle ahead of it in the
        #         # next time step (assuming the vehicle ahead of it is not
        #         # moving), then stop immediately
        #         if self.display_warnings:
        #             flow.controllers.hit_history.hit_id +=1
        #             print(
        #                 "=====================================\n"
        #                 "Vehicle {} is about to crash. Instantaneous acceleration "
        #                 "clipping applied.\n"
        #                 "=====================================".format(self.veh_id))
        #
        #         return -this_vel / sim_step
        #     else:
        #         # if the vehicle is not in danger of crashing, continue with
        #         # the requested action
        #         return action
        # else:
        #     return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel




class IDMController_with_noise_margin(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

        # if next_vel > 0:
        #     # the second and third terms cover (conservatively) the extra
        #     # distance the vehicle will cover before it fully decelerates
        #     if h < sim_step * next_vel + this_vel * 1e-3 + \
        #             0.5 * this_vel * sim_step:
        #         # if the vehicle will crash into the vehicle ahead of it in the
        #         # next time step (assuming the vehicle ahead of it is not
        #         # moving), then stop immediately
        #         if self.display_warnings:
        #             flow.controllers.hit_history.hit_id +=1
        #             print(
        #                 "=====================================\n"
        #                 "Vehicle {} is about to crash. Instantaneous acceleration "
        #                 "clipping applied.\n"
        #                 "=====================================".format(self.veh_id))
        #
        #         return -this_vel / sim_step
        #     else:
        #         # if the vehicle is not in danger of crashing, continue with
        #         # the requested action
        #         return action
        # else:
        #     return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel




class IDMController_with_noise_platooning(BaseController):
    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        import torch
        import torch.optim as optim
        from torch.autograd import Variable
        import torch.nn.functional as F
        import torch.nn as nn
        import numpy as np

        self.margin_list = np.linspace(-5, 5, 101)
        class Net(nn.Module):
            def __init__(self):
                super(Net, self).__init__()
                self.fc1 = nn.Linear(7, 256)
                self.fc1.weight.data.normal_(0, 0.1)
                self.fc2 = nn.Linear(256, 128)
                self.fc2.weight.data.normal_(0, 0.1)
                self.out = nn.Linear(128, 101)
                self.out.weight.data.normal_(0, 0.1)


            def forward(self, x):
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                actions_value = self.out(x)
                return actions_value

            def choose_action(self, x):
                x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
                action_value = self.forward(x)
                action = torch.max(action_value, 1)[1].data.numpy()
                action = action[0]
                return action

        self.margin_net = Net()
        self.load_path = "flow/controllers/trained_network/margin_net_mixed_fairness_IDM_d2p45.pth"
        self.margin_net.load_state_dict(torch.load(self.load_path))
        self.preference = [0.5, 0.5]
        self.lead_velocity_threshold = 50
        self.backway_threshold = 1000
        self.margin_list = np.linspace(-5, 5, 101)

    def get_state(self, veh_id,env):
        if not veh_id:
            return None
        headway = env.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = env.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = env.k.vehicle.get_leader(veh_id)
        follower_veh_id = env.k.vehicle.get_follower(veh_id)
        veh_ids = env.k.vehicle.get_ids()
        cur_traffic_throughput_with_noise = env.perception_system.get_traffic_throughput_with_noise(veh_ids)
        self_traffic_throughput_ratio = self_speed / (headway + 1e-5) / (cur_traffic_throughput_with_noise + 1e-5)
        if lead_veh_id and follower_veh_id:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = self.lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = self.lead_velocity_threshold
            backway = self.backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed, ]





    def predict_margin(self, veh_id, env):
        veh_state = self.get_state(veh_id, env)
        action = self.margin_net.choose_action(self.preference + veh_state)
        margin = self.margin_list[action]
        env.perception_system.set_margin(veh_id, margin)

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        self.predict_margin(self.veh_id, env)
        h = env.perception_system.get_data_with_noise("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)

        if lead_id and env.k.vehicle.get_type(self.veh_id) == env.k.vehicle.get_type(lead_id):
            h = env.perception_system.get_data_without_noise("distance", self.veh_id)

        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) < collision_distance_offline:
        #     h = env.perception_system.get_data_with_noise(sensor_type="distance", veh_id=self.veh_id)

        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) == 0 :
        #     print(env.perception_system.get_margin(self.veh_id))
        # print(env.perception_system.get_margin(self.veh_id))


        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) == 0 :
        #     return -self.max_deaccel

        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel :
            return -self.max_deaccel

        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
                if self.veh_id in hit_histroies:
                    hit_histroies[self.veh_id] += 1
                else:
                    hit_histroies[self.veh_id] = 1
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel




class IDMController_with_noise_s0_change(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))

        margin = env.perception_system.get_margin(self.veh_id)

        if margin > 0:
            s_star += margin
        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                if self.display_warnings:
                    print(
                        "=====================================\n"
                        "Vehicle {} is about to crash. Instantaneous acceleration "
                        "clipping applied.\n"
                        "=====================================".format(self.veh_id))
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action


    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel


class IDMController_predict_margin_with_noise2(BaseController):
    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        import torch
        import torch.optim as optim
        from torch.autograd import Variable
        import torch.nn.functional as F
        import torch.nn as nn
        import numpy as np

        self.margin_list = np.linspace(-5, 5, 101)
        class Net(nn.Module):
            def __init__(self):
                super(Net, self).__init__()
                self.fc1 = nn.Linear(7, 256)
                self.fc1.weight.data.normal_(0, 0.1)
                self.fc2 = nn.Linear(256, 128)
                self.fc2.weight.data.normal_(0, 0.1)
                self.out = nn.Linear(128, 101)
                self.out.weight.data.normal_(0, 0.1)


            def forward(self, x):
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                actions_value = self.out(x)
                return actions_value

            def choose_action(self, x):
                x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
                action_value = self.forward(x)
                action = torch.max(action_value, 1)[1].data.numpy()
                action = action[0]
                return action

        self.margin_net = Net()
        self.load_path = "flow/controllers/trained_network/margin_net_fairness_TTC.pth"
        self.margin_net.load_state_dict(torch.load(self.load_path))
        self.preference = [0.5, 0.5]
        self.lead_velocity_threshold = 50
        self.backway_threshold = 1000
        self.margin_list = np.linspace(-5, 5, 101)

    def get_state(self, veh_id,env):
        if not veh_id:
            return None
        headway = env.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = env.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = env.k.vehicle.get_leader(veh_id)
        follower_veh_id = env.k.vehicle.get_follower(veh_id)
        veh_ids = env.k.vehicle.get_ids()
        cur_traffic_throughput_with_noise = env.perception_system.get_traffic_throughput_with_noise(veh_ids)
        self_traffic_throughput_ratio = self_speed / (headway + 1e-5) / (cur_traffic_throughput_with_noise + 1e-5)
        if lead_veh_id and follower_veh_id:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = self.lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = self.lead_velocity_threshold
            backway = self.backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed, ]





    def predict_margin(self, veh_id, env):
        veh_state = self.get_state(veh_id, env)
        action = self.margin_net.choose_action(self.preference + veh_state)
        margin = self.margin_list[action]
        env.perception_system.set_margin(veh_id, margin)

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        self.predict_margin(self.veh_id, env)
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)

        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) < collision_distance_offline:
        #     h = env.perception_system.get_data_with_noise(sensor_type="distance", veh_id=self.veh_id)

        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) == 0 :
        #     print(env.perception_system.get_margin(self.veh_id))
        # print(env.perception_system.get_margin(self.veh_id))


        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))


        # if env.perception_system.get_data_with_noise(sensor_type="distance",
        #                                              veh_id=self.veh_id) == 0 :
        #     return -self.max_deaccel

        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel :
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
                if self.veh_id in hit_histroies:
                    hit_histroies[self.veh_id] += 1
                else:
                    hit_histroies[self.veh_id] = 1
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel


class IDMController_platooning_predict_margin(BaseController):
    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        import torch
        import torch.optim as optim
        from torch.autograd import Variable
        import torch.nn.functional as F
        import torch.nn as nn
        import numpy as np

        self.margin_list = np.linspace(-5, 5, 101)
        class Net(nn.Module):
            def __init__(self):
                super(Net, self).__init__()
                self.fc1 = nn.Linear(7, 256)
                self.fc1.weight.data.normal_(0, 0.1)
                self.fc2 = nn.Linear(256, 128)
                self.fc2.weight.data.normal_(0, 0.1)
                self.out = nn.Linear(128, 101)
                self.out.weight.data.normal_(0, 0.1)


            def forward(self, x):
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                actions_value = self.out(x)
                return actions_value

            def choose_action(self, x):
                x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
                action_value = self.forward(x)
                action = torch.max(action_value, 1)[1].data.numpy()
                action = action[0]
                return action

        self.margin_net = Net()
        self.load_path = "flow/controllers/trained_network/margin_net_fairness_TTC.pth"
        self.margin_net.load_state_dict(torch.load(self.load_path))
        self.preference = [0.5, 0.5]
        self.lead_velocity_threshold = 50
        self.backway_threshold = 1000
        self.margin_list = np.linspace(-5, 5, 101)

    def get_state(self, veh_id,env):
        if not veh_id:
            return None
        headway = env.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = env.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = env.k.vehicle.get_leader(veh_id)
        follower_veh_id = env.k.vehicle.get_follower(veh_id)
        veh_ids = env.k.vehicle.get_ids()
        cur_traffic_throughput_with_noise = env.perception_system.get_traffic_throughput_with_noise(veh_ids)
        self_traffic_throughput_ratio = self_speed / (headway + 1e-5) / (cur_traffic_throughput_with_noise + 1e-5)
        if lead_veh_id and follower_veh_id:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = self.lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = self.lead_velocity_threshold
            backway = self.backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed, ]





    def predict_margin(self, veh_id, env):
        veh_state = self.get_state(veh_id, env)
        action = self.margin_net.choose_action(self.preference + veh_state)
        margin = self.margin_list[action]
        env.perception_system.set_margin(veh_id, margin)

    def get_accel(self, env):
        """See parent class."""
        # in order to deal with ZeroDivisionError
        self.predict_margin(self.veh_id, env)
        h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if lead_id and env.k.vehicle.get_type(self.veh_id) == env.k.vehicle.get_type(lead_id):
            h = env.perception_system.get_data_without_noise("distance", self.veh_id)
        lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))

        # print(env.perception_system.get_margin(self.veh_id))
        # if lead_id and env.k.vehicle.get_type(self.veh_id) == env.k.vehicle.get_type(lead_id):
        #     if env.perception_system.get_data_without_noise("distance", self.veh_id) < 3:
        #         print(env.perception_system.get_margin(self.veh_id))
        #         print(h, self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2))
        if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
            return -self.max_deaccel
        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
                if self.veh_id in hit_histroies:
                    hit_histroies[self.veh_id] += 1
                else:
                    hit_histroies[self.veh_id] = 1
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

    def get_action(self, env):
        """Convert the get_accel() acceleration into an action.

        If no acceleration is specified, the action returns a None as well,
        signifying that sumo should control the accelerations for the current
        time step.

        This method also augments the controller with the desired level of
        stochastic noise, and utlizes the "instantaneous", "safe_velocity",
        "feasible_accel", and/or "obey_speed_limit" failsafes if requested.

        Parameters
        ----------
        env : flow.envs.Env
            state of the environment at the current time step

        Returns
        -------
        float
            the modified form of the acceleration
        """
        # clear the current stored accels of this vehicle to None
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=False, failsafe=True)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=False)
        env.k.vehicle.update_accel(self.veh_id, None, noise=True, failsafe=True)

        # this is to avoid abrupt decelerations when a vehicle has just entered
        # a network and it's data is still not subscribed
        if len(env.k.vehicle.get_edge(self.veh_id)) == 0:
            return None

        # this allows the acceleration behavior of vehicles in a junction be
        # described by sumo instead of an explicit model
        if env.k.vehicle.get_edge(self.veh_id)[0] == ":":
            return None

        accel = self.get_accel(env)

        # if no acceleration is specified, let sumo take over for the current
        # time step
        if accel is None:
            return None

        # store the acceleration without noise to each vehicle
        # run fail safe if requested
        env.k.vehicle.update_accel(self.veh_id, accel, noise=False, failsafe=False)
        accel_no_noise_with_failsafe = accel

        # for failsafe in self.failsafes:
        #     accel_no_noise_with_failsafe = failsafe(env, accel_no_noise_with_failsafe)
        #
        # env.k.vehicle.update_accel(self.veh_id, accel_no_noise_with_failsafe, noise=False, failsafe=True)

        # add noise to the accelerations, if requested
        if self.accel_noise > 0:
            accel += np.sqrt(env.sim_step) * np.random.normal(0, self.accel_noise)
        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=False)

        # run the fail-safes, if requested
        for failsafe in self.failsafes:
            accel = failsafe(env, accel)

        env.k.vehicle.update_accel(self.veh_id, accel, noise=True, failsafe=True)
        return accel




class BandoController_predict_margin_with_noise(BaseController):
    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=.5,
                 beta=20,
                 h_st=2,
                 h_go=100,
                 v_max=10,
                 want_max_accel=False,
                 time_delay=0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate an Bando controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go
        self.want_max_accel = want_max_accel
        self.max_accel = 2
        self.max_deaccel = 5

        import torch
        import torch.optim as optim
        from torch.autograd import Variable
        import torch.nn.functional as F
        import torch.nn as nn
        import numpy as np

        self.margin_list = np.linspace(-5, 5, 101)
        class Net(nn.Module):
            def __init__(self):
                super(Net, self).__init__()
                self.fc1 = nn.Linear(7, 256)
                self.fc1.weight.data.normal_(0, 0.1)
                self.fc2 = nn.Linear(256, 128)
                self.fc2.weight.data.normal_(0, 0.1)
                self.out = nn.Linear(128, 101)
                self.out.weight.data.normal_(0, 0.1)


            def forward(self, x):
                x = self.fc1(x)
                x = F.relu(x)
                x = self.fc2(x)
                x = F.relu(x)
                actions_value = self.out(x)
                return actions_value

            def choose_action(self, x):
                x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
                action_value = self.forward(x)
                action = torch.max(action_value, 1)[1].data.numpy()
                action = action[0]
                return action

        self.margin_net = Net()
        self.load_path = "flow/controllers/trained_network/margin_net_only_TTC_Bando_d2p45.pth"
        self.margin_net.load_state_dict(torch.load(self.load_path))
        self.preference = [0.5, 0.5]
        self.lead_velocity_threshold = 50
        self.backway_threshold = 1000
        self.margin_list = np.linspace(-5, 5, 101)


    def get_state(self, veh_id,env):
        if not veh_id:
            return None
        headway = env.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = env.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = env.k.vehicle.get_leader(veh_id)
        follower_veh_id = env.k.vehicle.get_follower(veh_id)
        veh_ids = env.k.vehicle.get_ids()
        cur_traffic_throughput_with_noise = env.perception_system.get_traffic_throughput_with_noise(veh_ids)
        self_traffic_throughput_ratio = self_speed / (headway + 1e-5) / (cur_traffic_throughput_with_noise + 1e-5)
        if lead_veh_id and follower_veh_id:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = env.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = env.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = self.lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = env.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = self.lead_velocity_threshold
            backway = self.backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed, ]


    def predict_margin(self, veh_id, env):
        veh_state = self.get_state(veh_id, env)
        action = self.margin_net.choose_action(self.preference + veh_state)
        margin = self.margin_list[action]
        env.perception_system.set_margin(veh_id, margin)

    # def get_accel(self, env):
    #     """See parent class."""
    #     # in order to deal with ZeroDivisionError
    #     self.predict_margin(self.veh_id, env)
    #     h = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
    #     v = env.perception_system.get_data_with_noise("velocity", self.veh_id)
    #     lead_id = env.k.vehicle.get_leader(self.veh_id)
    #     lead_vel = env.perception_system.get_data_with_noise("velocity", lead_id)
    #     if abs(h) < 1e-3:
    #         h = 1e-3
    #
    #     if lead_id is None or lead_id == '':  # no car ahead
    #         s_star = 0
    #     else:
    #         lead_vel = env.k.vehicle.get_speed(lead_id)
    #         s_star = self.s0 + max(
    #             0, v * self.T + v * (v - lead_vel) /
    #             (2 * np.sqrt(self.a * self.b)))
    #
    #
    #     if self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2) < -self.max_deaccel:
    #         return -self.max_deaccel
    #     return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        # if not lead_id:  # no car ahead
        #     if self.want_max_accel:
        #         return self.max_accel

        if lead_id:
            v_l = env.k.vehicle.get_speed(lead_id)
        else:
            v_l = 0
        self.predict_margin(self.veh_id, env)
        v = env.k.vehicle.get_speed(self.veh_id)
        # s = env.k.vehicle.get_headway(self.veh_id)
        # s = env.perception_system.get_data_with_noise("distance",self.veh_id)

        s = env.perception_system.get_data_with_noise_margin("distance", self.veh_id)
        if s <= 0 :
             s = 1e-3
        return self.accel_func(v, v_l, s)

    def accel_func(self, v, v_l, s):
        """Compute the acceleration function."""
        v_h = self.v_max * ((np.tanh(s/self.h_st-2)+np.tanh(2))/(1+np.tanh(2)))
        s_dot = v_l - v
        u = self.alpha * (v_h - v) + self.beta * s_dot/(s**2)
        if u > self.max_accel:
            return self.max_accel
        if u < -1*self.max_deaccel:
            return -self.max_deaccel
        return u

    def get_safe_action_instantaneous(self, env, action):
        import flow.controllers.hit_history
        from flow.controllers.hit_history import current_hit, hit_histroies
        if env.k.vehicle.num_vehicles == 1:
            return action

        lead_id = env.k.vehicle.get_leader(self.veh_id)

        # if there is no other vehicle in the lane, all actions are safe
        if lead_id is None:
            return action

        this_vel = env.k.vehicle.get_speed(self.veh_id)
        sim_step = env.sim_step
        lead_vel = env.k.vehicle.get_speed(lead_id)
        # next_vel = this_vel + action * sim_step
        h = env.k.vehicle.get_headway(self.veh_id)

        delta_v = lead_vel - this_vel
        if delta_v * sim_step + h <= collision_distance_offline:
            if env.k.simulation.time > env.k.simulation.sim_step*100:
                import flow.controllers.hit_history
                from flow.controllers.hit_history import current_hit, hit_histroies
                flow.controllers.hit_history.hit_id += 1
                # if self.display_warnings:
                #     print(
                #         "=====================================\n"
                #         "Vehicle {} is about to crash. Instantaneous acceleration "
                #         "clipping applied.\n"
                #         "=====================================".format(self.veh_id))
                if self.veh_id in hit_histroies:
                    hit_histroies[self.veh_id] += 1
                else:
                    hit_histroies[self.veh_id] = 1
            if delta_v * sim_step + h <= 0:
                return -this_vel / sim_step
        return action

