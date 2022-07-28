"""Environment for training the acceleration behavior of vehicles in a ring."""

from flow.core import rewards
from flow.envs.base import Env

from gym.spaces.box import Box

import numpy as np
from flow.utils.hyper_paras import collision_distance_offline
ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration for autonomous vehicles, in m/s^2
    'max_accel': 3,
    # maximum deceleration for autonomous vehicles, in m/s^2
    'max_decel': 3,
    # desired velocity for all vehicles in the network, in m/s
    'target_velocity': 10,
    # specifies whether vehicles are to be sorted by position during a
    # simulation step. If set to True, the environment parameter
    # self.sorted_ids will return a list of all vehicles sorted in accordance
    # with the environment
    'sort_vehicles': False
}


class OptimizationEnv(Env):
    """Fully observed acceleration environment.

    This environment used to train autonomous vehicles to improve traffic flows
    when acceleration actions are permitted by the rl agent.

    Required from env_params:

    * max_accel: maximum acceleration for autonomous vehicles, in m/s^2
    * max_decel: maximum deceleration for autonomous vehicles, in m/s^2
    * target_velocity: desired velocity for all vehicles in the network, in m/s
    * sort_vehicles: specifies whether vehicles are to be sorted by position
      during a simulation step. If set to True, the environment parameter
      self.sorted_ids will return a list of all vehicles sorted in accordance
      with the environment

    States
        The state consists of the velocities and absolute position of all
        vehicles in the network. This assumes a constant number of vehicles.

    Actions
        Actions are a list of acceleration for each rl vehicles, bounded by the
        maximum accelerations and decelerations specified in EnvParams.

    Rewards
        The reward function is the two-norm of the distance of the speed of the
        vehicles in the network from the "target_velocity" term. For a
        description of the reward, see: flow.core.rewards.desired_speed

    Termination
        A rollout is terminated if the time horizon is reached or if two
        vehicles collide into one another.

    Attributes
    ----------
    prev_pos : dict
        dictionary keeping track of each veh_id's previous position
    absolute_position : dict
        dictionary keeping track of each veh_id's absolute position
    obs_var_labels : list of str
        referenced in the visualizer. Tells the visualizer which
        metrics to track
    """

    def __init__(self, env_params, sim_params, network, simulator='traci', perception_system=None, safety_system=None):
        for p in ADDITIONAL_ENV_PARAMS.keys():
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter \'{}\' not supplied'.format(p))

        # variables used to sort vehicles by their initial position plus
        # distance traveled
        self.prev_pos = dict()
        self.absolute_position = dict()

        super().__init__(env_params=env_params,
                         sim_params=sim_params,
                         network=network,
                         simulator=simulator,
                         perception_system=perception_system,
                         safety_system=safety_system)

    @property
    def action_space(self):
        """See class definition."""
        return Box(
            low=-abs(self.env_params.additional_params['max_decel']),
            high=self.env_params.additional_params['max_accel'],
            shape=(self.initial_vehicles.num_rl_vehicles, ),
            dtype=np.float32)

    @property
    def observation_space(self):
        """See class definition."""
        self.obs_var_labels = ['Velocity', 'Absolute_pos']
        return Box(
            low=0,
            high=1,
            shape=(2 * self.initial_vehicles.num_vehicles, ),
            dtype=np.float32)

    def _apply_rl_actions(self, rl_actions):
        """See class definition."""
        sorted_rl_ids = [
            veh_id for veh_id in self.sorted_ids
            if veh_id in self.k.vehicle.get_rl_ids()
        ]
        self.k.vehicle.apply_acceleration(sorted_rl_ids, rl_actions)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        if self.env_params.evaluate:
            return np.mean(self.k.vehicle.get_speed(self.k.vehicle.get_ids()))
        else:
            return rewards.desired_velocity(self, fail=kwargs['fail'])

    def get_state(self):
        """See class definition."""
        speed = [self.k.vehicle.get_speed(veh_id) / self.k.network.max_speed()
                 for veh_id in self.sorted_ids]
        pos = [self.k.vehicle.get_x_by_id(veh_id) / self.k.network.length()
               for veh_id in self.sorted_ids]

        return np.array(speed + pos)

    def additional_command(self):
        """See parent class.

        Define which vehicles are observed for visualization purposes, and
        update the sorting of vehicles using the self.sorted_ids variable.
        """
        # specify observed vehicles
        if self.k.vehicle.num_rl_vehicles > 0:
            for veh_id in self.k.vehicle.get_human_ids():
                self.k.vehicle.set_observed(veh_id)

        # update the "absolute_position" variable
        for veh_id in self.k.vehicle.get_ids():
            this_pos = self.k.vehicle.get_x_by_id(veh_id)

            if this_pos == -1001:
                # in case the vehicle isn't in the network
                self.absolute_position[veh_id] = -1001
            else:
                change = this_pos - self.prev_pos.get(veh_id, this_pos)
                self.absolute_position[veh_id] = \
                    (self.absolute_position.get(veh_id, this_pos) + change) \
                    % self.k.network.length()
                self.prev_pos[veh_id] = this_pos

    @property
    def sorted_ids(self):
        """Sort the vehicle ids of vehicles in the network by position.

        This environment does this by sorting vehicles by their absolute
        position, defined as their initial position plus distance traveled.

        Returns
        -------
        list of str
            a list of all vehicle IDs sorted by position
        """
        if self.env_params.additional_params['sort_vehicles']:
            return sorted(self.k.vehicle.get_ids(), key=self._get_abs_position)
        else:
            return self.k.vehicle.get_ids()

    def _get_abs_position(self, veh_id):
        """Return the absolute position of a vehicle."""
        return self.absolute_position.get(veh_id, -1001)

    def reset(self):
        """See parent class.

        This also includes updating the initial absolute position and previous
        position.
        """
        obs = super().reset()

        for veh_id in self.k.vehicle.get_ids():
            self.absolute_position[veh_id] = self.k.vehicle.get_x_by_id(veh_id)
            self.prev_pos[veh_id] = self.k.vehicle.get_x_by_id(veh_id)

        return obs

    def step(self, rl_actions):
        """Advance the environment by one step.

        Assigns actions to autonomous and human-driven agents (i.e. vehicles,
        traffic lights, etc...). Actions that are not assigned are left to the
        control of the simulator. The actions are then used to advance the
        simulator by the number of time steps requested per environment step.

        Results from the simulations are processed through various classes,
        such as the Vehicle and TrafficLight kernels, to produce standardized
        methods for identifying specific network state features. Finally,
        results from the simulator are used to generate appropriate
        observations.

        Parameters
        ----------
        rl_actions : array_like
            an list of actions provided by the rl algorithm

        Returns
        -------
        observation : array_like
            agent's observation of the current environment
        reward : float
            amount of reward associated with the previous state/action pair
        done : bool
            indicates whether the episode has ended
        info : dict
            contains other diagnostic information from the previous action
        """
        for _ in range(self.env_params.sims_per_step):
            self.time_counter += 1
            self.step_counter += 1
            self.distance_infor = {}
            self.velocity_infor = {}
            self.leader_infor = {}
            if len(self.k.vehicle.get_ids()) > 0:
                for veh_id in self.k.vehicle.get_ids():
                    veh_type = self.k.vehicle.get_type(veh_id)
                    sensor_system = self.k.vehicle.type_parameters[veh_type]['sensor_system']
                    distance = sensor_system.get_data_with_noise("distance", self, veh_id)
                    velocity = sensor_system.get_data_with_noise("velocity", self, veh_id)

                    self.distance_infor[veh_id] = distance
                    self.velocity_infor[veh_id] = velocity
                    self.leader_infor[veh_id] = self.k.vehicle.get_leader(veh_id)

                    self.perception_system.update_data_with_noise("distance", veh_id, distance)
                    self.perception_system.update_data_with_noise("velocity", veh_id, velocity)
                    distance = sensor_system.get_data_without_noise("distance", self, veh_id)
                    velocity = sensor_system.get_data_without_noise("velocity", self, veh_id)
                    self.perception_system.update_data_without_noise("distance", veh_id, distance)
                    self.perception_system.update_data_without_noise("velocity", veh_id, velocity)

                    # self.perception_system
                for veh_id in self.k.vehicle.get_ids():
                    self_velocity = self.perception_system.get_data_without_noise("velocity", veh_id)
                    lead_veh_id = self.k.vehicle.get_leader(veh_id)
                    if lead_veh_id:
                        lead_velocity = self.perception_system.get_data_without_noise("velocity", lead_veh_id)
                        headway = self.perception_system.get_data_without_noise("distance", veh_id)
                        self.safety_system.calculate_safety_metrics(veh_id=veh_id,
                                                                    headway=headway,
                                                                    self_velocity=self_velocity,
                                                                    lead_velocity=lead_velocity)
                    else:
                        self.safety_system.calculate_safety_metrics(veh_id=veh_id,
                                                                    headway=1e5,
                                                                    self_velocity=self_velocity,
                                                                    lead_velocity=1e3)
                # print(self.safety_system.get_fairness_metric(0.09,0.1))
                # print(self.perception_system.get_traffic_throughput(self.k.vehicle.get_ids()))

            # perform acceleration actions for controlled human-driven
            self.distance_list = []
            self.velocity_list = []
            self.vehicle_list = []
            cur_veh_id = list(self.leader_infor.keys())[0]
            cur_num = 0
            veh_num = len(self.k.vehicle.get_ids())
            while cur_num < veh_num:
                self.distance_list.append(self.distance_infor[cur_veh_id])
                self.velocity_list.append(self.velocity_infor[cur_veh_id])
                self.vehicle_list.append(cur_veh_id)
                cur_num += 1
                cur_veh_id = self.k.vehicle.get_leader(cur_veh_id)
            def opt_func(margins):
                t = 0.1
                for i in range(len(margins)):
                    self.perception_system.set_margin(self.vehicle_list[i], margins[i])
                accel = []
                for i in range(len(margins)):
                    action  = self.k.vehicle.get_acc_controller(
                        self.vehicle_list[i]).get_action(self)
                    accel.append(action)
                throughput = 0
                safety = 0
                headway = []
                delta_v = []
                for i in range(len(accel)):
                    leader = self.k.vehicle.get_leader(self.vehicle_list[i])
                    leader_index = self.vehicle_list.index(leader)
                    temp_delta_v = 0
                    if leader_index and self.velocity_list[i] and accel[i] and self.velocity_list[leader_index] and accel[leader_index]:
                        if self.velocity_list[i]+accel[i]*t >=0 and self.velocity_list[leader_index]+accel[leader_index]*t >=0:
                            temp_delta_v = -(self.velocity_list[i]+accel[i]*t-self.velocity_list[leader_index]-accel[leader_index]*t)
                        elif self.velocity_list[i]+accel[i]*t <=0 and self.velocity_list[leader_index]+accel[leader_index]*t >=0:
                            temp_delta_v = self.velocity_list[leader_index] + accel[leader_index] * t
                        elif self.velocity_list[i]+accel[i]*t >=0:
                            temp_delta_v = -(self.velocity_list[i] + accel[i] * t)
                    temp_headway = self.distance_list[i] + temp_delta_v * t
                    headway.append(temp_headway)
                    delta_v.append(temp_delta_v)
                for i in range(len(accel)):
                    if headway[i] > collision_distance_offline and self.velocity_list[i] and accel[i]:
                        throughput += (self.velocity_list[i]+accel[i]*t)/headway[i]
                        if delta_v[i] >= 0:
                            safety += 30
                        elif abs(headway[i]/(delta_v[i])) > 30:
                            safety += 30
                        elif abs(headway[i]/(delta_v[i])) <= 30:
                            safety += abs(headway[i]/(delta_v[i]))
                return -0.5*safety - 0.5*throughput

            from sko.GA import GA

            ga = GA(func=opt_func, n_dim=len(self.k.vehicle.get_ids()), size_pop=50, max_iter=800, prob_mut=0.001,
                    lb=[-5]*len(self.k.vehicle.get_ids()), ub=[5]*len(self.k.vehicle.get_ids()),
                    precision=1e-7)
            best_margin, best_y = ga.run()
            for i in range(len(best_margin)):
                self.perception_system.set_margin(self.vehicle_list[i], best_margin[i])

            # if len(self.k.vehicle.get_controlled_ids()) > 0:
            #     accel = []
            #     for veh_id in self.k.vehicle.get_controlled_ids():
            #         action = self.k.vehicle.get_acc_controller(
            #             veh_id).get_action(self)
            #         accel.append(action)
            #     self.k.vehicle.apply_acceleration(
            #         self.k.vehicle.get_controlled_ids(), accel)



            if len(self.k.vehicle.get_controlled_ids()) > 0:
                accel = []
                for veh_id in self.k.vehicle.get_controlled_ids():
                    action = self.k.vehicle.get_acc_controller(
                        veh_id).get_action(self)
                    accel.append(action)
                self.k.vehicle.apply_acceleration(
                    self.k.vehicle.get_controlled_ids(), accel)

            # perform lane change actions for controlled human-driven vehicles
            if len(self.k.vehicle.get_controlled_lc_ids()) > 0:
                direction = []
                for veh_id in self.k.vehicle.get_controlled_lc_ids():
                    target_lane = self.k.vehicle.get_lane_changing_controller(
                        veh_id).get_action(self)
                    direction.append(target_lane)
                self.k.vehicle.apply_lane_change(
                    self.k.vehicle.get_controlled_lc_ids(),
                    direction=direction)

            # perform (optionally) routing actions for all vehicles in the
            # network, including RL and SUMO-controlled vehicles
            routing_ids = []
            routing_actions = []
            for veh_id in self.k.vehicle.get_ids():
                if self.k.vehicle.get_routing_controller(veh_id) \
                        is not None:
                    routing_ids.append(veh_id)
                    route_contr = self.k.vehicle.get_routing_controller(
                        veh_id)
                    routing_actions.append(route_contr.choose_route(self))

            self.k.vehicle.choose_routes(routing_ids, routing_actions)

            self.apply_rl_actions(rl_actions)

            self.additional_command()

            # advance the simulation in the simulator by one step
            self.k.simulation.simulation_step()

            # store new observations in the vehicles and traffic lights class
            self.k.update(reset=False)

            # update the colors of vehicles
            if self.sim_params.render:
                self.k.vehicle.update_vehicle_colors()

            # crash encodes whether the simulator experienced a collision
            crash = self.k.simulation.check_collision()

            # stop collecting new simulation steps if there is a collision
            if crash:
                break

            # render a frame
            self.render()

        states = self.get_state()

        # collect information of the state of the network based on the
        # environment class used
        self.state = np.asarray(states).T

        # collect observation new state associated with action
        next_observation = np.copy(states)

        # test if the environment should terminate due to a collision or the
        # time horizon being met
        done = (self.time_counter >= self.env_params.sims_per_step *
                (self.env_params.warmup_steps + self.env_params.horizon)
                or crash)

        # compute the info for each agent
        infos = {}

        # compute the reward
        if self.env_params.clip_actions:
            rl_clipped = self.clip_actions(rl_actions)
            reward = self.compute_reward(rl_clipped, fail=crash)
        else:
            reward = self.compute_reward(rl_actions, fail=crash)

        return next_observation, reward, done, infos
