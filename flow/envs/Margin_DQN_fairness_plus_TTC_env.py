from flow.core import rewards
from flow.envs.base import Env
from flow.utils.hyper_paras import collision_distance_offline
from copy import deepcopy
from gym.spaces.box import Box
import random
import numpy as np

import torch
import torch.optim as optim
from torch.autograd import Variable
import torch.nn.functional as F
import torch.nn as nn
import numpy as np
from traci.exceptions import FatalTraCIError
from traci.exceptions import TraCIException

import os
import atexit
import time
import traceback


preference = [0.5, 0.5]

margin_list = np.linspace(-5, 5, 101)
lead_velocity_threshold = 50
backway_threshold=1000
k = 1e-3

# parameters
Batch_size = 32
Lr = 0.01
Epsilon = 0.9  # greedy policy
Gamma = 0.9  # reward discount
Target_replace_iter = 100  # target update frequency
Memory_capacity = 2000
N_actions = len(margin_list)
N_states = 7
ENV_A_SHAPE = 0


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

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(N_states, 256)
        self.fc1.weight.data.normal_(0, 0.1)
        self.fc2 = nn.Linear(256, 128)
        self.fc2.weight.data.normal_(0, 0.1)
        self.out = nn.Linear(128, N_actions)
        self.out.weight.data.normal_(0, 0.1)

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        x = F.relu(x)
        actions_value = self.out(x)
        return actions_value


class DQN(object):
    def __init__(self, load_path=None):
        self.eval_net, self.target_net = Net(), Net()
        self.learn_step_counter = 0  # for target updating
        self.memory_counter = 0  # for storing memory
        self.memory = np.zeros((Memory_capacity, N_states * 2 + 2))  # innitialize memory
        self.optimizer = optim.Adam(self.eval_net.parameters(), lr=Lr)
        self.loss_func = nn.MSELoss()
        if load_path:
            self.load_path = load_path
            self.target_net.load_state_dict(torch.load(load_path))
            self.eval_net.load_state_dict(torch.load(load_path))
            print("network weights loaded successfully")
        else:
            self.load_path = "margin_net_fairness_TTC.pth"

    def choose_action(self, x):
        x = Variable(torch.unsqueeze(torch.FloatTensor(x), 0))
        if np.random.uniform() < Epsilon:
            action_value = self.eval_net.forward(x)
            action = torch.max(action_value, 1)[1].data.numpy()
            action = action[0]
        else:
            action = np.random.randint(0, N_actions)
            action = action
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, [a, r], s_))
        index = self.memory_counter % Memory_capacity
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self):
        # target net update
        if self.learn_step_counter % Target_replace_iter == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
            torch.save(self.target_net.state_dict(), self.load_path)
            # print("network weights saved")
        self.learn_step_counter += 1

        sample_index = np.random.choice(Memory_capacity, Batch_size)
        # print(sample_index)
        b_memory = self.memory[sample_index, :]
        # print(b_memory)
        b_s = Variable(torch.FloatTensor(b_memory[:, :N_states]))
        b_a = Variable(torch.LongTensor(b_memory[:, N_states:N_states + 1].astype(int)))
        b_r = Variable(torch.FloatTensor(b_memory[:, N_states + 1:N_states + 2]))
        b_s_ = Variable(torch.FloatTensor(b_memory[:, -N_states:]))

        # print(b_a)
        # print(self.eval_net(b_s))
        q_eval = self.eval_net(b_s).gather(1, b_a)
        # print(q_eval)
        q_next = self.target_net(b_s_).detach()
        q_target = b_r + Gamma * q_next.max(1)[0].view(Batch_size, 1)
        loss = self.loss_func(q_eval, q_target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()








class Margin_DQN_fairness_plus_TTC_env(Env):
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

        self.dqn = DQN(load_path="margin_net_fairness_TTC.pth")
        # self.dqn = DQN()
        # self.max_traffic_throughput = 0
        # self.max_fairness_metric = 0
        self.cur_fairness_metric = 0
        self.cur_traffic_throughput = 0
        self.cur_traffic_throughput_with_noise = 0

        self.max_traffic_throughput = 5
        self.max_safety_metric = 0.3

        self.max_traffic_throughput_reward_dic = {}
        self.min_traffic_throughput_reward_dic = {}
        self.max_safety_reward_dic = {}
        self.min_safety_reward_dic = {}

    @property
    def action_space(self):
        pass
        # """See class definition."""
        # return Box(
        #     low=-abs(self.env_params.additional_params['max_decel']),
        #     high=self.env_params.additional_params['max_accel'],
        #     shape=(self.initial_vehicles.num_rl_vehicles, ),
        #     dtype=np.float32)

    @property
    def observation_space(self):
        pass
        # """See class definition."""
        # self.obs_var_labels = ['Velocity', 'Absolute_pos']
        # return Box(
        #     low=0,
        #     high=1,
        #     shape=(2 * self.initial_vehicles.num_vehicles, ),
        #     dtype=np.float32)

    def _apply_rl_actions(self, rl_actions):
        pass
        # """See class definition."""
        # sorted_rl_ids = [
        #     veh_id for veh_id in self.sorted_ids
        #     if veh_id in self.k.vehicle.get_rl_ids()
        # ]
        # self.k.vehicle.apply_acceleration(sorted_rl_ids, rl_actions)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        pass
        # if self.env_params.evaluate:
        #     return np.mean(self.k.vehicle.get_speed(self.k.vehicle.get_ids()))
        # else:
        #     return rewards.desired_velocity(self, fail=kwargs['fail'])

    def get_state(self, veh_id=None):
        """See class definition."""
        # speed = [self.k.vehicle.get_speed(veh_id) / self.k.network.max_speed()
        #          for veh_id in self.sorted_ids]
        # pos = [self.k.vehicle.get_x_by_id(veh_id) / self.k.network.length()
        #        for veh_id in self.sorted_ids]
        if not veh_id:
            return None
        headway = self.perception_system.get_data_with_noise("distance", veh_id)
        self_speed = self.perception_system.get_data_with_noise("velocity", veh_id)
        lead_veh_id = self.k.vehicle.get_leader(veh_id)
        follower_veh_id = self.k.vehicle.get_follower(veh_id)
        # self_traffic_throughput_ratio = self_speed/(headway+k)/(self.cur_traffic_throughput_with_noise+k)
        # if self_traffic_throughput_ratio > 0.6:
        #     print(self_traffic_throughput_ratio)
        if lead_veh_id and follower_veh_id:
            lead_speed = self.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = self.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = self.perception_system.get_data_with_noise("velocity", follower_veh_id)
        elif lead_veh_id == None and follower_veh_id != None:
            backway = self.perception_system.get_data_with_noise("distance", follower_veh_id)
            follow_speed = self.perception_system.get_data_with_noise("velocity", follower_veh_id)
            lead_speed = lead_velocity_threshold
        elif lead_veh_id != None and follower_veh_id == None:
            lead_speed = self.perception_system.get_data_with_noise("velocity", lead_veh_id)
            backway = backway_threshold
            follow_speed = 0
        elif lead_veh_id == None and follower_veh_id == None:
            lead_speed = lead_velocity_threshold
            backway = backway_threshold
            follow_speed = 0
        return [headway, self_speed, lead_speed, backway, follow_speed,]

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

    def update_perception_safety_system(self):
        veh_ids = self.k.vehicle.get_ids()
        if len(veh_ids) > 0:
            for veh_id in veh_ids:
                veh_type = self.k.vehicle.get_type(veh_id)
                sensor_system = self.k.vehicle.type_parameters[veh_type]['sensor_system']
                distance = sensor_system.get_data_with_noise("distance", self, veh_id)
                velocity = sensor_system.get_data_with_noise("velocity", self, veh_id)
                self.perception_system.update_data_with_noise("distance", veh_id, distance)
                self.perception_system.update_data_with_noise("velocity", veh_id, velocity)
                distance = sensor_system.get_data_without_noise("distance", self, veh_id)
                velocity = sensor_system.get_data_without_noise("velocity", self, veh_id)
                self.perception_system.update_data_without_noise("distance", veh_id, distance)
                self.perception_system.update_data_without_noise("velocity", veh_id, velocity)

                # self.perception_system
            for veh_id in veh_ids:
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
                                                                lead_velocity=lead_velocity)
        self.cur_traffic_throughput = self.perception_system.get_traffic_throughput(veh_ids)
        self.cur_fairness_metric = self.safety_system.get_fairness_metric(lambda_para=1, beta=0.5)
        self.cur_traffic_throughput_with_noise = self.perception_system.get_traffic_throughput_with_noise(veh_ids)



    def step(self,rl_actions):
        # if not self.max_safety_reward_dic.keys():
        #     veh_ids = self.k.vehicle.get_ids()
        #     for veh_id in veh_ids:
        #         self.max_safety_reward_dic[veh_id] = 0
        #         self.min_safety_reward_dic[veh_id] = 1e5
        #         self.max_traffic_throughput_reward_dic[veh_id] = 0
        #         self.min_traffic_throughput_reward_dic[veh_id] = 1e5
        rewards = 0
        for _ in range(self.env_params.sims_per_step):
            self.time_counter += 1
            self.step_counter += 1
            self.update_perception_safety_system()
            cur_state = []
            actions = []
            for veh_id in self.k.vehicle.get_ids():
                veh_state = self.get_state(veh_id)
                action = self.dqn.choose_action(preference+veh_state)
                self.perception_system.set_margin(veh_id, margin_list[action])
                cur_state.append(veh_state[:])
                actions.append(action)

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

        self.update_perception_safety_system()
        veh_ids = self.k.vehicle.get_ids()
        for i in range(len(veh_ids)):
            veh_id = veh_ids[i]
            veh_state = self.get_state(veh_id)
            r1_ = self.perception_system.get_data_without_noise("velocity", veh_id)/\
                 self.perception_system.get_data_without_noise("distance", veh_id)
            if self.perception_system.get_data_without_noise("distance", veh_id) <= collision_distance_offline:
                r1_ = 0


            r2_ = self.cur_fairness_metric - self.safety_system.get_fairness_metric(lambda_para=1,beta=0.5,excluded_id=veh_id)



            r1 = r1_ / self.max_traffic_throughput
            r2 = r2_ / self.max_safety_metric


            if r1 < 0:
                r1 = 0

            if r2 < 0:
                r2 = 0

            # if r1 > 0 and r2 > 0:
            #     print(r1/r2)

            r_array = np.array([r1, r2])
            r = np.dot(np.array(preference).T, r_array)
            if self.k.simulation.time <= self.k.simulation.sim_step:
                r = 0

            rewards += r
            # self.max_traffic_throughput_reward_dic[veh_id] = max(self.max_traffic_throughput_reward_dic[veh_id],r1_)
            # self.min_traffic_throughput_reward_dic[veh_id] = min(self.min_traffic_throughput_reward_dic[veh_id],r1_)
            # self.max_safety_reward_dic[veh_id] = max(self.max_safety_reward_dic[veh_id],r2_)
            # self.min_safety_reward_dic[veh_id] = min(self.min_safety_reward_dic[veh_id],r2_)



            self.dqn.store_transition(preference+cur_state[i], actions[i], r, preference+veh_state)

        if self.dqn.memory_counter > Memory_capacity:
            self.dqn.learn()

        # states = self.get_state()

        # collect information of the state of the network based on the
        # environment class used
        # self.state = np.asarray(states).T

        # collect observation new state associated with action
        # next_observation = np.copy(states)

        # test if the environment should terminate due to a collision or the
        # time horizon being met
        done = (self.time_counter >= self.env_params.sims_per_step *
                (self.env_params.warmup_steps + self.env_params.horizon)
                or crash)

        # compute the info for each agent
        infos = {}

        return None, rewards, done, infos


