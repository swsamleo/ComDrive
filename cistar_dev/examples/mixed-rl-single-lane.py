''' Basic test of fully rl environment with accelerations as actions. Single lane. Mixed human and rl.  

Variables:
    sumo_params {dict} -- [Pass time step, whether safe mode is on or off]
    sumo_binary {str} -- [Use either sumo-gui or sumo for visual or non-visual]
    type_params {dict} -- [Types of cars in the system. 
    Format {"name": (number, (Model, {params}), (Lane Change Model, {params}), initial_speed)}]
    env_params {dict} -- [Params for reward function]
    net_params {dict} -- [Params for network.
                            length: road length
                            lanes
                            speed limit
                            resolution: number of edges comprising ring
                            net_path: where to store net]
    cfg_params {dict} -- [description]
    initial_config {dict} -- [shuffle: randomly reorder cars to start experiment
                                spacing: if gaussian, add noise in start positions
                                bunching: how close to place cars at experiment start]
    scenario {[type]} -- [Which road network to use]
'''
import logging
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import run_experiment_lite
from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.envs.gym_env import GymEnv

from cistar_dev.scenarios.loop.gen import CircleGenerator
from cistar_dev.scenarios.loop.loop_scenario import LoopScenario
from cistar_dev.controllers.rlcontroller import RLController
from cistar_dev.controllers.car_following_models import *
from cistar_dev.controllers.lane_change_controllers import *


def run_task(*_):
    import cistar_dev.envs as cistar_envs
    logging.basicConfig(level=logging.INFO)

    tot_cars = 8

    auton_cars = 4
    human_cars = tot_cars - auton_cars

    sumo_params = {"time_step": 0.1, "human_sm": 1, "rl_sm": 1}

    sumo_binary = "sumo"

    type_params = [("rl", auton_cars, (RLController, {}), (StaticLaneChanger, {}), 0),
                   ("cfm", human_cars, (BCMController, {"v_des": 10}), (StaticLaneChanger, {}), 0)]

    env_params = {"target_velocity": 8, "max-deacc": 3, "max-acc": 3, "num_steps": 1000}

    net_params = {"length": 200, "lanes": 1, "speed_limit": 30, "resolution": 40, "net_path": "debug/rl/net/"}

    cfg_params = {"start_time": 0, "end_time": 3000, "cfg_path": "debug/rl/cfg/"}

    initial_config = {"shuffle": False}

    scenario = LoopScenario("rl-test", CircleGenerator, type_params, net_params, cfg_params,
                            initial_config=initial_config)

    from cistar_dev import pass_params
    env_name = "SimpleAccelerationEnvironment"
    pass_params = (env_name, sumo_params, sumo_binary, type_params, env_params, net_params,
                cfg_params, initial_config, scenario)

    env = GymEnv(env_name, record_video=False, register_params=pass_params)
    horizon = env.horizon
    env = normalize(env)

    logging.info("Experiment Set Up complete")

    print("experiment initialized")

    env = normalize(env)

    policy = GaussianMLPPolicy(
        env_spec=env.spec,
        hidden_sizes=(32, 32)
    )

    baseline = LinearFeatureBaseline(env_spec=env.spec)

    algo = TRPO(
        env=env,
        policy=policy,
        baseline=baseline,
        batch_size=1000,
        max_path_length=horizon,
        # whole_paths=True,
        n_itr=2,
        # discount=0.99,
        # step_size=0.01,
    )
    algo.train()

for seed in [1]:  # [1, 5, 10, 73, 56]
    run_experiment_lite(
        run_task,
        # Number of parallel workers for sampling
        n_parallel=4,
        # Only keep the snapshot parameters for the last iteration
        snapshot_mode="last",
        # Specifies the seed for the experiment. If this is not provided, a random seed
        # will be used
        seed=seed,
        mode="local",
        exp_prefix="leah-test-exp",
        python_command="/home/aboudy/anaconda2/envs/rllab-distributed/bin/python3.5"
        # plot=True,
    )
