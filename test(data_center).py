
# for i in range(10):
import numpy as np
import time
from flow.networks.ring import RingNetwork
for i in range(1):
    name = "ring_example"

    from flow.core.params import VehicleParams, SimCarFollowingController,\
    SumoCarFollowingParams

    vehicles = VehicleParams()

    from flow.controllers.car_following_models import IDMController_predict_margin_with_noise,\
        IDMController_with_noise,IDMController,LACController, IDMController_predict_margin_with_noise2,\
        OVMController,LinearOVM,BandoFTLController,CFMController,BCMController,BandoFTLController_with_noise,\
        BandoController_predict_margin_with_noise,CFMController_with_noise
    from flow.controllers.routing_controllers import ContinuousRouter
    from flow.core.kernel.sensor.Sensor_System import Sensor_System

    from flow.core.kernel.sensor_system.Sensor_System import SensorSystem
    from flow.core.kernel.sensor_system.Sensor_Func import generate_Gaussian_noise
    sensorsystem = SensorSystem()
    sensorsystem.add_new_sensor("distance", "GPS", noise_function=generate_Gaussian_noise)

    #perception_module
    sensor_system = Sensor_System()
    sensor_system.add_new_sensor("distance", "G", error_type="Gaussian", error_size=2.45)
    sensor_system.add_new_sensor("velocity", "G", error_type="Gaussian", error_size=0)

    from flow.core.kernel.safety.Safety_System import Safety_System
    safety_measurement_obj = Safety_System()

    sumo_car_following_para1 = SumoCarFollowingParams(speed_mode="aggressive",decel=6, min_gap=0, max_speed=30, accel=3,
                                                      speed_dev=0)
    # vehicles.add("human_0",
    #              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
    #              routing_controller=(ContinuousRouter, {}),
    #              num_vehicles=15,
    #              car_following_params=sumo_car_following_para1,
    #              sensor_system=sensor_system,
    #              color="red")
    #
    # vehicles.add("human_1",
    #              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
    #              routing_controller=(ContinuousRouter, {}),
    #              num_vehicles=15,
    #              car_following_params=sumo_car_following_para1,
    #              sensor_system=sensor_system,
    #              color="white")

    vehicles.add("testing_car",
                 acceleration_controller=(IDMController, {"fail_safe": 'instantaneous'}),
                 routing_controller=(ContinuousRouter, {}),
                 num_vehicles=20,
                 car_following_params=sumo_car_following_para1,
                 sensor_system=sensor_system,
                 color="white",
                 sensorsystem=sensorsystem)

    from flow.networks.ring import ADDITIONAL_NET_PARAMS

    print(ADDITIONAL_NET_PARAMS)


    ADDITIONAL_NET_PARAMS = {
        # length of the ring road
        "length": 250,
        # number of lanes
        "lanes": 1,
        # speed limit for all edges
        "speed_limit": 30,
        # resolution of the curves on the ring
        "resolution": 40
    }


    from flow.core.params import NetParams

    net_params = NetParams(additional_params=ADDITIONAL_NET_PARAMS)

    from flow.core.params import InitialConfig

    initial_config = InitialConfig(spacing="random")

    from flow.core.params import TrafficLightParams

    traffic_lights = TrafficLightParams()


    from flow.envs.ring.accel import AccelEnv
    from flow.envs.Margin_DQN_withbackway_env import Margin_DQN_withbackway_Env

    from flow.core.params import SumoParams

    sim_params = SumoParams(sim_step=0.1, render=True, emission_path='data')

    from flow.envs.ring.accel import ADDITIONAL_ENV_PARAMS

    ADDITIONAL_ENV_PARAMS = {
        'max_accel': 2,
        'max_decel': 10,
        'target_velocity': 10,
        'sort_vehicles': False
    }


    print(ADDITIONAL_ENV_PARAMS)

    from flow.core.params import EnvParams

    env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS)

    from flow.core.contrast_experiment import Experiment

    from flow.core.kernel.perception.Perception_System import Perception_System

    perception_system = Perception_System()

    from flow.core.kernel.safety.Safety_System import Safety_System
    safety_system = Safety_System()
    from flow.core.kernel.data_center.data_center import DataCenter
    data_center = DataCenter()

    flow_params = dict(
        exp_tag='ring_example',
        env_name=AccelEnv,
        network=RingNetwork,
        simulator='traci',
        sim=sim_params,
        env=env_params,
        net=net_params,
        veh=vehicles,
        initial=initial_config,
        tls=traffic_lights,
        perception_system=perception_system,
        safety_system=safety_system,
        data_center=data_center
    )

    # number of time steps
    flow_params['env'].horizon = 2000
    exp = Experiment(flow_params)

    # run the sumo simulation
    _ = exp.run(1, convert_to_csv=False)

# from flow.controllers.hit_history import current_hit,hit_histroies
# import flow.controllers.hit_history
# print(flow.controllers.hit_history.hit_id)
# for value in hit_histroies.values():
#     print("passive_car:",value.passive_car)
#     print("active_car_speed:",value.active_car_speed)
#     print("step_time",value.step_time)

