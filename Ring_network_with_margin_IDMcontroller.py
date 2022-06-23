

import numpy as np
import time
from flow.networks.ring import RingNetwork
name = "ring_example"

from flow.core.params import VehicleParams, SimCarFollowingController,\
SumoCarFollowingParams

vehicles = VehicleParams()

from flow.controllers.car_following_models import IDMController_predict_margin_with_noise,\
    IDMController_with_noise,IDMController,LACController,\
    OVMController,LinearOVM,BandoFTLController,CFMController,BCMController
from flow.controllers.routing_controllers import ContinuousRouter
from flow.core.kernel.sensor.Sensor_System import Sensor_System


#perception_module
sensor_system = Sensor_System()
sensor_system.add_new_sensor("distance", "G", error_type="Gaussian", error_size=2.45)
sensor_system.add_new_sensor("velocity", "G", error_type="Gaussian", error_size=0)

from flow.core.kernel.safety.Safety_System import Safety_System
safety_measurement_obj = Safety_System()

sumo_car_following_para1 = SumoCarFollowingParams(speed_mode="aggressive",decel=6,min_gap=0,max_speed=30,accel=3,
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

vehicles.add("human_1",
             acceleration_controller=(IDMController, {"fail_safe": 'instantaneous'}),
             routing_controller=(ContinuousRouter, {}),
             num_vehicles=20,
             car_following_params=sumo_car_following_para1,
             sensor_system=sensor_system,
             color="white")

# vehicles.add("human_0",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")

# vehicles.add("human_1",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_2",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_3",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_4",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_5",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_6",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_7",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_8",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_9",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_10",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_11",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_12",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_13",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_14",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_15",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_16",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_17",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_18",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_19",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_20",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_21",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_22",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_23",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_24",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_25",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_26",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_27",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_28",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_29",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")

# vehicles.add("human_30",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_31",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_32",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_33",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_34",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_35",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_36",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_37",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")
#
# vehicles.add("human_38",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")
#
# vehicles.add("human_39",
#              acceleration_controller=(IDMController_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")




# vehicles.add("human",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")


# vehicles.add("human2",
#              acceleration_controller=(IDMController_with_noise, {}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=2,
#              car_following_params=sumo_car_following_para1,
#              perception=perception_layer)

# sumo_car_following_para2 = SumoCarFollowingParams(speed_mode="aggressive",decel=0.1,min_gap=0,max_speed=100)
# vehicles.add("human2",
#              acceleration_controller=(Hit_controller2, {}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=1,
#              car_following_params=sumo_car_following_para2)



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



# vehicles.add("human_0",
#              acceleration_controller=(IDMController_predict_margin_with_noise, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=15,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="red")


# vehicles.add("human_1",
#              acceleration_controller=(LACController, {"fail_safe": 'instantaneous'}),
#              routing_controller=(ContinuousRouter, {}),
#              num_vehicles=15,
#              car_following_params=sumo_car_following_para1,
#              sensor_system=sensor_system,
#              color="white")



from flow.core.params import NetParams

net_params = NetParams(additional_params=ADDITIONAL_NET_PARAMS)

from flow.core.params import InitialConfig

initial_config = InitialConfig(spacing="random")

from flow.core.params import TrafficLightParams

traffic_lights = TrafficLightParams()


from flow.envs.ring.accel import AccelEnv
from flow.envs.Margin_DQN_withbackway_env import Margin_DQN_withbackway_Env

from flow.core.params import SumoParams

sim_params = SumoParams(sim_step=0.1, render=True , emission_path='data')

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
    safety_system=safety_system
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

