
import numpy as np
import time
from flow.networks.ring import RingNetwork
name = "ring_example"

from flow.core.params import VehicleParams,SimCarFollowingController,\
SumoCarFollowingParams

vehicles = VehicleParams()

from flow.controllers.car_following_models import IDMController,Hit_controller
from flow.controllers.routing_controllers import ContinuousRouter


Hit_controller.headway_noise = 1000
sumo_car_following_para1 = SumoCarFollowingParams(speed_mode="aggressive",decel=6,min_gap=0,max_speed=30,accel=2,
                                                  speed_dev=0)
# unsafe_controller = Unsafe_Controller(car_following_params=sumo_car_following_para)
vehicles.add("human",
             acceleration_controller=(Hit_controller, {}),
             routing_controller=(ContinuousRouter, {}),
             num_vehicles=10,
             car_following_params=sumo_car_following_para1)

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
    "length": 230,
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

initial_config = InitialConfig(spacing="uniform", perturbation=1)

from flow.core.params import TrafficLightParams

traffic_lights = TrafficLightParams()


from flow.envs.ring.accel import AccelEnv

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

from flow.core.experiment import Experiment

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
)

# number of time steps
flow_params['env'].horizon = 3000
exp = Experiment(flow_params)

# run the sumo simulation
_ = exp.run(1, convert_to_csv=False)

from flow.controllers.hit_history import current_hit,hit_histroies
import flow.controllers.hit_history
print(flow.controllers.hit_history.hit_id)
for value in hit_histroies.values():
    print("passive_car:",value.passive_car)
    print("active_car_speed:",value.active_car_speed)
    print("step_time",value.step_time)


def record():
    import sqlite3
    conn = sqlite3.connect('hit_record.db')
    c = conn.cursor()
    c.execute('''INSERT INTO  hit_record (head_way_noise,hit_time,cur_time)
                values(%s,%s,%s)'''%(int(np.sqrt(Hit_controller.headway_noise)),
                                     flow.controllers.hit_history.hit_id,
                                     int(time.time())))
    conn.commit()
    conn.close()
#record()
flow.controllers.hit_history.initialize()

# for value in hit_histroies.values():
#     print("passive_car:",value.passive_car)
#     print("active_car_speed:",value.active_car_speed)
#     print("step_time",value.step_time)



# import os
#
# emission_location = os.path.join(exp.env.sim_params.emission_path, exp.env.network.name)
# print(emission_location + '-emission.xml')