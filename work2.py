import numpy as np
import time
from flow.networks.ring import RingNetwork
name = "ring_example"

from flow.core.params import VehicleParams,SimCarFollowingController,\
SumoCarFollowingParams

vehicles = VehicleParams()

from flow.controllers.car_following_models import IDMController,Hit_controller,IDMController_with_noise
from flow.controllers.routing_controllers import ContinuousRouter
from flow.core.kernel.perception.Perception_Obj import Perception_Obj
from flow.core.kernel.perception.Perception_Layer import Perception_Layer
from flow.core.kernel.perception.Fuse_Data_Obj import Quadratic_Avg_Fuse_Data_Obj
from flow.core.kernel.perception.Distance_Sensor_Obj import GPS_Distance_Sensor,Camera_Distance_Sensor,\
    Radar_Distance_Sensor,Base_Distance_Sensor_Obj
from flow.core.kernel.perception.Error_Obj import Gaussian_Error_Obj
error_size_list = np.linspace(0,10,21)
for error_size in error_size_list:
    for i in range(20):
        print(error_size,i)
        class Customed_GPS_Distance_Sensor(GPS_Distance_Sensor):
            def __init__(self,error_obj=Gaussian_Error_Obj,error_size=error_size):
                self.error_obj = error_obj(error_size)
                print(error_size)
                self.data = None
        #perception_module
        perception_layer = Perception_Layer()
        distance_perception_obj = Perception_Obj(Quadratic_Avg_Fuse_Data_Obj)
        distance_perception_obj.add_new_sensor(Customed_GPS_Distance_Sensor)
        perception_layer.set_distance_perception_obj(distance_perception_obj)


        sumo_car_following_para1 = SumoCarFollowingParams(speed_mode="aggressive")
        # unsafe_controller = Unsafe_Controller(car_following_params=sumo_car_following_para)
        vehicles.add("human",
                     acceleration_controller=(IDMController_with_noise, {"fail_safe":'instantaneous'}),
                     routing_controller=(ContinuousRouter, {}),
                     num_vehicles=15,
                     car_following_params=sumo_car_following_para1,
                     perception=perception_layer)

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
        flow_params['env'].horizon = 1000
        exp = Experiment(flow_params)

        # run the sumo simulation
        _ = exp.run(1, convert_to_csv=False)

        from flow.controllers.hit_history import current_hit,hit_histroies
        import flow.controllers.hit_history



        from flow.controllers.hit_history import current_hit,hit_histroies
        import flow.controllers.hit_history
        print(flow.controllers.hit_history.hit_id)



        def record(error_size):
            import sqlite3
            conn = sqlite3.connect('hit_record_changing_noise.db')
            c = conn.cursor()
            c.execute('''INSERT INTO  hit_record (head_way_noise,hit_num,cur_time,avg_speed)
                        values(%s,%s,%s,%s,%s,%s)'''%(error_size,flow.controllers.hit_history.hit_id,
                                                     int(time.time()),
                                                    np.mean(exp.env.k.vehicle.get_speed(vehicles.ids))))

            conn.commit()
            conn.close()
        record(error_size)
        flow.controllers.hit_history.initialize()