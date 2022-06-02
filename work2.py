
import numpy as np
import time
from flow.networks.ring import RingNetwork
name = "ring_example"
import numpy as np
s_list = np.linspace(0,7,15)
T_list = [0,0.5,1]
for s in s_list:
    for T in T_list:
        for i in range(20):
            print(s,T,i)
            from flow.core.params import VehicleParams,SimCarFollowingController,\
            SumoCarFollowingParams

            vehicles = VehicleParams()

            from flow.controllers.car_following_models import IDMController_with_noise,IDMController
            from flow.controllers.routing_controllers import ContinuousRouter
            from flow.core.kernel.perception.Perception import Perception


            #perception_module
            perception_layer = Perception()
            perception_layer.add(sensor_direction="front",error_type="Gaussian",error_size=2.45)
            # perception_layer.add(sensor_direction="front",error_type='Absolute',error_size=2)

            s0=0
            sumo_car_following_para1 = SumoCarFollowingParams(speed_mode="aggressive")
            # unsafe_controller = Unsafe_Controller(car_following_params=sumo_car_following_para)
            vehicles.add("human",
                         acceleration_controller=(IDMController_with_noise, {"fail_safe":"instantaneous","s0":7,"T":0}),
                         routing_controller=(ContinuousRouter, {}),
                         num_vehicles=20,
                         car_following_params=sumo_car_following_para1,
                         perception=perception_layer)

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



            from flow.core.params import NetParams

            net_params = NetParams(additional_params=ADDITIONAL_NET_PARAMS)

            from flow.core.params import InitialConfig

            initial_config = InitialConfig()

            from flow.core.params import TrafficLightParams

            traffic_lights = TrafficLightParams()


            from flow.envs.ring.accel import AccelEnv

            from flow.core.params import SumoParams

            sim_params = SumoParams(sim_step=0.1, render=False, emission_path='data')

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
            flow_params['env'].horizon = 5000
            exp = Experiment(flow_params)

            # run the sumo simulation
            _ = exp.run(1, convert_to_csv=False)

            from flow.controllers.hit_history import current_hit,hit_histroies
            import flow.controllers.hit_history
            print(flow.controllers.hit_history.hit_id)
            # for value in hit_histroies.values():
            #     print("passive_car:",value.passive_car)
            #     print("active_car_speed:",value.active_car_speed)
            #     print("step_time",value.step_time)


            def record(s0,T):
                import sqlite3
                conn = sqlite3.connect('hit_record.db')
                c = conn.cursor()
                c.execute('''INSERT INTO  hit_record (head_way_noise,hit_num,cur_time,s0,T,avg_speed)
                            values(2.45,%s,%s,%s,%s,%s)'''%(flow.controllers.hit_history.hit_id,
                                                         int(time.time()),
                                                        s0,
                                                        T,
                                                        np.mean(exp.env.k.vehicle.get_speed(vehicles.ids))))

                conn.commit()
                conn.close()
            record(s0,T)
            flow.controllers.hit_history.initialize()