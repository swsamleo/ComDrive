"prevent divide by zero"
k = 1e-3
from flow.utils.hyper_paras import collision_distance_offline
import math
class Perception_System():
    def __init__(self):
        self.__data_with_noise = {"distance": {},
                                  "velocity": {},
                                  "angle": {}
                                  }
        self.__data_without_noise = {"distance": {},
                                     "velocity": {},
                                     "angle": {}
                                    }
        self.__margin = {}

    def get_data_with_noise(self, sensor_type, veh_id):
        if veh_id in self.__data_with_noise[sensor_type].keys():
            return self.__data_with_noise[sensor_type][veh_id]
        else:
            return None

    def get_data_with_noise_margin(self, sensor_type, veh_id):
        if veh_id in self.__margin.keys():
            return self.__data_with_noise[sensor_type][veh_id] + self.__margin[veh_id]
        else:
            return self.__data_with_noise[sensor_type][veh_id]

    def get_data_without_noise(self, sensor_type, veh_id):
        if veh_id in self.__data_without_noise[sensor_type].keys():
            return self.__data_without_noise[sensor_type][veh_id]
        else:
            return None

    def get_margin(self, veh_id):
        if veh_id in self.__margin.keys():
            return self.__margin[veh_id]
        else:
            return 0

    def set_margin(self, veh_id, margin):
        self.__margin[veh_id] = margin

    def update_data_with_noise(self, sensor_type, veh_id, data_value):
        self.__data_with_noise[sensor_type][veh_id] = data_value

    def update_data_without_noise(self, sensor_type, veh_id, data_value):
        self.__data_without_noise[sensor_type][veh_id] = data_value

    def get_traffic_throughput(self, veh_ids, excluded_id=None):
        temp_traffic_throughput = 0
        for veh_id in veh_ids:
            if veh_id != excluded_id and self.__data_with_noise["distance"][veh_id] > collision_distance_offline:
                temp_traffic_throughput += self.__data_without_noise["velocity"][veh_id] / (self.__data_without_noise["distance"][veh_id]+k)
        return temp_traffic_throughput

    #REMOVE
    def get_traffic_throughput_with_noise(self, veh_ids, excluded_id=None):
        temp_traffic_throughput = 0
        for veh_id in veh_ids:
            if veh_id != excluded_id:
                temp_traffic_throughput += self.__data_with_noise["velocity"][veh_id] / (self.__data_with_noise["distance"][veh_id]+k)
        return temp_traffic_throughput

    def get_throughput_metrics(self,lambda_para, beta , veh_ids, excluded_id=None):
        sum_throughput_metric = 0
        for veh_id in veh_ids:
            if veh_id != excluded_id:
                sum_throughput_metric += self.__data_without_noise["velocity"][veh_id] / (self.__data_without_noise["distance"][veh_id]+k)
        temp = 0
        for veh_id in veh_ids:
            if veh_id != excluded_id:
                temp += (self.__data_without_noise["velocity"][veh_id] / (self.__data_without_noise["distance"][veh_id] + k)
                         / (sum_throughput_metric+k)) ** (1 - beta)
        f_b = temp ** (1 / beta)
        if not isinstance(f_b, float):
            return 0
        if f_b <= 0 or sum_throughput_metric <= 0:
            return 0
        fairness = lambda_para * math.log(f_b, 2.72) + math.log(sum_throughput_metric, 2.72)
        if sum_throughput_metric > 0 and f_b > 0:
            return fairness
        else:
            return 0
