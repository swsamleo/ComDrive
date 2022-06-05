
import copy
import numpy as np
from flow.core.kernel.perception.perception_params import Error, Sensor, Fuse_Function, Perception_Type 

'''
Perception hold different sensors into it. 
'''
class Perception_Obj():
    def __init__(self, perception_type="distance", fuse_function_name=None):
        if perception_type not in Perception_Type.keys():
            raise Exception("Unknown perception_type, please choose from %s" % Perception_Type.keys())
        self.__perception_type = perception_type
        self.__sensors = []
        if fuse_function_name not in Fuse_Function.keys():
            raise Exception("Unknown fuse function, please choose from %s" % Fuse_Function.keys())
        self.__fuse_function = Fuse_Function[fuse_function_name]
        self.__data = {}

    def add_new_sensor(self, sensor_name, error_type=None, error_size=0):
        temp_sensor = {}
        if sensor_name in Sensor:
            temp_sensor["error_type"] = Sensor[sensor_name][0]
            temp_sensor["error_size"] = Sensor[sensor_name][1]
        else:
            if error_type not in Error:
                raise Exception("Unknown Error Type, please choose from %s" % Error.keys())
            if error_type and error_size:
                temp_sensor["error_type"] = error_type
                temp_sensor["error_size"] = float(error_size)
        self.__sensors.append(copy.deepcopy(temp_sensor))

    def set_fuse_function_name(self,fuse_function_name):
        if fuse_function_name not in Fuse_Function.keys():
            raise Exception("Unknown fuse function, please choose from %s" % Fuse_Function.keys())
        self.__fuse_function = Fuse_Function[fuse_function_name]

    def update_data(self, env, vehicle_id):
        data_list = self.__detect_data(env, vehicle_id)
        self.__fuse_data(data_list, vehicle_id)

    def get_data(self, vehicle_id):
        if vehicle_id in self.__data.keys():
            return self.__data[vehicle_id]

    def __detect_data(self, env, vehicle_id):
        temp_data_list = []
        for sensor in self.__sensors:
            raw_data = Perception_Type[self.__perception_type](env, vehicle_id)
            temp_data = raw_data + Error[sensor["error_type"]](sensor["error_size"])
            temp_data_list.append(temp_data)
        return temp_data_list

    def __fuse_data(self, data_list, vehicle_id):
        fused_data = self.__fuse_function(data_list)
        if fused_data >= 0:
            self.__data[vehicle_id] = fused_data
        else:
            self.__data[vehicle_id] = 0






