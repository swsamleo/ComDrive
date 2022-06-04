
import copy
import numpy as np


'''
Perception hold different sensors into it. 
fuse_obj is used to process data from different sensors, such as using Arithmetic mean or Quadratic mean
add_new_sensor can add sensors into the perception_obj
get_data is the interface 
detect_data is used to get data from env
'''
class Perception_Obj():
    def __init__(self,fuse_data_obj):
        self.__sensors = []
        self.__fuse_obj = fuse_data_obj()

    def add_new_sensor(self,sensor):
        self.__sensors.append(sensor())

    def set_fuse_data_obj(self,fuse_data_obj):
        self.__fuse_obj = fuse_data_obj()


    def get_data(self,env,vehicle_id):
        data_list = self.__detect_data(env,vehicle_id)
        return self.__fuse_data(data_list)

    def __detect_data(self,env,vehicle_id):
        data_list = []
        for sensor in self.__sensors:
            sensor.detect_raw_data_from_env(env, vehicle_id)
            sensor.process_raw_data_with_noise()
            data_list.append(sensor.data)
        return data_list


    def __fuse_data(self,data_list):
        return self.__fuse_obj.fuse(data_list)





