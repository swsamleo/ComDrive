
import copy
import numpy as np
from flow.core.kernel.perception.Fuse_Data_Obj import Avg_Fuse_Data_Obj


'''
Perception hold different sensors into it. 
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





