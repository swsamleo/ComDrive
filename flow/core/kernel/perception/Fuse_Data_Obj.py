from flow.core.kernel.perception.Base_Fuse_Data_Obj import Base_Fuse_Data_Obj
import numpy as np


'''
Fuse Data obj tackles how to process data getting from differnet sensors.
'''
class Arithmetic_Avg_Fuse_Data_Obj(Base_Fuse_Data_Obj):
    def fuse(self, *data_list):
        return np.mean(data_list)

class Quadratic_Avg_Fuse_Data_Obj(Base_Fuse_Data_Obj):
    def fuse(self, *data_list):
        data_list = np.array(data_list)
        return np.sqrt(np.sum(data_list*data_list)/len(data_list))