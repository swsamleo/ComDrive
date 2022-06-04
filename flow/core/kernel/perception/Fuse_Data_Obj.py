from flow.core.kernel.perception.Base_Fuse_Data_Obj import Base_Fuse_Data_Obj
import numpy as np

class Avg_Fuse_Data_Obj(Base_Fuse_Data_Obj):
    def fuse(self, *data_list):
        return np.mean(data_list)