from flow.core.kernel.perception.Base_Error_Obj import Base_Error_Obj
import numpy as np
'''
Now there are three type of err_obj: None,Gaussian,Absolute
'''

class None_Error_Obj(Base_Error_Obj):
    def generate_error(self):
        self.error_val = 0
        return self.error_val

class Gaussian_Error_Obj(Base_Error_Obj):
    def generate_error(self):
        self.error_val = np.random.normal(0,self.error_size)
        return self.error_val

class Absolute_Error_Obj(Base_Error_Obj):
    def generate_error(self):
        self.error_val = self.error_size
        return self.error_val

# error_types = {"Gaussian": Gaussian_error_obj,
#                 "Absolute": Absolute_error_obj
#                     }