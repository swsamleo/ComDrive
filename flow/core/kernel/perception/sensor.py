
from flow.core.kernel.perception.error import Gaussian_error_obj, Absolute_error_obj,None_error_obj
class Sensor():
    def __init__(self,error_type=None,error_size=0):
        if error_type != None and error_size ==None:
            raise Exception("must give error_size with error_type")

        self.error_types = {"Gaussian":Gaussian_error_obj,
                            "Absolute":Absolute_error_obj
                            }
        if error_type not in self.error_types.keys() and error_type != None:
            raise Exception("unsupported error type,please choose from %s"%(self.error_types.keys()))
        if error_type in self.error_types.keys():
            self.error_obj = self.error_types[error_type](error_size)
        else:
            self.error_obj = None_error_obj(0)
            print(type(self.error_obj))
        self.raw_data = None

    def process_data(self):
        self.processed_data = self.raw_data + self.error_obj.generate_error()
        if self.processed_data < 0 :
            self.processed_data = 0
