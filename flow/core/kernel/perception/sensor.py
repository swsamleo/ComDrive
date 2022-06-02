
from flow.core.kernel.perception.error import error_types,Gaussian_error_obj, Absolute_error_obj,None_error_obj
class Sensor():
    def __init__(self,error_type=None,error_size=0):
        if error_type != None and error_size ==None:
            raise Exception("must give error_size with error_type")

        if error_type not in error_types.keys() and error_type != None:
            raise Exception("unsupported error type,please choose from %s"%(error_types.keys()))
        if error_type in error_types.keys():
            self.error_obj = error_types[error_type](error_size)
        else:
            self.error_obj = None_error_obj(0)
            print(type(self.error_obj))
        self.raw_data = None

    def process_data(self):
        self.processed_data = self.raw_data + self.error_obj.generate_error()
        if self.processed_data < 0 :
            self.processed_data = 0
