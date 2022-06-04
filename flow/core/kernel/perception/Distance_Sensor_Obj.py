from flow.core.kernel.perception.Base_Sensor_Obj import Base_Sensor_Obj,Base_Distance_Sensor_Obj
from flow.core.kernel.perception.Error_Obj import None_Error_Obj,Gaussian_Error_Obj,Absolute_Error_Obj

class GPS_Distance_Sensor(Base_Distance_Sensor_Obj):
    def __init__(self,error_obj=Gaussian_Error_Obj,error_size=2.45):
        self.error_obj = error_obj(error_size)
        self.data = None

class Camera_Distance_Sensor(Base_Distance_Sensor_Obj):
    def __init__(self,error_obj=Absolute_Error_Obj,error_size=2):
        self.error_obj = error_obj(error_size)
        self.data = None

class Radar_Distance_Sensor(Base_Distance_Sensor_Obj):
    def __init__(self,error_obj=Gaussian_Error_Obj,error_size=1):
        self.error_obj = error_obj(error_size)
        self.data = None