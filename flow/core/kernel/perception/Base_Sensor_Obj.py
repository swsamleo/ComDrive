from abc import ABCMeta,abstractmethod
class Base_Sensor_Obj(metaclass=ABCMeta):
    def __init__(self,error_type=None,error_size=0):
        self.error_type = error_type
        self.error_size = error_size

    @abstractmethod
    def detect_raw_data_from_env(self,data_type):
        pass

    @abstractmethod
    def process_raw_data_with_noise(self):
        pass

    @abstractmethod
    def get_data(self):
        pass


class Base_Distance_Sensor_Obj(Base_Sensor_Obj):
    def __init__(self,error_obj=object,error_size=None):
        self.detect_type = "distance"
        self.error_obj = error_obj(error_size)

    def detect_raw_data_from_env(self, env, vehicle_id):
        self.data = env.k.vehicle.get_headway(vehicle_id)


    def process_raw_data_with_noise(self):
        self.data += self.error_obj.generate_error()

    def get_data(self):
        return self.data


class Base_Velocity_Sensor_Obj(Base_Sensor_Obj):
    def __init__(self,error_obj=object,error_size=None):
        self.detect_type = "velocity"
        self.error_obj = error_obj(error_size)

    def detect_raw_data_from_env(self, env, vehicle_id):
        self.data = env.k.vehicle.get_speed(vehicle_id)

    def process_raw_data_with_noise(self):
        self.data += self.error_obj.generate_error()

    def get_data(self):
        return self.data