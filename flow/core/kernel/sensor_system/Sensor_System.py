
import pandas as pd
from flow.core.kernel.sensor_system.Sensor_Func import generate_no_noise, generate_Gaussian_noise
from flow.core.kernel.sensor_system.Sensor_Params import detect_func
sensor_data_columns = [
 'value']
import copy, numpy as np

class SensorSystem:
    def __init__(self):
        self.__sensors = []
        self.__detect_types = []
        self.__fuse_func = np.mean

    def add_new_sensor(self, detect_type, sensor_name, noise_function=generate_no_noise):
        self.__sensors.append((detect_type, sensor_name, noise_function))

    def detect_sensor_data_from_env(self, env, veh_id, detect_type):
        data = detect_func[detect_type](env, veh_id)
        return data

    def get_detect_types(self):
        return self.__detect_types

    def get_sensors(self):
        return self.__sensors

    def get_fuse_function(self):
        return self.__fuse_func