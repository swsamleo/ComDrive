

class Perception_System():
    def __init__(self):
        self.__data_with_noise = {"distance": {},
                                  "velocity": {},
                                  "angle": {}
                                  }
        self.__data_without_noise = {"distance": {},
                                     "velocity": {},
                                     "angle": {}
                                    }
        self.__margin = {}

    def get_data_with_noise(self, sensor_type, veh_id):
        if veh_id in self.__data_with_noise[sensor_type].keys():
            return self.__data_with_noise[sensor_type][veh_id]
        else:
            return None

    def get_data_without_noise(self, sensor_type, veh_id):
        if veh_id in self.__data_without_noise[sensor_type].keys():
            return self.__data_without_noise[sensor_type][veh_id]
        else:
            return None

    def get_margin(self, veh_id):
        if veh_id in self.__margin.keys():
            return self.__margin[veh_id]

    def set_margin(self, veh_id, margin):
        self.__margin[veh_id] = margin

    def update_data_with_noise(self, sensor_type, veh_id, data_value):
        if veh_id in self.__margin.keys():
            self.__data_with_noise[sensor_type][veh_id] = data_value + self.__margin[veh_id]
        else:
            self.__margin[veh_id] = 0
            self.__data_with_noise[sensor_type][veh_id] = data_value + self.__margin[veh_id]

    def update_data_without_noise(self, sensor_type, veh_id, data_value):
        self.__data_without_noise[sensor_type][veh_id] = data_value

    def get_traffic_throughput(self):
        pass

