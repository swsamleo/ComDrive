"prevent divide by zero"
k = 1e-3
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

    def get_data_with_noise_margin(self, sensor_type, veh_id):
        if veh_id in self.__margin.keys():
            return self.__data_with_noise[sensor_type][veh_id] + self.__margin[veh_id]
        else:
            return self.__data_with_noise[sensor_type][veh_id]

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
        self.__data_with_noise[sensor_type][veh_id] = data_value

    def update_data_without_noise(self, sensor_type, veh_id, data_value):
        self.__data_without_noise[sensor_type][veh_id] = data_value

    def get_traffic_throughput(self, veh_ids):
        temp_traffic_throughput = 0
        for veh_id in veh_ids:
            temp_traffic_throughput += self.__data_without_noise["velocity"][veh_id] / (self.__data_without_noise["distance"][veh_id]+k)
        return temp_traffic_throughput

    def get_traffic_throughput_with_noise(self, veh_ids):
        temp_traffic_throughput = 0
        for veh_id in veh_ids:
            temp_traffic_throughput += self.__data_with_noise["velocity"][veh_id] / (self.__data_with_noise["distance"][veh_id]+k)
        return temp_traffic_throughput
