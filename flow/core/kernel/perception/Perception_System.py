

class Perception_System():
    def __init__(self):
        self.__data = {"distance": {},
                       "velocity": {},
                       "angle": {}
                       }
        self.__margin = 0

    def get_data(self, sensor_type, veh_id):
        if veh_id in self.__data[sensor_type].keys():
            return self.__data[sensor_type][veh_id]
        else:
            return None

    def get_margin(self):
        return self.__margin

    def set_margin(self, margin):
        self.__margin = margin

    def update_data(self, sensor_type, veh_id, data_value):
        self.__data[sensor_type][veh_id] = data_value + self.__margin

    def get_traffic_throughput(self):
        pass

