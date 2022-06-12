from  flow.core.kernel.sensor.Sensor_Params import Sensor,Error,\
    Detect_Type,Fuse_Function
import copy
class Sensor_System():
    def __init__(self):
        self.__sensors = {"distance": [],
                          "velocity": [],
                          "angle": []
                          }
        self.__fuse_func = {"distance": None,
                            "velocity": None,
                            "angle": None
                            }
        self.__data_with_noise = {"distance": {},
                                  "velocity": {},
                                  "angle": {}
                                  }

        self.__data_without_noise = {"distance": {},
                                    "velocity": {},
                                    "angle": {}
                                    }
        self.__activated_sensors = {"distance": "all",
                                    "velocity": "all",
                                    "angle": "all"
                                    }
        self.__has_error = {"distance": True,
                            "velocity": True,
                            "angle": True
                            }

    def add_new_sensor(self,sensor_type, sensor_name, error_type=None,error_size=None):
        if sensor_type not in self.__sensors.keys():
            raise Exception("no supported sensor_type please choose from %s"%self.__sensors.keys())
        if sensor_name in Sensor[sensor_type].keys():
            temp_sensor = {"sensor_name": sensor_name,
                           "error_type": Sensor[sensor_type][sensor_name][0],
                           "error_size": Sensor[sensor_type][sensor_name][1]}
            self.__sensors[sensor_type].append(copy.deepcopy(temp_sensor))
        else:
            if error_type == None or error_size == None:
                raise Exception( "error_type and error_size have to be specified for customized sensor")
            temp_sensor = {"sensor_name": sensor_name,
                           "error_type": error_type,
                           "error_size": error_size}
            self.__sensors[sensor_type].append(copy.deepcopy(temp_sensor))

    def get_has_error(self):
        return self.__has_error

    def set_has_error(self, state=True):
        self.__has_error = state

    def get_fuse_func(self,sensor_type):
        return self.__fuse_func[sensor_type]

    def set_fuse_func(self,sensor_type,fuse_func):
        self.__fuse_func[sensor_type] = fuse_func

    def get_activated_sensor(self,sensor_type):
        return self.__activated_sensors[sensor_type]

    def set_activated_sensor(self,sensor_type,activated_sensor_namelist):
        self.__activated_sensors[sensor_type] = activated_sensor_namelist

    def get_data_with_noise(self,sensor_type,env,veh_id):
        temp_data_list = self.__detect_data(sensor_type,env,veh_id)
        self.__fuse_data(temp_data_list[:], sensor_type, veh_id)
        return self.__data_with_noise[sensor_type][veh_id]

    def get_data_without_noise(self,sensor_type,env,veh_id):
        self.__detect_data(sensor_type, env, veh_id)
        return self.__data_without_noise[sensor_type][veh_id]

    def __detect_data(self,sensor_type, env, veh_id):
        temp_data_list = []
        for sensor in self.__sensors[sensor_type]:
            if self.__activated_sensors[sensor_type] == "all" or sensor["sensor_name"] in self.__activated_sensors[sensor_type]:
                raw_data = Detect_Type[sensor_type](env, veh_id)
                self.__data_without_noise[sensor_type][veh_id] = raw_data
                if sensor["error_type"] != None:
                    raw_data = raw_data + Error[sensor["error_type"]](sensor["error_size"])
                temp_data_list.append(raw_data)
        return temp_data_list

    def __fuse_data(self, data_list, sensor_type, veh_id):
        fused_data = Fuse_Function[self.__fuse_func[sensor_type]](data_list)
        if fused_data >= 0:
            self.__data_with_noise[sensor_type][veh_id] = fused_data
        else:
            self.__data_with_noise[sensor_type][veh_id] = 0


