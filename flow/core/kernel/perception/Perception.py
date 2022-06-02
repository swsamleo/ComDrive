sensor_types = ["GPS","radar"]
import copy
import numpy as np
from flow.core.kernel.perception.sensor import Sensor


'''
Perception hold different sensors into it. 
Sensors get four directions, which are'front','back','left','right'
other modules can get data by get_data()
get_raw_data() directly gets data from sumo which has no noise
process_data() add noise to raw_data
give_out_data return the processed data
if there are multiple sensors in one direction fuse_data() can fuse different sensors'
data

'''
class Perception():
    def __init__(self):
        self.sensors = {}
        self.sensors["front"] = []
        self.sensors["back"] = []
        self.sensors["left"] = []
        self.sensors["right"] = []

    def add(self,sensor_direction,error_type=None,error_size=0):
        if sensor_direction not  in self.sensors.keys():
            raise Exception("dirctions must be %s"%(self.sensors.keys()))

        temp_sensor = Sensor(error_type,error_size)
        self.sensors[sensor_direction].append(copy.deepcopy(temp_sensor))

    def get_data(self,env,vehicle_id,direction):
        self.get_raw_data(env,vehicle_id)
        self.process_data(direction)
        return self.give_out_data(direction)

    def get_raw_data(self,env,vehicle_id,direction="front"):
        if self.sensors[direction] == []:
            raise Exception("%s sensors unassigned" % (direction))
        for sensor in self.sensors[direction]:
            sensor.raw_data = env.k.vehicle.get_headway(vehicle_id)

    def process_data(self,direction):
        for sensor in self.sensors[direction]:
            sensor.process_data()

    def give_out_data(self,direction):
        return self.fuse_data(direction)

    def fuse_data(self,direction):
        result = []
        for sensor in self.sensors[direction]:
            result.append(sensor.processed_data)
        return self.process_fused_data(result)

    def process_fused_data(self,result):
        res = np.mean(result)
        return res

