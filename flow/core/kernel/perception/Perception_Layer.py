'''
Perception_Layer contains several obj. Each obj has sensors inside.
For example distance_perception_obj has sensors detecting sensors.
'''


class Perception_Layer:
    def __init__(self):
        self.__distance_perception_obj = None
        self.__velocity_perception_obj = None
        self.__angle_perception_obj = None

    def set_distance_perception_obj(self,perception_obj):
        self.__distance_perception_obj = perception_obj

    def set_velocity_perception_obj(self,perception_obj):
        self.__velocity_perception_obj = perception_obj

    def set_angle_perception_obj(self,perception_obj):
        self.__angel_perception_obj = perception_obj

    def get_distance_perception_obj(self):
        return self.__distance_perception_obj

    def get_velocity_perception_obj(self):
        return self.__velocity_perception_obj

    def get_angle_perception_obj(self):
        return self.__angel_perception_obj