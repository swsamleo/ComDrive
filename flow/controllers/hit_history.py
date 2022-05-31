
hit_id = 0
current_hit = {}
hit_histroies = {}

def initialize():
    global hit_id,current_hit,hit_histroies
    hit_id = 0
    current_hit = {}
    hit_histroies = {}

class Hit_History():
    def __init__(self,
                 active_car=None,
                 passive_car=None,
                 active_car_speed=None,
                 passive_car_speed=None,
                 step_time=None,
                 **kwargs):
        self.active_car = active_car
        self.passive_car = passive_car
        self.active_car_speed = active_car_speed
        self.passive_car_speed = passive_car_speed
        self.delta_v = abs(active_car_speed-passive_car_speed)
        self.step_time = step_time

