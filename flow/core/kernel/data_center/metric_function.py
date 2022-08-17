
from flow.utils.hyper_paras import collision_distance_offline

def calculate_throughput(headway, self_velocity, **kwargs):
    if headway < collision_distance_offline:
        return 0
    return self_velocity / headway


def calculate_TTC(headway, self_velocity=0, lead_velocity=0, margin=0, headway_threshold=900, TTC_threshold=30, **kwargs):
    if headway <= collision_distance_offline:
        return 0
        if headway > headway_threshold:
            return TTC_threshold
        if not lead_velocity:
            return TTC_threshold
        if self_velocity > lead_velocity:
            if (headway - margin) / (self_velocity - lead_velocity) > TTC_threshold:
                return TTC_threshold
            return (headway - margin) / (self_velocity - lead_velocity)
    else:
        return TTC_threshold


individual_metric_functions = {'throughput':calculate_throughput, 
                                'TTC':calculate_TTC}