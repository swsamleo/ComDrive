
from flow.utils.hyper_paras import collision_distance_offline
def calculate_TTC(headway, self_velocity=0, lead_velocity=0, margin=0, headway_threshold=900,TTC_threshold=30):
    if headway <= collision_distance_offline:
        return 0
    if headway > headway_threshold:
        return TTC_threshold
    if not lead_velocity:
        return TTC_threshold
    if self_velocity > lead_velocity:
        if (headway-margin)/(self_velocity-lead_velocity) > TTC_threshold:
            return TTC_threshold
        else:
            return (headway - margin) / (self_velocity - lead_velocity)
    else:
        return TTC_threshold