# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/metric_function.py
# Compiled at: 2022-08-02 23:10:15
# Size of source mod 2**32: 1014 bytes
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