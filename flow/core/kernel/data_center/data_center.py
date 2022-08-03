# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/data_center.py
# Compiled at: 2022-08-03 00:08:52
# Size of source mod 2**32: 1575 bytes
from flow.core.kernel.data_center.sensor_data_center import SensorDataCenter
from flow.core.kernel.data_center.noise_data_center import NoiseDataCenter
from flow.core.kernel.data_center.fused_noise_data_center import FusedNoiseDataCenter
from flow.core.kernel.data_center.individual_metric_data_center import IndividualMetricDataCenter
from flow.core.kernel.data_center.overall_metric_data_center import OverallMetricDataCenter

class DataCenter:

    def __init__(self):
        self.sensor_data_center = SensorDataCenter()
        self.individual_noise_data_center = NoiseDataCenter()
        self.fused_noise_data_center = FusedNoiseDataCenter()
        self.individual_metric_data_center = IndividualMetricDataCenter()
        self.overall_metric_data_center = OverallMetricDataCenter()
        self.data_center_objects = {'sensor':self.sensor_data_center,  'individual_noise':self.individual_noise_data_center, 
         'fused_noise':self.fused_noise_data_center, 
         'individual_metric':self.individual_metric_data_center, 
         'overall_metric':self.overall_metric_data_center}

    def update_data(self, data_center_name, data):
        self.data_center_objects[data_center_name].update_data(data)

    def get_data(self, data_center_name, **kwargs):
        if data_center_name not in self.data_center_objects:
            raise Exception('Unknown data_name')
        return (self.data_center_objects[data_center_name].get_data)(**kwargs)