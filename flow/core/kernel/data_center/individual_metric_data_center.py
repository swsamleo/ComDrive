# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/individual_metric_data_center.py
# Compiled at: 2022-08-02 23:40:22
# Size of source mod 2**32: 3483 bytes
import pandas as pd
from flow.core.kernel.data_center.metric_function import individual_metric_functions
from flow.core.kernel.data_center.base_individual_data_center import BaseDataCenter

class IndividualMetricDataCenter(BaseDataCenter):

    def __init__(self):
        self.dataframe = pd.DataFrame(columns=['veh_id', 'metric_type', 'value', 'step'])
        self.metrics = ['throughput', 'TTC']
        self.dataframe_len = 0

    def calculate_metrics(self, env, veh_id):
        for metric in self.metrics:
            if metric in individual_metric_functions:
                metric_function = individual_metric_functions[metric]
                headway = env.data_center.get_data(data_center_name='sensor', veh_id=veh_id,
                  detect_type='distance',
                  step=(env.k.simulation.time))
                headway = float(headway)
                headway_noise = env.data_center.get_data(data_center_name='fused_noise', veh_id=veh_id,
                  detect_type='distance',
                  step=(env.k.simulation.time))
                headway_noise = float(headway_noise)
                headway += headway_noise
                self_velocity = env.data_center.get_data(data_center_name='sensor', veh_id=veh_id,
                  detect_type='velocity',
                  step=(env.k.simulation.time))
                self_velocity = float(self_velocity)
                lead_id = env.k.vehicle.get_leader(veh_id)
                lead_velocity = None
                lead_velocity = env.data_center.get_data(data_center_name='sensor', veh_id=lead_id,
                  detect_type='velocity',
                  step=(env.k.simulation.time))
                lead_velocity = float(lead_velocity)
                attributes = {'headway':headway, 
                 'self_velocity':self_velocity, 
                 'lead_velocity':lead_velocity}
                value = metric_function(**attributes)
                new_row = [veh_id, metric, value, env.k.simulation.time]
                self.update_data(new_row)

    def update_data(self, data):
        self.dataframe.loc[self.dataframe_len] = data
        self.dataframe_len += 1

    def get_data(self, veh_id, metric_type, step, **kwargs):
        return self.dataframe.loc[((self.dataframe['veh_id'] == veh_id) & (self.dataframe['metric_type'] == metric_type) & (self.dataframe['step'] == step))]['value']