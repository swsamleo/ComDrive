# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/individual_metric_data_center.py
# Compiled at: 2022-08-02 23:40:22
# Size of source mod 2**32: 3483 bytes
import pandas as pd
from flow.core.kernel.data_center.metric_function import individual_metric_functions
from flow.core.kernel.data_center.base_individual_data_center import BaseDataCenter
import numpy as np


class IndividualMetricDataCenter(BaseDataCenter):
    def __init__(self):
        self.data = {}
        self.metrics = ['throughput', 'TTC']

    # def __init__(self):
    #     self.columns = ['veh_id', 'metric_type', 'value', 'step']
    #     self.dataframe = pd.DataFrame(columns=['veh_id', 'metric_type', 'value', 'step'])
    #     self.metrics = ['throughput', 'TTC']
    #     self.dataframe_len = 0
    #     self.dataframe = np.array([[0] * len(self.columns)])

    def calculate_metrics(self, env, veh_id):
        for metric in self.metrics:
            if metric in individual_metric_functions:
                metric_function = individual_metric_functions[metric]
                headway = env.k.simulation.data_center.get_data(data_center_name='sensor', veh_id=veh_id,
                  key='headway',
                  t=round(env.k.simulation.time,2))
                print(headway)
                headway = float(headway)
                headway_noise = env.k.simulation.data_center.get_data(data_center_name='fused_noise', veh_id=veh_id,
                  key='headway_noise',
                  t=round(env.k.simulation.time,2))
                headway_noise = float(headway_noise)
                headway += headway_noise
                self_velocity = env.k.simulation.data_center.get_data(data_center_name='sensor', veh_id=veh_id,
                  key='velocity',
                  t=round(env.k.simulation.time,2))
                self_velocity = float(self_velocity)
                lead_id = env.k.vehicle.get_leader(veh_id)
                lead_velocity = None
                lead_velocity = env.k.simulation.data_center.get_data(data_center_name='sensor', veh_id=lead_id,
                                                                      key='velocity',
                                                                      t=round(env.k.simulation.time, 2))
                lead_velocity = float(lead_velocity)
                attributes = {'headway': headway,
                              'self_velocity': self_velocity,
                              'lead_velocity': lead_velocity}
                value = metric_function(**attributes)
                new_row = [veh_id, metric, value, env.k.simulation.time]
                if metric == "TTC":
                    self.update_data(t=round(env.k.simulation.time, 2), veh_id=veh_id, TTC=value)
                elif metric == "throughput":
                    self.update_data(t=round(env.k.simulation.time, 2), veh_id=veh_id, throughput=value)

    # def update_data(self, data):
    #     self.dataframe = np.row_stack((data, self.dataframe))
    #     # self.dataframe.loc[self.dataframe_len] = data
    #     # self.dataframe_len += 1
    #
    # def get_data(self, veh_id, metric_type, step, **kwargs):
    #     return self.dataframe[(self.dataframe[:, 0] == veh_id)
    #                           &(self.dataframe[:, 1] == metric_type)
    #                           &(self.dataframe[:, 3] == str(step))][0][2]