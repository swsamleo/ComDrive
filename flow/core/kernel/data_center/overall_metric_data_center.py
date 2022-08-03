# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/overall_metric_data_center.py
# Compiled at: 2022-08-02 23:10:15
# Size of source mod 2**32: 2397 bytes
import pandas as pd
from flow.core.kernel.data_center.metric_function import individual_metric_functions
from flow.core.kernel.data_center.base_individual_data_center import BaseDataCenter
import numpy as np, math

class OverallMetricDataCenter(BaseDataCenter):

    def __init__(self):
        self.columns = ['metric_type', 'value', 'step']
        self.metrics = []
        self.dataframe = np.array([[0]*len(self.columns)])

    def calculate_fairness_safety(self, env, beta_para=0.5, lambda_para=1):
        veh_ids = env.k.vehicle.get_ids()
        TTC_list = []
        for veh_id in veh_ids:
            temp = env.data_center.get_data(data_center_name='individual_metric', veh_id=veh_id,
              metric_type='TTC',
              step=(env.k.simulation.time))
            TTC_list.append(float(temp))

        safety = np.sum(TTC_list)
        fairness = 0
        for individual_TTC in TTC_list:
            fairness += (individual_TTC / safety) ** (1 - beta_para)

        fairness = lambda_para * math.log(fairness, 2.72)
        safety = math.log(safety, 2.72)
        new_row = ['fairness_safety', fairness + safety, env.k.simulation.time]
        self.update_data(new_row)
        return fairness + safety

    def calculate_overall_throughput(self, env):
        veh_ids = env.k.vehicle.get_ids()
        throughput_list = []
        for veh_id in veh_ids:
            temp = env.data_center.get_data(data_center_name='individual_metric', veh_id=veh_id,
              metric_type='throughput',
              step=(env.k.simulation.time))
            throughput_list.append(float(temp))

        overall_throughput = np.sum(throughput_list)
        new_row = ['overall_throughput', overall_throughput, env.k.simulation.time]
        self.update_data(new_row)
        return overall_throughput

    def update_data(self, data):
        self.dataframe = np.row_stack((data, self.dataframe))

    def get_data(self, metric_type, step, **kwargs):
        return self.dataframe[(self.dataframe[:, 0] == metric_type)
                              &(self.dataframe[:, 2] == str(step))][0][1]