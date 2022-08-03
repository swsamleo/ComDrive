# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/fused_noise_data_center.py
# Compiled at: 2022-08-02 23:15:25
# Size of source mod 2**32: 791 bytes
import pandas as pd, numpy as np
from flow.core.kernel.data_center.base_individual_data_center import BaseDataCenter

class FusedNoiseDataCenter(BaseDataCenter):
    def __init__(self):
        self.columns = ['veh_id', 'detect_type', 'value', 'step']
        self.fuse_func = np.mean
        self.dataframe = np.array([[0]*len(self.columns)])

    def update_data(self, data):
        self.dataframe = np.row_stack((data, self.dataframe))

    def get_data(self, veh_id, detect_type, step, **kwargs):
        return self.dataframe[(self.dataframe[:, 0] == veh_id)
                              &(self.dataframe[:, 1] == detect_type)
                              &(self.dataframe[:, 3] == str(step))][0][2]

    def get_fuse_function(self):
        return self.fuse_func