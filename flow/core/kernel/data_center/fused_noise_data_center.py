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
        self.dataframe = pd.DataFrame(columns=['veh_id', 'detect_type', 'value', 'step'])
        self.fuse_func = np.mean
        self.dataframe_len = 0

    def update_data(self, data):
        self.dataframe.loc[self.dataframe_len] = data
        self.dataframe_len += 1

    def get_data(self, veh_id, detect_type, step, **kwargs):
        return self.dataframe.loc[((self.dataframe['veh_id'] == veh_id) & (self.dataframe['detect_type'] == detect_type) & (self.dataframe['step'] == step))]['value']

    def get_fuse_function(self):
        return self.fuse_func