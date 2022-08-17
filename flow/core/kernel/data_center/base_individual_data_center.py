# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/base_individual_data_center.py
# Compiled at: 2022-08-02 12:09:46
# Size of source mod 2**32: 204 bytes
from abc import ABCMeta, abstractmethod
import numpy as np


class BaseDataCenter(metaclass=ABCMeta):
    def __init__(self):
        self.data = {}

    def update_data(self, t, veh_id=None, **kwarg):
        if veh_id:
            if veh_id not in self.data.keys():
                self.data[veh_id] = {}
            if t not in self.data[veh_id].keys():
                self.data[veh_id][t] = {}
            self.data[veh_id][t].update(**kwarg)
        else:
            if t not in self.data.keys():
                self.data[t] = {}
            self.data[t].update(**kwarg)

    def get_data(self, t, key, veh_id=None, ):
        if veh_id:
            return self.data[veh_id][t][key]
        else:
            return self.data[t][key]
