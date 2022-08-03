# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/data_center/base_individual_data_center.py
# Compiled at: 2022-08-02 12:09:46
# Size of source mod 2**32: 204 bytes
from abc import ABCMeta, abstractmethod

class BaseDataCenter(metaclass=ABCMeta):

    @abstractmethod
    def update_data(self, data):
        pass

    @abstractmethod
    def get_data(self):
        pass