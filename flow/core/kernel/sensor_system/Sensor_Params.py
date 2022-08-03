# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/sensor_system/Sensor_Params.py
# Compiled at: 2022-07-30 23:09:45
# Size of source mod 2**32: 233 bytes
from flow.core.kernel.sensor_system.Sensor_Func import detect_distance_data_from_env, detect_velocity_data_from_env
detect_func = {'distance':detect_distance_data_from_env, 
 'velocity':detect_velocity_data_from_env}