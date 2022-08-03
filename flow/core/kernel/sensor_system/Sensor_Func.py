# uncompyle6 version 3.8.0
# Python bytecode 3.7.0 (3394)
# Decompiled from: Python 3.7.4 (default, Aug  9 2019, 18:34:13) [MSC v.1915 64 bit (AMD64)]
# Embedded file name: /home/richzhou/Desktop/intern/ComDrive/flow/core/kernel/sensor_system/Sensor_Func.py
# Compiled at: 2022-08-02 21:32:49
# Size of source mod 2**32: 688 bytes
import numpy as np

def detect_distance_data_from_env(env, vehicle_id):
    return env.k.vehicle.get_headway(vehicle_id)


def detect_velocity_data_from_env(env, vehicle_id):
    return env.k.vehicle.get_speed(vehicle_id)


def generate_no_noise():
    return 0


def generate_Gaussian_noise(mu_para=0, sigma_para=5):
    return np.random.normal(mu_para, sigma_para)