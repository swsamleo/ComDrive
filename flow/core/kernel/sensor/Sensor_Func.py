import numpy as np
def generate_Gaussian_error(error_size):
    return np.random.normal(0, error_size)

def generate_Absolute_error(error_size):
    return  error_size

def generate_Uniform_error(error_size):
    return np.random.uniform(-error_size, error_size)

def generate_Laplace_error(error_size):
    if error_size > 0:
        return np.random.laplace(0, error_size)
    else:
        return 0


def generate_no_error(error_size=0):
    return 0

def fuse_by_arithmetic_average(data):
    return np.mean(data)

def detect_distance_data_from_env(env,vehicle_id):
    return env.k.vehicle.get_headway(vehicle_id)

def detect_velocity_data_from_env(env,vehicle_id):
    return env.k.vehicle.get_speed(vehicle_id)