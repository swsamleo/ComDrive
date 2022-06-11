import numpy as np

from flow.core.kernel.sensor.Sensor_Func import \
    generate_Gaussian_error,generate_Absolute_error,\
    generate_no_error,fuse_by_arithmetic_average,\
    detect_distance_data_from_env,detect_velocity_data_from_env

Detect_Type = {"distance": detect_distance_data_from_env,
               "velocity": detect_velocity_data_from_env}

Error = {"Gaussian": generate_Gaussian_error,
         "Absolute": generate_Absolute_error,
         None: generate_no_error}

Sensor = {"distance": {
            "GPS": ("Gaussian", 2.45),
            "Radar": ("Gaussian", 1),
            "Camera": ("Absolute", 2),
            None: ("Absolute", 0)},
          "velocity": {
            "GPS": ("Gaussian", 2.45),
            "Radar": ("Gaussian", 1),
            "Camera": ("Absolute", 2),
            None: ("Absolute", 0)},
          }

Fuse_Function = {None: fuse_by_arithmetic_average,
                "arithmetic_average": fuse_by_arithmetic_average}

