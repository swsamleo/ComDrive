import numpy as np

from flow.core.kernel.perception.perception_params_func import \
    generate_Gaussian_error,generate_Absolute_error,\
    generate_no_error,fuse_by_arithmetic_average,\
    detect_distance_data_from_env,detect_velocity_data_from_env

Perception_Type = {"distance": detect_distance_data_from_env,
                   "velocity": detect_velocity_data_from_env}

Error = {"Gaussian": generate_Gaussian_error,
         "Absolute": generate_Absolute_error,
         None: generate_no_error}

Sensor = {"GPS": ("Gaussian", 2.45),
          "Radar": ("Gaussian", 1),
          "Camera": ("Absolute", 2),
          }

Fuse_Function = {None: fuse_by_arithmetic_average,
                "arithmetic_average": fuse_by_arithmetic_average}

