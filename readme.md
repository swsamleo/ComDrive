# System Architecture

![architecture](pic_readme/architecture.svg)

# Class Section

## Simulation

![class](pic_readme/class.svg)

### Class Definition


| **Class Name** |                        **Definition**                        |
| :------------: | :----------------------------------------------------------: |
|  SensorSystem  |                     Collect sensor data                      |
|   DataCenter   |                         Process data                         |
|     Router     |         Re-route the vehicle in certain road network         |
|   Controller   | Control the vehicle, including steering, accelerating and braking |
|      Env       | Provide interface for interacting with various aspects of a traffic simulation |
|    NetWork     | Specify features of a network,  including the positions of nodes, properties of the edges, and junctions connecting these nodes, properties of vehicles and traffic lights, and other features as well |

### Function Explanation

- SensorSystem

  | **Function Name** | **Explanation** |
  | :------------:| :--------------------------:|
  |add_new_sensor(self, detect_type, sensor_name, noise_function=generate_no_noise)| Add new sensor to the SensorSystem |
  |detect_sensor_data_from_env(self, env, veh_id, detect_type)| Return sensor data from Env|
  |get_sensors(self)| Return all sensors |
  |get_fuse_function(self)| Return sensor fuse function |
  | get_detect_types(self) | Return types of all sensors |

- DataCenter 

  | Function Name | **Explanation** |
  | :------------:| :--------------------------:|
  | update_data(self, data_center_name, t, veh_id=None,**kwargs)| Update the data named data_center_name of the vehicle with index veh_id at step t in DataCenter |
  | get_data(self, data_center_name, **kwargs) | Get data names data_center_name from DataCenter |

- BaseRouter

  | Function Name | **Explanation** |
  | :------------:| :--------------------------:|
  | choose_route(self, env) | Return the routing method implemented by the controller |

- BaseController

  The detailed explanation is presented in [Controller](#Controller) section.

- Env

  | Function Name | **Explanation** |
  | :------------:| :--------------------------:|
  | restart_simulation(self, sim_params, render=None) | Restart an already initialized simulation instance |
  | setup_initial_state(self) | Store information on the initial state of vehicles in the network |
  | step(self, rl_actions) | Advance the environment by one step |
  | reset(self) | Reset the environment |
  | additional_command(self) | Additional commands that may be performed by the step method |
  | clip_actions(self, rl_actions=None) | Clip the actions passed from the RL agent |
  | apply_rl_actions(self, rl_actions=None) | Specify the actions to be performed by the rl agent(s) |
  | compute_reward(self, rl_actions, **kwargs) | Reward function for the RL agent(s) |
  | terminate(self) | Close the TraCI I/O connection |
  | render(self, reset=False, buffer_length=5) | Render a frame |
  | pyglet_render(self) | Render a frame using pyglet |
  | get_state(self) | Return the state of the simulation as perceived by the RL agent |
  | action_space(self) | Identify the dimensions and bounds of the action space |
  | observation_space(self) | Identify the dimensions and bounds of the observation space |

- NetWork

  | Function Name | **Explanation** |
  | :------------:| :--------------------------:|
  | specify_edge_starts(self) | Define edge starts for road sections in the network |
  | specify_internal_edge_starts(self) | Define the edge starts for internal edge nodes |
  | specify_nodes(self, net_params) | Specify the attributes of nodes in the network |
  | specify_edges(self, net_params) | Specify the attributes of edges connecting pairs on nodes |
  | specify_types(self, net_params) | Specify the attributes of various edge types (if any exist) |
  | specify_connections(self, net_params) | Specify the attributes of connections |
  | specify_routes(self, net_params) | Specify the routes vehicles can take starting from any edge |

## <div id="Controller">Controller</div>

We adopt four different typical controller to evaluate the impact of noise.

![controller](pic_readme/controller.svg)

We use IDMController and BandoFTLController to validate the effectiveness of our proposed margin in terms of traffic throughput and safety. 

![add margin](pic_readme/add_margin.svg)

## Experiment

Gaussian noise with $\mu=5$

![Gaussian_5](pic_readme/gaussian_5.gif)

Simulation with noise and without noise result:

![gaussian_5](pic_readme/gaussian_5.png)

Untrained model:

![Gaussian_5_untrained](pic_readme/gaussian_5_untrained.gif)

Trained model:

![Gaussian_5_train](pic_readme/gaussian_5_train.gif)

# Comdrive Project Status
## Overview
We're excited to share the Comdrive project with the community. Comdrive is an ***`in-development`*** simulation tool designed with specific goals in mind. It's important to note that the architecture of Comdrive is currently under development, and as such, ***`it may not yet offer full functionality`***. Our team is diligently working to enhance its capabilities and stability.

## Current State
- **Development Phase**: Please be aware that ***`Comdrive is in a development phase`***. We're in the process of refining its architecture and expanding its feature set.

- **Functionality**: Some features within Comdrive are in the beta stage and may exhibit instability or limited functionality. These features are being improved with each update.

- **Stability**: Given its current development status, ***`Comdrive may exhibit unstable behavior under certain conditions`***. We are committed to improving its stability and performance.

## Usage Considerations
- **Feedback**: Users are encouraged to provide feedback on any issues encountered or suggestions for improvement. Your input is invaluable as we strive to develop Comdrive into a more robust tool.

- **Updates**: Regular updates are planned for Comdrive, aimed at addressing current limitations and introducing new features. We recommend staying updated with the latest version for the best experience.

## Future Plans
- **Open Source**: While ***`Comdrive is not yet open-sourced due to its preliminary state`***, we intend to make it publicly available once we achieve a more stable and comprehensive version. This approach ensures that the community receives a reliable and effective tool.

- **Collaboration**: We are open to collaboration and contributions from the community. Detailed guidelines will be provided as Comdrive progresses towards a stable release.

## Contact and Support
- For any inquiries or support needs, please contact us. We appreciate your interest in Comdrive and look forward to growing with your support.
