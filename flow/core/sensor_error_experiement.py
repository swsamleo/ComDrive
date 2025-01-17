"""Contains an experiment class for running simulations."""
from flow.utils.registry import make_create_env
from datetime import datetime
import logging
import time
import numpy as np
import sqlite3
import math
class Experiment:
    """
    Class for systematically running simulations in any supported simulator.

    This class acts as a runner for a network and environment. In order to use
    it to run an network and environment in the absence of a method specifying
    the actions of RL agents in the network, type the following:

        >>> from flow.envs import Env
        >>> flow_params = dict(...)  # see the examples in exp_config
        >>> exp = Experiment(flow_params)  # for some experiment configuration
        >>> exp.run(num_runs=1)

    If you wish to specify the actions of RL agents in the network, this may be
    done as follows:

        >>> rl_actions = lambda state: 0  # replace with something appropriate
        >>> exp.run(num_runs=1, rl_actions=rl_actions)

    Finally, if you would like to like to plot and visualize your results, this
    class can generate csv files from emission files produced by sumo. These
    files will contain the speeds, positions, edges, etc... of every vehicle
    in the network at every time step.

    In order to ensure that the simulator constructs an emission file, set the
    ``emission_path`` attribute in ``SimParams`` to some path.

        >>> from flow.core.params import SimParams
        >>> flow_params['sim'] = SimParams(emission_path="./data")

    Once you have included this in your environment, run your Experiment object
    as follows:

        >>> exp.run(num_runs=1, convert_to_csv=True)

    After the experiment is complete, look at the "./data" directory. There
    will be two files, one with the suffix .xml and another with the suffix
    .csv. The latter should be easily interpretable from any csv reader (e.g.
    Excel), and can be parsed using tools such as numpy and pandas.

    Attributes
    ----------
    custom_callables : dict < str, lambda >
        strings and lambda functions corresponding to some information we want
        to extract from the environment. The lambda will be called at each step
        to extract information from the env and it will be stored in a dict
        keyed by the str.
    env : flow.envs.Env
        the environment object the simulator will run
    """

    def __init__(self, flow_params, custom_callables=None):
        """Instantiate the Experiment class.

        Parameters
        ----------
        flow_params : dict
            flow-specific parameters
        custom_callables : dict < str, lambda >
            strings and lambda functions corresponding to some information we
            want to extract from the environment. The lambda will be called at
            each step to extract information from the env and it will be stored
            in a dict keyed by the str.
        """
        self.custom_callables = custom_callables or {}

        # Get the env name and a creator for the environment.
        create_env, _ = make_create_env(flow_params)

        # Create the environment.
        self.env = create_env()

        logging.info(" Starting experiment {} at {}".format(
            self.env.network.name, str(datetime.utcnow())))

        logging.info("Initializing environment.")

    def run(self, num_runs, rl_actions=None, convert_to_csv=False):
        """Run the given network for a set number of runs.

        Parameters
        ----------
        num_runs : int
            number of runs the experiment should perform
        rl_actions : method, optional
            maps states to actions to be performed by the RL agents (if
            there are any)
        convert_to_csv : bool
            Specifies whether to convert the emission file created by sumo
            into a csv file

        Returns
        -------
        info_dict : dict < str, Any >
            contains returns, average speed per step
        """
        num_steps = self.env.env_params.horizon

        # raise an error if convert_to_csv is set to True but no emission
        # file will be generated, to avoid getting an error at the end of the
        # simulation
        if convert_to_csv and self.env.sim_params.emission_path is None:
            raise ValueError(
                'The experiment was run with convert_to_csv set '
                'to True, but no emission file will be generated. If you wish '
                'to generate an emission file, you should set the parameter '
                'emission_path in the simulation parameters (SumoParams or '
                'AimsunParams) to the path of the folder where emissions '
                'output should be generated. If you do not wish to generate '
                'emissions, set the convert_to_csv parameter to False.')

        # used to store
        info_dict = {
            "returns": [],
            "velocities": [],
            "outflows": [],
        }
        info_dict.update({
            key: [] for key in self.custom_callables.keys()
        })

        if rl_actions is None:
            def rl_actions(*_):
                return None

        # time profiling information
        t = time.time()
        times = []

        trained_vel = []
        untrained_vel = []

        safety_metric = []
        throughput_data = []
        velocity_data = {}
        headway_data = {}
        fairness_data = []
        TTC_data = []
        for i in range(num_runs):
            ret = 0
            vel = []
            custom_vals = {key: [] for key in self.custom_callables.keys()}
            state = self.env.reset()
            for j in range(num_steps):
                t0 = time.time()
                state, reward, done, _ = self.env.step(rl_actions(state))
                t1 = time.time()
                times.append(1 / (t1 - t0))

                # Compute the velocity speeds and cumulative returns.
                veh_ids = self.env.k.vehicle.get_ids()
                vel.append(np.mean(self.env.k.vehicle.get_speed(veh_ids)))
                ret += reward

                # Compute the results for the custom callables.
                for (key, lambda_func) in self.custom_callables.items():
                    custom_vals[key].append(lambda_func(self.env))

                if done:
                    break

                safety_metric.append(self.env.safety_system.get_fairness_metric(lambda_para=1, beta=0.5))
                throughput_data.append(self.env.perception_system.get_traffic_throughput(veh_ids=veh_ids))
                temp_sum_TTC = 0
                for veh_id in veh_ids:
                    if veh_id in velocity_data:
                        velocity_data[veh_id].append(self.env.perception_system.get_data_without_noise("velocity",veh_id))
                    else:
                        velocity_data[veh_id] = [self.env.perception_system.get_data_without_noise("velocity", veh_id)]
                    if veh_id in headway_data:
                        headway_data[veh_id].append(self.env.perception_system.get_data_without_noise("distance",veh_id))
                    else:
                        headway_data[veh_id] =[self.env.perception_system.get_data_without_noise("distance", veh_id)]
                    temp_sum_TTC += self.env.safety_system.get_safety_data("TTC",veh_id)
                TTC_data.append(temp_sum_TTC)
                fairness_data.append(safety_metric[-1]-math.log(temp_sum_TTC,2.72))

                # temp_trained_vel = []
                # temp_untrained_vel = []
                # switch = 1
                # for veh_id in veh_ids:
                #     if switch:
                #         temp = self.env.perception_system.get_data_without_noise("velocity",veh_id)/\
                #                 self.env.perception_system.get_data_without_noise("distance",veh_id)
                #         temp_trained_vel.append(temp)
                #         switch = 0
                #     else:
                #         temp = self.env.perception_system.get_data_without_noise("velocity",veh_id)/\
                #                 self.env.perception_system.get_data_without_noise("distance",veh_id)
                #         temp_untrained_vel.append(temp)
                #         switch = 1
                #
                # trained_vel.append(np.sum(temp_trained_vel))
                # untrained_vel.append(np.sum(temp_untrained_vel))


                # print(self.env.safety_system.get_fairness_metric(lambda_para=1, beta=0.5))
            import dill as pickle
            # f = open('train_vel_data.pkl', 'wb')
            # pickle.dump(trained_vel, f)
            # f1 = open('untrain_vel_data.pkl', 'wb')
            # pickle.dump(untrained_vel, f1)

            # f2 = open('safety_metric.pkl', 'wb')
            # pickle.dump(safety_metric, f2)
            f3 = open('metric_data/throughput_data.pkl',"wb")
            pickle.dump(throughput_data, f3)
            from flow.controllers.hit_history import hit_histroies
            # f4 = open("hit_histories.pkl","wb")
            # pickle.dump(hit_histroies,f4)

            f5 = open("metric_data/velocities_data.pkl","wb")
            pickle.dump(velocity_data, f5)

            f6 = open("metric_data/headway_data.pkl","wb")
            pickle.dump(headway_data,f6)

            # f7 = open("fairness_data.pkl","wb")
            # pickle.dump(fairness_data,f7)

            f8 = open('metric_data/TTC_data.pkl',"wb")
            pickle.dump(TTC_data,f8)

            f9 = open("metric_data/avg_vel_data.pkl","wb")
            pickle.dump(vel,f9)
            # Store the information from the run in info_dict.
            outflow = self.env.k.vehicle.get_outflow_rate(int(500))
            info_dict["returns"].append(ret)
            info_dict["velocities"].append(np.mean(vel))
            info_dict["outflows"].append(outflow)
            for key in custom_vals.keys():
                info_dict[key].append(np.mean(custom_vals[key]))
            error_size = 10

            print("Round {0}, return: {1}".format(i, ret))




            # Save emission data at the end of every rollout. This is skipped
            # by the internal method if no emission path was specified.
            if self.env.simulator == "traci":
                self.env.k.simulation.save_emission(run_id=i)

        # Print the averages/std for all variables in the info_dict.
        for key in info_dict.keys():
            print("Average, std {}: {}, {}".format(
                key, np.mean(info_dict[key]), np.std(info_dict[key])))
        print("Total time:", time.time() - t)
        print("steps/second:", np.mean(times))
        for ret in info_dict["returns"]:
            print(ret)
        self.env.terminate()

        return info_dict
