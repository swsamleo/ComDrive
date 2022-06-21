from flow.core.kernel.safety.Safety_System_Params import metric_types
import math
import time


class Safety_System():
    def __init__(self, safety_metric_types=["TTC"]):
        self.__safety_metric_types = safety_metric_types
        self.__safety_data = {}
        for metric in safety_metric_types:
            self.__safety_data[metric] = {}

    def calculate_safety_metrics(self, veh_id, **kwargs):
        for metric in self.__safety_metric_types:
            result = metric_types[metric](**kwargs)
            self.__safety_data[metric][veh_id] = result

    def get_safety_data(self, metric, veh_id):
        return self.__safety_data[metric][veh_id]

    def set_safety_metric_types(self, safety_metric_types):
        self.__safety_metric_types = safety_metric_types

    def get_fairness_metric(self, lambda_para, beta, safety_metric_type="TTC", excluded_id=None):
        sum_safety_metrics = 0
        for key in self.__safety_data[safety_metric_type]:
            if key != excluded_id:
                sum_safety_metrics += self.__safety_data[safety_metric_type][key]
        temp = 0
        for key in self.__safety_data[safety_metric_type]:
            if key != excluded_id:
                temp += (self.__safety_data[safety_metric_type][key] / sum_safety_metrics) ** (1 - beta)
        f_b = temp ** (1 / beta)

        if not isinstance(f_b, float):
            return 0
        elif f_b > 0 and sum_safety_metrics > 0:
            fairness = lambda_para * math.log(f_b, 2.72) + math.log(sum_safety_metrics, 2.72)
            return fairness
        else:
            return 0
