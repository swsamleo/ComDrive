from flow.core.kernel.safety.Safety_System_Params import metric_types


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
