from ..core.metric import *

from ..core.objective_function import ObjectiveFunction
from ..syntethic_functions.forrester import Forrester

def create_metric(metric_name: str) -> Metric:
    name = metric_name.lower()
    if name == "basic":
        return BasicMetric()
    else:
        print("Error: the metric " + metric_name + " does not exists")
        exit(-1)

def create_objective_function(function_name: str, metric_name: str, function_params: dict = {}) -> ObjectiveFunction:
    metric = create_metric(metric_name)

    name = function_name.lower()
    if name == "forrester":
        return Forrester()
    else:
        print("Error: the objective function " + function_name + " does not exists")
        exit(-1)
