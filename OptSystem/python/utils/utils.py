from ..core.metric import *

from ..core.objective_function import ObjectiveFunction, BatchObjectiveFunction
from ..syntethic_functions.forrester import Forrester

def create_metric(metric_name: str) -> Metric:
    name = metric_name.lower()
    if name == "basic":
        return BasicMetric()
    else:
        print("Error: the metric " + metric_name + " does not exists")
        exit(-1)

def create_objective_function(function_name: str, metric_name: str, function_params: dict = {}, batch_size: int = 1) -> ObjectiveFunction:
    if batch_size > 1:
        obj_funcs = [create_objective_function(function_name, metric_name, function_params=function_params) for _ in range(batch_size)]

        return BatchObjectiveFunction(obj_funcs)

    metric = create_metric(metric_name)
    
    name = function_name.lower()
    if name == "forrester":
        return Forrester()
    else:
        print("Error: the objective function " + function_name + " does not exists")
        exit(-1)
