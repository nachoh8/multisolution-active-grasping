from ..core.optimizer_executor import OptimizerExecutor
from ..executors.bayesopt_executor import BayesOptExecutor
from ..executors.sigopt_executor import SigOptExecutor

from ..core.metric import *

from ..core.objective_function import ObjectiveFunction, BatchObjectiveFunction
from ..syntethic_functions.function1d import STR_TO_1D
from ..syntethic_functions.function2d import STR_TO_2D

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
    func = None
    for fs in [STR_TO_1D, STR_TO_2D]:
        if name in fs:
            return fs[name](params=function_params)
    
    if func == None:
        print("Error: the objective function " + function_name + " does not exists")
        exit(-1)

def create_optimizer(optimizer_data: dict, obj_func_data: dict, metric: str) -> OptimizerExecutor:

    name = optimizer_data["name"].lower()
    params = optimizer_data["params"]
    flog = optimizer_data["flog"]

    is_batch = "bopt_params" in params and "par_type" in params["bopt_params"] and params["bopt_params"]["par_type"] == "PAR_BBO"
    batch_size = params["bopt_params"]["n_parallel_samples"] if is_batch else 1

    obj_function = create_objective_function(obj_func_data["name"], metric, function_params=obj_func_data["params"], batch_size=batch_size)
    
    if name == "bayesopt":
        return BayesOptExecutor(params, obj_function, log_file=flog)
    elif name == "sigopt":
        return SigOptExecutor(params, obj_function, log_file=flog)
    else:
        print("Error: the optimizer " + name + " does not exists")
        exit(-1)
