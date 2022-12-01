import traceback

from ..core.optimizer_executor import OptimizerExecutor
try:
    from ..executors.bayesopt_executor import BayesOptExecutor
    BAYESOPT_ENABLED=True
except Exception as err:
    print("ERROR:", err)
    print(traceback.print_exc())
    print("WARNING: bayesopt module not imported")
    BAYESOPT_ENABLED=False

try:
    from ..executors.sigopt_executor import SigOptExecutor
    SIGOPT_ENABLED=True
except Exception as err:
    print("ERROR:", err)
    print(traceback.print_exc())
    print("WARNING: sigopt module not imported")
    SIGOPT_ENABLED=False

try:
    from ..executors.gpyopt_executor import GPyOptExecutor
    GPYOPT_ENABLED=True
except Exception as err:
    print("ERROR:", err)
    print(traceback.print_exc())
    print("WARNING: gpyopt module not imported")
    GPYOPT_ENABLED=False

try:
    from ..executors.robot_executor import ROBOTExecutor
    ROBOT_ENABLED=True
except Exception as err:
    print("ERROR:", err)
    print(traceback.print_exc())
    print("WARNING: ROBOT module not imported")
    ROBOT_ENABLED=False

from ..core.metric import *

from ..core.objective_function import ObjectiveFunction, BatchObjectiveFunction
from ..synthetic_functions.function1d import create_1d_function
from ..synthetic_functions.function2d import create_2d_function
from ..cec2013.functionCEC2013 import create_cec2013_function

try:
    from ..grasp.grasp_models import create_grasp_function
    from ..grasp.grasp_metrics import create_grasp_metric
except Exception as err:
    print("ERROR:", err)
    print(traceback.print_exc())
    print("WARNING: Grasp module not imported")

    def create_grasp_function(name: str, metric: Metric, fparams: str):
        return None
    
    def create_grasp_metric(name):
        return None


def create_metric(metric_name: str) -> Metric:
    name = metric_name.lower()
    if name == "basic":
        return BasicMetric
    
    metric = create_grasp_metric(name)
    if metric == None:
        print("Error: the metric " + metric_name + " does not exists")
        exit(-1)
    
    return metric

def create_objective_function(function_name: str, metric_name: str, fparams: str = "", batch_size: int = 1, in_parallel: bool = False) -> ObjectiveFunction:
    if batch_size > 1:
        if in_parallel:
            obj_funcs = [create_objective_function(function_name, metric_name, fparams=fparams) for _ in range(batch_size)]
        else:
            obj_funcs = [create_objective_function(function_name, metric_name, fparams=fparams)]

        return BatchObjectiveFunction(obj_funcs)

    metric = create_metric(metric_name)
    
    func = None
    for cf in [create_1d_function, create_2d_function, create_grasp_function, create_cec2013_function]:
        func = cf(function_name, metric, fparams)
        if func != None:
            return func
    
    if func == None:
        print("Error: the objective function " + function_name + " does not exists")
        exit(-1)

def create_optimizer(optimizer_data: dict, obj_func_data: dict, metric: str, in_parallel: bool = False, verbose: bool = False) -> OptimizerExecutor:

    name = optimizer_data["name"].lower()
    params = optimizer_data["params"]
    flog = optimizer_data["flog"]

    is_batch = "bopt_params" in params and "par_type" in params["bopt_params"] and (params["bopt_params"]["par_type"] == "PAR_BBO" or params["bopt_params"]["par_type"] == "PAR_MCMC")
    batch_size = params["bopt_params"]["n_parallel_samples"] if is_batch else 1

    obj_function = create_objective_function(obj_func_data["name"], metric, fparams=obj_func_data["fparams"], batch_size=batch_size, in_parallel=in_parallel)
    
    if name == "bayesopt" and BAYESOPT_ENABLED:
        return BayesOptExecutor(params, obj_function, log_file=flog, verbose=verbose)
    elif name == "sigopt" and SIGOPT_ENABLED:
        return SigOptExecutor(params, obj_function, log_file=flog)
    elif name == "gpyopt" and GPYOPT_ENABLED:
        return GPyOptExecutor(params, obj_function, log_file=flog, verbose=verbose)
    elif name == "robot" and ROBOT_ENABLED:
        return ROBOTExecutor(params, obj_function, log_file=flog, verbose=verbose)
    else:
        print("Error: the optimizer " + name + " does not exists")
        exit(-1)
