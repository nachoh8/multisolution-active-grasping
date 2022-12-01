import json
import numpy as np

from pygrasp.pygrasp import *

from ..core.objective_function import ObjectiveFunction
from .grasp_metrics import *

def construct_grasp_executor(gtype: type, fparams: str = "") -> "BaseGraspExecutor":
    """
    return GraspExecutor
    """
    if gtype == GraspPlanner:
        grasp_params = EnvParameters()
        if not loadEnvParametersFile(fparams, grasp_params):
            print("Error: parsing GraspPlanner params")
            exit(-1)
        return GraspPlanner(grasp_params)
    else:
        print("Error: grasp executor " + str(gtype) + " does not exists")
        exit(-1)


class GraspModel(ObjectiveFunction):
    def __init__(self, grasp_executor: BaseGraspExecutor, ndim: int, var_names: "list[str]", metric: BaseGraspResultMetric, fparams: str = "") -> None:
        self.executor = construct_grasp_executor(grasp_executor, fparams)
        if fparams != "":
            f = open(fparams, 'r')
            json_params = json.load(f)
        else:
            json_params = {}

        super().__init__(ndim, var_names, metric, func_params=json_params)

    def _evaluate(self, query: np.ndarray) -> GraspResult:
        return self.executor.executeQueryGrasp(query)

class GraspPlannerModel(GraspModel):
    def __init__(self, fparams: str, metric: BaseGraspResultMetric = EpsilonMetric) -> None:
        super().__init__(GraspPlanner, CARTESIAN_VEC_SIZE, ["x", "y", "z", "rx", "ry", "rz"], metric, fparams=fparams)
    
    def get_name(self) -> str:
        return "GraspPlanner"

def create_grasp_function(name: str, metric: BaseGraspResultMetric, fparams: str) -> GraspModel:
    _name = name.lower()
    if _name == "gp" or _name == "graspplanner":
        if issubclass(metric, BaseGraspResultMetric):
            return GraspPlannerModel(fparams, metric=metric)
        print("Error: Metric " + str(metric) + " is not compatible with GraspPlannerModel")
        exit(-1)
    
    return None
