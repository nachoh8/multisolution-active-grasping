import json
import numpy as np

from pygrasp.pygrasp import *

from ..core.objective_function import ObjectiveFunction
from .grasp_metrics import *

def construct_grasp_executor(gtype: type, fparams: str = "") -> "GraspExecutor":
    """
    return GraspExecutor
    """
    if gtype == TestGramacyExecutor:
        return TestGramacyExecutor()
    elif gtype == GraspPlanner:
        grasp_params = GraspPlannerParams()
        if not load_GraspPlannerParams_json(fparams, grasp_params):
            print("Error: parsing GraspPlanner params")
            exit(-1)
        return GraspPlanner(grasp_params)
    else:
        print("Error: grasp executor " + str(gtype) + " does not exists")
        exit(-1)
    """elif gtype == GraspPlannerIK:
        grasp_params = GraspPlannerIKParams()
        if not load_GraspPlannerIKParams(fgrasp, grasp_params):
            print("Error: parsing grasp planner ik params")
            return None
        return GraspPlannerIK(grasp_params), grasp_params"""


class GraspModel(ObjectiveFunction):
    def __init__(self, grasp_executor: GraspExecutor, ndim: int, var_names: "list[str]", metric: BaseGraspResultMetric, fparams: str = "") -> None:
        self.executor = construct_grasp_executor(grasp_executor, fparams)
        if fparams != "":
            f = open(fparams, 'r')
            json_params = json.load(f)
        super().__init__(ndim, var_names, metric, func_params=json_params)

    def _evaluate(self, query: np.ndarray) -> GraspResult:
        return self.executor.executeQueryGrasp(query)

class TestGramacyGraspModel(GraspModel):
    def __init__(self) -> None:
        super().__init__(TestGramacyExecutor, 2, ["x1", "x2"], EpsilonMetric)
    
    def get_name(self) -> str:
        return "GramacyGP"

class GraspPlannerModel(GraspModel):
    def __init__(self, fparams: str, metric: BaseGraspResultMetric = EpsilonMetric) -> None:
        super().__init__(GraspPlanner, CARTESIAN_VARS_SIZE, ["x", "y", "z", "rx", "ry", "rz"], metric, fparams=fparams)
    
    def get_name(self) -> str:
        return "GraspPlanner"

def create_grasp_function(name: str, metric: BaseGraspResultMetric, fparams: str) -> GraspModel:
    if name == "gramacygp":
        return TestGramacyGraspModel()
    elif name == "gp":
        if issubclass(metric, BaseGraspResultMetric):
            return GraspPlannerModel(fparams, metric=metric)
        print("Error: Metric " + str(metric) + " is not compatible with GraspPlannerModel")
        exit(-1)
    
    return None
