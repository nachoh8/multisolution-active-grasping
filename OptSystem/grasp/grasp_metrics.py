from ..core.metric import Metric

from pygrasp.pygrasp import GraspResult

class BaseGraspResultMetric(Metric):
    def __init__(self, res: GraspResult) -> None:
        self.res: GraspResult = None # simply to infer type
        super().__init__(res)
    
    def get_failure(self) -> str:
        return self.res.error

class EpsilonMetric(BaseGraspResultMetric):

    def get_name(self) -> str:
        return "Epsilon"
    
    def get_metric_names(self) -> "list[str]":
        return ["epsilon"]

    def get_metrics(self) -> "list[tuple[str, any]]":
        return [("epsilon", self.res.measure)]

    def get_metadata(self) -> "list[tuple[str, any]]":
        return [("volume", self.res.volume), ("force_closure", self.res.force_closure)]

class EpsilonFCMetric(EpsilonMetric):
    def get_name(self) -> str:
        return "EpsilonFC"

    def get_metrics(self) -> "list[tuple[str, any]]":
        if self.res.force_closure:
            return [("epsilon", self.res.measure)]
        else:
            return [("epsilon", 0.0)]

STR_TO_GRASP_METRIC = {
    EpsilonMetric(res=None).get_name().lower(): EpsilonMetric,
    EpsilonFCMetric(res=None).get_name().lower(): EpsilonFCMetric
}

def create_grasp_metric(name: str) -> BaseGraspResultMetric:
    if name in STR_TO_GRASP_METRIC:
        return STR_TO_GRASP_METRIC[name]
    return None
