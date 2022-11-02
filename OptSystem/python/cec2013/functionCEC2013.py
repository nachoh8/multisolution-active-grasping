import numpy as np

from ..core.metric import Metric
from ..synthetic_functions.functionNd import FunctionND
from .cec2013.cec2013 import CEC2013

class FunctionCEC2013(FunctionND):
    def __init__(self, nfunc: int):
        self.cec_func = CEC2013(nfunc)
        super().__init__(self.cec_func.get_dimension())

    def get_name(self) -> str:
        return self.cec_func.get_name()
    
    def _evaluate(self, query: np.ndarray) -> any:
        return self.cec_func.evaluate(query)

def create_cec2013_function(name: str, metric: Metric, fparams: str) -> FunctionCEC2013:
    if len(name) == 2:
        if name[0] == "f" or name[0] == "F":
            idx_f = int(name[1])
            if idx_f >= 1 and idx_f <= 20:
                return FunctionCEC2013(idx_f)
    
    return None
