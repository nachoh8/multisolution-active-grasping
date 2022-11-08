import numpy as np

from ..core.metric import Metric
from ..synthetic_functions.functionNd import FunctionND
from .cec2013.cec2013 import CEC2013, FUNCTION_NAMES

class FunctionCEC2013(FunctionND):
    def __init__(self, nfunc: int):
        self.cec_func = CEC2013(nfunc)
        super().__init__(self.cec_func.get_dimension())

    def get_name(self) -> str:
        return self.cec_func.get_name()
    
    def get_lower_bounds(self) -> "list[float]":
        return None # TODO
    
    def get_upper_bounds(self) -> "list[float]":
        return None # TODO
    
    def get_global_optima(self) -> float:
        return self.cec_func.get_fitness_goptima()

    def get_num_global_optima(self) -> int:
        return self.cec_func.get_no_goptima()

    def get_exclusion_radius(self) -> float:
        return self.cec_func.get_rho()

    def _evaluate(self, query: np.ndarray) -> float:
        fval = self.cec_func.evaluate(query)
        return float(fval)

def create_cec2013_function(name: str, metric: Metric, fparams: str) -> FunctionCEC2013:
    if len(name) == 2 or len(name) == 3:
        if name[0] == "f" or name[0] == "F":
            idx_f = int(name[1:])
            if idx_f >= 1 and idx_f <= 20:
                return FunctionCEC2013(idx_f)
    
    try:
        idx = FUNCTION_NAMES.index(name)
        return FunctionCEC2013(idx + 1)
    except:
        pass
    
    return None
