import numpy as np

from ..core.metric import Metric
from ..synthetic_functions.functionNd import FunctionND
from .cec2013.cec2013 import CEC2013, FUNCTION_NAMES

class FunctionCEC2013(FunctionND):
    __radius_ = [
        2.0,
        0.1,
        0.1,
        2.0,
        1.0,
        0.5,
        0.2,
        0.5,
        0.2,
        0.2,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
    ]

    __accs_ = [
        4.0,
        0.01,
        0.01,
        4.0,
        0.1,
        4.0,
        0.01,
        0.01,
        0.01,
        0.2,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
        0.01,
    ]

    __GO_ = [
        np.array([[0.0], [30.0]]),
        np.array([[0.1], [0.3], [0.5], [0.7], [0.9]]),
        np.array([[0.0797]]),
        np.array([[3.0, 2.0], [-2.805118,3.131312], [-3.779310,-3.283186], [3.584428,-1.848126]]),
        np.array([[0.0898, -0.7126],[-0.0898,0.7126]]),
        None,
        np.array([[7.71666667, 7.66666667],
                [4.11868687, 7.66666667],
                [7.71666667, 4.08333333],
                [4.11868687, 4.08333333],
                [7.71666667, 2.16666667],
                [4.11868687, 2.16666667],
                [7.71666667, 1.17171717],
                [4.11868687, 1.17171717],
                [7.71666667, 0.62878788],
                [4.11868687, 0.62878788],
                [7.71666667, 0.33838384],
                [4.11868687, 0.33838384],
                [2.2       , 4.11060606],
                [2.2       , 7.69545455],
                [1.17777778, 4.11060606],
                [1.17777778, 7.69545455],
                [2.2       , 2.23282828],
                [1.17777778, 2.23282828],
                [2.2       , 1.17171717],
                [1.17777778, 1.17171717],
                [2.2       , 0.62878788],
                [1.17777778, 0.62878788],
                [2.2       , 0.33838384],
                [1.17777778, 0.33838384],
                [0.33333333, 4.11060606],
                [0.33333333, 7.69545455],
                [0.62222222, 4.11060606],
                [0.62222222, 7.69545455],
                [0.33333333, 2.23282828],
                [0.62222222, 2.23282828],
                [0.33333333, 1.17171717],
                [0.62222222, 1.17171717],
                [0.33333333, 0.62878788],
                [0.62222222, 0.62878788],
                [0.33333333, 0.33838384],
                [0.62222222, 0.33838384]]),
        None,
        None,
        None,
        None
    ]

    def __init__(self, nfunc: int):
        self.nfunc = nfunc
        self.cec_func = CEC2013(nfunc)
        super().__init__(self.cec_func.get_dimension())

        ndim = self.cec_func.get_dimension()
        self.lb = []
        self.ub = []
        for d in range(ndim):
            _ld = self.cec_func.get_lbound(d)
            self.lb.append(_ld)
            _ud = self.cec_func.get_ubound(d)
            self.ub.append(_ud)
        
    def get_name(self) -> str:
        return self.cec_func.get_name()
    
    def get_lower_bounds(self) -> "list[float]":
        return self.lb

    def get_upper_bounds(self) -> "list[float]":
        return self.ub
    
    def get_global_optima(self) -> float:
        return self.cec_func.get_fitness_goptima()

    def get_num_global_optima(self) -> int:
        return self.cec_func.get_no_goptima()
    
    def get_global_optima_points(self) -> np.ndarray:
        return self.__GO_[self.nfunc - 1]

    def get_exclusion_radius(self) -> float:
        return self.__radius_[self.nfunc - 1] # self.cec_func.get_rho()

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
