from ..core.objective_function import ObjectiveFunction
from ..core.metric import BasicMetric

class FunctionND(ObjectiveFunction):
    def __init__(self, ndim: int, params = {}):
        if ndim == 1:
            var_names = ["x"]
        else:
            var_names = ["x" + str(i) for i in range(1, ndim + 1)]
        super().__init__(ndim, var_names, BasicMetric, func_params=params)

