import numpy as np

from ..core.objective_function import ObjectiveFunction
from ..core.metric import BasicMetric

class Forrester(ObjectiveFunction):
    def __init__(self):
        super().__init__(1, ["x"], BasicMetric())
    
    def get_name(self) -> str:
        return "Forrester"
    
    def _evaluate(self, query: np.ndarray) -> float:
        x = query[0]
        
        fval = (6 * x - 2) ** 2
        fval = fval * np.sin(12 * x - 4)

        return fval
