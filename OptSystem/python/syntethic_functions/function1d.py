import numpy as np

from ..core.objective_function import ObjectiveFunction
from ..core.metric import BasicMetric

class Function1D(ObjectiveFunction):
    def __init__(self, params = {}):
        super().__init__(1, ["x"], BasicMetric(), func_params=params)

class Forrester(Function1D):
    """
    Description: 1D simple function and multimodal, with one global minimum, one local minimum and a zero-gradient inflection point.
    Domain: [0, 1]
    Global minima: (aprox) -6.0207 at x = 0.7572 \\
    Link: https://www.sfu.ca/~ssurjano/forretal08.html
    """
    
    def get_name(self) -> str:
        return "Forrester"
    
    def _evaluate(self, query: np.ndarray) -> float:
        x = query[0]
        
        fval = (6 * x - 2) ** 2
        fval = fval * np.sin(12 * x - 4)

        return fval

class Gramacy1D(Function1D):
    """
    Description: 1D simple function and multimodal, with one global minimun and multiple local minimum
    Domain: [0.5, 2.5]
    Global minima: (aprox) -0.869 at x = 0.5485 \\
    Link: https://www.sfu.ca/~ssurjano/grlee12.html
    """
    
    def get_name(self) -> str:
        return "Gramacy1D"
    
    def _evaluate(self, query: np.ndarray) -> float:
        x = query[0]

        fval = np.sin(10.0 * np.pi * x) / (2.0 * x) + np.power(x-1, 4)

        return fval

STR_TO_1D = {Forrester().get_name().lower(): Forrester, Gramacy1D().get_name().lower(): Gramacy1D}

