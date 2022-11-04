import json
import numpy as np

from ..core.metric import Metric

from .functionNd import FunctionND

class Function2D(FunctionND):
    def __init__(self, params: dict = {}):
        super().__init__(2, params=params)

class Gramacy2D(Function2D):
    """
    Description: 2D simple function and with one global minimun
    Domain: x_i in [-2, 6], for i = 1, 2
    Global minima: (aprox) -0.4288 at x = (-0.70809565, -0.00076767) \\
    Link: https://www.sfu.ca/~ssurjano/grlee08.html
    """
    
    def get_name(self) -> str:
        return "Gramacy2D"
    
    def get_global_optima(self) -> float:
        return -0.4288
    
    def get_num_global_optima(self) -> int:
        return 1
    
    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]

        fval = x1 * np.exp(-x1**2 - x2**2)

        return fval

class Branin(Function2D):
    """
    Description: 2D function with three global minima
    Domain: x1 in [-5, 10], x2 in [0, 15]
    Global minima: 0.397887 at x = [(pi,12.275), (pi,2.275), (9.42478,2.475)] \\
    Link: https://www.sfu.ca/~ssurjano/branin.html
    """
    def __init__(self, params: dict = {}):
        super().__init__(params=params)
        if "a" in self.params:
            self.a = self.params["a"]
        else:
            self.a = 1
        
        if "b" in self.params:
            self.b = self.params["b"]
        else:
            self.b = 5.1 / (4 * np.pi**2)
        
        if "c" in self.params:
            self.c = self.params["c"]
        else:
            self.c = 5/np.pi
        
        if "r" in self.params:
            self.r = self.params["r"]
        else:
            self.r = 6
        
        if "s" in self.params:
            self.s = self.params["s"]
        else:
            self.s = 10
        
        if "t" in self.params:
            self.t = self.params["t"]
        else:
            self.t = 1/(8*np.pi)

    def get_name(self) -> str:
        return "Branin"
    
    def get_global_optima(self) -> float:
        return 0.397887

    def get_num_global_optima(self) -> int:
        return 3

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]

        fval = self.a * (x2 - self.b * x1**2 + self.c * x1 - self.r)**2 + self.s * (1 - self.t) * np.cos(x1) + self.s

        return fval

class GoldsteinPrice(Function2D):
    """
    Description: 2D function with several local minima
    Domain: x_i in [-2, 2], for i = 1, 2
    Global minima: 3 at x = (0,-1) \\
    Link: https://www.sfu.ca/~ssurjano/goldpr.html
    """

    def get_name(self) -> str:
        return "Goldstein"
    
    def get_global_optima(self) -> float:
        return 3
    
    def get_num_global_optima(self) -> int:
        return 1

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]

        fact1a = (x1 + x2 + 1)**2
        fact1b = 19 - 14*x1 + 3*x1**2 - 14*x2 + 6*x1*x2 + 3*x2**2
        fact1 = 1 + fact1a*fact1b

        fact2a = (2*x1 - 3*x2)**2
        fact2b = 18 - 32*x1 + 12*x1**2 + 48*x2 - 36*x1*x2 + 27*x2**2
        fact2 = 30 + fact2a*fact2b
        
        fval = fact1*fact2

        return fval

class Rosenbrock(Function2D):
    """
    Description: 2D function. The function is unimodal, and the global minimum lies in a narrow, parabolic valley.
    Domain: x_i in [-5, 10] or in [-2.048, 2.048], for i = 1, 2
    Global minima: 0 at x = (1,1) \\
    Link: https://www.sfu.ca/~ssurjano/rosen.html
    """

    def get_name(self) -> str:
        return "Rosenbrock"
    
    def get_global_optima(self) -> float:
        return 0.0
    
    def get_num_global_optima(self) -> int:
        return 1

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]
        
        fval = 100*(x2 - x1**2)**2 + (x1 - 1)**2

        return fval

class EggHolder(Function2D):
    """
    Description: 2D function. The function is a difficult function to optimize, because of the large number of local minima.
    Domain: x_i in [-512,512], for i = 1, 2
    Global minima: -959.6407 at x = (512,404.2319) \\
    Link: https://www.sfu.ca/~ssurjano/egg.html
    """

    def get_name(self) -> str:
        return "EggHolder"
    
    def get_global_optima(self) -> float:
        return -959.6407
    
    def get_num_global_optima(self) -> int:
        return 1

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]
        
        fval = -(x2+47) * np.sin(np.sqrt(abs(x2 + x1/2 + 47))) - x1 * np.sin(np.sqrt(abs(x1-(x2+47))))

        return fval

class McCormick(Function2D):
    """
    Description: 2D function with one global minima.
    Domain: x1 in [-1.5, 4], x2 in [-3, 4]
    Global minima: -1.9133 at x = (-0.54719,-1.54719) \\
    Link: https://www.sfu.ca/~ssurjano/mccorm.html
    """

    def get_name(self) -> str:
        return "McCormick"
    
    def get_global_optima(self) -> float:
        return -1.9133
    
    def get_num_global_optima(self) -> int:
        return 1

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]
        
        term1 = np.sin(x1 + x2)
        term2 = (x1 - x2)**2
        term3 = -1.5*x1
        term4 = 2.5*x2
        fval = term1 + term2 + term3 + term4 + 1

        return fval

class SixHumpCamel(Function2D):
    """
    Description: 2D function with six local minima, both of which are global.
    Domain: x1 in [-3, 3], x2 in [-2, 2]
    Global minima: -1.0316 at x = [(0.0898,-0.7126),(-0.0898,0.7126)] \\
    Link: https://www.sfu.ca/~ssurjano/camel6.html
    """

    def get_name(self) -> str:
        return "SixHumpCamel"
    
    def get_global_optima(self) -> float:
        return -1.0316
    
    def get_num_global_optima(self) -> int:
        return 2

    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]
        
        term1 = (4-2.1*x1**2+(x1**4)/3) * x1**2
        term2 = x1*x2
        term3 = (-4+4*x2**2) * x2**2
        fval = term1 + term2 + term3

        return fval

class Beale(Function2D):
    """
    Description: 2D function, multimodal, with sharp peaks at the corners of the input domain.
    Domain: x_i in [-4.5, 4.5], for i = 1,2
    Global minima: 0 at x = (3, 0.5) \\
    Link: https://www.sfu.ca/~ssurjano/beale.html
    """

    def get_name(self) -> str:
        return "Beale"
    
    def get_global_optima(self) -> float:
        return 0.0

    def get_num_global_optima(self) -> int:
        return 1
    
    def _evaluate(self, query: np.ndarray) -> float:
        x1 = query[0]
        x2 = query[1]
        
        fval = (1.5 - x1 + x1*x2)**2 + (2.25 - x1 + x1 * x2**2)**2 + (2.625 - x1 + x1 * x2**3)**2

        return fval


STR_TO_2D = {
    Gramacy2D().get_name().lower(): Gramacy2D,
    Branin().get_name().lower(): Branin,
    GoldsteinPrice().get_name().lower(): GoldsteinPrice,
    Rosenbrock().get_name().lower(): Rosenbrock,
    EggHolder().get_name().lower(): EggHolder,
    McCormick().get_name().lower(): McCormick,
    SixHumpCamel().get_name().lower(): SixHumpCamel,
    Beale().get_name().lower(): Beale
}

def create_2d_function(name: str, metric: Metric, fparams: str) -> Function2D:
    if name in STR_TO_2D:
        if fparams != "":
            f = open(fparams, 'r')
            params = json.load(f)
        else:
            params = {}
        return STR_TO_2D[name](params=params)
    return None
