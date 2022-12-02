###############################################################################
# Version: 1.1
# Last modified on: 3 April, 2016
# Developers: Michael G. Epitropakis
#      email: m_(DOT)_epitropakis_(AT)_lancaster_(DOT)_ac_(DOT)_uk
###############################################################################
# from scipy.spatial.distance import pdist, squareform

# from scipy.spatial import distance
import numpy as np
import math
from .functions import *
from .cfunction import *
from .CF1 import *
from .CF2 import *
from .CF3 import *
from .CF4 import *

FUNCTION_NAMES = [
    "Five-Uneven-Peak Trap",
    "Equal Maxima",
    "Uneven Maxima",
    "Himmelblau",
    "Six-Hump Camel Back",
    "Shubert-2D",
    "Vincent-2D",
    "Shubert-3D",
    "Vincent-3D",
    "Modified Rastrigin",
    "CF1",
    "CF2",
    "CF3-2D",
    "CF3-3D",
    "CF4-3D",
    "CF3-5D",
    "CF4-5D",
    "CF3-10D",
    "CF4-10D",
    "CF4-20D"
]

class CEC2013(object):
    __nfunc_ = -1
    __functions_ = {
        1: five_uneven_peak_trap,
        2: equal_maxima,
        3: uneven_decreasing_maxima,
        4: himmelblau,
        5: six_hump_camel_back,
        6: shubert,
        7: vincent,
        8: shubert,
        9: vincent,
        10: modified_rastrigin_all,
        11: CF1,
        12: CF2,
        13: CF3,
        14: CF3,
        15: CF4,
        16: CF3,
        17: CF4,
        18: CF3,
        19: CF4,
        20: CF4,
    }
    __f_ = None
    __fopt_ = [
        200.0,
        1.0,
        1.0,
        200.0,
        1.031628453489877,
        186.7309088310239,
        1.0,
        2709.093505572820,
        1.0,
        -2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    __rho_ = [
        0.01,
        0.01,
        0.01,
        0.01,
        0.5,
        0.5,
        0.2,
        0.5,
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
        0.01,
    ]
    __nopt_ = [2, 5, 1, 4, 2, 18, 36, 81, 216, 12, 6, 8, 6, 6, 8, 6, 8, 6, 8, 8]
    __maxfes_ = [
        50000,
        50000,
        50000,
        50000,
        50000,
        200000,
        200000,
        400000,
        400000,
        200000,
        200000,
        200000,
        200000,
        400000,
        400000,
        400000,
        400000,
        400000,
        400000,
        400000,
    ]
    __dimensions_ = [1, 1, 1, 2, 2, 2, 2, 3, 3, 2, 2, 2, 2, 3, 3, 5, 5, 10, 10, 20]
    __names_ = FUNCTION_NAMES

    def __init__(self, nofunc):
        assert nofunc > 0 and nofunc <= 20
        self.__nfunc_ = nofunc
        if self.__nfunc_ > 0 and self.__nfunc_ < 11:
            self.__f_ = self.__functions_[self.__nfunc_]
        else:
            self.__f_ = self.__functions_[self.__nfunc_](self.get_dimension())

    def evaluate(self, x):
        x_ = np.asarray(x)
        assert len(x_) == self.get_dimension()
        if self.__nfunc_ > 0 and self.__nfunc_ < 11:
            return self.__f_(x_)
        else:
            return self.__f_.evaluate(x_)

    def get_lbound(self, n):
        assert n >= 0 and n < self.__dimensions_[self.__nfunc_ - 1]
        result = 0
        if self.__nfunc_ == 1 or self.__nfunc_ == 2 or self.__nfunc_ == 3:
            result = 0
        elif self.__nfunc_ == 4:
            result = -6
        elif self.__nfunc_ == 5:
            tmp = [-1.9, -1.1]
            result = tmp[n]
        elif self.__nfunc_ == 6 or self.__nfunc_ == 8:
            result = -10
        elif self.__nfunc_ == 7 or self.__nfunc_ == 9:
            result = 0.25
        elif self.__nfunc_ == 10:
            result = 0
        elif self.__nfunc_ > 10:
            result = self.__f_.get_lbound(n)
        return result

    def get_ubound(self, n):
        assert n >= 0 and n < self.__dimensions_[self.__nfunc_ - 1]
        result = 0
        if self.__nfunc_ == 1:
            result = 30
        elif self.__nfunc_ == 2 or self.__nfunc_ == 3:
            result = 1
        elif self.__nfunc_ == 4:
            result = 6
        elif self.__nfunc_ == 5:
            tmp = [1.9, 1.1]
            result = tmp[n]
        elif self.__nfunc_ == 6 or self.__nfunc_ == 8:
            result = 10
        elif self.__nfunc_ == 7 or self.__nfunc_ == 9:
            result = 10
        elif self.__nfunc_ == 10:
            result = 1
        elif self.__nfunc_ > 10:
            result = self.__f_.get_ubound(n)
        return result

    def get_fitness_goptima(self):
        return self.__fopt_[self.__nfunc_ - 1]

    def get_dimension(self):
        return self.__dimensions_[self.__nfunc_ - 1]

    def get_no_goptima(self):
        return self.__nopt_[self.__nfunc_ - 1]

    def get_rho(self):
        return self.__rho_[self.__nfunc_ - 1]

    def get_maxfes(self):
        return self.__maxfes_[self.__nfunc_ - 1]

    def get_name(self):
        return self.__names_[self.__nfunc_-1]

    def get_info(self):
        return {
            "name": self.__names_[self.__nfunc_-1],
            "fbest": self.get_fitness_goptima(),
            "dimension": self.get_dimension(),
            "nogoptima": self.get_no_goptima(),
            "maxfes": self.get_maxfes(),
            "rho": self.get_rho(),
        }