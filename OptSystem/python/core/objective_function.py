import numpy as np

from .metric import Metric

class ObjectiveFunction(object):
    def __init__(self, ndim: int, var_names: "list[str]", metric: Metric, func_params: dict = {}) -> None:
        self.ndim = ndim
        self.var_names = var_names
        self.metric = metric
        self.params: dict = func_params

        self.default_query: np.ndarray = np.zeros(ndim)
    
    def get_var_names(self) -> "list[str]":
        return self.var_names

    def _var_to_idx(self, var: str) -> int:
        return self.var_names.index(var)

    def _idx_to_var(self, idx: int) -> str:
        return self.var_names[idx]
    
    def _parse_query(self, query: dict) -> np.ndarray:
        n = len(query)
        if n == 0 or n > self.ndim:
            raise Exception("Query size must be 1 <= size <= " + str(self.ndim))
        
        final_query = self.default_query.copy()
        for k,v in list(query.items()):
            idx = self._var_to_idx(k)
            final_query[idx] = v
        
        return final_query

    def set_default_query(self, query: dict):
        if len(query) != self.ndim:
            raise Exception("Default query size must be " + str(self.ndim))
        
        self.default_query = self._parse_query(query)

    def get_params(self) -> dict:
        return self.params
    
    def get_name(self) -> str:
        raise Exception("This is an abstract class")
    
    def get_default_query(self) -> list:
        return self.default_query.tolist()
    
    def execute(self, query: dict) -> Metric:
        final_query = self._parse_query(query)

        if final_query.shape[0] != self.ndim:
            raise Exception("Query size must be " + str(self.ndim))

        res = self._evaluate(final_query)
        self.metric.set_result(res)

        return self.metric
        
    def _evaluate(self, query: np.ndarray) -> any:
        raise Exception("This is an abstract class")
