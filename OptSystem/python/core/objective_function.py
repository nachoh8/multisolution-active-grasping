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
    
    def get_global_optima(self) -> float:
        return None
    
    def get_num_global_optima(self) -> int:
        return None
    
    def get_exclusion_radius(self) -> float:
        return 0.01
    
    def get_metric(self) -> Metric:
        return self.metric(res=None)
    
    def get_default_query(self) -> list:
        return self.default_query.tolist()
    
    def execute(self, query: dict) -> Metric:
        final_query = self._parse_query(query)

        if final_query.shape[0] != self.ndim:
            raise Exception("Query size must be " + str(self.ndim))

        res = self._evaluate(final_query)

        return self.metric(res)
        
    def _evaluate(self, query: np.ndarray) -> any:
        raise Exception("This is an abstract class")

class BatchObjectiveFunction(ObjectiveFunction):
    def __init__(self, obj_functions: "list[ObjectiveFunction]") -> None:
        self.obj_functions = obj_functions
        self.ref_function = self.obj_functions[0]
        self.in_parallel = len(obj_functions) > 1

        self.ndim = self.ref_function.ndim
        self.var_names = self.ref_function.var_names
        self.metric = self.ref_function.metric
        self.params: dict = self.ref_function.params

        self.default_query: np.ndarray = np.zeros(self.ndim)

    def set_default_query(self, query: dict):
        for objf in self.obj_functions:
            objf.set_default_query(query)
        
        self.default_query = self.ref_function.default_query
    
    def get_name(self) -> str:
        return self.ref_function.get_name()
    
    def get_default_query(self) -> list:
        return self.default_query.tolist()
    
    def execute(self, query: dict) -> Metric:
        raise Exception("execute method is not compatible with this class")
    
    def executeBatch(self, queries: "list[dict]", ntrials: int) -> "tuple[list[Metric],list[int]]":
        if self.in_parallel:
            print("Error: parallel execution is not yet supported")
            exit(-1)
        
        res = []
        trials = []
        for query in queries:
            err = "err"
            i = 0
            while i < ntrials and err != "":
                m = self.ref_function.execute(query)
                err = m.get_failure()
                i +=1
            
            res.append(m)
            trials.append(i)

        return res, trials
        
    def _evaluate(self, query: np.ndarray) -> any:
        raise Exception("_evaluate method is not compatible with this class")
