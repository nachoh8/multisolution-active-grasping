import json
from datetime import datetime

from .metric import Metric

BASIC_PARAMS_KEY = "basic_params"
OPTIMIZER_KEY = "optimizer"
OBJ_FUNCTION_KEY = "objective_function"
BEST_RESULTS_KEY = "best_results"
OPTIMUM_KEY = "optimum"
ITERATIONS_KEY = "iterations"

class DataLog(object):
    def __init__(self, log_file: str) -> None:
        self.log_file: str = log_file

        self.data = dict()
        self.data["date"] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        self.exec_time = 0.0 # in seconds

        self.basic_params: dict = dict()
        self.obj_function: dict = dict()
        self.optimizer: dict = dict()
        self.best_results: list[dict] = []
        self.optimum: dict = dict()
        self.iterations: list[list[dict]] = []
    
    def _log(self, key: any, data: any):
        self.data[key] = data
    
    def log_basic_params(self, params: dict):
        self._log(BASIC_PARAMS_KEY, params)
    
    def log_optimizer(self, name: str, params: dict):
        self.optimizer = {"name": name, "params": params}
        self._log(OPTIMIZER_KEY, self.optimizer)
    
    def log_objective_function(self, name: str, params: dict):
        self.grasp_executor = {"name": name, "params": params}
        self._log(OBJ_FUNCTION_KEY, self.grasp_executor)
    
    def log_execution_time(self, exec_time_seconds: float):
        self.exec_time = exec_time_seconds
        self._log("execution_time", self.exec_time)

    def log_iteration(self, queries: "list[dict]", metrics: "list[Metric]", ntrials: "list[int]"):
        log_data = []
        for query, res, trials in zip(queries, metrics, ntrials):
            log_query = {}

            log_query["query"] = query

            err = res.get_failure()
            if err != "":
                log_query["error"] = err

            log_query["metrics"] = {}
            for name, value in res.get_metrics():
                log_query["metrics"][name] = value
            
            log_query["metadata"] = {}
            log_query["metadata"]["trials"] = trials
            
            for name, value in res.get_metadata():
                log_query["metadata"][name] = value
            
            log_data.append(log_query)
        
        self.iterations.append(log_data)

    def log_best_results(self, results: "list[dict]"):
        self.best_results = results
        self._log(BEST_RESULTS_KEY, self.best_results)
    
    def log_optimum(self, optimum: dict):
        self.optimum = optimum
        self._log(OPTIMUM_KEY, self.optimum)

    def save_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log destination file")    
        
        self._log(ITERATIONS_KEY, self.iterations)
        
        json_str = json.dumps(self.data, indent=4)
        with open(file, 'w') as outfile:
            outfile.write(json_str)

    def load_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log file")

        with open(file, 'r') as f:
            self.data = json.load(f)

            self.exec_time      = self.data.get("execution_time", 0.0)
            self.basic_params   = self.data[BASIC_PARAMS_KEY]
            self.optimizer      = self.data[OPTIMIZER_KEY]
            self.obj_function = self.data[OBJ_FUNCTION_KEY]
            self.best_results   = self.data.get(BEST_RESULTS_KEY, [])
            self.optimum   = self.data.get(OPTIMUM_KEY, {})
            self.iterations         = self.data[ITERATIONS_KEY]
            f.close()
    
    def get_optimizer_name(self) -> str:
        return self.optimizer["name"]
    
    def get_execution_time(self) -> float:
        return self.exec_time

    def get_active_vars(self) -> "list[str]":
        return self.basic_params["active_variables"]
    
    def get_lower_bounds(self) -> "list[str]":
        opt_name = self.get_optimizer_name()
        if opt_name[:2] == 'BO':
            return self.optimizer["params"].get("lower_bound", None)
        return None
    def get_upper_bounds(self) -> "list[str]":
        opt_name = self.get_optimizer_name()
        if opt_name[:2] == 'BO':
            return self.optimizer["params"].get("upper_bound", None)
        return None

    def get_num_init_points(self) -> int:
        opt_name = self.get_optimizer_name()
        if opt_name == "ROBOT":
            return self.optimizer["params"]["num_init_points"]
        elif opt_name[:2] == "BO":
            return self.optimizer["params"]["bopt_params"]["n_init_samples"]
        else:
            return 0
    
    def get_optimum(self, metric: str = "outcome") -> "tuple[list, float]":
        if len(self.optimum) == 0:
            return None, None
        
        act_vars = self.get_active_vars()
        n_var = len(act_vars)

        best_q = [None] * n_var
        q = self.optimum["query"]
        for var in act_vars:
            idx = act_vars.index(var)
            best_q[idx] = q[var]
        
        m = self.optimum["metrics"][metric]

        return best_q, m

    def get_queries(self, metric: str = "outcome", minimize = False, best_per_iteration = True) -> "tuple[list, list]":
        act_vars = self.get_active_vars()
        n_var = len(act_vars)
        
        queries = []
        metrics = []
        for data_iteration in self.iterations:
            if best_per_iteration:
                best_idx = 0
                best_v = data_iteration[0]["metrics"][metric]
                if len(data_iteration) > 1:
                    for i in range(1, len(data_iteration)):
                        m = data_iteration[i]["metrics"][metric]
                        
                        if (minimize and m < best_v) or (not minimize and m > best_v):
                            best_v = m
                            best_idx = i
                    
                best_q = [None] * n_var
                q = data_iteration[best_idx]["query"]
                for var in act_vars:
                    idx = act_vars.index(var)
                    best_q[idx] = q[var]
                
                queries.append(best_q)
                metrics.append(best_v)
            else:
                for i in range(len(data_iteration)):
                    q = data_iteration[i]["query"]
                    m = data_iteration[i]["metrics"][metric]
                    fq = [None] * n_var
                    for var in act_vars:
                        idx = act_vars.index(var)
                        fq[idx] = q[var]
                    queries.append(fq)
                    metrics.append(m)
        
        return queries, metrics

    def get_best_queries(self, metric: str = "outcome") -> "tuple[list, list]":
        act_vars = self.get_active_vars()
        n_var = len(act_vars)
        
        queries = []
        metrics = []
        for data_query in self.best_results:
            query = [None] * n_var
            
            q = data_query["query"]
            for var in act_vars:
                idx = act_vars.index(var)
                query[idx] = q[var]
            
            queries.append(query)

            for m in data_query["metrics"]:
                if m["name"] == metric:
                    metrics.append(m['value'])
        
        return queries, metrics

    def get_batch_size(self) -> int:
        return len(self.iterations[-1])
    
    def get_batches(self, metric: str = "outcome") -> "tuple[list[list], list[list]]":
        batches = []
        values = []
        for it in self.iterations:
            if len(it) > 1:
                qs, vs = self._parse_iteration(it, metric)
                batches.append(qs)
                values.append(vs)
        
        return batches, values
    
    def _parse_iteration(self, iteration: "list[dict]", metric: str = "outcome") -> "tuple[list[list[float]], list[float]]":
        act_vars = self.get_active_vars()
        n_var = len(act_vars)

        queries = []
        values = []
        for i in range(len(iteration)):
            q = iteration[i]["query"]
            m = iteration[i]["metrics"][metric]
            fq = [None] * n_var
            for var in act_vars:
                idx = act_vars.index(var)
                fq[idx] = q[var]
            queries.append(fq)
            values.append(m)
        
        return queries, values