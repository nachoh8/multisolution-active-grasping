import json
from datetime import datetime

from .metric import Metric

BASIC_PARAMS_KEY = "basic_params"
OPTIMIZER_KEY = "optimizer"
OBJ_FUNCTION_KEY = "objective_function"
BEST_RESULTS_KEY = "best_results"
QUERIES_KEY = "queries"

class DataLog(object):
    def __init__(self, log_file: str) -> None:
        self.log_file: str = log_file

        self.data = dict()
        self.data["date"] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        self.basic_params: dict = dict()
        self.obj_function: dict = dict()
        self.optimizer: dict = dict()
        self.best_results: list[dict] = []
        self.queries: list[dict] = []
    
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
    
    def log_query(self, query: dict, res: Metric, trials: int):
        log_data = {}

        log_data["query"] = query

        err = res.get_failure()
        if err != "":
            log_data["error"] = err

        log_data["metrics"] = {}
        for name, value in res.get_metrics():
            log_data["metrics"][name] = value
        
        log_data["metadata"] = {}
        log_data["metadata"]["trials"] = trials
        
        for name, value in res.get_metadata():
            log_data["metadata"][name] = value
        
        self.queries.append(log_data)
        self._log(QUERIES_KEY, self.queries)

    def log_best_results(self, results: "list[dict]"):
        self.best_results = results
        self._log(BEST_RESULTS_KEY, self.best_results)

    def save_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log destination file")    

        json_str = json.dumps(self.data, indent=4)
        with open(file, 'w') as outfile:
            outfile.write(json_str)

    