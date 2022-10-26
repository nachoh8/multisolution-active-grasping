from .objective_function import ObjectiveFunction, BatchObjectiveFunction
from .metric import Metric
from .datalog import DataLog

class OptimizerExecutor(object):
    def __init__(self, name: str, opt_params: dict, obj_func: ObjectiveFunction, active_variables: "list[str]", default_query: dict = {},
                    n_trials: int = 1, log_file: str = "") -> None:
        
        self.name = name
        self.optimizer_params = opt_params

        self.active_variables: list[int] = active_variables
        self.default_query: dict = default_query
        self.n_trials: int = n_trials
        
        self.obj_func: ObjectiveFunction = obj_func
        if len(self.default_query) > 0:
            self.obj_func.set_default_query(self.default_query)

        self.best_results: list[dict] = []

        if log_file != "":
            self.logger: DataLog = DataLog(log_file)
            params_log = {"active_variables": self.active_variables, "default_query": self.default_query, "metric": self.obj_func.get_metric().get_name(), "n_trials": self.n_trials}
            self.logger.log_basic_params(params_log)
            self.logger.log_objective_function(self.obj_func.get_name(), self.obj_func.get_params())
            self.logger.log_optimizer(self.name, self.optimizer_params)
        else:
            self.logger: DataLog = None

    def executeQuery(self, query: dict) -> Metric:
        err = "err"
        i = 0
        while i < self.n_trials and err != "":
            res: Metric = self.obj_func.execute(query)
            err = res.get_failure()
            i +=1

        if self.logger:
            self.logger.log_iteration([query], [res], [i])

        if err != "":
            print("Query:", query, "-> Error:", err, " trials:", i)
        else:
            print("Query:", query)
            print("Metrics:", res.get_metrics())
            print("Metadata:", res.get_metadata())

        return res
    
    def executeBatch(self, queries: "list[dict]") -> "list[Metric]":
        if type(self.obj_func) == BatchObjectiveFunction:
            res: tuple[list[Metric],list[int]] = self.obj_func.executeBatch(queries, self.n_trials)

            for query, m, trials in zip(queries, res[0], res[1]):
                err = m.get_failure()
                if err != "":
                    print("Query:", query, "-> Error:", err, " trials:", trials)
                else:
                    print("Query:", query)
                    print("Metrics:", m.get_metrics())
                    print("Metadata:", m.get_metadata())
            
            if self.logger:
                self.logger.log_iteration(queries, res[0], res[1])

            return res[0]

        raise Exception("Batch optimization needs a BatchObjectiveFunction")
    
    def _run(self):
        raise Exception("This is an abstract class")

    def start_optimization(self):
        self._run()
        if self.logger:
            self.logger.log_best_results(self.best_results)
            self.logger.save_json()
