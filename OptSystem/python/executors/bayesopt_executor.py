import numpy as np

from bayesoptmodule import BayesOptContinuous

from ..core.optimizer_executor import OptimizerExecutor
from ..core.objective_function import ObjectiveFunction
from ..core.metric import Metric

class BayesOptExecutor(OptimizerExecutor, BayesOptContinuous):
    
    def __init__(self, params: dict, obj_func: ObjectiveFunction, log_file: str = ""):
        n_trials = int(params['n_trials'])
        active_variables = params['active_variables']
        default_query = params['default_query']
        lower_bound = np.array(params['lower_bound'], dtype=np.float64)
        upper_bound = np.array(params['upper_bound'], dtype=np.float64)

        self.invert_metric = params['invert_metric']
        
        bopt_params = params['bopt_params']
        name = bopt_params["name"]
        bopt_params.pop("name")

        opt_params = {"lower_bound": list(lower_bound), "upper_bound": list(upper_bound), "invert_metric": self.invert_metric ,"bopt_params": bopt_params}
        
        OptimizerExecutor.__init__(self, name, opt_params, obj_func, active_variables, default_query=default_query, n_trials=n_trials, log_file=log_file)
        BayesOptContinuous.__init__(self, len(active_variables))

        self.params = bopt_params
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
    
    def _run(self) -> None:
        print("------------------------")
        print("BAYESOPT EXECUTOR")
        print("Optimizer: " + self.name)
        print("Objective function: " + self.obj_func.get_name())
        print("Metric: " + self.obj_func.metric.get_name())
        print("Invert Metric: " + str(self.invert_metric))
        print("Active variables: " + str(self.active_variables))
        print("Default query: " + str(self.default_query))
        print("------------------------")

        quality, x_out, _ = self.optimize()     

        query = dict(zip(self.active_variables, list(x_out)))
        value = -quality if self.invert_metric else quality
        r = {"query": query, "metrics": [{"name": self.obj_func.metric.get_metric_names()[0], "value": value}]} # TODO: revisar
        self.best_results = [r]

        print("------------------------")
        print("Best:")
        print("\tPoint:", x_out)
        print("\tOutcome:", quality)
        
    def evaluateSample(self, x_in) -> float:
        query = dict(zip(self.active_variables, list(x_in)))
        
        res: Metric = self.executeQuery(query)

        main_metric = res.get_metrics()[0]
        value = main_metric[1]

        return -value if self.invert_metric else value

