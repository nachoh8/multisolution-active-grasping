import numpy as np
from pathlib import Path
import os

from bayesoptmodule import BayesOptContinuous

from ..core.optimizer_executor import OptimizerExecutor
from ..core.objective_function import ObjectiveFunction
from ..core.metric import Metric

class BayesOptExecutor(OptimizerExecutor, BayesOptContinuous):
    
    def __init__(self, params: dict, obj_func: ObjectiveFunction, log_file: str = "", verbose = False):
        n_trials = int(params.get('n_trials', 1))
        default_query = params.get('default_query', {})
        active_variables = params.get('active_variables', None)
        if active_variables == None:
            active_variables = obj_func.get_var_names()
            lb = obj_func.get_lower_bounds()
            ub = obj_func.get_upper_bounds()
            lower_bound = np.array(lb, dtype=np.float64)
            upper_bound = np.array(ub, dtype=np.float64)
        else:
            lower_bound = np.array(params['lower_bound'], dtype=np.float64)
            upper_bound = np.array(params['upper_bound'], dtype=np.float64)
        
        bopt_params = params['bopt_params']
        name = bopt_params["name"]
        bopt_params.pop("name")
        self.invert_metric = bopt_params['invert_metric']
        bopt_params.pop("invert_metric")

        bopt_verbose_lvl = bopt_params.get("verbose_level", 1)
        self.bopt_verbose = verbose
        if verbose and bopt_verbose_lvl > 0:
            verbose = False
            self.bopt_verbose = True
        elif not verbose and bopt_verbose_lvl > 0:
            bopt_params["verbose_level"] = 0
            self.bopt_verbose = False

        if log_file != "":
            bopt_params["load_save_flag"] = 2 # save model data
            data_file = os.path.splitext(os.path.basename(log_file))[0] + ".data"
            bopt_params["save_filename"] = str(Path(log_file).parent.absolute()) + "/" + data_file

        opt_params = {"lower_bound": list(lower_bound), "upper_bound": list(upper_bound), "invert_metric": self.invert_metric ,"bopt_params": bopt_params}
        
        OptimizerExecutor.__init__(self, name, opt_params, obj_func, active_variables, default_query=default_query, n_trials=n_trials, log_file=log_file, verbose=verbose)
        BayesOptContinuous.__init__(self, len(active_variables))

        self.params = bopt_params
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
    
    def _run(self) -> None:
        if self.bopt_verbose:
            print("------------------------")
            print("BAYESOPT EXECUTOR")
            print("Optimizer: " + self.name)
            print("Objective function: " + self.obj_func.get_name())
            print("Metric: " + self.obj_func.get_metric().get_name())
            print("Invert Metric: " + str(self.invert_metric))
            print("Active variables: " + str(self.active_variables))
            print("Default query: " + str(self.default_query))
            print("------------------------")
        
        data_res = self.optimize()
        multisolution = self.params.get("num_solutions", 1) > 1 and self.params.get("diversity_level", 0.0) > 0.0
        if multisolution:
            quality, x_out, solutions, sols_value = data_res
        else:
            quality, x_out, _ = data_res

        query = dict(zip(self.active_variables, list(x_out)))
        value = -quality if self.invert_metric else quality
        r = {"query": query, "metrics": [{"name": self.obj_func.get_metric().get_metric_names()[0], "value": value}]}
        self.optimum = r

        if self.bopt_verbose:
            print("------------------------")
            print("Best:")
            print("\tPoint:", x_out)
            print("\tOutcome:", value)

        if multisolution:
            if self.bopt_verbose:
                print("\nMultisolutions:")

            for sx, sv in zip(solutions, sols_value):
                q = dict(zip(self.active_variables, list(sx)))
                v = -sv if self.invert_metric else sv
                sol = {"query": q, "metrics": [{"name": self.obj_func.get_metric().get_metric_names()[0], "value": v}]}
                self.best_results.append(sol)
                
                if self.bopt_verbose:
                    print("\tPoint:", sx, "->", v)
        else:
            self.best_results = [r]
        
    def evaluateSample(self, x_in: np.ndarray) -> float:
        query = dict(zip(self.active_variables, list(x_in)))
        
        res: Metric = self.executeQuery(query)

        main_metric = res.get_metrics()[0]
        value = main_metric[1]

        return -value if self.invert_metric else value

    def evaluateSamples(self, x_in: np.ndarray) -> np.ndarray:
        queries = [dict(zip(self.active_variables, list(x))) for x in x_in]
        res = self.executeBatch(queries)
        
        res_np = np.zeros(len(res))
        for i in range(len(res)):
            main_metric = res[i].get_metrics()[0]
            value = main_metric[1]
            res_np[i] = -value if self.invert_metric else value
        
        return res_np

