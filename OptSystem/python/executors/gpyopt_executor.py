import numpy as np

import GPyOpt as go

from ..core.optimizer_executor import OptimizerExecutor
from ..core.objective_function import ObjectiveFunction
from ..core.metric import Metric

class GPyOptExecutor(OptimizerExecutor):
    
    def __init__(self, params: dict, obj_func: ObjectiveFunction, log_file: str = "", verbose = False):
        n_trials = int(params['n_trials'])
        default_query = params['default_query']
        domain = params["domain"] # [{'name': 'x', 'type': 'continuous', 'domain': (lb,ub)}, ...]
        active_variables = [var['name'] for var in domain]

        initial_design_numdata = params["initial_design_numdata"]
        self.num_iters = params["num_iters"]

        batch_size = params["batch_size"]

        model_update_interval = params["model_update_interval"]
        self.maximize = params['maximize']
        
        name = params["name"]

        opt_params = {
            "maximize": self.maximize ,
            "domain": domain,
            "initial_design_numdata": initial_design_numdata,
            "num_iters": self.num_iters,
            "batch_size": batch_size,
            "model_update_interval": model_update_interval
        }

        OptimizerExecutor.__init__(self, name, opt_params, obj_func, active_variables, default_query=default_query, n_trials=n_trials, log_file=log_file, verbose=verbose)
        
    
    def _run(self) -> None:
        if self.verbose:
            print("------------------------")
            print("GPyOpt EXECUTOR")
            print("Optimizer: " + self.name)
            print("Objective function: " + self.obj_func.get_name())
            print("Metric: " + self.obj_func.get_metric().get_name())
            print("Maximize metric: " + str(self.maximize))
            print("Active variables: " + str(self.active_variables))
            print("Default query: " + str(self.default_query))
            print("------------------------")
        
        self.executor = go.methods.BayesianOptimization(
            self.evaluateSample,
            domain=self.optimizer_params["domain"],
            acquisition_type = 'LCB',              
            normalize_Y = False,
            initial_design_numdata = self.optimizer_params["initial_design_numdata"],
            initial_design_type='latin',
            evaluator_type = 'local_penalization',
            batch_size = self.optimizer_params["batch_size"],
            num_cores = 1,
            model_update_interval=self.optimizer_params["model_update_interval"],
            maximize=self.maximize,
            ARD=True
        )

        self.executor.run_optimization(self.num_iters)

        x_best = self.executor.x_opt
        y_best = self.executor.fx_opt

        query = dict(zip(self.active_variables, list(x_best)))
        value = -y_best if self.maximize else y_best
        r = {"query": query, "metrics": [{"name": self.obj_func.get_metric().get_metric_names()[0], "value": value}]} # TODO: revisar
        self.best_results = [r]

        print("------------------------")
        print("Best:")
        print("\tPoint:", x_best)
        print("\tOutcome:", value)

        if self.logger: # convert to batch mode
            iterations = self.logger.iterations
            init_samples = self.optimizer_params["initial_design_numdata"]
            batch_size = self.optimizer_params["batch_size"]

            final_iterations = iterations[:init_samples]
            i = init_samples
            while i < len(iterations):
                data_batch = [iterations[j][0] for j in range(i, i+batch_size)]
                i += batch_size
                final_iterations.append(data_batch)
            
            self.logger.iterations = final_iterations

        
    def evaluateSample(self, x_in: np.ndarray) -> np.ndarray:
        """
        x_in(1, ndim) \\
        return y(1,1)
        """
        query = dict(zip(self.active_variables, list(x_in.flatten())))
        
        res: Metric = self.executeQuery(query)

        main_metric = res.get_metrics()[0]
        value = main_metric[1]

        return np.array(value).reshape(1, 1)


