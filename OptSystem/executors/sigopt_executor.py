import os
import sigopt

from ..core.optimizer_executor import OptimizerExecutor
from ..core.objective_function import ObjectiveFunction
from ..core.metric import Metric

class SigOptExecutor(OptimizerExecutor):
    def __init__(self, params: dict, obj_func: ObjectiveFunction, log_file: str = "", verbose = False, exp_id: str = None) -> None:
        n_trials = int(params.get('n_trials', 1))
        default_query = params.get('default_query', {})
        
        self.project_name: str = params["project"]

        self.token_type: str = params["mode"]
        if self.token_type == "dev":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_DEV_TOKEN"]
        elif self.token_type == "prod":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_PROD_TOKEN"]
        else:
            raise Exception("Mode not valid, use dev or prod")
        
        self.exp_id: str = exp_id # sigopt experiment id

        self.report_failures: bool = params.get("report_failures",False)

        self.exp_params: dict = params["exp_params"]
        os.environ["SIGOPT_PROJECT"] = self.project_name
        self.num_runs: int = self.exp_params["budget"]

        active_variables = [param["name"] for param in self.exp_params["parameters"]]

        name = params["name"]
        opt_params = {"project": self.project_name, "mode": self.token_type, "report_failures": self.report_failures, "exp_params": self.exp_params}

        OptimizerExecutor.__init__(self, name, opt_params, obj_func, active_variables, default_query=default_query, n_trials=n_trials, log_file=log_file, verbose=verbose)

        sigopt.set_project(self.project_name)
        if self.exp_id is None:
            self.experiment = sigopt.create_experiment(**self.exp_params)
            self.exp_id = self.experiment.id
        else:
            self.experiment = sigopt.get_experiment(self.exp_id)
            if self.experiment.is_finished():
                print("------------------------")
                print("SigOpt: Experiment " + self.exp_params["name"] + " (" + str(self.exp_id) + ") is finished")
                exit(0)
            if self.logger:
                self.logger.load_json()
            self.n_iterations = self.num_runs - self.experiment.progress.remaining_budget

        if self.logger:
            self.save_log_iteration = True
            self.optimizer_params["exp_id"] = self.exp_id
            self.log_params()
        
    def _run(self):
        if self.executor_verbose:
            print("------------------------")
            print("SIGOPT")
            print("Optimizer: " + self.name)
            print("Project: " + self.project_name)
            print("Experiment: " + self.exp_params["name"])
            print("ID: " + self.exp_id)
            print("Mode: " + ("dev" if self.token_type == "dev" else "production"))
            print("Report failures: " + str(self.report_failures))
            print("Objective function: " + self.obj_func.get_name())
            print("Metric: " + self.obj_func.get_metric().get_name())
            print("Active variables: " + str(self.active_variables))
            print("Default query: " + str(self.default_query))
            print("------------------------")
            if self.n_iterations == 0:
                print("Begin experiment")
            else:
                print("Continue experiment from iteration " + str(self.n_iterations+1))
            print("-------------------------------")
        
        for run in self.experiment.loop():
            with run:
                self.execute_run_query(run)
            if self.logger:
                self.logger.save_json()
        
        if self.executor_verbose:
            print("-------------------------------")
            print("End experiment")
            print("-------------------------------")
        
        best_runs = self.experiment.get_best_runs() # generator type
        if self.executor_verbose:
            print("Best results:")
        self.best_results = []
        for run in best_runs: # sigopt.objects.TrainingRun type
            if self.executor_verbose:
                print("Query:", list(run.assignments.items())) # sigopt.objects.Assignments = dict[param_name: value]
                print("Metrics:", [(metric, value.value) for metric, value in run.values.items()]) # dict[metric_name: sigopt.objects.MetricEvaluation]
            
            r = {"query": run.assignments, "metrics": [{"name": metric, "value": value.value} for metric, value in run.values.items()]}
            self.best_results.append(r)
    
    def execute_run_query(self, run: sigopt.run_context.RunContext):
        run.log_model(self.obj_func.get_name())

        query = dict(run.params)
        res: Metric = self.executeQuery(query)

        metrics = res.get_metrics()
        error = res.get_failure()
        metadata = res.get_metadata()

        not_log_metric = (error != "" and self.report_failures)

        if not not_log_metric:
            for name, value in metrics:
                run.log_metric(name, value)
        
        for name, value in metadata:
            run.log_metadata(name, value)

        if len(self.default_query) > 0:
            run.log_metadata("default_query", self.default_query)

        if error:
            run.log_metadata("error", error)
            if self.report_failures:
                run.log_failure()
