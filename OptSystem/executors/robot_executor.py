import numpy as np
import torch

from .robot_lib.scripts.optimize import Optimize
from .robot_lib.robot.objective import Objective
from .robot_lib.robot.tasks.test import Himmelblau

from ..core.optimizer_executor import OptimizerExecutor
from ..core.objective_function import ObjectiveFunction
from ..core.metric import Metric

class ROBOTObjectiveWrap(Objective):
    def __init__(self, obj_name: str, ndim: int, numpy_obj_function, unnormalize_query):
        self.numpy_obj_function = numpy_obj_function
        self.unnormalize_query = unnormalize_query

        super().__init__(
            task_id=obj_name,
            dim=ndim,
            lb=0.0,
            ub=1.0
            )

    def query_oracle(self, x: torch.tensor):
        x = x.detach().cpu().numpy()

        x_query = self.unnormalize_query(x)
        
        return self.numpy_obj_function(x_query)
    
    def divf(self, x1: torch.tensor, x2: torch.tensor):
        x1_np = x1.detach().cpu().numpy()
        x1_np = self.unnormalize_query(x1_np)

        x2_np = x2.detach().cpu().numpy()
        x2_np = self.unnormalize_query(x2_np)

        return np.linalg.norm(x1_np - x2_np)

class ROBOTExecutor(OptimizerExecutor, Optimize):
    
    def __init__(self, params: dict, obj_func: ObjectiveFunction, log_file: str = "", verbose = False):
        ### base params
        n_trials = int(params.get('n_trials', 1))
        default_query = params.get('default_query', {})
        opt_name = params["name"]
        minimize = params['minimize']

        active_variables = params.get('active_variables', None)
        if active_variables == None:
            active_variables = obj_func.get_var_names()
            lb = obj_func.get_lower_bounds()
            ub = obj_func.get_upper_bounds()
            self.lower_bound = np.array(lb, dtype=np.float64)
            self.upper_bound = np.array(ub, dtype=np.float64)
        else:
            self.lower_bound = np.array(params['lower_bound'], dtype=np.float64)
            self.upper_bound = np.array(params['upper_bound'], dtype=np.float64)
        
        self.robot_verbose = verbose

        ### robot params
        num_init_points = params["num_init_points"]
        num_total_evaluations = params["num_total_evaluations"]
        bsz = params["max_batch_size_per_sol"]
        num_solutions = params["num_solutions"]
        diversity_level = params["diversity_level"]
        print_freq = params.get("print_freq", 5)
        
        
        opt_params = {
            "minimize": minimize,
            "lower_bound": list(self.lower_bound),
            "upper_bound": list(self.upper_bound),
            "num_init_points": num_init_points,
            "num_total_evaluations": num_total_evaluations,
            "max_batch_size_per_sol": bsz,
            "num_solutions": num_solutions,
            "diversity_level": diversity_level,
            "print_freq": print_freq
            }

        OptimizerExecutor.__init__(
            self,
            opt_name,
            opt_params,
            obj_func,
            active_variables,
            default_query=default_query,
            n_trials=n_trials,
            log_file=log_file,
            verbose=False
        )
    
    def evaluateSample(self, x_in: np.ndarray) -> float:
        query = dict(zip(self.active_variables, list(x_in)))
        
        res: Metric = self.executeQuery(query)

        main_metric = res.get_metrics()[0]
        value = main_metric[1]

        return value
    
    def unnormalize(self, x: np.ndarray):
        return x * (self.upper_bound - self.lower_bound) + self.lower_bound

    def initialize_objective(self):
        self.objective = ROBOTObjectiveWrap(
            self.obj_func.get_name(),
            self.lower_bound.shape[0],
            self.evaluateSample,
            self.unnormalize
        )
        return self
    
    def load_train_data(self):
        lb, ub = self.objective.lb, self.objective.ub 
        xs = torch.rand(self.num_initialization_points, self.objective.dim)*(ub - lb) + lb
        out_dict = self.objective(xs)
        self.init_train_x = torch.from_numpy(out_dict['valid_xs']).float() 
        self.init_train_y = torch.tensor(out_dict['scores']).float() 
        self.init_train_y = self.init_train_y.unsqueeze(-1)
        
        if self.verbose:
            print("Loaded initial training data")
            print("train x shape:", self.init_train_x.shape)
            print("train y shape:", self.init_train_y.shape)

        return self 

    def _run(self):
        if self.robot_verbose:
            print("------------------------")
            print("ROBOT EXECUTOR")
            print("Objective function: " + self.obj_func.get_name())
            print("Metric: " + self.obj_func.get_metric().get_name())
            print("Minimize: " + str(self.optimizer_params["minimize"]))
            print("Active variables: " + str(self.active_variables))
            print("Default query: " + str(self.default_query))
            print("------------------------")
        
        Optimize.__init__(
            self,
            self.obj_func.get_name(),
            M=self.optimizer_params["num_solutions"],
            tau=self.optimizer_params["diversity_level"],
            minimize=self.optimizer_params["minimize"],
            max_n_oracle_calls=self.optimizer_params["num_total_evaluations"],
            bsz=self.optimizer_params["max_batch_size_per_sol"],
            num_initialization_points=self.optimizer_params["num_init_points"],
            print_freq=self.optimizer_params["print_freq"],
            verbose=self.robot_verbose
        )

        self.method_args['opt'] = locals()
        del self.method_args['opt']['self']
        
        self.run_robot()

        self.done()

        if self.robot_verbose:
            print("-------------------------------")
            print("End experiment")
            print("-------------------------------")
            print("Best results:")
        
        self.best_results = []
        for t_query, score in zip(self.robot_state.M_diverse_xs, self.robot_state.M_diverse_scores):
            q_np = t_query.detach().cpu().numpy()
            q = self.unnormalize(q_np).tolist()
            query = dict(zip(self.active_variables, q))
            v = float(score)
            if self.verbose:
                print("Query:", query)
                print("Outcome:", v)
            
            r = {"query": query, "metrics": [{"name": self.obj_func.get_metric().get_metric_names()[0], "value": v}]}
            self.best_results.append(r)

