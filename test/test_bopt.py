import json

from multisolution_active_grasping.executors.bayesopt_executor import BayesOptExecutor
from multisolution_active_grasping.syntethic_functions.forrester import Forrester

if __name__ == "__main__":
    
    f = open("config/tests/forrester/bopt_params.json", 'r')
    opt_params = json.load(f)
    f2 = open("config/tests/forrester/gopt_params.json", 'r')
    gopt_params = json.load(f2)
    
    params = {"bopt_params": opt_params}
    params.update(gopt_params)

    model = BayesOptExecutor(params, Forrester())
    model.start_optimization()
