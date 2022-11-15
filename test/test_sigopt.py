import json

from multisolution_active_grasping.executors.sigopt_executor import SigOptExecutor
from multisolution_active_grasping.synthetic_functions.function1d import Forrester
    
if __name__ == "__main__":
    # Active grasping params

    f = open("config/tests/forrester/sigopt_params.json", 'r')
    sigopt_params = json.load(f)

    model = SigOptExecutor(sigopt_params, Forrester())

    model.start_optimization()
    

    
