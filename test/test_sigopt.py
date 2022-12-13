import sys
sys.path.append("../")

import json

from OptSystem.executors.sigopt_executor import SigOptExecutor
from OptSystem.synthetic_functions.function1d import Forrester
    
if __name__ == "__main__":
    # Active grasping params

    f = open("../config/tests/forrester/sigopt_params.json", 'r')
    sigopt_params = json.load(f)

    model = SigOptExecutor(sigopt_params, Forrester(), verbose=True)

    model.start_optimization()
    

    
