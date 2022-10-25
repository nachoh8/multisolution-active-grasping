import argparse
import json

from multisolution_active_grasping.executors.bayesopt_executor import BayesOptExecutor
from multisolution_active_grasping.executors.sigopt_executor import SigOptExecutor
from multisolution_active_grasping.utils.utils import create_objective_function

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-objf", nargs=2, help="objective function", metavar=('<objective_function>', '<params_file>'), required=True)
    parser.add_argument("-bopt", nargs=2, help="bayesopt params", metavar=('<bayesopt_params>', '<exp_params>'))
    parser.add_argument("-sopt", type=str, help="sigopt params", metavar='<sigopt_params>')
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")
    parser.add_argument("-metric", type=str, help="metric type", metavar='<metric_type>', default="basic")
    parser.add_argument("-p", type=int, help="batch size", metavar='<batch_size>', default=1)

    args = parser.parse_args()
    
    obj_function_name = args.objf[0]
    obj_function_params_f = args.objf[1]
    if obj_function_params_f != "":
        fparams = open(obj_function_params_f, 'r')
        obj_function_params = json.load(fparams)
    else:
        obj_function_params = {}

    metric = args.metric
    batch_size = args.p

    obj_function = create_objective_function(obj_function_name, metric, function_params=obj_function_params, batch_size=batch_size)
    
    flog = args.flog

    if args.bopt: # bayesopt
        f = open(args.bopt[0], 'r')
        opt_params = json.load(f)
        f2 = open(args.bopt[1], 'r')
        gopt_params = json.load(f2)
        
        params = {"bopt_params": opt_params}
        params.update(gopt_params)
        optimizer = BayesOptExecutor(params, obj_function, log_file=flog)

    elif args.sopt: # sigopt
        f = open(args.sopt, 'r')
        opt_params = json.load(f)

        optimizer = SigOptExecutor(opt_params, obj_function, log_file=flog)

    else:
        print("Error: you must provide BayesoptExecutor (-bopt) or SigOptExecutor (-sopt)")
        exit(-1)

    optimizer.start_optimization()
        
