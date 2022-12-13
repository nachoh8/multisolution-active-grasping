import argparse
import json

from OptSystem.utils.utils import create_optimizer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-objf", nargs='+', help="objective function", metavar=('<objective_function>', '<params_file>'), required=True)
    parser.add_argument("-bopt", nargs='+', help="bayesopt params", metavar=('<bayesopt_params>', '<exp_params>'))
    parser.add_argument("-sopt", nargs='+', help="sigopt params", metavar=('<sigopt_params>', '<exp_id>'))
    parser.add_argument("-gpyopt", type=str, help="GPyOpt params", metavar='<gpyopt_params>')
    parser.add_argument("-robot", type=str, help="ROBOT params", metavar='<robot_params>')
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")
    parser.add_argument("-metric", type=str, help="metric type", metavar='<metric_type>', default="basic")
    parser.add_argument('-p', help='Batch optimization in parallel', action='store_true')
    parser.add_argument('-v', help='Verbose mode', action='store_true')

    args = parser.parse_args()
    
    obj_func_data = {}
    obj_func_data["name"] = args.objf[0]
    obj_func_data["fparams"] = args.objf[1] if len(args.objf) == 2 else ""
    
    metric = args.metric

    in_parallel = args.p
    verbose = args.v

    optimizer_data = {}

    optimizer_data["flog"] = args.flog

    if args.bopt: # bayesopt
        f = open(args.bopt[0], 'r')
        opt_params = json.load(f)
        params = {"bopt_params": opt_params}

        if len(args.bopt) == 2:
            f2 = open(args.bopt[1], 'r')
            gopt_params = json.load(f2)
            params.update(gopt_params)   
        
        optimizer_data["name"] = "bayesopt"
        optimizer_data["params"] = params

    elif args.sopt: # sigopt
        f = open(args.sopt[0], 'r')
        opt_params = json.load(f)

        optimizer_data["name"] = "sigopt"
        optimizer_data["params"] = opt_params
        if len(args.sopt) == 2:
            optimizer_data["exp_id"] = args.sopt[1]
    elif args.gpyopt: # GPyOpt
        f = open(args.gpyopt, 'r')
        opt_params = json.load(f)

        optimizer_data["name"] = "gpyopt"
        optimizer_data["params"] = opt_params
    elif args.robot: # GPyOpt
        f = open(args.robot, 'r')
        opt_params = json.load(f)

        optimizer_data["name"] = "robot"
        optimizer_data["params"] = opt_params

    optimizer = create_optimizer(optimizer_data, obj_func_data, metric, in_parallel=in_parallel, verbose=verbose)
    
    optimizer.start_optimization()
        
