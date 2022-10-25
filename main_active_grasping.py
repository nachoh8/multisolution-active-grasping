import argparse
import json

from active_grasping.utils import construct_grasp_executor_model
from active_grasping.bayesopt_executor import BayesOptExecutor
from active_grasping.sigopt_executor import SigOptExecutor
from active_grasping.datalog import DataLog

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-fgrasp", nargs=2, help="grasp executor params", metavar=('<executor>', '<params_file>'), required=True)
    parser.add_argument("-fbopt", nargs=2, help="bayesopt params", metavar=('<bayesopt_params>', '<exp_params>'))
    parser.add_argument("-fsopt", type=str, help="sigopt params", metavar='<sigopt_params>')
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")
    parser.add_argument("-metric", type=int, help="metric type for Grasp Planner IK", metavar='<metric_type>', default=-1)

    args = parser.parse_args()
    
    grasp_executor = int(args.fgrasp[0])
    fgrasp = args.fgrasp[1]

    if grasp_executor > 3:
        print("Error: executor must be {0: TestGramacyExecutor, 1: GraspPlanner, 2: GraspPlannerIK, 3: GraspPlannerS}")
        exit(-1)

    metric = int(args.metric)
    executor = construct_grasp_executor_model(grasp_executor, fgrasp=fgrasp, mtype=metric)
    
    flog = args.flog
    if flog:
        logger = DataLog(log_file=flog)
    else:
        logger = None
    

    if args.fbopt: # bayesopt
        f = open(args.fbopt[0], 'r')
        opt_params = json.load(f)
        f2 = open(args.fbopt[1], 'r')
        gopt_params = json.load(f2)
        
        params = {"bopt_params": opt_params}
        params.update(gopt_params)
        model = BayesOptExecutor(params, executor, logger=logger)

    elif args.fsopt: # sigopt
        f = open(args.fsopt, 'r')
        opt_params = json.load(f)

        model = SigOptExecutor(opt_params, executor, logger=logger)

    else:
        print("Error: you must provide BayesoptExecutor (-fbopt) or SigOptExecutor (-fsopt)")
        exit(-1)


    model.execute()
        
