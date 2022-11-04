import numpy as np
import math
import argparse
import os
import glob
import matplotlib.pyplot as plt
from tabulate import tabulate

# from multisolution_active_grasping.cec2013.cec2013.cec2013 import CEC2013, FUNCTION_NAMES
from multisolution_active_grasping.core.datalog import DataLog
from multisolution_active_grasping.core.objective_function import ObjectiveFunction
from multisolution_active_grasping.utils.utils import create_objective_function

ACCURACY = 0.001

def find_seeds_indices(sorted_pop, radius):
    seeds = []
    seeds_idx = []

    # Determine the species seeds: iterate through sorted population
    for i, x in enumerate(sorted_pop):
        found = False
        # Iterate seeds
        for j, sx in enumerate(seeds):
            # Calculate distance from seeds
            dist = np.sqrt(np.sum((x - sx) ** 2))

            # If the Euclidean distance is less than the radius
            if dist <= radius:
                found = True
                break
        if not found:
            seeds.append(x)
            seeds_idx.append(i)

    return seeds_idx

def global_optima_found(queries: np.ndarray, values: np.ndarray, go_value: float, n_go: int, radius: float, accuracy: float = ACCURACY, minimize = False):
    order = np.argsort(values)
    if not minimize:
        order = order[::-1]
    sorted_v = values[order]
    sorted_q = queries[order, :]

    
    seeds_idx = find_seeds_indices(sorted_q, radius) # get different global optimums

    count = 0
    goidx = []
    for idx in seeds_idx:
        # evaluate seed
        seed_fitness = sorted_v[idx]  # f.evaluate(sorted_pop[idx])

        # |F_seed - F_goptimum| <= accuracy
        if math.fabs(seed_fitness - go_value) <= accuracy:
            count = count + 1
            goidx.append(idx)

        # save time
        if count == n_go:
            break

    # gather seeds
    go_q = sorted_q[goidx]
    go_v = sorted_v[goidx]
    values

    return go_q, go_v

"""def convergence_speed_old(queries: np.ndarray, values: np.ndarray, f: CEC2013, accuracy: float = ACCURACY):
    q = queries.reshape(values.shape[0], -1)
    seeds_idx = find_seeds_indices(q, f.get_rho()) # get different global optimum candidates
    
    all_founds = False
    goidx = []
    for idx in seeds_idx:
        # evaluate seed
        seed_fitness = values[idx]  # f.evaluate(sorted_pop[idx])

        # |F_seed - F_goptimum| <= accuracy
        if math.fabs(seed_fitness - f.get_fitness_goptima()) <= accuracy:
            goidx.append(idx)

        # save time
        if len(goidx) == f.get_no_goptima():
            all_founds = True
            break

    total_evals = values.shape[0]
    if all_founds:
        go_np = np.array([goidx])
        return np.max(go_np+1) / total_evals
    else:
        return 1.0
"""
def convergence_speed(queries: np.ndarray, values: np.ndarray, go_value: float, n_go: int, radius: float, accuracy: float = ACCURACY):
    total_evals = values.shape[0]

    fe = -1
    go_idx = []
    for i in range(total_evals):
        q = queries[i]
        v = values[i]

        # |F_seed - F_goptimum| <= accuracy
        if math.fabs(v - go_value) <= accuracy:
            found = False
            for idx in go_idx:
                dist = np.sqrt(np.sum((q - queries[idx-1]) ** 2))
                # If the Euclidean distance is less than the radius
                if dist <= radius:
                    found = True
                    break
            
            if not found:
                go_idx.append(i+1)

        # save time
        if len(go_idx) == n_go:
            fe = i + 1
            break

    if fe > -1:
        return fe / total_evals, go_idx
    else:
        return 1.0, go_idx

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    
    args = parser.parse_args()
    flogs = args.flogs

    print(70 * '=')

    data = []
    for flog in flogs:
        # print(70 * '=')
        # print("LOG: " + flog)

        if os.path.isdir(flog):
            logs = glob.glob(flog + "/*.json")
            # print("Num. log files: " + str(len(logs)))
        else:
            logs = [flog]

        num_runs = len(logs)

        _runs_queries = []
        _runs_values = []
        _runs_best_queries = []
        _runs_best_values = []
        for log_file in logs:
            logger = DataLog(log_file=log_file)
            logger.load_json()

            _queries, _outcomes = logger.get_queries(best_per_iteration=False)
            queries = np.array(_queries)
            values = np.array(_outcomes).reshape(-1)
            _best = logger.get_best_queries()
            best_querys = np.array(_best[0])
            best_values = np.array(_best[1]).reshape(-1)

            _runs_queries.append(queries)
            _runs_values.append(values)
            _runs_best_queries.append(best_querys)
            _runs_best_values.append(best_values)

        runs_queries = np.array(_runs_queries)
        runs_values = np.array(_runs_values)
        runs_best_queries = np.array(_runs_best_queries)
        runs_best_values = np.array(_runs_best_values)

        total_evals_executed = runs_values.shape[1]

        data_exp = []
        data_exp.append(logger.get_optimizer_name())
        function_name = logger.obj_function["name"]
        obj_function = create_objective_function(function_name, "basic")

        # idx_f = FUNCTION_NAMES.index(function_name) + 1

        # fcec = CEC2013(idx_f)

        # print(10 * '-')

        # print("Function: " + function_name) # + " (F" + str(idx_f) + ")")

        # info = fcec.get_info()
        nGO = obj_function.get_num_global_optima() # info["nogoptima"]
        go_value = obj_function.get_global_optima()
        radius = 0.01

        # print("Num GO: " + str(nGO))
        # print("GO value: " + str(go_value))
        # print("Max num evals: " + str(info["maxfes"]))

        # print(10 * '-')

        # print("Total evals executed: " + str(total_evals_executed))
        # print("Solutions:", runs_best_values.flatten())
        # print("Solutions:", np.array(runs_best_values.flatten() - go_value <= ACCURACY))
        # print("Best solution: " + str(best_querys[0]) + " -> " + str(best_values[0]))

        runs_nGO = np.zeros(num_runs)
        runs_cspeed = np.zeros(num_runs)
        for i in range(num_runs):
            go_q, go_v = global_optima_found(runs_queries[i], runs_values[i], go_value, nGO, radius, minimize=True)
            runs_nGO[i] = go_v.shape[0]
            runs_cspeed[i], go_idx = convergence_speed(runs_queries[i], runs_values[i], go_value, nGO, radius)
        
        _founds = runs_nGO / nGO
        # print("Num of GOs found: " + str(_founds))

        peak_ratio = np.sum(runs_nGO) / (nGO * num_runs)
        data_exp.append(peak_ratio)
        # print("Peak ratio: " + str(peak_ratio))

        success_rate = np.count_nonzero(runs_nGO==nGO) / num_runs
        data_exp.append(success_rate)
        # print("Success rate: " + str(success_rate))
        
        avg_conv_speed = np.mean(runs_cspeed)
        data_exp.append(avg_conv_speed)
        # print("Avg. Convergence speed: " + str(avg_conv_speed))

        data_exp.append(np.min(runs_best_values))
        data_exp.append(np.mean(runs_best_values))

        data.append(data_exp)
    
    #print(70 * '=')
    print("Function:", function_name)
    print("Num GO: " + str(nGO))
    print("GO value: " + str(go_value))
    print(tabulate(data, headers=["Optimizer", "PR", "SR", "AvgFEs", "Best value", "Avg. Best"]))



