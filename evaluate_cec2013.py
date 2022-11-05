import numpy as np
import math
import argparse
import os
import glob
import matplotlib.pyplot as plt
from tabulate import tabulate

from multisolution_active_grasping.core.datalog import DataLog
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

def distance_queries_batch(batches: "tuple[np.ndarray, np.ndarray]") -> "tuple[np.ndarray, np.ndarray]":
    queries, values = batches
    num_b = values.shape[0]

    var_q = np.zeros((num_b, queries.shape[2]))
    var_v = np.zeros(num_b)
    for i, qs, vs in zip(range(num_b), queries, values):
        var_q[i] = np.var(qs, axis=0)
        var_v[i] = np.var(vs)
    
    return var_q, var_v

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument("-acc", type=float, help="accuracy", metavar='<accuracy>', default=ACCURACY)
    
    args = parser.parse_args()
    flogs = args.flogs
    ACCURACY = args.acc

    print(70 * '=')

    data = []
    for flog in flogs:
        if os.path.isdir(flog):
            logs = glob.glob(flog + "/*.json")
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

        data_exp = []
        data_exp.append(logger.get_optimizer_name())
        function_name = logger.obj_function["name"]
        obj_function = create_objective_function(function_name, "basic")

        # idx_f = FUNCTION_NAMES.index(function_name) + 1
        # fcec = CEC2013(idx_f)

        total_evals_executed = runs_values.shape[1]
        data_exp.append(total_evals_executed)

        nGO = obj_function.get_num_global_optima()
        go_value = obj_function.get_global_optima()
        radius = obj_function.get_exclusion_radius()

        runs_nGO = np.zeros(num_runs)
        runs_cspeed = np.zeros(num_runs)
        for i in range(num_runs):
            go_q, go_v = global_optima_found(runs_queries[i], runs_values[i], go_value, nGO, radius, minimize=True)
            runs_nGO[i] = go_v.shape[0]
            runs_cspeed[i], go_idx = convergence_speed(runs_queries[i], runs_values[i], go_value, nGO, radius)
        
        peak_ratio = np.sum(runs_nGO) / (nGO * num_runs)
        data_exp.append(peak_ratio)

        success_rate = np.count_nonzero(runs_nGO==nGO) / num_runs
        data_exp.append(success_rate)
        
        avg_conv_speed = np.mean(runs_cspeed)
        data_exp.append(avg_conv_speed)

        data_exp.append(np.min(runs_best_values))
        data_exp.append(np.mean(runs_best_values))
        data_exp.append(np.std(runs_best_values))

        _batch_queries, _batch_values = logger.get_batches()
        data_exp.append(logger.get_batch_size())
        if len(_batch_queries) > 0:
            batch_queries = np.array(_batch_queries)
            batch_values = np.array(_batch_values)
            # for b,v in zip(batch_queries, batch_values):
            #     print(b)
            
            batch_var_q, batch_var_v = distance_queries_batch((batch_queries, batch_values))
            # print("var")
            # print(batch_var_q)
            data_exp.append(np.mean(batch_var_q, axis=0))
            data_exp.append(np.std(batch_var_q, axis=0))
        else:
            data_exp.append(0.0)
            data_exp.append(0.0)

        data.append(data_exp)
    
    print("Function:", function_name)
    print("Num GO: " + str(nGO))
    print("GO value: " + str(go_value))
    print(
        tabulate(
            data,
            headers=["Optimizer", "FE", "PR", "SR", "AvgFEs", 
                    "Best value", "Avg. Best", "Std. Best",
                    "Batch size", "Var Batch (mean)", "Var Batch (std)"
                    ],
            floatfmt=".4f"
        )
    )



