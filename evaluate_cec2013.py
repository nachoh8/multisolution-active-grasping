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
RADIUS=None
MINIMIZE=False
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'g', '#ff7700', '#ff77ff', 'y' ]

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

def global_optima_found(queries: np.ndarray, values: np.ndarray, go_value: float, n_go: int):
    order = np.argsort(values)
    if not MINIMIZE:
        order = order[::-1]
    sorted_v = values[order]
    sorted_q = queries[order, :]
    radius = RADIUS
    
    seeds_idx = find_seeds_indices(sorted_q, radius) # get different global optimums

    count = 0
    goidx = []
    for idx in seeds_idx:
        # evaluate seed
        seed_fitness = sorted_v[idx]  # f.evaluate(sorted_pop[idx])

        # |F_seed - F_goptimum| <= accuracy
        if math.fabs(seed_fitness - go_value) <= ACCURACY:
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

def convergence_speed(queries: np.ndarray, values: np.ndarray, go_value: float, n_go: int):
    total_evals = values.shape[0]
    radius = RADIUS

    fe = -1
    go_idx = []
    for i in range(total_evals):
        q = queries[i]
        v = values[i]

        # |F_seed - F_goptimum| <= accuracy
        if math.fabs(v - go_value) <= ACCURACY:
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

    if fe > 0:
        return fe / total_evals, go_idx
    else:
        return 1.0, go_idx

def var_queries_batch(batches: "tuple[np.ndarray, np.ndarray]") -> "tuple[np.ndarray, np.ndarray]":
    queries, values = batches
    num_b = values.shape[0]

    var_q = np.zeros((num_b, queries.shape[2]))
    var_v = np.zeros(num_b)
    for i, qs, vs in zip(range(num_b), queries, values):
        var_q[i] = np.var(qs, axis=0)
        var_v[i] = np.var(vs)
    
    return var_q, var_v

def plot_var_batch(means: np.ndarray, stds: np.ndarray, var_names: "list[str]", optimizer_name: str):
    iterations = range(1, means.shape[0]+1)

    plt.figure()

    for i in range(means.shape[1]): # per dim
        means_it = means[:, i]
        plt.plot(iterations, means_it, label=var_names[i], color=COLORS[i])
        
        stds_it = stds[:, i]
        plt.fill_between(iterations, means_it - stds_it, means_it + stds_it, alpha=0.3, color=COLORS[i])

    plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Batch var')
    plt.title('Batch variance per dimension - ' + optimizer_name)

def compute_best_until_iteration(outcomes: np.ndarray) -> np.ndarray:
    res = np.array([outcomes[0]])

    for i in range(1, outcomes.shape[0]):
        if MINIMIZE:
            v = outcomes[i] if outcomes[i] < res[i-1] else res[i-1]
        else:
            v = outcomes[i] if outcomes[i] > res[i-1] else res[i-1]
        res = np.append(res, v)
    return res

def plot_outcome_iterations(outcomes: "list[tuple[np.ndarray, np.ndarray]]", names: "list[str]" = None, go: float = None):
    max_it = np.max([v[0].shape[0] for v in outcomes])
    iterations = range(1, max_it+1)

    plt.figure()

    if go != None:
        plt.plot(iterations, [go] * max_it, '--', label="Global optima")

    for i, v in enumerate(outcomes):
        mean_it = v[0]
        std_it = v[1]
        n = mean_it.shape[0]
        if n < max_it:
            mean_it = np.append(mean_it, np.full(max_it - n, mean_it[-1]))

        plt.plot(iterations, mean_it, label=names[i], color=COLORS[i])
        if std_it is not None:
            std_it = np.append(std_it, np.full(max_it - n, std_it[-1]))
            plt.fill_between(iterations, mean_it - std_it, mean_it + std_it, alpha=0.3, color=COLORS[i])
        
    if names:
        plt.legend()

    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title('Value of best selected sample - ' + OBJ_FUNCTION_NAME)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument("-acc", type=float, help="accuracy", metavar='<accuracy>', default=ACCURACY)
    parser.add_argument("-r", type=float, help="radius", metavar='<radius>', default=None)
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument('-no-plot', help='Minimize result', action='store_true')
    
    args = parser.parse_args()
    flogs = args.flogs
    ACCURACY = args.acc
    MINIMIZE = args.minimize
    RADIUS = args.r
    no_plot = args.no_plot

    print(70 * '=')

    table_data = []
    best_value_iterations = []
    for flog in flogs:
        if os.path.isdir(flog):
            logs = glob.glob(flog + "/*.json")
        else:
            logs = [flog]

        ### EXTRACT DATA

        num_runs = len(logs)

        _runs_queries = []
        _runs_values = []
        _runs_queries_best_it = []
        _runs_values_best_it = []
        _runs_batches_q = []
        _runs_batches_v = []
        _runs_best_queries = []
        _runs_best_values = []
        for log_file in logs:
            logger = DataLog(log_file=log_file)
            logger.load_json()

            _queries, _outcomes = logger.get_queries(minimize=MINIMIZE, best_per_iteration=False)
            queries = np.array(_queries)
            values = np.array(_outcomes).reshape(-1)
            
            _queries_best_it, _outcomes_best_it = logger.get_queries(minimize=MINIMIZE, best_per_iteration=True)
            queries_best_it = np.array(_queries_best_it)
            values_best_it = np.array(_outcomes_best_it).reshape(-1)

            _batch_queries, _batch_values = logger.get_batches()
            if len(_batch_queries) > 0:
                _runs_batches_q.append(np.array(_batch_queries))
                _runs_batches_v.append(np.array(_batch_values))

            _best = logger.get_best_queries()
            best_querys = np.array(_best[0])
            best_values = np.array(_best[1]).reshape(-1)

            _runs_queries.append(queries)
            _runs_values.append(values)
            _runs_queries_best_it.append(queries_best_it)
            _runs_values_best_it.append(values_best_it)
            _runs_best_queries.append(best_querys)
            _runs_best_values.append(best_values)

        runs_queries = np.array(_runs_queries)
        runs_values = np.array(_runs_values)
        runs_queries_best_it = np.array(_runs_queries_best_it)
        runs_values_best_it = np.array(_runs_values_best_it)
        runs_batches_q = np.array(_runs_batches_q)
        runs_batches_v = np.array(_runs_batches_v)
        runs_best_queries = np.array(_runs_best_queries)
        runs_best_values = np.array(_runs_best_values)
        
        ### ADD BASIC INFO

        data_exp = []
        data_exp.append(logger.get_optimizer_name())
        function_name = logger.obj_function["name"]
        obj_function = create_objective_function(function_name, "basic")

        total_evals_executed = runs_values.shape[1]
        data_exp.append(total_evals_executed)

        nGO = obj_function.get_num_global_optima()
        go_value = obj_function.get_global_optima()
        radius = obj_function.get_exclusion_radius()
        if RADIUS == None:
            RADIUS=radius

        ### COMPUTE Best value it, PK, SR, AvgFE

        runs_nGO = np.zeros(num_runs)
        runs_cspeed = np.zeros(num_runs)
        runs_best_value_it = np.zeros((num_runs, runs_values_best_it.shape[1]))
        for i in range(num_runs):
            go_q, go_v = global_optima_found(runs_queries[i], runs_values[i], go_value, nGO)
            runs_nGO[i] = go_v.shape[0]
            runs_cspeed[i], go_idx = convergence_speed(runs_queries[i], runs_values[i], go_value, nGO)
            runs_best_value_it[i] = compute_best_until_iteration(runs_values_best_it[i])
        
        mean_best_until_it = np.mean(runs_best_value_it, axis=0)
        std_best_until_it = np.std(runs_best_value_it, axis=0)
        best_value_iterations.append((mean_best_until_it, std_best_until_it))

        peak_ratio = np.sum(runs_nGO) / (nGO * num_runs)
        data_exp.append(peak_ratio)

        success_rate = np.count_nonzero(runs_nGO==nGO) / num_runs
        data_exp.append(success_rate)
        
        avg_conv_speed = np.mean(runs_cspeed)
        data_exp.append(avg_conv_speed)

        ### Solutions measure
        
        if MINIMIZE:
            data_exp.append(np.min(runs_best_values))
        else:
            data_exp.append(np.max(runs_best_values))
        data_exp.append(np.mean(runs_best_values))
        data_exp.append(np.std(runs_best_values))

        ### Variance Batch measure
        
        if len(runs_batches_q) > 0:
            data_exp.append(runs_batches_v.shape[2])
            
            runs_batch_var_q = np.array([var_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])

            mean_var_batch_it = np.mean(runs_batch_var_q, axis=0)
            std_var_batch_it = np.std(runs_batch_var_q, axis=0)
            if not no_plot:
                plot_var_batch(mean_var_batch_it, std_var_batch_it, logger.get_active_vars(), logger.get_optimizer_name())
            
            # TODO: per run -> var batch -> (n_iter, n_dim) -> variance mean/std of iterations (2, n_dim)
            # TODO: mean of means and mean of stds?
            _m_run = np.mean(runs_batch_var_q, axis=1)
            _m_dim = np.mean(_m_run, axis=0)
            data_exp.append(_m_dim)
            _s_run = np.std(runs_batch_var_q, axis=1)
            _s_dim = np.mean(_s_run, axis=0)
            data_exp.append(_s_dim)
        else:
            data_exp.append(1)
            data_exp.append(0.0)
            data_exp.append(0.0)

        table_data.append(data_exp)
    
    OBJ_FUNCTION_NAME=function_name
    print("Function:", OBJ_FUNCTION_NAME)
    print("Num GO: " + str(nGO))
    print("GO value: " + str(go_value))
    print("Accuracy:", ACCURACY)
    print("Radius:", RADIUS)
    print(
        tabulate(
            table_data,
            headers=["Optimizer", "FE", "PR", "SR", "AvgFEs", 
                    "Best value", "Avg. Best", "Std. Best",
                    "Batch size", "Var Batch (mean)", "Var Batch (std)"
                    ],
            floatfmt=".4f"
        )
    )

    
    if not no_plot:
        plot_outcome_iterations(best_value_iterations, names=[row[0] for row in table_data], go=go_value)
        plt.show()



