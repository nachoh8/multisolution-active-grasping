import numpy as np
import math
import argparse
import os
import glob
import matplotlib.pyplot as plt
from tabulate import tabulate
from itertools import combinations

from multisolution_active_grasping.core.datalog import DataLog
from multisolution_active_grasping.utils.utils import create_objective_function

from kmeans import compute as kmeans

ACCURACY = 0.95
RADIUS=None
MINIMIZE=False
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'b', '#ff7700', '#ff77ff', 'y' ]
ACTIVE_VARS=["x"]

TABLE_HEADERS=[
    "Optimizer", "FE", "PR", "SR", "CS", 
    "Best solutions", "Avg. Best", "Std. Best",
    "Avg. GO",
    "Batch size", "Var Batch (mean)", "Var Batch (std)",
    "Dist Batch (mean)", "Dist Batch (std)",
    "Solutions var. (mean)", "Solutions outcome. (mean)"
]

INFO_TABLE=[0,1,2,3,4,6,7,8,10,11, 12, 13]

euclidean_distance = lambda x1, x2: np.sqrt(np.sum((x1 - x2) ** 2))

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

def get_global_optimas(queries: np.ndarray, values: np.ndarray, go: np.ndarray, go_value: float):
    total_evals = queries.shape[0]
    go_found = [(None, None) for _ in go] # [(pt, v)]
    fe = 0
    for q, v, i in zip(queries, values, range(total_evals)):
        if math.fabs(v - go_value) <= ACCURACY:
            min_dist = np.array([euclidean_distance(q, g_i) for g_i in go])
            min_idx = np.argmin(min_dist)
            if go_found[min_idx][0] is None:
                go_found[min_idx] = (q, v)
                fe = i + 1
            else:
                _gv = go_found[min_idx][1]
                if (MINIMIZE and v < _gv) or ((not MINIMIZE) and v > _gv):
                    go_found[min_idx] = (q, v)
                    fe = i + 1

    go_q = np.array([q for q,_ in go_found if q is not None])
    go_v = np.array([v for _, v in go_found if v is not None])
    conv_speed = fe / float(total_evals) if go_q.shape[0] == go.shape[0] else 1.0

    return go_q, go_v, conv_speed

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

def distance_queries_batch(batches: "tuple[np.ndarray, np.ndarray]") -> "tuple[np.ndarray, np.ndarray]":
    queries, values = batches
    num_b = queries.shape[0]
    num_q = queries.shape[1]

    var_q = np.zeros(num_b)
    var_v = np.zeros(num_b)
    for i, qs, vs in zip(range(num_b), queries, values):
        d = 0.0
        n = 0
        for q1, q2 in combinations(range(num_q), 2):
            d += np.linalg.norm(qs[q1] - qs[q2])
            n += 1
        var_q[i] = d / float(n)
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
        # if n < max_it:
        #     mean_it = np.append(mean_it, np.full(max_it - n, mean_it[-1]))
        plt.plot(range(1, n+1), mean_it, label=names[i], color=COLORS[i])
        if std_it is not None:
            std_minus = mean_it - std_it
            std_plus = mean_it + std_it
            plt.fill_between(range(1, n+1), std_minus, std_plus, alpha=0.3, color=COLORS[i])
            # std_it = np.append(std_it, np.full(max_it - n, std_it[-1]))
            # plt.fill_between(iterations, mean_it - std_it, mean_it + std_it, alpha=0.3, color=COLORS[i])
        
    if names:
        plt.legend()
    plt.ylim((go-1, 10))
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(OBJ_FUNCTION_NAME)
    # plt.title('Value of best selected sample - ' + OBJ_FUNCTION_NAME)

def plot_solutions(CX, CY, SX, SY, tittle, value_title):
    plot2d = False

    ndim = CX[0].shape[1]
    fig = plt.figure()
    if ndim >= 2 and not plot2d:
        ax = fig.add_subplot(projection='3d')
    for cx, cy, sx, sy in zip(CX, CY, SX, SY):
        if ndim == 1:
            sc_plot = plt.scatter(cx, cy, alpha=0.5)
            plt.scatter(sx, sy, c='k')
            plt.xlabel(ACTIVE_VARS[0])
            plt.ylabel(value_title)
        elif ndim == 2 and plot2d:
            sc_plot = plt.scatter(cx[:, 0], cx[:, 1], alpha=0.5)
            plt.xlabel(ACTIVE_VARS[0])
            plt.ylabel(ACTIVE_VARS[1])
        else:
            if ndim == 2:
                ax.scatter(cx[:, 0], cx[:, 1], cy, alpha=0.5)
                ax.scatter(sx[0], sx[1], sy, c='k')
                ax.set_zlabel(value_title)
            else:
                ax.scatter(cx[:, 0], cx[:, 1], cx[:, 2], alpha=0.5)
                ax.scatter(sx[0], sx[1], sx[2], c='k')
                ax.set_zlabel(ACTIVE_VARS[2])

            ax.set_xlabel(ACTIVE_VARS[0])
            ax.set_ylabel(ACTIVE_VARS[1])
    
    # plt.colorbar(sc_plot, label=value_title, orientation="vertical")
    plt.title(tittle)

def normalize(v, max, min):
    return (v-min) / (max-min)

def unnormalize(v, max, min):
    return v * (max-min) + min

def get_solutions(num_solutions: int, min_outcome: float, queries: np.ndarray, values: np.ndarray, lb: np.ndarray, ub: np.ndarray, name: str):
    ndim = queries.shape[1]

    min_idx = np.argmin(values)
    max_idx = np.argmax(values)
    Ymin = values[min_idx]
    Ymax = values[max_idx]

    Qs = (queries - lb) / (ub - lb)
    Ys = (values - Ymin) / (Ymax - Ymin)

    if Ymax < 0:
        Yminv = (Ymax*(1 + 1 - min_outcome) - Ymin) / (Ymax - Ymin)
    else:
        Yminv = (Ymax*min_outcome - Ymin) / (Ymax - Ymin)
    
    idxs = Ys >= Yminv
    Yv = values[idxs]
    Qs = queries[idxs]
    # print("Max:", Ymax, "Min value:", Yminv)#, normalize(Yminv, Ymax, Ymin), unnormalize(0.7, Ymax, Ymin))
    # print("Num candidates:", Qs.shape[0])
    CX, CY = kmeans(Qs, Yv, num_solutions)
    SX = np.zeros((num_solutions, ndim))
    SY = np.zeros(num_solutions)
    # print("Solutions")
    for cx, cy, i in zip(CX, CY, range(len(CY))):
        max_idx = np.argmax(cy)
        SX[i] = cx[max_idx]
        SY[i] = cy[max_idx]
        # print(SX[i], SY[i])
    
    # plot_solutions(CX, CY, SX, SY, "Clusters - " + name, "y")
    return SX, SY

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument("-acc", type=float, help="accuracy over GO (%)", metavar='<accuracy>', default=ACCURACY)
    parser.add_argument("-r", type=float, help="radius", metavar='<radius>', default=None)
    parser.add_argument("-nsols", type=int, help="num solutions", metavar='<num_solutions>', default=4)
    parser.add_argument("-minv", type=float, help="min optimum value", metavar='<min_optimum>', default=0.7)
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument('-no-plot', help='No show plots', action='store_true')

    
    args = parser.parse_args()
    flogs = args.flogs
    MINIMIZE = args.minimize
    RADIUS = args.r
    num_solutions = args.nsols
    min_value = args.minv
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
        ACTIVE_VARS=obj_function.get_var_names()

        total_evals_executed = runs_values.shape[1]
        data_exp.append(total_evals_executed)

        nGO = obj_function.get_num_global_optima()
        go_value = obj_function.get_global_optima()
        radius = obj_function.get_exclusion_radius()
        if RADIUS == None:
            RADIUS=radius

        if go_value == 0.0:
            ACCURACY = 1.0 - args.acc
        else:
            ACCURACY = abs(go_value) * (1.0 - args.acc)

        ### COMPUTE Best value it, PK, SR, AvgFE

        runs_mean_go = np.zeros(num_runs)
        runs_nGO = np.zeros(num_runs)
        runs_cspeed = np.zeros(num_runs)
        runs_best_value_it = np.zeros((num_runs, runs_values_best_it.shape[1]))
        true_go_points = obj_function.get_global_optima_points()
        # print(flog)
        for i in range(num_runs):
            if true_go_points is None:
                go_q, go_v = global_optima_found(runs_queries[i], runs_values[i], go_value, nGO)
                runs_cspeed[i], _ = convergence_speed(runs_queries[i], runs_values[i], go_value, nGO)
            else:
                go_q, go_v, conv_speed = get_global_optimas(runs_queries[i], runs_values[i], true_go_points, go_value)
                runs_cspeed[i] = conv_speed
            # print(i)
            # print(go_q)
            """for gq, gv in zip(go_q, go_v):
                print(gq, gv)"""
            
            n_go = go_v.shape[0]
            runs_mean_go[i] = np.mean(go_v) if n_go > 0 else 0.0
            runs_nGO[i] = n_go
            runs_best_value_it[i] = compute_best_until_iteration(runs_values_best_it[i])
        
        mean_best_until_it = np.mean(runs_best_value_it, axis=0)
        std_best_until_it = np.std(runs_best_value_it, axis=0)
        best_value_iterations.append((mean_best_until_it, std_best_until_it))

        total_go_found = np.sum(runs_nGO)
        peak_ratio = total_go_found / (nGO * num_runs)
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

        # combined mean of go
        
        if total_go_found > 0:
            comb_mean = np.sum(runs_mean_go * runs_nGO) / total_go_found
        else:
            comb_mean = '-'
        data_exp.append(comb_mean)

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

            runs_batch_dist_q = np.array([distance_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])
            _m_run = np.mean(runs_batch_dist_q, axis=1)
            data_exp.append(np.mean(_m_run))
            data_exp.append(np.std(_m_run))
        else:
            data_exp.append(1)
            data_exp.append(0.0)
            data_exp.append(0.0)
            data_exp.append(0.0)
            data_exp.append(0.0)

        # Get optimum solutions
        
        """SQ = []
        SV = []
        lb = np.array(obj_function.get_lower_bounds())
        ub = np.array(obj_function.get_upper_bounds())
        for rq, rv in zip(runs_queries, runs_values):
            Sq, Sv = get_solutions(num_solutions, min_value, rq, rv, lb, ub, logger.get_optimizer_name())
            SQ.append(Sq)
            SV.append(Sv)
        
        SQ = np.array(SQ)
        SV = np.array(SV)

        sq_var = np.array([np.var(rq, axis=0) for rq in SQ])
        sqvar_mean = np.mean(sq_var, axis=0)
        data_exp.append(sqvar_mean)

        sv_mean = np.array([np.mean(rv) for rv in SV])
        sv_mean = np.mean(sv_mean)
        data_exp.append(sv_mean)"""

        table_data.append(data_exp)
    
    OBJ_FUNCTION_NAME=function_name
    print("Function:", OBJ_FUNCTION_NAME)
    print("Num GO: " + str(nGO))
    print("GO value: " + str(go_value))
    print("Accuracy:", ACCURACY)
    print("Radius:", RADIUS)
    # print("Num. solutions:", num_solutions)
    # print("Min. value (%):", min_value)
    print(
        tabulate(
            [[data[i] for i in INFO_TABLE] for data in table_data],
            headers=[TABLE_HEADERS[i] for i in INFO_TABLE],
            floatfmt=".4f"
        )
    )

    
    if not no_plot:
        plot_outcome_iterations(best_value_iterations, names=[row[0] for row in table_data], go=go_value)
    plt.show()



