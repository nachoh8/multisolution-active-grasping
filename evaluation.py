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
METRIC="outcome"
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'b', '#ff7700', '#ff77ff', 'y' ]
ACTIVE_VARS=["x"]

class PlotData:
    def __init__(self, optimizer_name: str, total_evaluations: int) -> None:
        self.optimizer = optimizer_name
        self.total_evals = total_evaluations
        
        self.best_solution = 0.0
        self.avg_best = 0.0
        self.std_best = 0.0
        self.mean_best_value_iterations = np.zeros(1)
        self.std_best_value_iterations = np.zeros(1)

        self.peak_ratio = 0
        self.succes_rate = 0
        self.conv_speed = 1.0
        self.avg_global_optimas = 0

        self.batch_size = 1
        self.mean_var_batch = 0.0
        self.std_var_batch = 0.0

        self.num_solutions = 0
        self.var_solutions = 0.0
        self.var_outcome_solutions = 0.0
    
    def set_cec_metrics(self, PK: float, SR: float, CS: float, MGOs: float):
        self.peak_ratio = PK
        self.succes_rate = SR
        self.conv_speed = CS
        self.avg_global_optimas = MGOs
    
    def set_best_solution_metrics(self, best: float, avg_best: float, std_best: float, mean_best_value_iterations: np.ndarray, std_best_value_iterations: np.ndarray):
        self.best_solution = best
        self.avg_best = avg_best
        self.std_best = std_best
        self.mean_best_value_iterations = mean_best_value_iterations
        self.std_best_value_iterations = std_best_value_iterations
    
    def set_batch_metrics(self, batch_size: int, mean_var_batch: float, std_var_batch: float):
        self.batch_size = batch_size
        self.mean_var_batch = mean_var_batch
        self.std_var_batch = std_var_batch
    
    def set_solutions_metrics(self, num_solutions: int, var_solutions: float, var_outcome_solutions: float):
        self.num_solutions = num_solutions
        self.var_solutions = var_solutions
        self.var_outcome_solutions = var_outcome_solutions
    
    def get_data(self, data: "list[str]"):
        data_v = dict()

        data_v["Optimizer"] = self.optimizer
        data_v["FE"] = self.total_evals

        data_v["PR"] = self.peak_ratio
        data_v["SR"] = self.succes_rate
        data_v["CS"] = self.conv_speed
        data_v["MGOs"] = self.avg_global_optimas
        
        data_v["Best solution"] = self.best_solution
        data_v["Avg. Best"] = self.avg_best
        data_v["Std. Best"] = self.std_best

        data_v["Batch size"] = self.batch_size
        data_v["Var Batch (mean)"] = self.mean_var_batch
        data_v["Var Batch (std)"] = self.std_var_batch

        data_v["Num. sols."] = self.num_solutions
        data_v["Solutions var. (mean)"] = self.var_solutions
        data_v["Solutions outcome (mean)"] = self.var_outcome_solutions

        return [data_v[idx] for idx in data]


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

    max_y = -99999999999999999999
    min_y = 99999999999999999999
    for i, v in enumerate(outcomes):
        mean_it = v[0]
        std_it = v[1]
        n = mean_it.shape[0]

        _min_v = np.min(mean_it)
        _max_v = np.max(mean_it)
        if _min_v < min_y:
            min_y = _min_v
        if _max_v < max_y:
            max_y = _max_v
        # if n < max_it:
        #     mean_it = np.append(mean_it, np.full(max_it - n, mean_it[-1]))
        plt.plot(range(1, n+1), mean_it, label=names[i], color=COLORS[i])
        if std_it is not None:
            std_minus = mean_it - std_it
            std_plus = mean_it + std_it
            plt.fill_between(range(1, n+1), std_minus, std_plus, alpha=0.3, color=COLORS[i])
            _min_v = np.min(std_minus)
            _max_v = np.max(std_plus)
            if _min_v < min_y:
                min_y = _min_v
            if _max_v < max_y:
                max_y = _max_v
            # std_it = np.append(std_it, np.full(max_it - n, std_it[-1]))
            # plt.fill_between(iterations, mean_it - std_it, mean_it + std_it, alpha=0.3, color=COLORS[i])
        
    if names:
        plt.legend()
    
    """if go is not None:
        pass
        if MINIMIZE:
            _min = min_y if min_y < go else go
            plt.ylim((_min - abs(_min) * 0.1, max_y))
        else:
            _max = max_y if max_y > go else go
            _min = _max * 0.25 if min_y < _max * 0.0 else min_y
            plt.ylim((_min, _max + abs(_max) * 0.1))
    else:
        pass"""
    #elif go != None and not MINIMIZE:
    #    plt.ylim((go-1))
    plt.xlabel('Iteration')
    plt.ylabel(METRIC)
    plt.title(OBJ_FUNCTION_NAME)
    # plt.title('Value of best selected sample - ' + OBJ_FUNCTION_NAME)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument("-acc", type=float, help="accuracy over GO (%)", metavar='<accuracy>', default=ACCURACY)
    parser.add_argument("-r", type=float, help="radius", metavar='<radius>', default=None)
    parser.add_argument("-metric", type=str, help="metric", metavar='<metric>', default=METRIC)
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument('-no-plot', help='No show plots', action='store_true')
    parser.add_argument('-best', help='Compute best solution metrics', action='store_true')
    parser.add_argument('-cec', help='Compute cec metrics', action='store_true')
    parser.add_argument('-batch', help='Compute batch metrics', action='store_true')
    parser.add_argument('-sols', help='Compute solutions metrics', action='store_true')

    
    args = parser.parse_args()
    flogs = args.flogs
    no_plot = args.no_plot

    best_metrics=args.best
    cec_metrics=args.cec
    batch_metrics=args.batch
    sols_metrics=args.sols
    
    MINIMIZE = args.minimize
    METRIC=args.metric
    
    RADIUS = args.r

    obj_function = None

    print(70 * '=')

    table_data: "list[PlotData]" = []
    go_value = None

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

            _queries, _outcomes = logger.get_queries(minimize=MINIMIZE, best_per_iteration=False, metric=METRIC)
            queries = np.array(_queries)
            values = np.array(_outcomes).reshape(-1)
            
            _queries_best_it, _outcomes_best_it = logger.get_queries(minimize=MINIMIZE, best_per_iteration=True, metric=METRIC)
            queries_best_it = np.array(_queries_best_it)
            values_best_it = np.array(_outcomes_best_it).reshape(-1)

            _batch_queries, _batch_values = logger.get_batches(metric=METRIC)
            if len(_batch_queries) > 0:
                _runs_batches_q.append(np.array(_batch_queries))
                _runs_batches_v.append(np.array(_batch_values))

            _best = logger.get_best_queries(metric=METRIC)
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

        function_name = logger.obj_function["name"]
        
        total_evals_executed = runs_values.shape[1]
        data_exp = PlotData(logger.get_optimizer_name(), total_evals_executed)

        ### BEST SOLUTION FOUND METRIC
        if best_metrics:
            runs_best_value_it = np.zeros((num_runs, runs_values_best_it.shape[1]))
            for i in range(num_runs):
                runs_best_value_it[i] = compute_best_until_iteration(runs_values_best_it[i])
            
            mean_best_until_it = np.mean(runs_best_value_it, axis=0)
            std_best_until_it = np.std(runs_best_value_it, axis=0)

            if MINIMIZE:
                best_run = np.array([np.min(r) for r in runs_best_values])
                best_sol = np.min(best_run)
            else:
                best_run = np.array([np.max(r) for r in runs_best_values])
                best_sol = np.max(best_run)
            avg_best = np.mean(best_run)
            std_best = np.std(best_run)

            data_exp.set_best_solution_metrics(best_sol, avg_best, std_best, mean_best_until_it, std_best_until_it)

        ### CEC13 METRICS

        if cec_metrics:
            if obj_function is None:
                obj_function = create_objective_function(function_name, "basic" if METRIC == "outcome" else METRIC)
            ACTIVE_VARS=obj_function.get_var_names()

            nGO = obj_function.get_num_global_optima()
            go_value = obj_function.get_global_optima()
            radius = obj_function.get_exclusion_radius()
            if RADIUS == None:
                RADIUS=radius

            if go_value == 0.0:
                ACCURACY = 1.0 - args.acc
            else:
                ACCURACY = abs(go_value) * (1.0 - args.acc)
            
            runs_mean_go = np.zeros(num_runs)
            runs_nGO = np.zeros(num_runs)
            runs_cspeed = np.zeros(num_runs)
            true_go_points = obj_function.get_global_optima_points()
            
            for i in range(num_runs):
                if true_go_points is None:
                    go_q, go_v = global_optima_found(runs_queries[i], runs_values[i], go_value, nGO)
                    runs_cspeed[i], _ = convergence_speed(runs_queries[i], runs_values[i], go_value, nGO)
                else:
                    go_q, go_v, conv_speed = get_global_optimas(runs_queries[i], runs_values[i], true_go_points, go_value)
                    runs_cspeed[i] = conv_speed
                
                n_go = go_v.shape[0]
                runs_mean_go[i] = np.mean(go_v) if n_go > 0 else 0.0
                runs_nGO[i] = n_go
            
            total_go_found = np.sum(runs_nGO)
            peak_ratio = total_go_found / (nGO * num_runs)

            success_rate = np.count_nonzero(runs_nGO==nGO) / num_runs
            
            avg_conv_speed = np.mean(runs_cspeed)

            if total_go_found > 0:
                comb_mean = np.sum(runs_mean_go * runs_nGO) / total_go_found
            else:
                comb_mean = '-'

            data_exp.set_cec_metrics(peak_ratio, success_rate, avg_conv_speed, comb_mean)
        else:
            ACTIVE_VARS=logger.get_active_vars()

        ### BATCH METRICS
        
        if batch_metrics and len(runs_batches_q) > 0:
            batch_size = runs_batches_v.shape[2]
            
            runs_batch_var_q = np.array([var_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])

            mean_var_batch_it = np.mean(runs_batch_var_q, axis=0)
            std_var_batch_it = np.std(runs_batch_var_q, axis=0)
            if not no_plot:
                plot_var_batch(mean_var_batch_it, std_var_batch_it, logger.get_active_vars(), logger.get_optimizer_name())
            
            # TODO: per run -> var batch -> (n_iter, n_dim) -> variance mean/std of iterations (2, n_dim)
            # TODO: mean of means and mean of stds?
            _m_run = np.mean(runs_batch_var_q, axis=1)
            _m_dim = np.mean(_m_run, axis=0)
            
            _s_run = np.std(runs_batch_var_q, axis=1)
            _s_dim = np.mean(_s_run, axis=0)

            # runs_batch_dist_q = np.array([distance_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])
            # _m_run = np.mean(runs_batch_dist_q, axis=1)
            # data_exp.append(np.mean(_m_run))
            # data_exp.append(np.std(_m_run))

            data_exp.set_batch_metrics(batch_size, _m_dim, _s_dim)
        
        ### MULTISOLUTION METRICS
        
        if sols_metrics:
            n_solutions = np.array([vs.shape[0] for vs in runs_best_values])

            sq_var = np.array([np.var(rq, axis=0) for rq in runs_best_queries])
            sqvar_mean = np.mean(sq_var, axis=0)

            sv_mean = np.array([np.mean(rv) for rv in runs_best_values])
            sv_mean = np.mean(sv_mean)

            data_exp.set_solutions_metrics(np.mean(n_solutions), sqvar_mean, sv_mean)

        table_data.append(data_exp)
    
    OBJ_FUNCTION_NAME=function_name
    print("Function:", OBJ_FUNCTION_NAME)
    
    info_table=["Optimizer", "FE"]
    if best_metrics:
        info_table += ["Best solution", "Avg. Best", "Std. Best",]

    if cec_metrics:
        info_table += ["PR", "SR", "CS", "MGOs"]

        print("Num GO: " + str(nGO))
        print("GO value: " + str(go_value))
        print("Accuracy:", ACCURACY)
        print("Radius:", RADIUS)
    
    if batch_metrics:
        info_table += ["Batch size", "Var Batch (mean)", "Var Batch (std)"]
    
    if sols_metrics:
        info_table += ["Num. sols.", "Solutions var. (mean)", "Solutions outcome (mean)"]
    
    print(
        tabulate(
            [t_data.get_data(info_table) for t_data in table_data],
            headers=[h for h in info_table],
            floatfmt=".4f"
        )
    )

    
    if best_metrics and not no_plot:
        plot_outcome_iterations(
            [(t_data.mean_best_value_iterations, t_data.std_best_value_iterations) for t_data in table_data],
            names=[t_data.optimizer for t_data in table_data], go=go_value
            )
    
    if not no_plot:
        plt.show()



