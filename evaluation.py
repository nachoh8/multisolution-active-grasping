import numpy as np
import math
import argparse
import os
import glob
import matplotlib.pyplot as plt
from tabulate import tabulate
from itertools import combinations

from OptSystem.core.datalog import DataLog
from OptSystem.utils.utils import create_objective_function

ACCURACY = 0.95
RADIUS=None
MINIMIZE=False
METRIC="outcome"
OBJ_FUNCTION_NAME=""
COLORS=['b', 'magenta'] # so-ms, robot
COLORS=['k', 'r', 'gold', 'g'] # mebo: k --, e-mebo: r -, cl-mebo: gold :
LINESTYLE=["-", "-", "-"]
ACTIVE_VARS=["x"]

class PlotData:
    def __init__(self, optimizer_name: str, total_evaluations: int, total_runs: int, mean_exec_time: float = 0.0) -> None:
        self.optimizer = optimizer_name
        self.total_evals = total_evaluations
        self.total_runs = total_runs
        self.mean_exec_time = mean_exec_time
        
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
        self.mean_var_batch = []
        self.std_var_batch = []

        self.mean_num_solutions = 0
        self.std_num_solutions = 0
        self.mean_var_solutions = []
        self.mean_dist_solutions = 0.0
        self.std_dist_solutions = 0.0
        self.mean_outcome_solutions = 0.0
        self.std_outcome_solutions = 0.0

        self.no_valid_queries = 0.0
        self.no_valid_queries_std = 0.0

        self.small_occ_best = 0.0
        self.small_occ_best_std = 0.0
        self.large_occ_best = 0.0
        self.large_occ_best_std = 0.0
    
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
    
    def set_solutions_metrics(self, mean_num_solutions: float, std_num_solutions: float, mean_var_solutions: float, mean_dist_solutions: float, std_dist_solutions: float, mean_outcome_solutions: float, std_outcome_solutions: float):
        self.mean_num_solutions = mean_num_solutions
        self.std_num_solutions = std_num_solutions
        self.mean_var_solutions = mean_var_solutions
        self.mean_dist_solutions = mean_dist_solutions
        self.std_dist_solutions = std_dist_solutions
        self.mean_outcome_solutions = mean_outcome_solutions
        self.std_outcome_solutions = std_outcome_solutions
    
    def set_no_valid_metrics(self, num_no_valid_queries: float, num_no_valid_queries_std: float):
        self.no_valid_queries = num_no_valid_queries
        self.no_valid_queries_std = num_no_valid_queries_std
    
    def set_occlusion_metrics(self, _small_occ_best: float, _small_occ_best_std: float, _large_occ_best: float, _large_occ_best_std: float):
        self.small_occ_best = _small_occ_best
        self.small_occ_best_std = _small_occ_best_std
        self.large_occ_best = _large_occ_best
        self.large_occ_best_std = _large_occ_best_std

    def _format_float(self, n: float) -> str:
        return "{:.4f}".format(n)

    def get_data(self, data: "list[str]"):
        data_v = dict()

        data_v["Optimizer"] = self.optimizer
        data_v["#Runs"] = self.total_runs
        data_v["FE"] = self.total_evals

        t = int(self.mean_exec_time)
        h = t // 3600
        m = (t % 3600) // 60
        s = t % 60

        h_str = str(h).zfill(2)
        m_str = str(m).zfill(2)
        s_str = str(s).zfill(2)

        data_v["T"] = h_str + ':' + m_str + ':' + s_str

        data_v["0-Value"] = self._format_float(self.no_valid_queries) + " \u00B1 " + self._format_float(self.no_valid_queries_std)

        data_v["PR"] = self.peak_ratio
        data_v["SR"] = self.succes_rate
        data_v["CS"] = self.conv_speed
        data_v["MGOs"] = self.avg_global_optimas
        
        data_v["Best solution"] = self.best_solution
        data_v["Avg. Best"] = self._format_float(self.avg_best) + " \u00B1 " + self._format_float(self.std_best)
        # data_v["Std. Best"] = self.std_best

        data_v["Batch size"] = self.batch_size
        data_v["Var Batch (mean)"] = [float(self._format_float(v)) for v in self.mean_var_batch]
        data_v["Var Batch (std)"] = [float(self._format_float(v)) for v in self.std_var_batch]

        data_v["Num. sols. (mean)"] = self._format_float(self.mean_num_solutions) + " \u00B1 " + self._format_float(self.std_num_solutions)
        data_v["Solutions var. (mean)"] = [float(self._format_float(v)) for v in self.mean_var_solutions]
        data_v["Solutions D metric (mean)"] = self._format_float(self.mean_dist_solutions) + " \u00B1 " + self._format_float(self.std_dist_solutions)
        data_v["Solutions Q metric (mean)"] = self._format_float(self.mean_outcome_solutions) + " \u00B1 " + self._format_float(self.std_outcome_solutions)

        data_v["S-OCC"] = self._format_float(self.small_occ_best) + " \u00B1 " + self._format_float(self.small_occ_best_std)
        data_v["L-OCC"] = self._format_float(self.large_occ_best) + " \u00B1 " + self._format_float(self.large_occ_best_std)

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
        var_v[i] = np.mean(vs)
    
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
        plt.plot(range(1, n+1), mean_it, LINESTYLE[i], label=names[i], color=COLORS[i])
        if std_it is not None:
            std_minus = mean_it - std_it
            std_plus = mean_it + std_it
            plt.fill_between(range(1, n+1), std_minus, std_plus, alpha=0.4, color=COLORS[i], linewidth=2, linestyle=LINESTYLE[i])
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
    # bottle -> pos: plt.ylim([0.35,0.92]), pose: plt.ylim([0.25,0.84])
    # trophy -> pos: plt.ylim([0.04,0.34]), pose: plt.ylim([0.12,0.42])
    # drill -> pos: plt.ylim([0.04,0.34]), pose: plt.ylim([0.08,0.26])
    plt.ylim([0.08,0.245])
    plt.xlabel('Function Evaluations')
    plt.ylabel(METRIC)
    plt.title("Best grasp found")
    # plt.title('Value of best selected sample - ' + OBJ_FUNCTION_NAME)

def weitzman_metric(solutions: "np.ndarray", S: set) -> float:
    """
    Weitzman, M.L.(1992). On diversity. Quarterly Journal of Economics
    """
    n_solutions = len(S)
    if n_solutions == 1:
        return 0.0, 0.0

    w_metrics = []
    for s in S:
        dist = []
        S_s = S-{s}
        for si in S_s:
            _d = np.linalg.norm(solutions[si] - solutions[s])
            dist.append(_d)
        
        w = np.min(dist) + weitzman_metric(solutions, S_s)[0]

        w_metrics.append(w)

    return np.min(w_metrics), np.std(w_metrics)

def solutions_diversity_metric(solutions_orig: "np.ndarray", metric_type = 0, cluster_sols = False, mind = None) -> float:
    if solutions_orig.shape[0] <= 1:
        return [[0],[]], 0.0, 0.0
    
    if cluster_sols: # cluster grasps and mean dist (with 20% object size or search space on height axis)
        """n_dist_sols = solutions_orig.shape[0]+1
        sols_final = None
        sols_joined = None

        for start in range(solutions_orig.shape[0]):
            s = [start] + [i for i in range(solutions_orig.shape[0]) if i != start]
            print("s", s)
            
            _sols_final = []
            _sols_joined = []
            _n_dist_sols = 0
            while len(s) > 0:
                si = s[0]
                found = [si]
                for sj in s:
                    if sj == si: continue
                    d = np.linalg.norm(solutions_orig[si] - solutions_orig[sj])
                    if d < mind:
                        found.append(sj)
                
                n = len(found)
                print(n, found)
                if n > 1:
                    pt_center = np.sum(solutions_orig[found], axis=0) / n
                    _sols_final += [pt_center for _ in found]
                    _sols_joined.append(found)
                else:
                    _sols_final.append(solutions_orig[si])
                # s = s - set(found)
                for f in found:
                    s.remove(f)
                _n_dist_sols += 1
            
            if _n_dist_sols < n_dist_sols:
                n_dist_sols = _n_dist_sols
                sols_final = _sols_final
                sols_joined = _sols_joined"""
        n_sols = solutions_orig.shape[0]
        valid_sols = set()
        novalid_sols = set()
        for s in range(n_sols):
            s_pt = solutions_orig[s]
            _nsls = []
            for s2 in range(n_sols):
                if s == s2: continue
                if np.linalg.norm(s_pt - solutions_orig[s2]) < mind:
                    if s < s2 and s not in _nsls:
                        _nsls.append(s)
                    _nsls.append(s2)
            if len(_nsls) > 0:
                if s not in _nsls:
                    _nsls.append(s)
                novalid_sols.add(tuple(_nsls))
            else:
                valid_sols.add(s)
        to_join_sols = set()
        for s in range(n_sols):
            if s in valid_sols: continue
            lmax = 0
            _sl = None
            for nv in novalid_sols:
                if s in nv:
                    l = len(nv)
                    if l > lmax:
                        _sl = nv
                        lmax = l
            to_join_sols.add(_sl)
        
        sols_valid = list(valid_sols)
        sols_joined = [list(jsls) for jsls in to_join_sols] 
        solutions = [solutions_orig[s] for s in valid_sols]
        for jsls in sols_joined:
            solutions += [np.sum(solutions_orig[jsls], axis=0) / len(jsls) for _ in jsls]
        # n_dist_sols = len(valid_sols)
        solutions = np.array(solutions)
    else:
        solutions = solutions_orig
        # n_dist_sols = solutions.shape[0]
        sols_valid = [i for i in range(4)]
        sols_joined = []
    
    n_solutions = solutions.shape[0]
    
    if metric_type == 0: # mean euclidean distance
        dist = []
        for q1, q2 in combinations(range(n_solutions), 2):
            d = np.linalg.norm(solutions[q1] - solutions[q2])
            dist.append(d)
        dist = np.array(dist)

        std_d_metric = np.std(dist)
        d_metric = np.mean(dist)
    
    elif metric_type == 1: # mean of min Sum of Distances to Nearest Neighbor (SDNN), with N=1
        """dist = np.zeros(n_solutions)
        d_matrix = np.zeros((n_solutions, n_solutions))
        for i in range(n_solutions):
            _dist = []
            for j in range(n_solutions):
                if i == j:
                    d_matrix[i, j] = 0.0
                    continue
                _d = np.linalg.norm(solutions[i] - solutions[j])
                d_matrix[i, j] = _d
                _dist.append(_d)
            
            _dist = np.array(_dist)
            # print(_dist)
            dist[i] = np.min(_dist)
        # print(d_matrix)"""
        m1 = np.hstack([solutions]*n_solutions).reshape(n_solutions,n_solutions,-1)
        m2 = np.vstack([solutions]*n_solutions).reshape(n_solutions,n_solutions,-1)
        distM = np.linalg.norm(m1-m2, axis=2)
        dist = np.array([np.min(distM[i,list(set(range(n_solutions))-set([i]))]) for i in range(distM.shape[0])])
        # print(dist)
        # d_metric = 2 * np.min(dist) + np.mean(dist)
        std_d_metric = np.std(dist)
        d_metric = np.mean(dist)#  + np.min(dist) * (np.min(dist) / np.max(dist))
        _min = np.min(dist)
        _max = np.max(dist)
        if _min > 0.0 and _max > 0.0:
            d_metric += _min * (_min / _max)
    
    elif metric_type == 2: # Weitzman
        d_metric, std_d_metric = weitzman_metric(solutions, set([i for i in range(n_solutions)]))
    
    elif metric_type == 3:
        d_metric = np.var(solutions[:, :3], axis=0)
        std_d_metric = 0.0
    elif metric_type == 4:
        shortest = 0.0
        for path in combinations(range(n_solutions), ):
            d = np.linalg.norm(solutions[q1] - solutions[q2])


    return (sols_valid,sols_joined), d_metric, std_d_metric

def solutions_quality_metric(q_solutions: "np.ndarray", metric_type = 0, minimize = False) -> float:
    n_solutions = q_solutions.shape[0]

    if n_solutions <= 1:
        return q_solutions[0], 0.0
    
    if metric_type == 0: # mean quality
        q_metric = np.mean(q_solutions)
        std_q_metric = np.std(q_solutions)
    
    elif metric_type == 1: # mean quality + min quality
        q_metric = np.mean(q_solutions)
        if minimize:
            q_metric += np.max(q_solutions)
        else:
            q_metric += np.min(q_solutions)
            
        std_q_metric = np.std(q_solutions)

    return q_metric, std_q_metric


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
    parser.add_argument("-sols", nargs='+', help="multisolutions metrics", metavar='<metric>, <diversity_lvl>', default=None)
    parser.add_argument("-occ_metric", nargs=2, help="occlusion metric: less than z1, greather than z2", metavar='<z_lt> <z_gt>', default=None)
    
    args = parser.parse_args()
    flogs = args.flogs
    no_plot = args.no_plot

    best_metrics=args.best
    cec_metrics=args.cec
    batch_metrics=args.batch
    if args.sols is None:
        sols_metrics=False
        sols_div_metric=0
        sols_diversity=None
    else:
        sols_metrics=True
        sols_div_metric=int(args.sols[0])
        if args.sols[1] != "":
            sols_diversity=float(args.sols[1])
        else:
            sols_diversity=None
    
    if args.occ_metric is None:
        occ_ranges = (None,None)
    else:
        occ_ranges = (float(args.occ_metric[0]), float(args.occ_metric[1])) # grasp[z < occ_ranges[0] || z > occ_ranges[1]]
    
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
        _runs_exec_time = []
        for log_file in logs:
            # print(log_file)
            logger = DataLog(log_file=log_file)
            logger.load_json()

            _runs_exec_time.append(logger.get_execution_time())

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

        runs_exec_time = np.array(_runs_exec_time)
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
        data_exp = PlotData(logger.get_optimizer_name(), total_evals_executed, len(logs), mean_exec_time=np.mean(runs_exec_time))
        if function_name == "GraspPlanner" or function_name == "GP" or function_name == "EigenGraspPlanner" or function_name == "EGP": # is grasp experiment
            m0 = np.count_nonzero(runs_values==0.0, axis=1) / total_evals_executed
            data_exp.set_no_valid_metrics(np.mean(m0), np.std(m0))

        ### BEST SOLUTION FOUND METRIC
        if best_metrics:
            runs_best_value_it = np.zeros((num_runs, runs_values_best_it.shape[1]))
            _bests = np.zeros(num_runs)
            for i in range(num_runs):
                runs_best_value_it[i] = compute_best_until_iteration(runs_values_best_it[i])
                _bests[i] = runs_best_value_it[i][-1]
            mean_best_until_it = np.mean(runs_best_value_it, axis=0)
            std_best_until_it = np.std(runs_best_value_it, axis=0)

            if MINIMIZE:
                best_run = np.array([np.min(r) for r in _bests])
                best_sol = np.min(best_run)
            else:
                best_run = np.array([np.max(r) for r in _bests])
                best_sol = np.max(best_run)
            avg_best = np.mean(best_run)
            std_best = np.std(best_run)

            b_size = logger.get_batch_size()
            if b_size > 1:
                init_pts = logger.get_num_init_points()
                _aux_mean = np.zeros(total_evals_executed)
                _aux_mean[:init_pts] = mean_best_until_it[:init_pts]
                _aux_std = np.zeros(total_evals_executed)
                _aux_std[:init_pts] = std_best_until_it[:init_pts]
                j = init_pts
                for i in range(init_pts, mean_best_until_it.shape[0]):
                    _aux_mean[j:j+b_size] = mean_best_until_it[i]
                    _aux_std[j:j+b_size] = std_best_until_it[i]
                    j += b_size
                mean_best_until_it = _aux_mean
                std_best_until_it = _aux_std

            data_exp.set_best_solution_metrics(best_sol, avg_best, std_best, mean_best_until_it, std_best_until_it)

        ### BEST SOLUTION with OCCLUSIONS
        if occ_ranges[0]:
            runs_best_value_small_occ = np.zeros(num_runs)
            runs_best_value_large_occ = np.zeros(num_runs)
            for i in range(num_runs):
                qs = runs_queries[i]
                vs = runs_values[i]
                
                # small occlusion
                small_vs = vs[(qs[:,2] < occ_ranges[0]) | (qs[:,2] > occ_ranges[1])]
                small_qs = qs[(qs[:,2] < occ_ranges[0]) | (qs[:,2] > occ_ranges[1])]
                
                # print("RUN", i)
                # print("Small occlusion - canidates:", small_vs.shape[0])
                if small_vs.shape[0] > 0:
                    best_idx = np.argmax(small_vs)
                    runs_best_value_small_occ[i] = small_vs[best_idx]
                    # print("Best:", small_qs[best_idx], "->", small_vs[best_idx])
                else:
                    runs_best_value_small_occ[i] = 0.0
                
                # large occlusion
                large_vs = vs[qs[:,2] < occ_ranges[0]]
                large_qs = qs[qs[:,2] < occ_ranges[0]]
                
                # print("Large occlusion - canidates:", large_vs.shape[0])
                if large_vs.shape[0] > 0:
                    best_idx = np.argmax(large_vs)
                    runs_best_value_large_occ[i] = large_vs[best_idx]
                    # print("Best:", large_qs[best_idx], "->", large_vs[best_idx])
                else:
                    runs_best_value_large_occ[i] = 0.0
            
            data_exp.set_occlusion_metrics(np.mean(runs_best_value_small_occ), np.std(runs_best_value_small_occ), np.mean(runs_best_value_large_occ), np.std(runs_best_value_large_occ))

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
            print(runs_batch_var_q.shape)
            """mean_var_batch_it = np.mean(runs_batch_var_q, axis=0)
            std_var_batch_it = np.std(runs_batch_var_q, axis=0)
            if not no_plot:
                plot_var_batch(mean_var_batch_it, std_var_batch_it, logger.get_active_vars(), logger.get_optimizer_name())"""

            # runs_batch_dist_q = np.array([distance_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])
            # _m_run = np.mean(runs_batch_dist_q, axis=1)
            # data_exp.append(np.mean(_m_run))
            # data_exp.append(np.std(_m_run))

            # runs_batch_var_q = np.array([distance_queries_batch((rbq, rbv))[0] for rbq, rbv in zip(runs_batches_q, runs_batches_v)])
            """runs_batch_var_q = np.zeros(runs_batches_q.shape[0])#, runs_batches_q.shape[1]))
            for i, r in enumerate(runs_batches_q):
                r_d_pts = np.zeros(r.shape[0])
                r_d_m = np.zeros(r.shape[0])
                n_eq = [0 for _ in range(4)]
                for j, b in enumerate(r):
                    if False:
                        d_pts, d_m, d_s = solutions_diversity_metric(b, 0, cluster_sols=True, mind=40)
                        n_eq[d_pts-1] += 1
                        if d_pts == 1:
                            print("all equals", solutions_diversity_metric(b, 0))
                    else:
                        d_pts, d_m, d_s = solutions_diversity_metric(b, metric_type=1)
                    r_d_pts[j] = len(d_pts[0]) + len(d_pts[1])
                    r_d_m[j] = d_m
                runs_batch_var_q[i] = np.mean(r_d_m)"""
                # print(i, np.mean(r_d_pts), np.std(r_d_pts), "-", np.mean(r_d_m), np.std(r_d_m))
                # print(n_eq)
                    
                    
            # TODO: per run -> var batch -> (n_iter, n_dim) -> variance mean/std of iterations (2, n_dim)
            # TODO: mean of means and mean of stds?
            _m_run = np.mean(runs_batch_var_q, axis=1)
            _m_dim = np.mean(_m_run, axis=0)
            _s_run = np.std(runs_batch_var_q, axis=1)
            _s_dim = np.mean(_s_run, axis=0)

            data_exp.set_batch_metrics(batch_size, _m_dim, _s_dim)
        
        ### MULTISOLUTION METRICS
        
        if sols_metrics:
            # n_solutions = np.array([vs.shape[0] for vs in runs_best_values])
            # print(data_exp.optimizer)
            sq_var = np.array([np.var(rq, axis=0) for rq in runs_best_queries])
            sqvar_mean = np.mean(sq_var, axis=0)
            
            if sols_diversity is None:
                runs_dist_sols = [solutions_diversity_metric(rq, metric_type=sols_div_metric) for rq in runs_best_queries]
            else:
                runs_dist_sols = [solutions_diversity_metric(rq, metric_type=sols_div_metric, cluster_sols=True, mind=sols_diversity) for rq in runs_best_queries]
            
            # _test = np.array([solutions_diversity_metric(rq, metric_type=1) for rq in runs_best_queries])
            # _test2 = np.array([solutions_diversity_metric(rq, metric_type=1, cluster_sols=True, mind=sols_diversity) for rq in runs_best_queries])
            n_solutions = np.array([len(r[0]) + len(r[1]) for r, _, _ in runs_dist_sols])
            runs_mean_dist_sols = np.array([r[1] for r in runs_dist_sols])
            runs_std_dist_sols = np.array([r[2] for r in runs_dist_sols])
            runs_sv_mean = np.mean(runs_best_values, axis=1) # np.array([np.mean(rv) for rv in runs_best_values])
            
            """for l, rq, vv in zip(logs, runs_best_queries, runs_sv_mean):
                print("----" + l + "----")
                s = ""
                for i, m_t in zip([0,1,2], ["mean dist", "mean min dist", "W"]):
                    d_m = solutions_diversity_metric(rq, metric_type=i, cluster_sols=True)
                    s += m_t + ": " + str(d_m) + " | "
                print(s) # + "Q: " + str(vv))"""

            
            """for l, nd, m, s, vv in zip(logs, n_solutions, runs_mean_dist_sols, runs_std_dist_sols, runs_sv_mean):
                print("----" + l + "----")
                print(nd, m, s, vv)"""
                # print(t[1], np.mean(t[1]))
                # print(t2[1], np.mean(t2[1]))
                
            mean_dist_sols = np.mean(runs_mean_dist_sols)
            std_dist_sols = np.std(runs_mean_dist_sols)

            sv_mean = np.mean(runs_sv_mean)
            sv_std = np.std(runs_sv_mean)

            data_exp.set_solutions_metrics(np.mean(n_solutions), np.std(n_solutions), sqvar_mean, mean_dist_sols, std_dist_sols, sv_mean, sv_std)

        table_data.append(data_exp)
    
    OBJ_FUNCTION_NAME=function_name
    print("Function:", OBJ_FUNCTION_NAME)
    
    info_table=["Optimizer", "#Runs", "FE", "T"]
    if OBJ_FUNCTION_NAME == "GraspPlanner" or OBJ_FUNCTION_NAME == "EigenGraspPlanner":
        info_table += ["0-Value"]
    
    if best_metrics:
        info_table += ["Best solution", "Avg. Best"] #, "Std. Best",]

    if occ_ranges[0]:
        info_table += ["S-OCC", "L-OCC"]
        print("Occlusion metric:")
        print("\tSmall: z < " + str(occ_ranges[0]) + " or z > " + str(occ_ranges[1]))
        print("\tLarge: z < " + str(occ_ranges[0]))

    if cec_metrics:
        info_table += ["PR", "SR", "CS", "MGOs"]

        print("Num GO: " + str(nGO))
        print("GO value: " + str(go_value))
        print("Accuracy:", ACCURACY)
        print("Radius:", RADIUS)
    
    if batch_metrics:
        info_table += ["Batch size", "Var Batch (mean)", "Var Batch (std)"]
    
    if sols_metrics:
        print("Diversity metric:", sols_div_metric)
        if sols_diversity is not None:
            print("Diversity level:", sols_diversity)
        info_table += ["Num. sols. (mean)", "Solutions D metric (mean)", "Solutions Q metric (mean)"] # "Solutions var. (mean)", 
    
    print(70 * '=')
    
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
            ) # ["BBO-LP", "SO-MS", "ROBOT", "MEBO"] [t_data.optimizer for t_data in table_data]
    
    if not no_plot:
        plt.show()



