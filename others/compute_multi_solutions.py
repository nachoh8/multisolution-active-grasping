import sys
sys.path.append("../")

from itertools import product, combinations
import math
import multiprocessing
import numpy as np
import argparse
import os
import glob
import matplotlib.pyplot as plt

from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer

from OptSystem.core.datalog import DataLog
from OptSystem.utils.utils import create_objective_function

from evaluation import solutions_diversity_metric

from kmeans import compute as kmeans

MINIMIZE=False
METRIC="outcome"
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'b', '#ff7700', '#ff77ff', 'y' ]
ACTIVE_VARS=["x"]

def plot_solutions(CX, CY, SX, SY, tittle, value_title, best=None):
    plot2d = False

    ndim = CX[0].shape[1]
    fig = plt.figure()
    if ndim >= 2 and not plot2d:
        ax = fig.add_subplot(projection='3d')
    for cx, cy in zip(CX, CY):
        if ndim == 1:
            sc_plot = plt.scatter(cx, cy, alpha=0.5)
            plt.xlabel(ACTIVE_VARS[0])
            plt.ylabel(value_title)
        elif ndim == 2 and plot2d:
            sc_plot = plt.scatter(cx[:, 0], cx[:, 1], alpha=0.5)
            plt.xlabel(ACTIVE_VARS[0])
            plt.ylabel(ACTIVE_VARS[1])
        else:
            if ndim == 2:
                ax.scatter(cx[:, 0], cx[:, 1], cy, alpha=0.5)
                ax.set_zlabel(value_title)
            else:
                ax.scatter(cx[:, 0], cx[:, 1], cx[:, 2], alpha=0.5)
                ax.set_zlabel(ACTIVE_VARS[2])

            ax.set_xlabel(ACTIVE_VARS[0])
            ax.set_ylabel(ACTIVE_VARS[1])
    
    for sx, sy in zip(SX, SY):
        if ndim == 1:
            plt.scatter(sx, sy, c='k')
        elif ndim == 2 and plot2d:
            pass
        else:
            if ndim == 2:
                ax.scatter(sx[0], sx[1], sy, c='k')
            else:
                ax.scatter(sx[0], sx[1], sx[2], c='k')
    if best:
        bq = best[0]
        bv = best[1]
        if ndim == 1:
            plt.scatter(bq, bv, c='r')
        elif ndim == 2 and plot2d:
            pass
        else:
            if ndim == 2:
                ax.scatter(bq[0], bq[1], bv, c='r')
            else:
                ax.scatter(bq[0], bq[1], bq[2], c='r', marker='+', s=80)

    # plt.colorbar(sc_plot, label=value_title, orientation="vertical")
    plt.title(tittle)

def normalize(v, max, min):
    return (v-min) / (max-min)

def unnormalize(v, max, min):
    return v * (max-min) + min

def parallel_select_solutions(args):
    all_candidates: "list[tuple[int]]" = args[0]
    Qs: np.ndarray = args[1]
    Yv: np.ndarray = args[2]
    num_solutions: int = args[3]

    SX = None
    SY = None
    
    best_d_m = 0.0
    for candidates in all_candidates:
        """if len(candidates) > num_solutions:
            _SX = None
            _SY = None
            _best_d_m = 0.0
            for cands2 in combinations(candidates, num_solutions):
                _idxs = list(cands2)
                _Q = Qs[_idxs]
                n_sols, d_m, _ = solutions_diversity_metric(_Q, cluster_sols=True, mind=40)
                _V = Yv[_idxs]
                d = n_sols * d_m * (np.mean(_V) + np.min(_V))
                if d > _best_d_m:
                    _SX = _Q
                    _SY = _V
                    _best_d_m = d
        else:"""
        _idxs = list(candidates)
        _SX = Qs[_idxs]
        _SY = Yv[_idxs]
        n_sols, d_m, _ = solutions_diversity_metric(_SX, cluster_sols=True, mind=40)
        _best_d_m = n_sols * d_m * (np.mean(_SY) + np.min(_SY))

        if _best_d_m >= best_d_m:
            SX = _SX
            SY = _SY
            best_d_m = _best_d_m
    
    return SX, SY, best_d_m

def select_solutions(clusters: "list[list[int]]", Qs: np.ndarray, Yv: np.ndarray, num_solutions):
    SX = None
    SY = None
    best_d_m = 0.0
    
    all_combinations = list(product(*clusters))
    n_combs = len(all_combinations)
    n_clusters = len(clusters)

    if n_clusters > num_solutions: # reduce number of candidates
        _combs = set()
        for cluster_comb in all_combinations:
            for candidate_sol in combinations(cluster_comb, num_solutions):
                _combs.add(candidate_sol)
        all_combinations = list(_combs)
        n_combs = len(all_combinations)

    if n_combs < 500:
        SX, SY, best_d_m = parallel_select_solutions((all_combinations, Qs, Yv, num_solutions))
    else: # parallel
        with multiprocessing.Pool() as pool:
            num_proc = len(multiprocessing.active_children())
            task_len = math.ceil(n_combs / (num_proc*4))
            for sx, sy, m in pool.imap_unordered(parallel_select_solutions, [(all_combinations[i*task_len:i*task_len + task_len], Qs, Yv, num_solutions) for i in range(num_proc*2)]):
                if m >= best_d_m:
                    SX = sx
                    SY = sy
                    best_d_m = m
    
    return SX, SY, best_d_m

def get_solutions(num_solutions: int, min_outcome: float, queries: np.ndarray, values: np.ndarray, lb: np.ndarray, ub: np.ndarray, name: str, clustering: str = "kmeanspp"):
    ndim = queries.shape[1]

    min_idx = np.argmin(values)
    max_idx = np.argmax(values)
    Ymin = values[min_idx]
    Ymax = values[max_idx]

    # Qs = (queries - lb) / (ub - lb)
    Ys = (values - Ymin) / (Ymax - Ymin)

    if Ymax < 0:
        Yminv = (Ymax*(1 + 1 - min_outcome) - Ymin) / (Ymax - Ymin)
    else:
        Yminv = (Ymax*min_outcome - Ymin) / (Ymax - Ymin)
    
    idxs = Ys >= Yminv
    Yv = values[idxs]
    Qs = queries[idxs]
    # print("Max:", Ymax, "Min value:", Yminv)#, normalize(Yminv, Ymax, Ymin), unnormalize(0.7, Ymax, Ymin))
    # print(queries[max_idx])
    print("Num candidates:", Qs.shape[0])

    if clustering == "kmeanspp":
        max_div = 0.0
        CX = None
        CY = None
        SX = None
        SY = None

        for k in range(200):
            clusters = kmeans(Qs, Yv, num_solutions)
            
            _CX = []
            _CY = []
            for c in clusters:
                _cQ = Qs[c]
                _CX.append(_cQ)
                _cV = Yv[c]
                _CY.append(_cV)
            
            # _SX = np.zeros((num_solutions, ndim))
            # _SY = np.zeros(num_solutions)

            # print("====Solutions (" + str(k) + ")====")
            """for cx, cy, i in zip(_CX, _CY, range(len(_CY))):
                max_idx = np.argmax(cy)
                _SX[i] = cx[max_idx]
                _SY[i] = cy[max_idx]
                # print("\t", _SX[i], _SY[i])"""
            
            # d_metric = solutions_diversity_metric(_SX, cluster_sols=True, mind=40)
            # print(_SX)
            # print("D metric:", d_metric)
            # print("Q metric:", np.mean(_SY), np.std(_SY))
            # d_m = d_metric[0] * d_metric[1]

            _SX, _SY, d_m = select_solutions(clusters, Qs, Yv, num_solutions)
            if d_m > max_div:
                CX = _CX
                CY = _CY
                SX = _SX
                SY = _SY
                max_div = d_m
        
        # plot_solutions(CX, CY, SX, SY, "Clusters - " + name, "y")
    elif clustering == "xmeans":
        clusters = None
        SX = None
        SY = None
        best_m = 0.0
        n_init_centroids = num_solutions
        for _ in range(20):
            initial_centroids = kmeans_plusplus_initializer(Qs, n_init_centroids).initialize()

            xmeans_instance = xmeans(Qs, initial_centers=initial_centroids, kmax=num_solutions*2, repeat=10)
            xmeans_instance.process()

            _clusters = xmeans_instance.get_clusters()
            _SX, _SY, d_m = select_solutions(_clusters, Qs, Yv, num_solutions)
            
            """_SX2 = np.zeros((num_solutions, ndim))
            _SY2 = np.zeros(num_solutions)
            for i, c in enumerate(_clusters):
                best_idx = np.argmax(Yv[c])
                _SX2[i] = Qs[best_idx]
                _SX2[i] = Yv[best_idx]

            n_sols, _d_m, _ = solutions_diversity_metric(_SX2, cluster_sols=True, mind=40)
            d_m2 = n_sols * _d_m * (np.mean(_SY2) + np.min(_SY2))
            if d_m2 > d_m:
                print("Best per cluster better!")
                print(d_m, d_m2)
                d_m = d_m2
                _SX = _SX2
                _SY = _SY2
            """
            if d_m > best_m:
                best_m = d_m
                SX = _SX
                SY = _SY
                clusters = _clusters
        CX = []
        CY = []
        for c in clusters:
            _cQ = Qs[c]
            CX.append(_cQ)
            _cV = Yv[c]
            CY.append(_cV)
            
        n_clusters = len(clusters)
        print("num clusters:", n_clusters, [len(c) for c in clusters])
        
        """centers = xmeans_instance.get_centers()        
        visualizer = cluster_visualizer()
        visualizer.append_clusters(clusters, Qs)
        visualizer.append_cluster(centers, None, marker='*', markersize=10)
        visualizer.show(display=False)"""
    
    plot_solutions(CX, CY, SX, SY, clustering + " - " + name, "y", best=(queries[max_idx], Ymax))
    return SX, SY

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument('-save', help='Overwrite solutions', action='store_true')
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument("-metric", type=str, help="metric", metavar='<metric>', default=METRIC)
    parser.add_argument('-no-plot', help='No show plots', action='store_true')
    parser.add_argument("-nsols", type=int, help="num solutions", metavar='<num_solutions>', default=4)
    parser.add_argument("-minv", type=float, help="min optimum value", metavar='<min_optimum>', default=0.7)
    
    args = parser.parse_args()
    flogs = args.flogs
    MINIMIZE = args.minimize
    METRIC=args.metric
    no_plot = args.no_plot
    save_solutions=args.save

    num_solutions = args.nsols
    min_value = args.minv

    print("Num. solutions:", num_solutions)
    print("Min. value (%):", min_value)

    for flog in flogs:
        if os.path.isdir(flog):
            logs = glob.glob(flog + "/*.json")
        else:
            logs = [flog]

        ### EXTRACT DATA

        num_runs = len(logs)

        for log_file in logs:
            print(70 * '=')
            print(log_file)
            logger = DataLog(log_file=log_file)
            logger.load_json()

            _queries, _outcomes = logger.get_queries(minimize=MINIMIZE, best_per_iteration=False, metric=METRIC)
            queries = np.array(_queries)
            values = np.array(_outcomes).reshape(-1)
            
            _best = logger.get_best_queries(metric=METRIC)
            best_querys = np.array(_best[0])
            best_values = np.array(_best[1]).reshape(-1)
        
            ACTIVE_VARS = logger.get_active_vars()

            # sq_var = np.var(best_querys, axis=0)
            # sv_mean = np.mean(best_values)

            d_metric = solutions_diversity_metric(best_querys, cluster_sols=True, mind=40)
            # d_metric_2 = solutions_diversity_metric(best_querys, metric_type=1)

            q_metric = (np.mean(best_values), np.std(best_values))

            print("Prev solutions")
            print("\tNum. solutions:", best_values.shape[0])
            print("\tD metric:", d_metric)
            print("\tQ metric:", np.mean(best_values) + np.min(best_values))
            print("\tOutcomes", best_values, q_metric)

            ### COMPUTE SOLUTIONS

            lb = np.array(logger.get_lower_bounds())
            ub = np.array(logger.get_upper_bounds())
            
            for c in ["xmeans"]:
                q_solutions, v_solutions = get_solutions(num_solutions, min_value, queries, values, lb, ub, logger.get_optimizer_name(), clustering=c)
                    
                n_solutions = v_solutions.shape[0]

                # sq_var = np.var(q_solutions, axis=0)
                # sv_mean = np.mean(v_solutions)

                d_metric = solutions_diversity_metric(q_solutions, cluster_sols=True, mind=40)
                # d_metric_2 = solutions_diversity_metric(q_solutions, metric_type=1)

                q_metric = (np.mean(v_solutions), np.std(v_solutions))

                print(c + " algorithm:")
                print("\tNum. solutions:", n_solutions)
                print("\tD metric:", d_metric)
                print("\tQ metric:", np.mean(v_solutions) + np.min(v_solutions))
                print("\tOutcomes", v_solutions, q_metric)

            if save_solutions:
                new_solutions = []
                for qs, vs, in zip(q_solutions, v_solutions):    
                    query = dict(zip(ACTIVE_VARS, list(qs)))
                    r = {"query": query, "metrics": [{"name": METRIC, "value": vs}]}
                    new_solutions.append(r)
                logger.log_best_results(new_solutions)
                logger.save_json()

    if not no_plot:
        plt.show()



