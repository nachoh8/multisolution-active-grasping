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
from sklearn.mixture import BayesianGaussianMixture

from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
from pyclustering.utils import metric

from OptSystem.core.datalog import DataLog
from OptSystem.utils.utils import create_objective_function

from evaluation import solutions_diversity_metric

from kmeans import compute as kmeans

MINIMIZE=False
METRIC="outcome"
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'b', '#ff7700', '#ff77ff', 'y' ]
ACTIVE_VARS=["x"]
DIVF=0
PLOT2D=False
PLOT_ENABLED=True

def euclid_dist(pt1, pt2):
    return np.linalg.norm(pt1[:3] - pt2[:3])

def plot_solutions(CX, CY, SX, SY, tittle, value_title, best=None):
    plot2d = PLOT2D

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
    
    if SX is not None and SY is not None:
        for sx, sy in zip(SX, SY):
            if ndim == 1:
                plt.scatter(sx, sy, c='k')
            elif ndim == 2 and plot2d:
                plt.scatter(sx[0], sx[1], c='k', marker='+')
            else:
                if ndim == 2:
                    ax.scatter(sx[0], sx[1], sy, c='k', marker='+')
                else:
                    ax.scatter(sx[0], sx[1], sx[2], c='k', marker='+')
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
        if DIVF > 0.0: # with diversity level
            sols_sets, d_m, _ = solutions_diversity_metric(_SX, cluster_sols=DIVF>0.0, mind=DIVF, metric_type=1)
            n_sols = len(sols_sets[0]) + len(sols_sets[1])
            _best_d_m = n_sols * d_m * (np.mean(_SY) + np.min(_SY))
        else:
            _, d_m, _ = solutions_diversity_metric(_SX, metric_type=1)
            _best_d_m = d_m * (np.mean(_SY) + np.min(_SY))

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
    print("Num of candidates:", n_combs)
    if n_combs < 500:
        SX, SY, best_d_m = parallel_select_solutions((all_combinations, Qs, Yv, num_solutions))
    else: # parallel
        with multiprocessing.Pool() as pool:
            num_proc = len(multiprocessing.active_children())
            task_len = math.ceil(n_combs / (num_proc*4))
            n_finished = 0
            for sx, sy, m in pool.imap_unordered(parallel_select_solutions, [(all_combinations[i*task_len:i*task_len + task_len], Qs, Yv, num_solutions) for i in range(num_proc*4)]):
                n_finished += 1
                if n_finished % 4 == 0: print("Finished:", n_finished, "/", num_proc*4)
                if m >= best_d_m:
                    SX = sx
                    SY = sy
                    best_d_m = m
    
    return SX, SY, best_d_m

def cluster_score(clusters: "list[list[int]]", Qs: np.ndarray, Yv: np.ndarray, batch_size: int, Ymin: float):
    num_pts = Qs.shape[0]
    n_cl = len(clusters)
    
    if Ymin < 0:
        Yv = abs(Ymin) + Yv

    len_cl = np.array([len(cl) for cl in clusters])
    vmax_cl = np.array([np.max(Yv[cl]) for cl in clusters])
    vmin_cl = np.array([np.min(Yv[cl]) for cl in clusters])
    value_score_factor = 1 - np.min(vmax_cl) / np.max(vmax_cl)
    Ymean = np.mean(vmax_cl)
    s = np.max(vmax_cl) - np.min(vmax_cl)

    cl_socre = np.zeros(n_cl)
    for i in range(n_cl):
        print("Cluster", i+1)
        print("\tSize:", len_cl[i], "Minv:", vmin_cl[i], "Maxv:", vmax_cl[i])
        csize = 1-len_cl[i]/np.max(len_cl)
        csize2 = (len_cl[i] - np.mean(len_cl)) / (np.max(len_cl) - np.min(len_cl))
        print("\tCluster size:", csize, csize2) # 1-len_cl[i]/num_pts
        vdist = 1-vmax_cl[i]/np.max(vmax_cl)
        vdist = vdist / value_score_factor
        vdist2 = (vmax_cl[i] - Ymean) / s
        print("\tValue distance:", vdist, vdist2)
        print("\tScore:", csize+vdist, csize2+vdist2)
        cl_socre[i] = csize2+vdist2
    
    smin = np.min(cl_socre)
    if smin < 0:
        cl_socre = cl_socre + abs(smin)

    cl_socre = np.max(cl_socre) - cl_socre

    batch = np.zeros(batch_size)
    n_pts = np.zeros(n_cl)
    for i in range(batch_size):
        print("Point", i+1, "/", batch_size)
        print("\tScores:", cl_socre)
        cl_idx = np.argmax(cl_socre)
        batch[i] = cl_idx
        n_pts[cl_idx] += 1
        cl_socre[cl_idx] *= 1 - 0.1 * n_pts[cl_idx]
        print("\tSelect cluster ", cl_idx+1)

def cluster_validation(centroids: np.ndarray):
    n_cls = centroids.shape[0]
    valid_cls = set()
    novalid_cls = set()
    for cl in range(n_cls):
        cx = centroids[cl]
        _ncl = []
        for cl2 in range(n_cls):
            if cl == cl2: continue
            if np.linalg.norm(cx - centroids[cl2]) < DIVF:
                if cl < cl2 and cl not in _ncl:
                    _ncl.append(cl)
                _ncl.append(cl2)
        if len(_ncl) > 0:
            if cl not in _ncl:
                _ncl.append(cl)
            novalid_cls.add(tuple(_ncl))
        else:
            valid_cls.add(cl)

    to_join_clusters = set()
    for cl in range(n_cls):
        if cl in valid_cls: continue
        lmax = 0
        _cl = None
        for nv in novalid_cls:
            if cl in nv:
                # cx = np.sum(centroids[list(nv)], axis=0) / len(nv)
                l = len(nv)
                if l > lmax:
                    _cl = nv
                    lmax = l
        to_join_clusters.add(_cl)
    
    return valid_cls, to_join_clusters

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
    print(Yminv)
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
            _SX = np.zeros((num_solutions, ndim))
            _SY = np.zeros(num_solutions)
            for i, c in enumerate(clusters):
                _cQ = Qs[c]
                _CX.append(_cQ)
                _cV = Yv[c]
                _CY.append(_cV)
                max_idx = np.argmax(_cV)
                _SX[i] = _cQ[max_idx]
                _SY[i] = _cV[max_idx]
            
            _, d_m, _ = solutions_diversity_metric(_SX, cluster_sols=True, mind=40)
            # print(_SX)
            # print("D metric:", d_metric)
            # print("Q metric:", np.mean(_SY), np.std(_SY))
            # d_m = d_metric[0] * d_metric[1]

            # _SX, _SY, d_m = select_solutions(clusters, Qs, Yv, num_solutions)
            if d_m > max_div:
                CX = _CX
                CY = _CY
                SX = _SX
                SY = _SY
                max_div = d_m
        
        # plot_solutions(CX, CY, SX, SY, "Clusters - " + name, "y")
    elif clustering == "xmeans":
        clusters = None
        centroids = None
        SX = None
        SY = None
        best_m = 0.0
        n_init_centroids = num_solutions
        for _ in range(1):
            initial_centroids = kmeans_plusplus_initializer(Qs, n_init_centroids, random_state=0).initialize()

            xmeans_instance = xmeans(Qs, initial_centers=initial_centroids, kmax=num_solutions, repeat=10, random_state=0) #, metric=metric.distance_metric(metric.type_metric.USER_DEFINED, func=euclid_dist, numpy_usage=True))
            xmeans_instance.process()

            _clusters = xmeans_instance.get_clusters()
            _centroids = xmeans_instance.get_centers()
            _SX, _SY, d_m = (np.zeros((1,3)), np.zeros(1), 0.1) # select_solutions(_clusters, Qs, Yv, num_solutions)
            # cluster_score(_clusters, Qs, Yv, 3)
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
                centroids = np.array(_centroids)
        CX = []
        CY = []
        for c in clusters:
            _cQ = Qs[c]
            CX.append(_cQ)
            _cV = Yv[c]
            CY.append(_cV)
            
        n_clusters = len(clusters)
        print("num clusters:", n_clusters, [len(c) for c in clusters])
        valid_cls, to_join_cls = cluster_validation(clusters, centroids)

        """centers = xmeans_instance.get_centers()        
        visualizer = cluster_visualizer()
        visualizer.append_clusters(clusters, Qs)
        visualizer.append_cluster(centers, None, marker='*', markersize=10)
        visualizer.show(display=False)"""
    
    elif clustering == "bgm": # bayesian gaussian mixture
        # TODO: a veces devuelve menos clusters
        SX = None
        SY = None
        best_m = 0.0
        clusters = []
        n = 0
        for _ in range(20):
            bgm = BayesianGaussianMixture(n_components=num_solutions, init_params="kmeans", n_init=num_solutions, max_iter=200)
            label_prediction = bgm.fit_predict(Qs)
            # prob_prediction = bgm.predict_proba(Qs)
            _clusters = [[] for _ in range(num_solutions)]
            for idx in range(Qs.shape[0]):
                p = label_prediction[idx]
                _clusters[p].append(idx)
                # print(p, prob_prediction[idx])
            clusters = _clusters
            SX = np.zeros((1,3))
            SY = np.zeros(1)
            break
            filled = True
            """for c in _clusters:
                if len(c) == 0:
                    filled = False
                    break"""
            if filled:
                n += 1
                _SX, _SY, d_m = select_solutions(_clusters, Qs, Yv, num_solutions)
                if d_m > best_m:
                    best_m = d_m
                    SX = _SX
                    SY = _SY
                    clusters = _clusters
        
        print("Filled:", n)
        
        CX = []
        CY = []
        for c in clusters:
            _cQ = Qs[c]
            CX.append(_cQ)
            _cV = Yv[c]
            CY.append(_cV)

    plot_solutions(CX, CY, SX, SY, clustering + " - " + name, "y", best=(queries[max_idx], Ymax))
    return SX, SY

def compute_clusters(num_solutions: int, queries: np.ndarray, values: np.ndarray, name: str, compute_solutions=True, best_per_cluster=True):
    print("Min:", np.min(values), "| Max:", np.max(values), "| Mean:", np.mean(values))

    vsorted = np.sort(values)

    max_sols = 0
    clusters = None
    centroids = None
    Qs = None
    Yv = None
    valid_cls = None
    to_join_cls = None
    q = None
    
    prev_n_pts = 0
    for d in range(9,0,-1):
        if compute_solutions and d <= 5:
            break
        lmin = np.quantile(vsorted, 0.1*d)
        # print('='*50)
        # print("Quantil:", 0.1*d, "->", lmin)
        
        idxs = values >= lmin
        _Yv = values[idxs]
        _Qs = queries[idxs]

        # print("Min:", np.min(_Yv), "| Max:", np.max(_Yv), "| Mean:", np.mean(_Yv))

        n_pts = _Yv.shape[0]

        # print("Num. points:", n_pts)
        if n_pts < num_solutions or n_pts == prev_n_pts:
            continue
        prev_n_pts = n_pts

        _clusters = None
        _centroids = None
        _valid_cls = None
        _to_join_cls = None
        n = 0
        for _ in range(10):
            initial_centroids = kmeans_plusplus_initializer(_Qs, num_solutions).initialize()

            xmeans_instance = xmeans(_Qs, initial_centers=initial_centroids, kmax=num_solutions, repeat=50)#, random_state=0) #, metric=metric.distance_metric(metric.type_metric.USER_DEFINED, func=euclid_dist, numpy_usage=True))
            xmeans_instance.process()

            __clusters = xmeans_instance.get_clusters()
            __centroids = np.array(xmeans_instance.get_centers())
            __valid_cls, __to_join_cls = cluster_validation(__centroids)

            _n = len(__valid_cls) + len(__to_join_cls)
            if _n > n:
                _clusters = __clusters
                _centroids = __centroids
                _valid_cls = __valid_cls
                _to_join_cls = __to_join_cls
                n = _n

        # print("Num clusters:", len(_clusters), [len(c) for c in _clusters])
        # print("Num. Valid:", len(_valid_cls), "Num. No Valid:", len(_to_join_cls))

        n = len(_valid_cls) + len(_to_join_cls)
        if n > max_sols:
            clusters = _clusters
            centroids = _centroids
            Qs = _Qs
            Yv = _Yv
            valid_cls = _valid_cls
            to_join_cls = _to_join_cls
            q = (0.1*d, lmin)
            
            max_sols = n
            if n == num_solutions:
                break
    
    print('='*15)
    print("Final Quantil:", q[0], "->", q[1])
    print("Num. Valid:", len(valid_cls), "Num. No Valid:", len(to_join_cls))

    if compute_solutions:
        best_idx = np.argmax(Yv)
        SX, SY, _ = select_solutions(clusters, Qs, Yv, num_solutions)
        if PLOT_ENABLED:
            plot_solutions([Qs[cl] for cl in clusters], [Yv[cl] for cl in clusters], SX, SY, "Solutions - " + name, "y", best=(Qs[best_idx], Yv[best_idx]))
        return SX, SY
    else:
        final_clusters = [clusters[cl] for cl in valid_cls]
        final_centroids = [centroids[cl] for cl in valid_cls]
        for jcl in to_join_cls:
            print("Joining:", jcl)
            join_cls = []
            for _cl in jcl:
                join_cls += clusters[_cl]
            final_clusters.append(join_cls)
            print(centroids[list(jcl)])
            cx = np.sum(centroids[list(jcl)], axis=0) / len(jcl)
            final_centroids.append(cx)
            print("New centroid:", cx)

        print('='*15)

        cluster_score(final_clusters, Qs, Yv, 3, np.min(values))

        if PLOT_ENABLED:
            plot_solutions([Qs[cl] for cl in clusters], [Yv[cl] for cl in clusters], centroids, [0 for _ in centroids], "Original clustering - " + name, "y")
            plot_solutions([Qs[cl] for cl in final_clusters], [Yv[cl] for cl in final_clusters], final_centroids, [0 for _ in final_centroids], "Fixed clustering - " + name, "y")

def compute_clusters_no_divf(num_solutions: int, queries: np.ndarray, values: np.ndarray, name: str):
    print("Min:", np.min(values), "| Max:", np.max(values), "| Mean:", np.mean(values))

    vsorted = np.sort(values)

    max_d_metric = 0
    clusters = None
    Qs = None
    Yv = None
    
    prev_n_pts = 0
    for d in range(9,0,-1):
        if d <= 5:
            break
        lmin = np.quantile(vsorted, 0.1*d)
        print('='*50)
        print("Quantil:", 0.1*d, "->", lmin)
        
        idxs = values >= lmin
        _Yv = values[idxs]
        _Qs = queries[idxs]

        n_pts = _Yv.shape[0]

        print("Num. points:", n_pts)
        if n_pts < num_solutions or n_pts == prev_n_pts:
            continue
        prev_n_pts = n_pts

        _clusters = None
        _max_d_metric = 0.0
        
        for _ in range(10):
            initial_centroids = kmeans_plusplus_initializer(_Qs, num_solutions).initialize()

            xmeans_instance = xmeans(_Qs, initial_centers=initial_centroids, kmax=num_solutions, repeat=50)#, random_state=0) #, metric=metric.distance_metric(metric.type_metric.USER_DEFINED, func=euclid_dist, numpy_usage=True))
            xmeans_instance.process()

            __clusters = xmeans_instance.get_clusters()
            _centroids = np.array(xmeans_instance.get_centers())
            _, d_m, _ = solutions_diversity_metric(_centroids, metric_type=1)
            
            cl_mean = np.array([np.mean(_Yv[cl]) for cl in __clusters])
            cl_min = np.array([np.min(_Yv[cl]) for cl in __clusters])
            d_m = d_m * (np.mean(cl_mean) + np.min(cl_min))

            if d_m > _max_d_metric:
                _clusters = __clusters
                _max_d_metric = d_m

        print("D metric:", _max_d_metric)

        if _max_d_metric > max_d_metric:
            clusters = _clusters
            Qs = _Qs
            Yv = _Yv
            
            max_d_metric = _max_d_metric
    
    SX, SY, _ = select_solutions(clusters, Qs, Yv, num_solutions)
    best_idx = np.argmax(Yv)
    if PLOT_ENABLED:
        plot_solutions([Qs[cl] for cl in clusters], [Yv[cl] for cl in clusters], SX, SY, "Solutions - " + name, "y", best=(Qs[best_idx], Yv[best_idx]))
    return SX, SY

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CEC2013 permormance measures')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument('-save', help='Overwrite solutions', action='store_true')
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument("-metric", type=str, help="metric", metavar='<metric>', default=METRIC)
    parser.add_argument('-no-plot', help='No show plots', action='store_true')
    parser.add_argument('-p2d', help='Plot 2D', action='store_true')
    parser.add_argument("-nsols", type=int, help="num solutions", metavar='<num_solutions>', default=4)
    # parser.add_argument("-minv", type=float, help="min optimum value", metavar='<min_optimum>', default=0.7)
    parser.add_argument("-fe", type=float, help="max func evals.", metavar='<func_evals>', default=1.0)
    parser.add_argument("-divf", type=float, help="min divisersity level.", metavar='<min_diversity_level>', required=True)
    
    args = parser.parse_args()
    flogs = args.flogs
    MINIMIZE = args.minimize
    METRIC=args.metric
    PLOT_ENABLED = not args.no_plot
    save_solutions=args.save
    max_fe=args.fe
    DIVF=args.divf
    PLOT2D=args.p2d

    num_solutions = args.nsols
    # min_value = args.minv

    print("Num. solutions:", num_solutions)
    print("Min. diversity level:", DIVF)
    # print("Min. value (%):", min_value)

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

            d_metric = solutions_diversity_metric(best_querys, cluster_sols=True, mind=DIVF, metric_type=1)
            d_metric = (len(d_metric[0][0]) + len(d_metric[0][1]), d_metric[1], d_metric[2])
            # d_metric_2 = solutions_diversity_metric(best_querys, metric_type=1)

            q_metric = (np.min(best_values), np.max(best_values), np.mean(best_values), np.std(best_values))

            print("Prev solutions")
            print("\tNum. solutions:", best_values.shape[0])
            print("\tD metric:", d_metric)
            print("\tQ metric:", q_metric) #  + np.min(best_values))
            # print("\tOutcomes", best_values, q_metric)

            """_n = best_values.shape[0]
            dist = np.zeros((_n, _n))
            for i in range(_n):
                for j in range(_n):
                    _d = np.linalg.norm(best_querys[i] - best_querys[j])
                    dist[i, j] = _d

            print(dist)"""

            # continue

            ### COMPUTE SOLUTIONS

            lb = np.array(logger.get_lower_bounds())
            ub = np.array(logger.get_upper_bounds())
            
            for c in ["xmeans"]: #, "xmeans"]:
                if max_fe < 1.0:
                    n = queries.shape[0] * max_fe
                    n = math.ceil((n - logger.get_num_init_points()) / logger.get_batch_size())
                    n = int(logger.get_num_init_points() + n*logger.get_batch_size())
                    queries = queries[:n]
                    values = values[:n]
                # q_solutions, v_solutions = get_solutions(num_solutions, min_value, queries, values, lb, ub, logger.get_optimizer_name(), clustering=c)
                if DIVF > 0.0:
                    q_solutions, v_solutions = compute_clusters(num_solutions, queries, values, logger.get_optimizer_name())
                else:
                    q_solutions, v_solutions = compute_clusters_no_divf(num_solutions, queries, values, logger.get_optimizer_name())
                    
                n_solutions = v_solutions.shape[0]

                # sq_var = np.var(q_solutions, axis=0)
                # sv_mean = np.mean(v_solutions)

                if DIVF > 0.0:
                    d_metric = solutions_diversity_metric(q_solutions, cluster_sols=True, mind=DIVF, metric_type=1)
                else:
                    d_metric = solutions_diversity_metric(q_solutions, metric_type=1)
                d_metric = (len(d_metric[0][0]) + len(d_metric[0][1]), d_metric[1], d_metric[2])
                # d_metric_2 = solutions_diversity_metric(q_solutions, metric_type=1)

                q_metric = (np.min(v_solutions), np.max(v_solutions), np.mean(v_solutions), np.std(v_solutions))

                print(c + " algorithm:")
                print("\tNum. solutions:", n_solutions)
                print("\tD metric:", d_metric)
                print("\tQ metric:", q_metric) # + np.min(v_solutions))
                print("\tOutcomes", v_solutions, )

                """dist = np.zeros((num_solutions, num_solutions))
                for i in range(n_solutions):
                    for j in range(n_solutions):
                        _d = np.linalg.norm(q_solutions[i] - q_solutions[j])
                        dist[i, j] = _d

                print(dist)"""

            if save_solutions:
                new_solutions = []
                for qs, vs, in zip(q_solutions, v_solutions):    
                    query = dict(zip(ACTIVE_VARS, list(qs)))
                    r = {"query": query, "metrics": [{"name": METRIC, "value": vs}]}
                    new_solutions.append(r)
                logger.log_best_results(new_solutions)
                logger.save_json()

    if PLOT_ENABLED:
        plt.show()



