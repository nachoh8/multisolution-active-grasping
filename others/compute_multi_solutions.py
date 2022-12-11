from itertools import combinations
import sys
sys.path.append("../")

import numpy as np
import argparse
import os
import glob
import matplotlib.pyplot as plt

from OptSystem.core.datalog import DataLog
from OptSystem.utils.utils import create_objective_function

from evaluation import solutions_diversity_metric

from kmeans import compute as kmeans

MINIMIZE=False
METRIC="outcome"
OBJ_FUNCTION_NAME=""
COLORS=['k', 'r', 'b', '#ff7700', '#ff77ff', 'y' ]
ACTIVE_VARS=["x"]

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
    print("Max:", Ymax, "Min value:", Yminv)#, normalize(Yminv, Ymax, Ymin), unnormalize(0.7, Ymax, Ymin))
    print(queries[max_idx])
    print("Num candidates:", Qs.shape[0])

    max_div = 0.0
    CX = None
    CY = None
    SX = None
    SY = None

    for k in range(5):
        _CX, _CY = kmeans(Qs, Yv, num_solutions)
        _SX = np.zeros((num_solutions, ndim))
        _SY = np.zeros(num_solutions)
        print("====Solutions (" + str(k) + ")====")
        for cx, cy, i in zip(_CX, _CY, range(len(_CY))):
            max_idx = np.argmax(cy)
            _SX[i] = cx[max_idx]
            _SY[i] = cy[max_idx]
            # print("\t", _SX[i], _SY[i])
        
        d_metric = solutions_diversity_metric(_SX, cluster_sols=True, mind=40)
        print(_SX)
        print("D metric:", d_metric)
        print("Q metric:", np.mean(_SY), np.std(_SY))
        if d_metric[1] > max_div:
            CX = _CX
            CY = _CY
            SX = _SX
            SY = _SY
            max_div = d_metric[1]
    
    plot_solutions(CX, CY, SX, SY, "Clusters - " + name, "y")
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

            d_metric = solutions_diversity_metric(best_querys)
            d_metric_2 = solutions_diversity_metric(best_querys, metric_type=1)

            q_metric = (np.mean(best_values), np.std(best_values))

            print("Prev solutions")
            print("\tNum. solutions:", best_values.shape[0])
            print("\tD metric:", d_metric, d_metric_2)
            print("\tQ metric:", q_metric)

            ### COMPUTE SOLUTIONS

            lb = np.array(logger.get_lower_bounds())
            ub = np.array(logger.get_upper_bounds())
            
            q_solutions, v_solutions = get_solutions(num_solutions, min_value, queries, values, lb, ub, logger.get_optimizer_name())
                
            n_solutions = v_solutions.shape[0]

            # sq_var = np.var(q_solutions, axis=0)
            # sv_mean = np.mean(v_solutions)

            d_metric = solutions_diversity_metric(q_solutions)
            d_metric_2 = solutions_diversity_metric(q_solutions, metric_type=1)

            q_metric = (np.mean(v_solutions), np.std(v_solutions))

            print("New solutions")
            print("\tNum. solutions:", n_solutions)
            print("\tD metric:", d_metric, d_metric_2)
            print("\tQ metric:", q_metric)

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



