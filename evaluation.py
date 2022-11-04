import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

from multisolution_active_grasping.core.objective_function import ObjectiveFunction
from multisolution_active_grasping.core.datalog import DataLog
from multisolution_active_grasping.utils.utils import create_objective_function

COLORS=['k', 'r', 'g', '#ff7700', '#ff77ff']
OBJ_FUNC_NAME = ""
METRIC = ""
MINIMIZE = False
OBJ_FUNCTION: ObjectiveFunction =None

def compute_max_until_iteration(outcomes: np.ndarray, minimize = False) -> np.ndarray:
    res = np.array([outcomes[0]])

    for i in range(1, outcomes.shape[0]):
        if minimize:
            v = outcomes[i] if outcomes[i] < res[i-1] else res[i-1]
        else:
            v = outcomes[i] if outcomes[i] > res[i-1] else res[i-1]
        res = np.append(res, v)
    return res

def outcome_iterations(outcomes: "list[np.ndarray]", best_acum=False, errors: "list[np.ndarray]" = None, names: "list[str]" = None):
    
    plt.figure()
    for i in range(len(outcomes)):
        outs = outcomes[i]
        iterations = range(1, outs.shape[0]+1)
        if best_acum:
            title = 'Best outcome until iteration x'
            Y = compute_max_until_iteration(outs)
        else:
            Y = outs
            title = 'Value of best selected sample - ' + OBJ_FUNC_NAME

        """if errors is not None:
            plt.errorbar(iterations, Y, yerr=errors[i], fmt='o', label=names[i], alpha=0.7)
        else:
            plt.plot(iterations, Y, 'o-')"""
        plt.plot(iterations, Y, label=names[i], color=COLORS[i])
        if errors is not None:
            plt.fill_between(iterations, Y - errors[i], Y + errors[i], alpha=0.3, color=COLORS[i])
    
    if names:
        plt.legend()
    
    if OBJ_FUNCTION != None:
        go = OBJ_FUNCTION.get_global_optima()
        if go != None:
            plt.plot(iterations, [go] * len(outcomes[-1]), '--', label="Global optima")

    plt.xlabel('Iteration')
    plt.ylabel(METRIC)
    plt.title(title)

def outcome_vars(queries: np.ndarray, outcomes: np.ndarray, var_labels: "list[str]", plot2D=False, name: str = ""):
    n_vars = len(var_labels)

    fig = plt.figure()
    if n_vars == 1:
        sc_plot = plt.scatter(queries[:, 0], outcomes, c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel(METRIC)
    elif plot2D:
        sc_plot = plt.scatter(queries[:, 0], queries[:, 1], c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel(var_labels[1])
    else:
        ax = fig.add_subplot(projection='3d')
        if n_vars == 2:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], outcomes, c=outcomes, alpha=0.5)
            ax.set_zlabel(METRIC)
        else:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], queries[:, 2], c=outcomes, alpha=0.5)
            ax.set_zlabel(var_labels[2])

        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])

    plt.colorbar(sc_plot, label=METRIC, orientation="vertical")
    
    plt.title("Distribution of the outcome " + name)

def plot_best(grasps: np.ndarray, outcomes: np.ndarray, var_labels: "list[str]", plot_text = False, name: str = ""):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for i, gs in zip(range(grasps.shape[0]), grasps):
        ax.scatter(gs[:, 0], gs[:, 1], gs[:, 2], marker='o', alpha=1.0)
        if plot_text:
            for j in range(gs.shape[0]):
                ax.text(gs[j, 0], gs[j, 1], gs[j, 2], str(round(outcomes[i][j], 3)))

    ax.set_xlabel(var_labels[0])
    ax.set_ylabel(var_labels[1])
    ax.set_zlabel(var_labels[2])
    
    
    plt.title("Best grasps " + name)

def plot_best_cmp(grasps: "list[np.ndarray]", names: "list[str]", var_labels: "list[str]"):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for exp_graps, name in zip(grasps, names):
        ax.scatter(exp_graps[:, 0], exp_graps[:, 1], exp_graps[:, 2], marker='o', alpha=0.3, label=name, s=200)

    ax.set_xlabel(var_labels[0])
    ax.set_ylabel(var_labels[1])
    ax.set_zlabel(var_labels[2])
    
    plt.legend()
    
    plt.title("Best grasps")

def get_values(file_path: str) -> "tuple[str, list[str], np.ndarray, np.ndarray, tuple[np.ndarray, np.ndarray]]":
    """
    input: log file
    output: optimizer name, active variables, grasps, outcomes, best grasps
    """
    global OBJ_FUNC_NAME

    logger = DataLog(log_file=file_path)
    logger.load_json()

    OBJ_FUNC_NAME = logger.obj_function["name"]

    act_vars = logger.get_active_vars()
    queries, outcomes = logger.get_queries(metric=METRIC, minimize=MINIMIZE)
    best = logger.get_best_queries(metric=METRIC)

    q_np = np.array(queries)
    res_np = np.array(outcomes).reshape(-1)

    return logger.get_optimizer_name(), act_vars, q_np, res_np, (np.array(best[0]), np.array(best[1]).reshape(-1)), None #np.array([mean_dist, var_norm, coef_var])

def get_folder_values(folder_path: str) -> "tuple[str, list[str], np.ndarray, np.ndarray, np.ndarray, np.ndarray]":
    """
    input: log file
    output: optimizer name, active variables, all grasps, all outcomes, best grasps, best outcomes
    """

    logs = glob.glob(folder_path + "/*.json")
    print("Num. log files: " + str(len(logs)))

    print("Adding", logs[0])
    optimizer_name, act_vars, grasps, outcomes, best, dist = get_values(logs[0])
    all_grasps = [grasps]
    all_outcomes = [outcomes]
    all_dists = [dist]
    if best[0].shape[0] > 0:
        all_best_grasps = [best[0]]
        all_best_outcomes = [best[1]]
    else:
        all_best_grasps = []
        all_best_outcomes = []

    for file in logs[1:]:
        print("Adding", file)
        _, _, grasps, outcomes, best, dist = get_values(file)
        all_grasps.append(grasps)
        all_outcomes.append(outcomes)
        all_dists.append(dist)
        if best[0].shape[0] > 0:
            all_best_grasps.append(best[0])
            all_best_outcomes.append(best[1])
            
    
    all_grasps = np.array(all_grasps)
    all_outcomes = np.array(all_outcomes)
    all_dists = None # np.array(all_dists)

    return optimizer_name, act_vars, all_grasps, all_outcomes, all_best_grasps, all_best_outcomes, all_dists


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)
    parser.add_argument('-minimize', help='Minimize result', action='store_true')
    parser.add_argument('-pbest', help='Plot best results', action='store_true')
    parser.add_argument('-no-plot', help='No show plots', action='store_true')
    parser.add_argument("-save", type=str, help="save plot", metavar=('<image_name_prefix>'))
    parser.add_argument("-metric", type=str, help="metric name to evaluate", metavar=('<metric_name>'), default="outcome")

    args = parser.parse_args()
    flogs = args.flogs
    MINIMIZE = args.minimize
    plot_best_enabled = args.pbest
    no_plot = args.no_plot
    if args.save:
        img_name_prefix = args.save
    else:
        img_name_prefix = ""

    METRIC = args.metric

    names = []
    mean_max_outcomes = []
    std_dev_max_outcomes = []
    best_grasps = [[], []]

    for flog in flogs:
        print("-------------------------------------------------")
        n_logs = 0
        var_queries = []
        if os.path.isdir(flog):  
            print("Loading results from " + flog + " folder")
            optimizer_name, act_vars, all_grasps, all_outcomes, all_best_grasps, all_best_outcomes, all_dists = get_folder_values(flog)
            n_logs = all_outcomes.shape[0]
            max_outcomes = np.array([compute_max_until_iteration(outs, minimize=MINIMIZE) for outs in all_outcomes])
            mean_max_outcomes.append(np.mean(max_outcomes, axis=0))
            std_dev_max_outcomes.append(np.std(max_outcomes, axis=0))

            ndim = all_grasps[0].shape[1]
            all_grasps = all_grasps.reshape(-1,ndim)
            all_outcomes = all_outcomes.reshape(-1,)

        else:
            print("Loading results from " + flog + " file")
            n_logs=1
            optimizer_name, act_vars, grasps, outcomes, best, all_dists = get_values(flog)

            all_grasps = grasps
            all_outcomes = outcomes

            if best[0].shape[0] > 0:
                all_best_grasps = [best[0]]
                all_best_outcomes = [best[1]]
            else:
                all_best_grasps = []
                all_best_outcomes = []

            max_outcomes = compute_max_until_iteration(outcomes, minimize=MINIMIZE)
            mean_max_outcomes.append(max_outcomes)
            std_dev_max_outcomes.append(np.zeros((max_outcomes.shape)))
        
        names.append(optimizer_name)

        print("Optimizer: " + optimizer_name)
        print("Active variables: " + str(act_vars))
        print("Num. total samples: " + str(all_outcomes.shape[0]))
        print("Mean Best solution: " + str(mean_max_outcomes[-1][-1]))
        if MINIMIZE:
            b_idx = np.argmin(all_outcomes)
        else:
            b_idx = np.argmax(all_outcomes)
        print("Best solution: " + str(all_outcomes[b_idx]) + " at " + str(all_grasps[b_idx]))
        print("All solutions:")
        for q, v in zip(all_best_grasps, all_best_outcomes):
            print("\tQuery: " + str(q) + " -> " + str(v))

    
    OBJ_FUNCTION = create_objective_function(OBJ_FUNC_NAME, METRIC if METRIC != "outcome" else "basic")
    outcome_iterations(mean_max_outcomes, errors=std_dev_max_outcomes, names=names)

    if plot_best_enabled and len(best_grasps) > 0:
        plot_best_cmp(best_grasps[0], names, act_vars)

    if not no_plot:
        plt.show()
    
    if img_name_prefix != "":
        plt.savefig(img_name_prefix + "_cmp_it.png")
