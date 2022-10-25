import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

from multisolution_active_grasping.core.datalog import DataLog

COLORS=['b', 'r', 'g', '#ff7700']

def compute_max_until_iteration(outcomes: np.ndarray, minimize = False) -> np.ndarray:
    res = np.array([outcomes[0]])

    for i in range(1, outcomes.shape[0]):
        if minimize:
            v = outcomes[i] if outcomes[i] < res[i-1] else res[i-1]
        else:
            v = outcomes[i] if outcomes[i] > res[i-1] else res[i-1]
        res = np.append(res, v)
    return res

def outcome_iterations(outcomes: np.ndarray, best_acum=False, errors: np.ndarray = None, names: "list[str]" = None):
    
    iterations = range(1, outcomes.shape[1]+1)
    plt.figure()
    for i, outs in zip(range(outcomes.shape[0]), outcomes):
        if best_acum:
            title = 'Best outcome until iteration x'
            Y = compute_max_until_iteration(outs)
        else:
            Y = outs
            title = 'Value of best selected sample'

        """if errors is not None:
            plt.errorbar(iterations, Y, yerr=errors[i], fmt='o', label=names[i], alpha=0.7)
        else:
            plt.plot(iterations, Y, 'o-')"""
        plt.plot(iterations, Y, label=names[i], color=COLORS[i])
        if errors is not None:
            plt.fill_between(iterations, Y - errors[i], Y + errors[i], alpha=0.3, color=COLORS[i])
    
    if names:
        plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(title)

def outcome_vars(queries: np.ndarray, outcomes: np.ndarray, var_labels: "list[str]", plot2D=False, name: str = ""):
    n_vars = len(var_labels)

    fig = plt.figure()
    if n_vars == 1:
        sc_plot = plt.scatter(queries[:, 0], outcomes, c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel("outcome")
    elif plot2D:
        sc_plot = plt.scatter(queries[:, 0], queries[:, 1], c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel(var_labels[1])
    else:
        ax = fig.add_subplot(projection='3d')
        if n_vars == 2:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], outcomes, c=outcomes, alpha=0.5)
            ax.set_zlabel("outcome")
        else:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], queries[:, 2], c=outcomes, alpha=0.5)
            ax.set_zlabel(var_labels[2])

        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])

    plt.colorbar(sc_plot, label="outcome", orientation="vertical")
    
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

    logger = DataLog(log_file=file_path)
    logger.load_json()

    act_vars = logger.get_active_vars()
    queries, outcomes = logger.get_queries()
    best = logger.get_best_queries()

    q_np = np.array(queries)
    res_np = np.array(outcomes).reshape(-1)

    # n_grasps = grasps.shape[0]
    """dist = np.zeros(n_grasps)
    for i in range(n_grasps):
        dg = 0
        n = 1
        for j in range(n_grasps):
            if i == j: continue
            dg += np.linalg.norm(grasps[i] - grasps[j])
            n += 1
        dist[i] = dg / n
    
    mean_dist = np.mean(dist)
    std_dist = np.std(dist)
    
    # var_dist = np.var(grasps, axis=0)
    # var_dist = np.mean(var_dist)
    gs = grasps + np.abs(np.min(grasps, axis=0))
    _std = np.std(gs, axis=0)
    _mean = np.mean(gs, axis=0)
    _coef = _std / _mean
    coef_var = np.mean(_coef)
    
    _size = np.max(grasps, axis=0) - np.min(grasps, axis=0)
    gs = gs / _size
    var_norm = np.var(gs)"""

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

    args = parser.parse_args()
    flogs = args.flogs
    minimize = args.minimize

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
            max_outcomes = np.array([compute_max_until_iteration(outs, minimize=minimize) for outs in all_outcomes])
            mean_max_outcomes.append(np.mean(max_outcomes, axis=0))
            std_dev_max_outcomes.append(np.std(max_outcomes, axis=0))

            all_grasps = all_grasps.reshape(-1,3)
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

            max_outcomes = compute_max_until_iteration(outcomes, minimize=minimize)
            mean_max_outcomes.append(max_outcomes)
            std_dev_max_outcomes.append(np.zeros((max_outcomes.shape)))
        
        names.append(optimizer_name)

        print("Optimizer: " + optimizer_name)
        print("Active variables: " + str(act_vars))
        print("Num. total grasps: " + str(all_grasps.shape[0]))
        # print("Avg. distance between grasps: " + str(np.mean(all_dists[:,0])))
        # print("Avg. normalized variance between queries: " + str(np.mean(all_dists[:,1])))
        # print("Avg. coef. of var. between queries: " + str(np.mean(all_dists[:,2])))
        print("Mean Best grasp: " + str(mean_max_outcomes[-1][-1]))
        print("Best grasp: " + str(np.max(all_outcomes)))

        # outcome_vars(all_grasps, all_outcomes, plot2D=False, var_labels=act_vars, name=optimizer_name)
        
        # plot_best(all_best_grasps, all_best_outcomes.reshape((-1,)), act_vars, name=optimizer_name, plot_text=True)

        n_exp = len(all_best_grasps)
        print("Experiments with solution: " + str(n_exp))
        if n_exp > 0:
            """best_dists = np.zeros(n_exp)
            best_axis_dists = np.zeros((n_exp, 3))
            best_std_dists = np.zeros(n_exp)
            best_std_axis_dists = np.zeros((n_exp, 3))"""
            n_bests = np.zeros(n_exp)
            
            """_b_g = all_best_grasps[0]
            _b_o = all_best_outcomes[0]

            for idx_exp in range(n_exp):
                best_g = all_best_grasps[idx_exp]
                best_o = all_best_outcomes[idx_exp]
                if idx_exp > 0:
                    _b_g = np.append(_b_g, best_g, axis=0)
                    _b_o = np.append(_b_o, best_o)
                n_g_exp = best_g.shape[0]
                n_bests[idx_exp] = n_g_exp

                exp_dist = np.zeros(n_g_exp)
                exp_axis_dist = np.zeros((n_g_exp, 3))
                for i in range(n_g_exp):
                    dist = 0
                    ax_dist = np.zeros(3)
                    n = 1
                    for j in range(n_g_exp):
                        if i == j: continue
                        d = best_g[i] - best_g[j]
                        ax_dist += np.abs(d)
                        dist += np.linalg.norm(d)
                        n += 1
                    exp_dist[i] = dist / n
                    exp_axis_dist[i] = ax_dist / n
                
                mean_dist = np.mean(exp_dist)
                std_dev_dist = np.std(exp_dist)
                mean_axis_dist = np.mean(exp_axis_dist, axis=0)
                std_dev_axis_dist = np.std(exp_axis_dist, axis=0)

                best_dists[idx_exp] = mean_dist
                best_std_dists[idx_exp] = std_dev_dist
                best_axis_dists[idx_exp] = mean_axis_dist
                best_std_axis_dists[idx_exp] = std_dev_axis_dist
                
                # print("\tExp " + str(idx_exp) + " | " + str(n_g_exp) + " grasps:")
                # print("\t\tDistance between best grasps: mean = " + str(mean_dist) + " mm, std dev = " + str(std_dev_dist) + " mm")
                # print("\t\tAxis distance between best grasps: " + str(mean_axis_dist) + " mm, std dev = " + str(std_dev_axis_dist) + " mm")

            total_bests = np.sum(n_bests)
            weights = n_bests / total_bests

            comb_mean_dist = np.sum(weights * best_dists)
            comb_std_dev_dist = np.sqrt(np.sum(weights * (np.power(best_std_dists, 2) + np.power(best_dists - comb_mean_dist, 2))))
            print("Distance between best grasps per experiment: mean = " + str(comb_mean_dist) + " mm, std dev = " + str(comb_std_dev_dist) + " mm")

            comb_mean_axis_dist = np.sum((weights * best_axis_dists.T).T, axis=0)
            d = best_axis_dists - comb_mean_axis_dist
            d2 = np.power(d, 2)
            sd2 = np.power(best_std_axis_dists, 2)
            comb_std_axis_dist = np.sqrt(np.sum((weights * (sd2 + d2).T).T, axis=0))
            print("Axis distance between best grasps per experiment: mean = " + str(comb_mean_axis_dist) + " mm, std dev = " + str(comb_std_axis_dist) + " mm")"""
            
            var_exps = np.zeros((n_exp,3))
            var_outcomes = np.zeros(n_exp)
            a = 0
            for i in range(n_exp):
                bg = all_best_grasps[i]
                n_bests[i] = bg.shape[0]

                gs = bg + np.abs(np.min(bg, axis=0))
                var_exps[i] = np.var(gs, axis=0)
                a += np.var(gs[:,0])
                #print(var_exps[i])

                var_outcomes[i] = np.var(all_best_outcomes[i])
                #print(var_outcomes[i])
            #print(var_exps)
            #print(a/n_exp)
            total_bests = np.sum(n_bests)
            print("Num. total best grasps: " + str(total_bests))
            n_sols = np.zeros(n_logs)
            n_sols[:n_exp] = n_bests
            avg_solutions = np.mean(n_sols)
            std_dev_solutions = np.std(n_sols)
            print("Avg. solutions per experiment: " + str(avg_solutions))
            print("Std. dev. solutions per experiment: " + str(std_dev_solutions))

            print("Variance best grasps (params): mean = " + str(np.mean(var_exps, axis=0)) + ", std dev = " + str(np.std(var_exps, axis=0)))
            #print(list(np.mean(var_exps, axis=0)), list(np.std(var_exps, axis=0)))
            print("Variance best grasps (metric): mean = " + str(np.mean(var_outcomes)) + ", std dev = " + str(np.std(var_outcomes)))
            #print(np.mean(var_outcomes), np.std(var_outcomes))


            
            # best_grasps[0].append(_b_g)
            # best_grasps[1].append(_b_o)

    # names = ['BO', 'MS', 'CAS50', 'CAS80']
    outcome_iterations(np.array(mean_max_outcomes), errors=np.array(std_dev_max_outcomes), names=names)

    # if len(best_grasps) > 0:
    #     plot_best_cmp(best_grasps[0], names, act_vars)

    plt.show()
