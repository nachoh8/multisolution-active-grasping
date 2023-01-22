import sys
sys.path.append("../")

import numpy as np
import math
import matplotlib.pyplot as plt
import argparse
import json

from OptSystem.core.datalog import DataLog
from OptSystem.core.objective_function import ObjectiveFunction
from OptSystem.utils.utils import create_objective_function

NUM_SAMPLES_DIM = 100
ACTIVE_VARS = ["x"]
OBJ_FUNCTION: ObjectiveFunction =None
PLOT2D=False

def plot_function(X, Y,tittle, value_title, go=None):
    plot2d = PLOT2D

    ndim = X.shape[1]
    fig = plt.figure()
    if ndim == 1:
        sc_plot = plt.scatter(X, Y, c=Y, alpha=0.5)
        if go is not None:
            plt.scatter(go[0], go[1], c='r')
        plt.xlabel(ACTIVE_VARS[0])
        plt.ylabel(value_title)
    elif ndim == 2 and plot2d:
        sc_plot = plt.scatter(X[:, 0], X[:, 1], c=Y, alpha=0.5)
        if go is not None:
            plt.scatter(go[0][:, 0], go[0][:, 1], c='r')
        plt.xlabel(ACTIVE_VARS[0])
        plt.ylabel(ACTIVE_VARS[1])
    else:
        ax = fig.add_subplot(projection='3d')
        if ndim == 2:
            sc_plot = ax.scatter(X[:, 0], X[:, 1], Y, c=Y, alpha=0.5)
            if go is not None:
                ax.scatter(go[0][:, 0], go[0][:, 1], go[1], c='r')
            ax.set_zlabel(value_title)
        else:
            sc_plot = ax.scatter(X[:, 0], X[:, 1], X[:, 2], c=Y, alpha=0.5)
            if go is not None:
                ax.scatter(go[0][:, 0], go[0][:, 1], go[0][:, 2], c='r', s=100)
            ax.set_zlabel(ACTIVE_VARS[2])

        ax.set_xlabel(ACTIVE_VARS[0])
        ax.set_ylabel(ACTIVE_VARS[1])
    
    plt.colorbar(sc_plot, label=value_title, orientation="vertical")
    plt.title(tittle)

def L_aprox(x: np.ndarray) -> float:
    epsilon = 1e-7
    x = np.atleast_2d(x)
    n = x.shape[1]
    g_aprox = np.zeros(x.shape)
    for d in range(n):
        e = np.zeros(n)
        e[d] = epsilon
        
        v_p = OBJ_FUNCTION._evaluate(x.flatten() + e)
        v_m = OBJ_FUNCTION._evaluate(x.flatten() - e)
        g_aprox[0, d] = (v_p - v_m) / (2.0 * epsilon)

    l_aprox = np.sqrt((g_aprox*g_aprox).sum())
    return -l_aprox

def show_L(X: np.ndarray):
    v_laprox = np.vectorize(L_aprox, signature='(n)->()')
    l_aprox = v_laprox(X)

    min_idx = np.argmin(l_aprox)
    max_idx = np.argmax(l_aprox)
    print("Lmin:", X[min_idx], " -> ", l_aprox[min_idx])
    print("Lmax:", X[max_idx], " -> ", l_aprox[max_idx])
    print("Lmean:", np.mean(l_aprox))

    plot_function(X, l_aprox, OBJ_FUNCTION.get_name() + " - L", "L")

def evaluate_f(x: np.ndarray):
    x = np.atleast_1d(x)
    query = dict(zip(ACTIVE_VARS, list(x)))
    return OBJ_FUNCTION.execute(query).get_metrics()[0][1]

def show_function(X: np.ndarray):
    v_evaluate = np.vectorize(evaluate_f, signature='(n)->()')
    Y = v_evaluate(X)

    min_idx = np.argmin(Y)
    max_idx = np.argmax(Y)
    print("Fmin:", X[min_idx], " -> ", Y[min_idx])
    print("Fmax:", X[max_idx], " -> ", Y[max_idx])
    print("Fmean:", np.mean(Y))
    
    plot_function(X, Y, OBJ_FUNCTION.get_name(), "outcome")

    return Y
    
def plot_file(fname: str):
    f = open(fname, 'r')
    lines = f.readlines()
    points = []
    l_v = []
    m_v = []
    s_v = []
    for line in lines:
        line_data = line.split()
        if line_data[0] == "point:":
            pt_str = line_data[1]
            pt = [float(pt_str[1:len(pt_str)-1])]
            points.append(pt)

            l_str = line_data[3]
            l = float(l_str)
            l_v.append(l)

            m_str = line_data[5]
            m = float(m_str)
            m_v.append(m)

            s_str = line_data[7]
            s = float(s_str)
            s_v.append(s)
    
    points = np.array(points)
    l_v = np.array(l_v)
    m_v = np.array(m_v)
    s_v = np.array(s_v)

    print("Num points: " + str(l_v.shape[0]))

    min_idx = np.argmin(m_v)
    max_idx = np.argmax(m_v)
    print("Fmin:", points[min_idx], " -> ", m_v[min_idx])
    print("Fmax:", points[max_idx], " -> ", m_v[max_idx])
    print("Fmean:", np.mean(m_v))

    min_idx = np.argmin(l_v)
    max_idx = np.argmax(l_v)
    print("Lmin:", points[min_idx], " -> ", l_v[min_idx])
    print("Lmax:", points[max_idx], " -> ", l_v[max_idx])
    print("Lmean:", np.mean(l_v))

    plot_function(points, m_v, fname + " - mean prediction", "outcome")
    plot_function(points, l_v, fname + " - L prediction", "L")

def get_global_optimas(queries: np.ndarray, values: np.ndarray, go: np.ndarray, go_value: float):
    euclidean_distance = lambda x1, x2: np.sqrt(np.sum((x1 - x2) ** 2))
    MINIMIZE = False
    if go_value == 0.0:
        ACCURACY = 1.0 - 0.95
    else:
        ACCURACY = abs(go_value) * (1.0 - 0.95)

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
    order = order[::-1]
    sorted_v = values[order]
    sorted_q = queries[order, :]
    radius = OBJ_FUNCTION.get_exclusion_radius()
    if go_value == 0.0:
        ACCURACY = 1.0 - 0.9
    else:
        ACCURACY = abs(go_value) * (1.0 - 0.9)
    
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-objf", nargs=2, help="objective function", metavar=('<objective_function>', '<params_file>'), default=("", ""))
    parser.add_argument("-flogs", nargs='+', help="exec files", metavar='<exec_files>+', default=[])
    parser.add_argument("-exp", help="exp params", metavar='<exp_params>', default="")
    parser.add_argument("-metric", type=str, help="metric type", metavar='<metric_type>', default="basic")
    parser.add_argument("-n", type=int, help="num samples", metavar='<num_samples>', default=NUM_SAMPLES_DIM)
    parser.add_argument('-p2d', help='plot in 2D', action='store_true')

    args = parser.parse_args()
    objf_name = args.objf[0]
    objf_fparams = args.objf[1]
    metric = args.metric
    NUM_SAMPLES_DIM = args.n
    PLOT2D=args.p2d

    OBJ_FUNCTION = create_objective_function(objf_name, metric, fparams=objf_fparams)    
    
    if args.exp != "":
        f = open(args.exp, 'r')
        exp_params = json.load(f)
    else:
        exp_params = {}
        exp_params["active_variables"] = OBJ_FUNCTION.get_var_names()
        exp_params["lower_bound"] = OBJ_FUNCTION.get_lower_bounds()
        exp_params["upper_bound"] = OBJ_FUNCTION.get_upper_bounds()
        exp_params["default_query"] = {}
    
    logs = args.flogs

    
    ACTIVE_VARS = exp_params["active_variables"]
    lower_bound = np.array(exp_params["lower_bound"])
    upper_bound = np.array(exp_params["upper_bound"])
    default_query = exp_params["default_query"]
    """if len(default_query) > 0:
            OBJ_FUNCTION.set_default_query(default_query)"""
    
    ndim = lower_bound.shape[0]
    n_samples = NUM_SAMPLES_DIM # ** ndim
    if ndim == 1:
        X = np.linspace(lower_bound[0], upper_bound[0], n_samples).reshape(-1,1)
    else:
        samples_dim = [np.linspace(lb, ub, n_samples) for lb, ub in zip(lower_bound, upper_bound)]
        X = np.array(np.meshgrid(*samples_dim)).T.reshape(-1, ndim)
        # F7 GLOBAL OPTIMA
        # Region top-right: [3.15, 1.75] - [10,10] -> 6 global optima
        #    [7.71666667 7.66666667] -> 0.9992908574916917
        #    [4.11868687 7.66666667] -> 0.9992536367084481
        #    [7.71666667 4.08333333] -> 0.9987980328487298
        #    [4.11868687 4.08333333] -> 0.9987608120654864
        #    [7.71666667 2.16666667] -> 0.9962331984297024
        #    [4.11868687 2.16666667] -> 0.9961959776464588
        # Region bottom-right: [3.15, 0.25] - [10,1.5] -> 6 global optima
        #    [7.71666667 1.17171717] -> 0.9999062715155211
        #    [4.11868687 1.17171717] -> 0.9998690507322776
        #    [7.71666667 0.62878788] -> 0.9986311263197253
        #    [4.11868687 0.62878788] -> 0.9985939055364819
        #    [7.71666667 0.33838384] -> 0.9935817851895261
        #    [4.11868687 0.33838384] -> 0.9935445644062826
        # Region top-middle: [0.8, 1.55] - [3,10] -> 6 global optima
        #    [2.2        4.11060606] -> 0.9997655166129035
        #    [2.2        7.69545455] -> 0.9997166739472552
        #    [1.17777778 4.11060606] -> 0.9989273547912422
        #    [1.17777778 7.69545455] -> 0.9988785121255939
        #    [2.2        2.23282828] -> 0.9918030451168477
        #    [1.17777778 2.23282828] -> 0.9909648832951864
        # Region bottom-middle: [0.8, 0.25] - [3, 1.5] -> 6 global optima
        #    [2.2        1.17171717] -> 0.9997177001346976
        #    [1.17777778 1.17171717] -> 0.9988795383130363
        #    [2.2        0.62878788] -> 0.9984425549389018
        #    [1.17777778 0.62878788] -> 0.9976043931172405
        #    [2.2        0.33838384] -> 0.9933932138087025
        #    [1.17777778 0.33838384] -> 0.9925550519870412
        # Region top-left: [0.25, 1.55] - [0.8, 10] -> 6 global optima
        #    [0.33333333 4.11060606] -> 0.9999771334402208
        #    [0.33333333 7.69545455] -> 0.9999282907745726
        #    [0.62222222 4.11060606] -> 0.9997404257372268
        #    [0.62222222 7.69545455] -> 0.9996915830715785
        #    [0.33333333 2.23282828] -> 0.992014661944165
        #    [0.62222222 2.23282828] -> 0.991777954241171
        # Region bottom-left: [0.25, 0.25] - [0.8, 1.5] -> 6 global optima
        #    [0.33333333 1.17171717] -> 0.9999293169620149
        #    [0.62222222 1.17171717] -> 0.9996926092590209
        #    [0.33333333 0.62878788] -> 0.9986541717662192
        #    [0.62222222 0.62878788] -> 0.9984174640632251
        #    [0.33333333 0.33838384] -> 0.9936048306360199
        #    [0.62222222 0.33838384] -> 0.9933681229330258

    
    print(70 * '=')
    print("TRUE FUNCTION")
    v_evaluate = np.vectorize(evaluate_f, signature='(n)->()')
    Y = v_evaluate(X)

    min_idx = np.argmin(Y)
    max_idx = np.argmax(Y)
    print("Fmin:", X[min_idx], " -> ", Y[min_idx])
    print("Fmax:", X[max_idx], " -> ", Y[max_idx])
    print("Fmean:", np.mean(Y))

    # plot_function(X, Y, OBJ_FUNCTION.get_name(), "outcome")

    # Y = show_function(X)
    if False:
        plot_function(X, Y, OBJ_FUNCTION.get_name(), "outcome")
    else:
        true_go_points = OBJ_FUNCTION.get_global_optima_points()
        go_value = OBJ_FUNCTION.get_global_optima()
        nGO = OBJ_FUNCTION.get_num_global_optima()
                
        if true_go_points is None:
            go_q, go_v = global_optima_found(X,Y, go_value, nGO)
            for q, v, in zip(go_q, go_v):
                print(q, "->", v)
        else:
            go_q = true_go_points
            go_v = v_evaluate(go_q)
        # plot_function(X, Y, OBJ_FUNCTION.get_name(), "outcome", (go_q, go_v))
    
    # dist = np.zeros((go_v.shape[0], go_v.shape[0] - 1))
    # for q, v, i, in zip(go_q, go_v, range(go_v.shape[0])):
    #     print(q, "->", v)
        # d = []
    """for j in range(go_v.shape[0]):
            if i == j: continue
            d.append(np.linalg.norm(q - go_q[j]))
        dist[i] = np.array(d)"""
    # print("Mean dist between GO:", np.mean(np.mean(dist, axis=1)))
    # print("GO found:", go_v.shape[0], "/", OBJ_FUNCTION.get_num_global_optima())
    # show_L(X)

    """min_v = 0.95
    if go_value == 0.0:
        acc = 1.0 - min_v
    else:
        acc = abs(go_value) * (1.0 - min_v)"""

    for f in logs:
        print(70 * '=')
        print("FILE: " + f)
        # plot_file(f)
        logger = DataLog(log_file=f)
        logger.load_json()

        _queries, _outcomes = logger.get_queries(minimize=False, best_per_iteration=False, metric="outcome")
        queries = np.array(_queries)
        values = np.array(_outcomes).reshape(-1)
        min_idx = np.argmin(values)
        max_idx = np.argmax(values)
        print("Fmin:", queries[min_idx], " -> ", values[min_idx])
        print("Fmax:", queries[max_idx], " -> ", values[max_idx])
        print("Fmean:", np.mean(values))

        _best_q, _best_v = logger.get_best_queries(metric=metric)
        best_q = np.array(_best_q)
        best_v = np.array(_best_v)

        log_go_q, log_go_v, _ = get_global_optimas(queries, values, go_q, go_value)
        print("Global optima found:", log_go_q.shape[0])
        # print("Solutions:")
        # for q, v in zip(best_q, best_v):
        #     print("\t", q, "->", v) #, "| found:", math.fabs(v - go_value) <= acc)
        # print(values[values==0.0].shape[0], values[values>0.2].shape[0])
        """n = values.shape[0]
        l = int(n*0.7)
        b_size = logger.get_batch_size()
        if b_size > 1:
            n_init_pts = logger.get_num_init_points()
            _n = l - n_init_pts
            n_it = int(_n / b_size)
            n = n_init_pts + n_it * b_size"""
        # print(l)
        _q = queries
        _v = values
        
        plot_function(_q, _v, "Optimization - " + logger.get_optimizer_name(), metric, (log_go_q, log_go_v))
    plt.show()
    
