import numpy as np
import matplotlib.pyplot as plt
import argparse
import json

from multisolution_active_grasping.core.objective_function import ObjectiveFunction
from multisolution_active_grasping.utils.utils import create_objective_function

NUM_SAMPLES_DIM = 1000
ACTIVE_VARS = ["x"]
OBJ_FUNCTION=None

def plot_function(X, Y,tittle, value_title, plot2d = False):
    ndim = X.shape[1]
    plot2D = False
    fig = plt.figure()
    if ndim == 1:
        sc_plot = plt.scatter(X, Y, c=Y, alpha=0.5)
        plt.xlabel(ACTIVE_VARS[0])
        plt.ylabel(value_title)
    elif ndim == 2 and plot2D:
        sc_plot = plt.scatter(X[:, 0], X[:, 1], c=Y, alpha=0.5)
        plt.xlabel(ACTIVE_VARS[0])
        plt.ylabel(ACTIVE_VARS[1])
    else:
        ax = fig.add_subplot(projection='3d')
        if ndim == 2:
            sc_plot = ax.scatter(X[:, 0], X[:, 1], Y, c=Y, alpha=0.5)
            ax.set_zlabel(value_title)
        else:
            sc_plot = ax.scatter(X[:, 0], X[:, 1], X[:, 2], c=Y, alpha=0.5)
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-objf", nargs=2, help="objective function", metavar=('<objective_function>', '<params_file>'), required=True)
    parser.add_argument("-exp", help="exp params", metavar='<exp_params>', required=True)
    parser.add_argument("-metric", type=str, help="metric type", metavar='<metric_type>', default="basic")
    parser.add_argument("-n", type=int, help="num samples", metavar='<num_samples>', default=NUM_SAMPLES_DIM)
    parser.add_argument("-f", nargs='+', help="exec files", metavar='<exec_files>+', default=[])

    args = parser.parse_args()
    objf_name = args.objf[0]
    objf_fparams = args.objf[1]
    metric = args.metric
    NUM_SAMPLES_DIM = args.n
    f = open(args.exp, 'r')
    exp_params = json.load(f)
    exec_files = args.f

    OBJ_FUNCTION = create_objective_function(objf_name, metric, fparams=objf_fparams)
    
    ACTIVE_VARS = exp_params["active_variables"]
    lower_bound = np.array(exp_params["lower_bound"])
    upper_bound = np.array(exp_params["upper_bound"])
    default_query = exp_params["default_query"]
    if len(default_query) > 0:
            OBJ_FUNCTION.set_default_query(default_query)
    
    ndim = lower_bound.shape[0]
    n_samples = NUM_SAMPLES_DIM # ** ndim
    X = np.random.uniform(lower_bound, upper_bound, (n_samples, ndim))
    
    print(70 * '=')
    print("TRUE FUNCTION")
    show_function(X)
    show_L(X)

    for f in exec_files:
        print(70 * '=')
        print("FILE: " + f)
        plot_file(f)
    plt.show()
    
