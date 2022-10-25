import numpy as np
from scipy.stats import norm

def hammer_precompute(v: np.ndarray, L: float, Min: float):
    m = np.sum(v, axis=1)
    pred = m / 2.0

    # print("mean",m,m.shape)
    # print("std",pred, pred.shape)
    pred[pred<1e-16] = 1e-16
    s = np.sqrt(pred)
    r_x0 = (m-Min)/L
    s_x0 = s/L
    r_x0 = r_x0.flatten()
    s_x0 = s_x0.flatten()
    # print("r",r_x0, r_x0.shape)
    # print("s",s_x0, s_x0.shape)

    return r_x0, s_x0

def hammer_function(x, x0,r_x0, s_x0):
    '''
    Creates the function to define the exclusion zones
    '''

    print("x", x, x.shape)
    print("x0", x0, x0.shape)
    subs_num = np.atleast_2d(x)[:,None,:] - np.atleast_2d(x0)[None,:,:]
    print("subs_num", subs_num, subs_num.shape)
    square_num = np.square(subs_num)
    print("square_num", square_num, square_num.shape)
    sum_num = (square_num).sum(-1)
    print("sum_num", sum_num, sum_num.shape)
    sqrt_num = np.sqrt(sum_num)
    print("sqrt_num", sqrt_num, sqrt_num.shape)
    num = sqrt_num - r_x0

    print("num", num)
    div = num / s_x0
    print("div", div)
    return norm.logcdf(div)

def penalized_acquisition(x, X_batch, r_x0, s_x0, transform='none'):
        '''
        Creates a penalized acquisition function using 'hammer' functions around the points collected in the batch

        .. Note:: the penalized acquisition is always mapped to the log space. This way gradients can be computed additively and are more stable.
        '''
        fval = -x.sum()/100000
        fval = -fval
        print("fval", fval)

        if transform=='softplus':
            fval = np.log(np.log1p(np.exp(fval)))
        elif transform=='none':
            fval = np.log(fval+1e-50)

        fval = -fval
        print("fval", fval)
        if X_batch is not None:
            h_vals = hammer_function(x, X_batch, r_x0, s_x0)
            print("h_vals", h_vals)
            print(h_vals.sum(axis=-1))
            fval += -h_vals.sum(axis=-1)
        return fval

if __name__ == '__main__':
    X = np.array([[0.0, 0.23], [0.142, 0.122], [0.05, 0.89]])
    L = 10.0
    Min = 0.0
    x = np.array([1.0, 0.5]).reshape(1,2)
    
    r,s = hammer_precompute(X, L, Min)
    print("r",r)
    print("s", s)

    # exclusion = hammer_function(x, X, r, s)
    # print("exclusion", exclusion, exclusion.shape)
    fval = penalized_acquisition(x, X, r, s)
    print("fval", fval)
