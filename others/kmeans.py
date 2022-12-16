import math
import numpy as np

def exists(E,i):
    # return 'True' if the point i exists 
    # in the array E, and 'False' unless otherwise
    return 0 < np.shape(np.array( \
        [ e for e in np.array(E) if (e == i).all() ]))[0]

def eucld(i1,i2):
    # Compute the squared Euclidean distance d=|i1-i2|^2 as the sum of squared 
    # distances between points i1 and i2, at each dimension
    return np.sum(np.array([ \
        math.pow(i1 - i2, 2.0) \
          for i1, i2 in zip(i1, i2) ]))

def initialize(X):
    # Get the random centroid c0
    c0 = np.random.randint(0, np.shape(X)[0] - 1) + 1
    # Compute the distance from centroid c0 to each point in X
    c0_d = np.array([ eucld(X[c0], x) for x in X ])
    
    # Get the centroid c1's as one of the points in X,
    # having the maximum distance to the centroid c0
    c1 = np.where(c0_d >= np.max(c0_d))[0][0]
    
    return np.array([c0,c1]) # Return the indexes of c0 and c1

def intra_cluster_d(X,S,r):
    di = 0; count = 0
    # For each cluster in S compute within-cluster average distance
    # as the sum of the observation distances to the centroid C[r]
    # divided by their quantity
    for i in range(np.shape(S)[0]):
        # Accumulate the distances of each observation, mapped onto
        # the centroid c[r] of the cluster s[i], to the variable di
        if S[i]['c'] == r: 
            di = di + math.sqrt(S[i]['d']); 
            count = count + 1
    
    return float(di) / count # Return an average of the distances

def inter_cluster_d(X,C):
    di = 0; count = np.shape(C)[0]
    # Get the number of inter-cluster distances 
    # between pairs of centroids c[i] and c[j]
    count = math.pow(count, 2.0) - count
    # For each pair of centroids c[i] and c[j]
    for i in range(np.shape(C)[0]):
        for j in range(np.shape(C)[0]):
            # Compute the distance between C[i] and C[j], 
            # accumulating it to the variable di
            di = di + math.sqrt(eucld(X[C[i]], X[C[j]]))
            
    return float(di) / count  # Return an average of the distances

def compute(X,Y,k):
    X = np.array(X)    # X - an input dataset of n-observations
    C = initialize(X)   # C - an initial set of centroids
    
    # Perform the dataset clustering iteratively, 
    # until the resultant set of k-clusters has been computed
    
    while True:
        S = np.empty(0)  # S - a set of newly built clusters
        
        # For each observation x[t] in X, do the following:
        for t in range(np.shape(X)[0]):
            # Check if the observation x[t] has already been
            # selected as one of the new centroids
            if exists(C, t) == False:
                # If not, compute the distance from 
                # the observation x[t] to each of the existing centroids in C
                cn_ds = np.array([ eucld(X[t], X[c]) for c in C ])
                # Get the centroid c[r] for which the distance to x[t] is the smallest
                cn_min_di = np.where(cn_ds == np.min(cn_ds))[0][0]

                # Assign the observation x[t] to the new cluster s[r], appending
                # the observation x[t]'s and centroid c[r]'s indexes to the set S
                S = np.append(S, { 'c': cn_min_di, 'i': t, 'd': cn_ds[cn_min_di] })

        # Terminate the clustering process, if the number of centroids 
        # in C is equal to the total number of clusters k, initially specified.
        
        # Otherwise, compute the next centroid c[r] in C
                
        if np.shape(C)[0] >= k: break
        
        # Get the distances |x-c| from the observations 
        # accross all clusters in S to each of the centroids in C
        cn_ds = np.array([s['d'] for s in S ])
        
        # Compute the index of an observation, for which 
        # the distance to one of the centroids in C is the largest
        cn_max_ci = np.where(cn_ds == np.max(cn_ds))[0][0]

        # Append the index of a new centroid c[r] to the set C
        C = np.append(C, S[cn_max_ci]['i'])
    
    """clusters_x = []
    clusters_y = []
    for ci in range(k):
        Xs = np.array([ X[s['i']] for s in S if s['c'] == ci ])
        Ys = np.array([ Y[s['i']] for s in S if s['c'] == ci ])
        if Xs.shape[0] == 0:
            Xs = np.array(X[C[ci]]).reshape(1, -1)
            Ys = np.array(Y[C[ci]]).reshape(-1)
        else:
            Xs = np.vstack((Xs, X[C[ci]]))
            Ys = np.append(Ys, Y[C[ci]])
        clusters_x.append(Xs)
        clusters_y.append(Ys)

    return clusters_x, clusters_y"""

    clusters = []
    n = 0
    for ci in range(k):
        _c = [s['i'] for s in S if s['c'] == ci ]
        _c += [C[ci]]
        n += len(_c)
        clusters.append(_c)
    
    return clusters
