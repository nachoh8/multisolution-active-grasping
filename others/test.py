import numpy as np
import matplotlib.pyplot as plt


X = np.array([i for i in range(4, 20)])
S = np.array([i for i in range(2,5)])
plt.figure()
for x in X:
    for ns in S:
        if ns > x:
            Y = x
        else:
            Y = np.math.factorial(x) / (np.math.factorial(ns) * np.math.factorial(x-ns))
        sc_plot = plt.scatter(x, ns, c=Y)

plt.xlabel("#points")
plt.ylabel("#solutions")
plt.colorbar(sc_plot, label="#combinations", orientation="vertical")

plt.figure()
Y = np.array([np.math.factorial(x) / (np.math.factorial(4) * np.math.factorial(x-4)) for x in X])
sc_plot = plt.scatter(X, Y, c=Y)
plt.xlabel("#points")
plt.ylabel("#combinations")

plt.show()