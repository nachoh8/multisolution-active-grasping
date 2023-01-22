import numpy as np
from pygrasp.pygrasp import *

print("----TEST EIGEN/NUMPY----")
eigen_v3f = test_eigen_numpy_type()
print(eigen_v3f)
print(type(eigen_v3f), eigen_v3f.shape, eigen_v3f.dtype)

np_v3f = np.ndarray((3,1), buffer=np.array([1.1, 2.2, 3.3]), dtype=np.float32)
print(eigen_v3f)
print(type(eigen_v3f), eigen_v3f.shape, eigen_v3f.dtype)

print()
print("----TEST GRASP PLANNER----")

params = EnvParameters() # empty
is_valid = loadEnvParametersFile("../config/tests/GP/bottle_params.json", params) # or from file
if not is_valid:
        print("Error: parsing grasp planner params")
        exit(1)

print("Scene file: " + params.scene_file)
print("Object: " + params.object)
print("EEF: " + params.eef)
print("EEF preshape: " + params.eef_preshape)

# set obj pose

planner = GraspPlanner(params)

# WARNING
# !!!!!
# c++ could access elements outside the range of the query vector and does not throw any errors
# !!!!
q1 = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
q2 = vectord(6, 0.0)

for q in [q1, q2]:
    res: GraspResult = planner.executeQueryGrasp(q)
    print("Result:")
    print("\tEpsilon measure:", res.measure)
    print("\tVolume measure:", res.volume)
    print("\tForce closure:", res.force_closure)
    print("\tError:", res.error)

print()
print("----TEST EIGEN GRASP PLANNER----")

print("Scene file: " + params.scene_file)
print("Object: " + params.object)
print("EEF: " + params.eef)
print("EEF preshape: " + params.eef_preshape)

# set obj pose

planner = EigenGraspPlanner(params)

# WARNING
# !!!!!
# c++ could access elements outside the range of the query vector and does not throw any errors
# !!!!
q1 = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
q2 = vectord(8, 0.0)

for q in [q1, q2]:
    res: GraspResult = planner.executeQueryGrasp(q)
    print("Result:")
    print("\tEpsilon measure:", res.measure)
    print("\tVolume measure:", res.volume)
    print("\tForce closure:", res.force_closure)
    print("\tEigengrasp1:", res.eigengrasp1)
    print("\tEigengrasp2:", res.eigengrasp2.shape)
    print("\tError:", res.error)
