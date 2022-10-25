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
print("----TEST GRAMACY----")
grmacy = TestGramacyExecutor()

# make query with np.array, [], vectord
q1 = np.array([0.28, 0.14])
q2 = [0.28, 0.14]
q3 = vectord()
q3.append(0.28)
q3.append(0.14)

for q in [q1, q2, q3]:
    res = grmacy.executeQueryGrasp(q)
    print(res.measure, res.volume, res.force_closure)

print()
print("----TEST GRASP PLANNER----")

params = GraspPlannerParams() # empty
is_valid = load_GraspPlannerParams_json("config/grasp/tests/grasp_params.json", params) # or from file
if not is_valid:
        print("Error: parsing grasp planner params")
        exit(1)

print("Robot file: " + params.robot_file)

# set obj pose
params.has_obj_pose = True
params.obj_position = np.ndarray((3,1), buffer=np.array([93, 34, 45]), dtype=np.float32)
params.obj_orientation = np.ndarray((3,1), buffer=np.array([1.4, 2.84, -3.1]), dtype=np.float32)

planner = GraspPlanner(params)

# WARNING
# !!!!!
# c++ could access elements outside the range of the query vector and does not throw any errors
# !!!!
q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q3 = vectord(6, 0.0)

for q in [q1, q2, q3]:
    res = planner.executeQueryGrasp(q)
    print(res.measure, res.volume, res.force_closure)

print("----TEST GRASP PLANNER IK----")

ik_params = GraspPlannerIKParams() # empty
is_valid = load_GraspPlannerIKParams("config/graspIK/tests/grasp_params.json", ik_params)
if not is_valid:
        print("Error: parsing grasp planner ik params")
        exit(1)


plannerIK = GraspPlannerIK(ik_params)

# WARNING
# !!!!!
# c++ could access elements outside the range of the query vector and does not throw any errors
# !!!!
query = np.array([-239.204, -32.972, 586.954, -1.57, 0.0,  3.14])

res = plannerIK.executeQueryGrasp(query)
print("Result:", res.measure, res.volume, res.force_closure)
