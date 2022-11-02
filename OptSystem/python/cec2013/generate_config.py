#!/usr/bin/env python
###############################################################################
# Version: 1.1
# Last modified on: 3 April, 2016
# Developers: Michael G. Epitropakis
#      email: m_(DOT)_epitropakis_(AT)_lancaster_(DOT)_ac_(DOT)_uk
###############################################################################
import numpy as np
import json

from cec2013.cec2013 import *

def main():
    config_folder = "/home/nacho/ActiveGrasping/multisolution-active-grasping/config/cec2013"
    print(config_folder)

    # Demonstration of using how_many_goptima function
    for i in range(1, 21):
        print(70 * "=")
        # Create function
        f = CEC2013(i)

        config_file = "F"+str(i) + "_params.json"
        print("F"+str(i) + " -> " + f.get_name())
        print(config_file)

        data = {
        "n_trials": 1,
        "default_query": {},
        "invert_metric": True
        }

        ndim = f.get_dimension()
        if ndim == 1:
            data["active_variables"] = ["x"]
        else:
            data["active_variables"] = ["x" + str(i) for i in range(1, ndim + 1)]

        # Get lower, upper bounds
        ub = np.zeros(ndim)
        lb = np.zeros(ndim)
        for k in range(ndim):
            ub[k] = f.get_ubound(k)
            lb[k] = f.get_lbound(k)
        data["lower_bound"] = lb.tolist()
        data["upper_bound"] = ub.tolist()
        print(data)

        json_str = json.dumps(data, indent=4)
        res_file = config_folder + "/" + config_file
        with open(res_file, 'w') as outfile:
            outfile.write(json_str)

    print(70 * "=")


if __name__ == "__main__":
    main()
