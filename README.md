# Active Grasping with Multisolution Bayesian Optimization

This is the repository of the master thesis ["Multisolution Bayesian optimization for robotic manipulation tasks"](https://deposita.unizar.es/record/72622) of the _Master in Robotics, Graphics and Computer vision_ at _Universidad de Zaragoza_.
This work focuses on Multisolution Bayesian optimization to obatain multiple efficient grasp configurations for unknown objects.

In this repository you can find the grasp evaluation environments and the synthetic functions used for the experiments. The implementation of the Multisolution Bayesian optimization method can be found in [BayesOpt-Multisolution](https://github.com/nachoh8/bayesopt_multisolution.git) repository.

This work is a continuation of our previous work [Optimización bayesiana multisolución para la exploración eficiente de agarres robóticos](http://hdl.handle.net/2183/31496) published at XLIII Jornadas Automaticas 2022.
The results and code can be found in the open-source repository
[Active Grasping](https://github.com/nachoh8/active-grasping).

## Requirements

* Python 3
* C++ 20
* [Simox](https://git.h2t.iar.kit.edu/sw/simox/simox)
* [BayesOpt - Multisolution](https://github.com/nachoh8/bayesopt_multisolution.git)
* [SigOpt - Python](https://sigopt.com/)
* SWIG (to create the grasping simulator python interface)

## Configuration

### BayesOpt
Set the environment variable BayesOpt_DIR to the path of BayesOpt.

### Grasping simulator
    >> mkdir build && cd build
    >> cmake ..
    >> make -j4
    >> sudo make install

### Python Experiment execution system
    >> python3 -m venv venv
    >> source venv/bin/activate
    >> pip install -r requirements.txt
    >> mkdir -p venv/lib/python3.8/site-packages/pygrasp
    >> cp build/lib/_pygrasp.so venv/lib/python3.8/site-packages/pygrasp/
    >> cp build/Grasp/python/pygrasp.py venv/lib/python3.8/site-packages/pygrasp/
    >> cp ${BayesOpt_DIR}/build/lib/bayesopt.so venv/lib/python3.8/site-packages/
    >> cp ${BayesOpt_DIR}/python/bayesoptmodule.py venv/lib/python3.8/site-packages/

### SigOpt setup

If you want to use active grasping with SigOpt platfform you have to add two environment variables:

* SIGOPT_DEV_TOKEN, your sigopt development token
* SIGOPT_PROD_TOKEN, your sigopt production token

You can choose Dev/Production token by setting the param "mode" to "dev"/"prod" in the sigopt params file.

## Tests
    >> cd test
    >> python3 test_grasp.py
    >> python3 test_bopt.py
    >> python3 test_sigopt.py

## Execution

### Active Grasping Optimization

    >> python3 main_active_grasping.py
                            -objf <objective_function> [<params_file>]
                            (-bopt <bayesopt_params> [<exp_params>] |
                            -sopt <sigopt_params> [<experiment_id>] |
                            -gpyopt <gpyopt_params>|
                            -robot <robot_params>)
                            [-flog <log_file>]
                            [-metric <metric>]
                            [-p] [-v]

* -objf: objective funcion to optimize
  * <objective_function>: objective function name
  * <params_file>: objective function parameters file, optional.
* -bopt: BayesOpt
  * <bayesopt_params>: bayesopt optimizer parameters file
  * <exp_params>: optional experiment file
* -sopt: SigOpt
  * <sigopt_params>: sigopt experiment params
  * <experiment_id>: optional, sigopt experiment id, allow continue running an experiment
* -gpyopt: GPyOpt
* -robot: ROBOT optimizer
* -flog: optional, file to save results
* -metric: metric to be used, by the default normal.
* -p: BBO in parallel
* -v: verbose

### Optimization evaluation

    >> python3 evaluation.py -flogs (<log_file> | <log_folder>)+
                            [-minimize] [-no-plot] [-metric <metric>]
                            [-best] [-cec] [-batch] [-sols]

* -best: compute best solution found metrics
* -cec: compute CEC 2013 metrics
* -batch: compute batch metrics
* -sols: compute multi solutions metrics

### Grasp visualization
    >> ./build/bin/grasp_visualization (-param <params_file> [eigen] | -log <log_file>)

 * -param: load the simulation environment fron a configuration file. If _-eigen_ then the eigengrasp environment is used, **NOT WORKS**.
 * -log: load the simulation environment fron a result file.

