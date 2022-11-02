# Active Grasping with Multisolution Bayesian Optimization

This is a modified version of the previous work [Active Grasping](https://github.com/nachoh8/active-grasping). With the results obtained in that work, the paper [Optimización bayesiana multisolución para la exploración eficiente de agarres robóticos](http://hdl.handle.net/2183/31496) was published at XLIII Jornadas Automaticas 2022.

Now, in this work we focus on Multisolution Bayesian optimization to obatain multiple efficient grasp configurations for unknown objects.
This work is for master's thesis "Multisolution Bayesian optimization for robotic manipulation tasks" of the _Master in Robotics, Graphics and Computer vision_ at _Universidad de Zaragoza_.

In this repository you can find the grasp evaluation environments and the synthetic functions used for the experiments. The implementation of the Multisolution BO method can be found in [BayesOpt](https://github.com/rmcantin/bayesoptpro) repository.

## Requirements

* Python 3
* C++ 20
* [Simox](https://gitlab.com/Simox/simox)
* [BayesOpt V2](https://github.com/rmcantin/bayesoptpro)
* [SigOpt - Python](https://sigopt.com/)
* SWIG (to create Grasp lib python interface)

## Compile

    >> mkdir build && cd build
    >> cmake ..
    >> make -j4
    >> sudo make install

## SigOpt setup

If you want to use active grasping with SigOpt platfform you have to add two environment variables:

* SIGOPT_DEV_TOKEN, your sigopt development token
* SIGOPT_PROD_TOKEN, your sigopt production token

You can choose Dev/Production token by setting the param "mode" to "dev"/"prod" in the sigopt params file.

## Tests

Tests to check that everything is working

Test C++ lib:

    >> cd build && ./bin/test_libs

Test python lib:

    >> cd ..
    >> python3 test/test_grasp.py
    >> python3 test/test_bopt.py
    >> python3 test/test_sigopt.py

## Execution

### Active Grasping Optimization

    >> python3 main_active_grasping.py
                            -objf <objective_function> <params_file>
                            (-bopt <bayesopt_params> <exp_params> | -sopt <sigopt_params>)
                            [-flog <log_file>]
                            [-metric <metric>]
                            [-p]

* -objf: objective funcion to optimize
  * <objective_function>: objective function name
  * <params_file>: objective function parameters file, if not params set it to ""
* -bopt: BayesOpt
  * <bayesopt_params>: bayesopt optimizer parameters file
  * <exp_params>: experiment file
* -sopt: SigOpt
* -flog: log file
* -metric: metric to be used
* -p: BBO in parallel

### Optimization evaluation

    >> python3 evaluation.py -flogs (<log_file> | <log_folder>)+
                            [-minimize] [-pbest] [-no-plot]
                            [-save <img_prefix>] [-metric <metric>]

### Grasp visualization

#### Grasp Planner

    >> ./build/bin/grasp_visualization (-param <params_file> | -log <log_file>)
