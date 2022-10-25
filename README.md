# Active Grasping with Multisolution Bayesian Optimization

With the results obtained, the paper [Optimización bayesiana multisolución para la exploración eficiente de agarres robóticos](http://hdl.handle.net/2183/31496) was published at the XLIII Jornadas Automaticas 2022.

Cite: Herrera Seara, I., García-Lechuz Sierra, J., García Barcos, J., Martínez-Cantín, R. (2022) Optimización bayesiana multisolución para la exploración eficiente de agarres robóticos. XLIII Jornadas de Automática: libro de actas, pp. 714-720.

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

Test c++ lib:

    >> cd build && ./bin/test_libs

Test python lib:

    >> cd ..
    >> python3 test/test_grasp.py
    >> python3 test/test_bopt.py
    >> python3 test/test_sigopt.py

## Execution

### Active Grasping Optimization

    >> python3 main_active_grasping.py
                            -fgrasp <executor> <params_file>
                            (-fbopt <bayesopt_params> <exp_params> |
                            -fsopt <sigopt_params>)
                            [-flog <log_file>]

* <executor\>: {0: TestGramacyExecutor, 1: GraspPlanner, 2: GraspPlannerIK, 3: GraspPlannerS}
* -fbopt: BayesOpt
* -fsopt: SigOpt

### Optimization evaluation

    >> python3 evaluation.py -flogs (<log_file> | <log_folder>)+

### Grasp visualization

#### Grasp EEF

    >> ./build/bin/grasp_visualization <mode> <file>

* <mode\>: 0: from configuration; 1: from log
* <file\>: configuration or log file

#### Grasp Spherical

    >> ./build/bin/grasp_visualizationS <grasp_params> [<log_file>]

* <grasp_params\>: config file
* <log_file\>: log file

#### Grasp IK BiRRT

    >> ./grasp_ik_visualization <mode> <file>

* <mode\>: 0: from configuration; 1: from log
* <file\>: configuration or log file
