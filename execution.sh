#!/bin/bash

### PARAMS

OPT_EXECUTOR=0 # 0: bayesopt, 1: sigopt

TYPE_FUNC="synthetic_functions" # grasp or synthetic_functions
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )

OBJ_FUNC=${SYNT_FUNCS[9]} # objective function name
OBJ_FUNC_PARAMS=""

METRIC="basic" # metric name

START=1
NUM_RUNS=10

FBO="config/${TYPE_FUNC}/bopt_params.json"
FBBO="config/${TYPE_FUNC}/bbo_lp_params.json"
FBOPT=$FBBO
FBOEXP="config/${TYPE_FUNC}/${OBJ_FUNC}/exp_params.json"

RES_LOG_PREFIX="res"
RES_FOLDER="logs/${TYPE_FUNC}/${OBJ_FUNC}/bbo"

if [ $OPT_EXECUTOR -eq 0 ]; then
    echo "Using: Bayesopt"
    echo "Bopt Params: $FBOPT"
    echo "Experiment Params: $FBOEXP"

    OPT_PARAMS="-bopt $FBOPT $FBOEXP"
elif [ $OPT_EXECUTOR -eq 1 ]; then
    echo "Using: SigOpt"
    echo "SigOpt Params: $FSOPT"

    OPT_PARAMS="-sopt $FSOPT"
else
    echo "Error: mode must be 0: Bayesopt, 1: Sigopt"
    exit 1
fi

echo "Res folder: $RES_FOLDER"
echo "Objective function: ${OBJ_FUNC}"
echo "Objective function params: ${OBJ_FUNC_PARAMS}"

### Execution
mkdir -p $RES_FOLDER
FLOG="$RES_FOLDER/$RES_LOG_PREFIX"

N_ERR=0
for (( i=$START; i<=$NUM_RUNS; ))
do
    echo "-------------------------------------"
    log="${FLOG}_$i.json"
    echo "Execution $i/$NUM_RUNS -> $log"
    python3 main_active_grasping.py $OPT_PARAMS -objf $OBJ_FUNC "$OBJ_FUNC_PARAMS" -flog $log -metric $METRIC
    if [ $? -eq 0 ]; then
        i=$(( $i + 1 ))
        N_ERR=0
    else
        echo "Error: error in execution, trying again"
        N_ERR=$(( $N_ERR + 1 ))
        if [ $N_ERR -eq 3 ]; then
            echo "END EEXECUTIONS: execution $i fails 3 times"
            exit 1
        fi
    fi
done
