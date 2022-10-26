#!/bin/bash

### PARAMS

OPT_EXECUTOR=0 # 0: bayesopt, 1: sigopt
OBJ_FUNC="forrester" # objective function name
METRIC="basic" # metric name

START=1
NUM_RUNS=10

FBO="config/synthetic_functions/bopt_params.json"
FBBO="config/synthetic_functions/bbo_lp_1d_params.json"
FBOPT=$FBBO
FBOEXP="config/synthetic_functions/${OBJ_FUNC}/exp_params.json"

RES_LOG_PREFIX="res"
RES_FOLDER="logs/${OBJ_FUNC}/bbo"

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

### Execution
mkdir -p $RES_FOLDER
FLOG="$RES_FOLDER/$RES_LOG_PREFIX"

N_ERR=0
for (( i=$START; i<=$NUM_RUNS; ))
do
    echo "-------------------------------------"
    log="${FLOG}_$i.json"
    echo "Execution $i/$NUM_RUNS -> $log"
    python3 main_active_grasping.py $OPT_PARAMS -objf $OBJ_FUNC "" -flog $log -metric $METRIC
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
