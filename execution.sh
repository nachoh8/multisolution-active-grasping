#!/bin/bash

### CONSTANTS

OPTIMIZERS=( "bo" "bbo_lp" "sigopt" )
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
GRASP_FUNCS=( "GramacyGP" "GP" )
GRASP_OBJECTS=( "bottle" "animal_statue" "trophy" )
GRASP_METRICS=( "epsilon" "epsilonfc" )
RES_LOG_PREFIX="res"

### PARAMS

START=1
NUM_RUNS=10

OPT_EXECUTOR=0 # 0: bayesopt, 1: sigopt
IDX_OPTIMIZER=1

TYPE_FUNC=1 # 0: synthetic_functions, 1: grasp
IDX_OBJ_FUNC=1
IDX_GRASP_OBJECT=0

IDX_GRASP_METRIC=0

SAVE_LOG=1 # 0: not save, 1: save to log file

TEST_FOLDER=0

### CONFIG EXPERIMENT

OPTIMIZER_NAME=${OPTIMIZERS[IDX_OPTIMIZER]}

if [ $TYPE_FUNC -eq 0 ]; then
    TYPE_FUNC_NAME="synthetic_functions"
    OBJ_FUNC=${SYNT_FUNCS[IDX_OBJ_FUNC]} # objective function name
    OBJ_FUNC_PARAMS=""

    METRIC="basic"

    FBOEXP="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/exp_params.json"
    FBOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}_params.json"

    RES_SUBFOLDER="${OPTIMIZER_NAME}"

elif [ $TYPE_FUNC -eq 1 ]; then
    TYPE_FUNC_NAME="grasp"
    OBJ_FUNC=${GRASP_FUNCS[IDX_OBJ_FUNC]} # objective function name
    if [ $IDX_OBJ_FUNC -eq 0 ]; then # gramacy grasp model test
        OBJ_FUNC_PARAMS=""
        OBJECT=""

        METRIC=${GRASP_METRICS[0]}

        FBOEXP="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/bopt/exp_params.json"
        
        RES_SUBFOLDER="${OPTIMIZER_NAME}"
    else
        OBJECT=${GRASP_OBJECTS[IDX_GRASP_OBJECT]} # object to grasp
        OBJ_FUNC_PARAMS="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/params/${OBJECT}_params.json"
        
        METRIC=${GRASP_METRICS[IDX_GRASP_METRIC]}

        FBOEXP="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/bopt/${OBJECT}/exp_params.json"
        
        RES_SUBFOLDER="${OBJECT}/${OPTIMIZER_NAME}"
    fi


    FBOPT="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/bopt/${OPTIMIZER_NAME}_params.json"

else
    echo "Error: objective function type must be -> 0: synthetic functions, 1: grasp"
    exit 1
fi

if [ $TEST_FOLDER -eq 0 ]; then
    RES_FOLDER="logs/${TYPE_FUNC_NAME}/${OBJ_FUNC}/${RES_SUBFOLDER}"
else
    RES_FOLDER="logs/tests/${TYPE_FUNC_NAME}/${OBJ_FUNC}/${RES_SUBFOLDER}"
fi

if [ $OPT_EXECUTOR -eq 0 ]; then
    echo "Using: Bayesopt"
    echo "Bopt Params: $FBOPT"
    echo "Experiment Params: $FBOEXP"

    OPT_ARGS="-bopt $FBOPT $FBOEXP"
elif [ $OPT_EXECUTOR -eq 1 ]; then
    echo "Using: SigOpt"
    echo "SigOpt Params: $FSOPT"

    OPT_ARGS="-sopt $FSOPT"
else
    echo "Error: optimizer must be -> 0: Bayesopt, 1: Sigopt"
    exit 1
fi

echo "Objective function: ${OBJ_FUNC}"
echo "Objective function params: ${OBJ_FUNC_PARAMS}"
echo "Metric: ${METRIC}"

### Execution

if [ $SAVE_LOG -eq 1 ]; then
    mkdir -p $RES_FOLDER
    FLOG="$RES_FOLDER/$RES_LOG_PREFIX"
    echo "Res folder: $RES_FOLDER"
fi

N_ERR=0
for (( i=$START; i<=$NUM_RUNS; ))
do
    echo "-------------------------------------"
    if [ $SAVE_LOG -eq 1 ]; then
        log="${FLOG}_$i.json"
    else
        log=""
    fi
    echo "Execution $i/$NUM_RUNS -> $log"

    python3 main_active_grasping.py $OPT_ARGS -objf $OBJ_FUNC "${OBJ_FUNC_PARAMS}" -flog "${log}" -metric $METRIC
    
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
