#!/bin/bash

### CONSTANTS

# OPTIMIZERS=( "cluster_mebo_gp" "cluster_mebo" "e_mebo" "e_mebo_gp" "mebo_gp" "bbo_mcmc_250_ei_lcb" "bbo_mcmc_250_ei" "bbo_mcmc_250_ei2" "bbo_mcmc_2500_ei2" "bbo_mcmc_250_lcb" "bbo_mcmc_2500" "gpyopt_bo" "gpyopt_lp" "sigopt_ms" "robot" )
OPTIMIZERS=( "cluster_mebo_gp" )
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "rosenbrock" "goldstein" "eggholder" "mccormick" "sixhumpcamel" "beale" )
GRASP_FUNCS=( "GP" )
GRASP_OBJECTS=( "bottle" "trophy" "drill" ) #0: bottle, 1: trophy, 2: drill
GRASP_METRICS=( "epsilon" "epsilonfc" )
GRASP_OPT=( "pos" "pos_ori" )
RES_LOG_PREFIX="res"

### PARAMS

# for i in `seq 4 6`; do ./exec_opt.sh $i; done

START=$1
NUM_RUNS=$2

OPT_EXECUTOR=0 # 0: bayesopt, 1: gpyopt, 2: sigopt, 3: robot
IDX_OPTIMIZER=0

TYPE_FUNC=1 # 0: synthetic_functions, 1: grasp, 2: cec2013 benchmark
IDX_OBJ_FUNC=0
IDX_GRASP_OBJECT=1
IDX_GRASP_METRIC=0
IDX_GRASP_OPT=1

SAVE_LOG=1 # 0: not save, 1: save to log file

TEST_FOLDER=0

### CONFIG EXPERIMENT

OPTIMIZER_NAME=${OPTIMIZERS[IDX_OPTIMIZER]}

if [ $TYPE_FUNC -eq 0 ]; then
    TYPE_FUNC_NAME="synthetic_functions"
    OBJ_FUNC=${SYNT_FUNCS[IDX_OBJ_FUNC]} # objective function name
    OBJ_FUNC_PARAMS=""

    METRIC="basic"

    FBOEXP="" #"config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/exp_params.json"
    FBOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}_params.json"

    FGPYOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}_params.json"

    RES_SUBFOLDER="${OPTIMIZER_NAME}_lcb"

elif [ $TYPE_FUNC -eq 1 ]; then
    TYPE_FUNC_NAME="grasp"
    OBJ_FUNC=${GRASP_FUNCS[IDX_OBJ_FUNC]} # objective function name
    OBJECT=${GRASP_OBJECTS[IDX_GRASP_OBJECT]} # object to grasp
    OBJ_FUNC_PARAMS="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/params/${OBJECT}_params.json"
    
    METRIC=${GRASP_METRICS[IDX_GRASP_METRIC]}
    G_OPT=${GRASP_OPT[IDX_GRASP_OPT]}

    FBOEXP="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/bopt/${OBJECT}/${G_OPT}_params.json"
    FBOPT="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/bopt/${OPTIMIZER_NAME}_params.json"
    
    FSOPT="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/sigopt/${OBJECT}/${G_OPT}_params.json"

    FGPYOPT="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/gpyopt/${OPTIMIZER_NAME}_params.json"

    FROBOT="config/${TYPE_FUNC_NAME}/${OBJ_FUNC}/robot/${OBJECT}/${G_OPT}_params.json"

    RES_SUBFOLDER="${OBJECT}/${G_OPT}/${OPTIMIZER_NAME}"

elif [ $TYPE_FUNC -eq 2 ]; then
    TYPE_FUNC_NAME="cec2013"
    OBJ_FUNC="F${IDX_OBJ_FUNC}" # objective function name
    OBJ_FUNC_PARAMS=""

    METRIC="basic"

    FBOEXP="" # "config/${TYPE_FUNC_NAME}/${OBJ_FUNC}_params.json"
    FBOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}/${OBJ_FUNC}_params.json"
    FSOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}/${OBJ_FUNC}_params.json"
    FGPYOPT="config/${TYPE_FUNC_NAME}/${OPTIMIZER_NAME}_params.json"
    FROBOT="config/${TYPE_FUNC_NAME}/robot/${OBJ_FUNC}_params.json"

    RES_SUBFOLDER="${OPTIMIZER_NAME}"

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
    if [ ! -z "$FBOEXP" ]; then
        echo "Experiment Params: $FBOEXP"
        OPT_ARGS="-bopt $FBOPT $FBOEXP"
    else
        OPT_ARGS="-bopt $FBOPT"
    fi
elif [ $OPT_EXECUTOR -eq 1 ]; then
    echo "Using: GPyOpt"
    echo "GPyOpt Params: $FGPYOPT"

    OPT_ARGS="-gpyopt ${FGPYOPT}"
elif [ $OPT_EXECUTOR -eq 2 ]; then
    echo "Using: SigOpt"
    echo "SigOpt Params: $FSOPT"

    OPT_ARGS="-sopt $FSOPT"
elif [ $OPT_EXECUTOR -eq 3 ]; then
    echo "Using: ROBOT"
    echo "ROBOT Params: $FROBOT"

    OPT_ARGS="-robot $FROBOT"
else
    echo "Error: optimizer must be -> 0: Bayesopt, 1: GPyOpt, 2: Sigopt, 3: ROBOT"
    exit 1
fi

echo "Objective function: ${OBJ_FUNC}"
if [ ! -z "$OBJ_FUNC_PARAMS" ]; then
    echo "Objective function params: ${OBJ_FUNC_PARAMS}"
    OBJ_ARGS="-objf ${OBJ_FUNC} ${OBJ_FUNC_PARAMS}"
else
    OBJ_ARGS="-objf ${OBJ_FUNC}"
fi

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

    python3 main_active_grasping.py ${OPT_ARGS} ${OBJ_ARGS} -flog "${log}" -metric $METRIC
    
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
