#!/bin/bash

### PARAMS

OPT_EXECUTOR=1 # 0: bayesopt, 1: sigopt
GRASP_EXECUTOR=2 # 0: TestGramacyExecutor, 1: GraspPlanner, 2: GraspPlannerIK, 3: GraspPlannerS
METRIC=2 # 0: Gramacy, 1: ForceClosure_GraspPlanner, 2: ForceClosure_GraspPlannerIK, 3: 2: ForceClosure_TIME_GraspPlannerIK, 4: ForceClosure_GraspPlannerS
START=9
NUM_RUNS=10

RES_LOG_PREFIX="res_yzr"
OBJ="animal_statue"

if [ $GRASP_EXECUTOR -eq 1 ]; then
    FGRASP="config/grasp/params/grasp_params.json"
    
    FBOPT="config/bayesopt/bopt_default.json"
    FGOPT="config/grasp/gopt/${OBJ}/gopt_yzr.json"

    FSOPT_OPT="config/grasp/sigopt/${OBJ}/opt.json"
    FSOPT_MS="config/grasp/sigopt/${OBJ}/multisol.json"
    FSOPT_CNT="config/grasp/sigopt/${OBJ}/all_constraint.json"
    FSOPT=$FSOPT_CNT

    RES_FOLDER_BOPT="bayesopt/${OBJ}"
    RES_FOLDER_SOPT="sigopt/${OBJ}/opt"
    RES_FOLDER_SMS="sigopt/${OBJ}/ms"
    RES_FOLDER_SCNT="sigopt/${OBJ}/all_constraint_2"
    RES_FOLDER="logs/grasp/${RES_FOLDER_SCNT}"
elif [ $GRASP_EXECUTOR -eq 2 ]; then
    FGRASP="config/graspIK/params/grasp_params_preshape.json"

    FBOPT="config/bayesopt/bopt_default.json"
    FGOPT="config/graspIK/gopt/${OBJ}/gopt_xyz.json"

    FSOPT_OPT="config/graspIK/sigopt/${OBJ}/opt_ntrials.json"
    FSOPT_MS="config/graspIK/sigopt/${OBJ}/ms_ntrials.json"
    FSOPT_CNT="config/graspIK/sigopt/${OBJ}/cas50_ntrials.json"
    FSOPT=$FSOPT_CNT

    RES_FOLDER_BOPT="bayesopt/${OBJ}_ntrials"
    RES_FOLDER_SOPT="sigopt/${OBJ}/opt_ntrials"
    RES_FOLDER_SMS="sigopt/${OBJ}/ms_ntrials"
    RES_FOLDER_SCNT="sigopt/${OBJ}/cas50_ntrials"
    RES_FOLDER="logs/graspIK/ICRA/${RES_FOLDER_SCNT}"
else
    echo "Error: Grasp executor must be 1: GraspPlanner, 2: GraspPlannerIK"
    exit 1
fi

if [ $OPT_EXECUTOR -eq 0 ]; then
    echo "Using: Bayesopt"
    echo "Bopt Params: $FBOPT"
    echo "Experiment Params: $FGOPT"

    PARAMS="-fbopt $FBOPT $FGOPT"
elif [ $OPT_EXECUTOR -eq 1 ]; then
    echo "Using: SigOpt"
    echo "SigOpt Params: $FSOPT"

    PARAMS="-fsopt $FSOPT"
else
    echo "Error: mode must be 0: Bayesopt, 1: Sigopt"
    exit 1
fi

echo "Res folder: $RES_FOLDER"
echo "Grasp Executor: ${GRASP_EXECUTOR}"
echo "Grasp Executor Params: ${FGRASP}"

### Execution
mkdir -p $RES_FOLDER
FLOG="$RES_FOLDER/$RES_LOG_PREFIX"

N_ERR=0
for (( i=$START; i<=$NUM_RUNS; ))
do
    echo "-------------------------------------"
    log="${FLOG}_$i.json"
    echo "Execution $i/$NUM_RUNS -> $log"
    python3 main_active_grasping.py -fgrasp $GRASP_EXECUTOR $FGRASP $PARAMS -flog $log -metric $METRIC
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
