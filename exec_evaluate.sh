#!/bin/bash

### CONSTANTS

OPTIMIZERS=( "bo" "bbo_lp_lcb" "bbo_lp_lcba" "gpyopt_lp" "sigopt" )
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
GRASP_FUNCS=( "GP" )
GRASP_OBJECTS=( "bottle" "animal_statue" "trophy" )

### PARAMS

TYPE_FUNC=1 # 0: synthetic_functions, 1: grasp, 2: CEC2013
NO_PLOT=0 # 0: show plots, 1: not show plots
SAVE_PLOTS=0

OBJFS_SYNT=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
OBJFS_SYNT=( "F1" "F2" "F3" "F4" "F7" "F10" ) # "F5" "F6" "F7" "F8" "F9" "F10" )
OBJFS_GP=( "GP" )
ALL_CMP_OPTIMIZERS=( "bo" "bbo_lp_lcb" "bbo_lp_lcba" "bbo_lp_lcb_fod" "bbo_lp_lcba_fod" "bbo_mcmc_250" "bbo_mcmc_250_lcb" "bbo_mcmc_2500" "gpyopt_bo" "gpyopt_lp" )
CMP_OPTIMIZERS=( "bbo_mcmc_250_ei_lcb" "bbo_mcmc_250_ei_lcb_dist" "bbo_mcmc_250_ei_lcb_dist_2" "cluster_mcmc" "sigopt_ms" )
CMP_OPTIMIZERS=( "bbo_mcmc_250_ei_lcb" "cluster_mcmc" "sigopt_ms" "robot" )
# CMP_OPTIMIZERS=( "bbo_mcmc_250" "bbo_mcmc_2500" "bbo_mcmc_250_lcb" "gpyopt_lp" )
GRASP_EXPS=( "bottle/pos_ori" "trophy/pos_ori" )

OBJFS=$OBJFS_GP
### CONFIG EXECUTION

EXEC_ARGS=""

NUM_OPTS=${#CMP_OPTIMIZERS[@]}

if [ $TYPE_FUNC -eq 0 ]; then
  TYPE_FUNC_NAME="synthetic_functions"
  EXEC_ARGS="-minimize" # -metric outcome"
  EXPS=( "" )
elif [ $TYPE_FUNC -eq 1 ]; then
  TYPE_FUNC_NAME="grasp"
  EXEC_ARGS="-metric epsilon -best -sols 1 40"
  EXPS=( ${GRASP_EXPS[@]} )
elif [ $TYPE_FUNC -eq 2 ]; then
  TYPE_FUNC_NAME="cec2013"
  EXEC_ARGS="-acc 0.95 -cec -batch"
  EXPS=( "" )
fi

if [ $NO_PLOT -eq 1 ]; then
  EXEC_ARGS="${EXEC_ARGS} -no-plot"
fi

if [ $SAVE_PLOTS -eq 1 ]; then
  EXEC_ARGS="${EXEC_ARGS} -save " # TODO
fi

### EXECUTE

for var in "${OBJFS[@]}"
do
  for exp in "${EXPS[@]}"
  do
    if [ ! -z "$exp" ]
    then
      exp="${exp}/"
    fi

    FILES_CMP="logs/${TYPE_FUNC_NAME}/${var}/${exp}${CMP_OPTIMIZERS[0]}"
    if [ $NUM_OPTS -gt 1 ]; then
      for (( i=1; i<$NUM_OPTS; ))
      do
        FILES_CMP="${FILES_CMP} logs/${TYPE_FUNC_NAME}/${var}/${exp}${CMP_OPTIMIZERS[i]}"
        i=$(( $i + 1 ))
      done
    fi
    # python3 evaluation.py $EXEC_ARGS -flogs $FILES_CMP
    python3 evaluation.py $EXEC_ARGS -flogs $FILES_CMP
  done
done
