#!/bin/bash

### CONSTANTS

OPTIMIZERS=( "bo" "bbo_lp_lcb" "bbo_lp_lcba" "gpyopt" "sigopt" )
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
GRASP_FUNCS=( "GP" )
GRASP_OBJECTS=( "bottle" "animal_statue" "trophy" )

### PARAMS

TYPE_FUNC=0 # 0: synthetic_functions, 1: grasp
NO_PLOT=0 # 0: show plots, 1: not show plots
SAVE_PLOTS=0

OBJFS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
CMP_OPTIMIZERS=( "bo" "bbo_lp_lcb" "bbo_lp_lcba" "bbo_lp_lcb_fod" "bbo_lp_lcba_fod" )
GRASP_EXPS=( "bottle" )

### CONFIG EXECUTION

EXEC_ARGS=""

NUM_OPTS=${#CMP_OPTIMIZERS[@]}

if [ $TYPE_FUNC -eq 0 ]; then
  TYPE_FUNC_NAME="synthetic_functions"
  EXEC_ARGS="-minimize -metric outcome"
  EXPS=( "" )
else
  TYPE_FUNC_NAME="grasp"
  EXEC_ARGS="-metric epsilon"
  EXPS=( ${GRASP_EXPS[@]} )
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
        FILES_CMP="${FILES_CMP} logs/${TYPE_FUNC_NAME}/${var}/${exp}${CMP_OPTIMIZERS[i]}_6"
        i=$(( $i + 1 ))
      done
    fi
    # python3 evaluation.py $EXEC_ARGS -flogs $FILES_CMP
    python3 evaluate_cec2013.py -flogs $FILES_CMP
  done
done
