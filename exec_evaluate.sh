#!/bin/bash

### CONSTANTS

OPTIMIZERS=( "bo" "bbo_lp" "sigopt" )
SYNT_FUNCS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )
GRASP_FUNCS=( "GramacyGP" "GP" )
GRASP_OBJECTS=( "bottle" "animal_statue" "trophy" )

### PARAMS

TYPE_FUNC=1 # 0: synthetic_functions, 1: grasp
NO_PLOT=0 # 0: show plots, 1: not show plots
SAVE_PLOTS=0

OBJFS=( "GP" )
CMP_OPTIMIZERS=( "bo" "bbo_lp" )

IDX_OBJECT=0

### CONFIG EXECUTION

EXEC_ARGS=""

NUM_OPTS=${#CMP_OPTIMIZERS[@]}

if [ $TYPE_FUNC -eq 0 ]; then
  TYPE_FUNC_NAME="synthetic_functions"
  EXEC_ARGS="-minimize -metric outcome"
  SUB_FOLDER=""
else
  TYPE_FUNC_NAME="grasp"
  EXEC_ARGS="-metric epsilon"
  SUB_FOLDER="${GRASP_OBJECTS[IDX_OBJECT]}/"
fi

if [ $NO_PLOT -eq 1 ]; then
  EXEC_ARGS="${EXEC_ARGS} -no-plot"
fi

if [ $SAVE_PLOTS -eq 1 ]; then
  EXEC_ARGS="${EXEC_ARGS }-save " # TODO
fi

### EXECUTE

for var in "${OBJFS[@]}"
do
  FILES_CMP="logs/${TYPE_FUNC_NAME}/${var}/${SUB_FOLDER}${CMP_OPTIMIZERS[0]}"
  if [ $NUM_OPTS -gt 1 ]; then
    for (( i=1; i<$NUM_OPTS; ))
    do
      FILES_CMP="${FILES_CMP} logs/${TYPE_FUNC_NAME}/${var}/${SUB_FOLDER}${CMP_OPTIMIZERS[i]}"
      i=$(( $i + 1 ))
    done
  fi

  python3 evaluation.py $EXEC_ARGS -flogs $FILES_CMP
done
