#!/bin/bash

### PARAMS

OBJFS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )

for var in "${OBJFS[@]}"
do
  python3 evaluation.py -minimize -flogs "logs/synthetic_functions/${var}/bo" "logs/synthetic_functions/${var}/bbo" # -save "logs/$var" -no-plot
done
