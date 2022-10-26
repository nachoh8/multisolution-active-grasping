#!/bin/bash

### PARAMS

OBJFS=( "forrester" "gramacy1d" "gramacy2d" "branin" "goldstein" "rosenbrock" "eggholder" "mccormick" "sixhumpcamel" "beale" )

for var in "${OBJFS[@]}"
do
  python3 evaluation.py -minimize -no-plot -save "logs/$var" -flogs "logs/${var}/bo" "logs/${var}/bbo"
done
