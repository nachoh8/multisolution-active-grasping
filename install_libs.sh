#!/bin/bash

python3 -m venv venv

source venv/bin/activate

pip install -r requirements.txt

mkdir -p venv/lib/python3.6/site-packages/pygrasp
cp build/lib/_pygrasp.so venv/lib/python3.6/site-packages/pygrasp/
cp build/Grasp/python/pygrasp.py venv/lib/python3.6/site-packages/pygrasp/

cp ${BayesOpt_DIR}/build/lib/bayesopt.so venv/lib/python3.6/site-packages/
cp ${BayesOpt_DIR}/python/bayesoptmodule.py venv/lib/python3.6/site-packages/
