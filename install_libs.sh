#!/bin/bash

source venv/bin/activate

cp ${BayesOpt_DIR}/build/lib/bayesopt.so venv/lib/python3.8/site-packages/
cp ${BayesOpt_DIR}/python/bayesoptmodule.py venv/lib/python3.8/site-packages/

pip install -r requirements.txt

mkdir -p venv/lib/python3.8/site-packages/pygrasp
cp build/lib/_pygrasp.so venv/lib/python3.8/site-packages/pygrasp/
cp build/Grasp/python/pygrasp.py venv/lib/python3.8/site-packages/pygrasp/
