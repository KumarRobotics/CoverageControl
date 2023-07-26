#!/bin/bash

for i in {7..9}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/datasets/pc128/data_params.yaml $i
done
