#!/bin/bash

for i in {4..9}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/datasets/1254/data_params.yaml $i
done
for i in {4..9}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/datasets/1774/data_params.yaml $i
done
for i in {4..9}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/datasets/1620/data_params.yaml $i
done
for i in {4..9}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/datasets/1916/data_params.yaml $i
done
