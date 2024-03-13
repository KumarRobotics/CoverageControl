#!/bin/bash

for i in {0..5}
do
	echo "$i"
	python simple_data_generation.py ~/CoverageControl_ws/src/CoverageControl/params/data_params.toml $i
done
