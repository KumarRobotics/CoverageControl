python -c 'import dataset_utils; dataset_utils.combine_dataset("${CoverageControl_ws}/lpac/params/data_params.toml", ["0", "1"])'
python -c 'import dataset_utils; dataset_utils.split_dataset("${CoverageControl_ws}/lpac/params/data_params.toml")'
