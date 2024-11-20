data_prams=${CoverageControl_ws}/lpac/params/data_params.toml
num_subsets=10
num_subsets_str="["
for i in $(seq 0 $((num_subsets-1))); do
    num_subsets_str="${num_subsets_str} \"${i}\", "
done
num_subsets_str="${num_subsets_str%??}"
num_subsets_str="${num_subsets_str} ]"
python -c "import dataset_utils; dataset_utils.combine_dataset(\"${data_prams}\", ${num_subsets_str})"
python -c "import dataset_utils; dataset_utils.split_dataset(\"${data_prams}\")"
