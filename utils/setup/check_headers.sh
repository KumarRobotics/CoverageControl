#!/bin/bash

# Temporary file to hold the first file's header
first_file_header=$(mktemp)

# Flag to indicate if all files are the same
all_same=true

# Path to the directory containing the files
directory_path=$1

# Define the pattern of files you want to check, e.g., "*.c" for C source files
file_pattern="*"

# Extract the first 22 lines of a specific file as the reference
# Replace 'specific_file_to_compare' with a real file name you want to use as a reference
head -n 22 "${2}" > "$first_file_header"

# Use find to recursively get all files matching the pattern
find "$directory_path" -type f -name "$file_pattern" | while read file; do
    # Temporary file for the current file's header
    current_file_header=$(mktemp)
  
    # Extract the first 22 lines of the current file
    head -n 22 "$file" > "$current_file_header"
  
    # Compare the current file's header with the first file's header
    if ! diff "$first_file_header" "$current_file_header" > /dev/null; then
        echo "Difference found in: $file"
        all_same=false
    fi
  
    # Clean up the temporary file
    rm "$current_file_header"
done

# Clean up
rm "$first_file_header"
