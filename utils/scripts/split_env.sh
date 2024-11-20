#!/bin/bash

# Check if there are any .env files
if [ -z "$(ls *.env 2>/dev/null)" ]; then
  echo "No .env files found in the current directory."
  exit 1
fi

# Loop over all files ending in .env
for file in *.env; do
  # Extract the base name of the file without the .env extension
  # This strips the last four characters, assuming they are '.env'
  base_name="${file%.*}"
  
  # Split the file into parts, each with 16 lines
  if ! split -l $1 -d --additional-suffix=".env" "$file" "${base_name}_"; then
    echo "Failed to split file $file"
    continue  # Skip to the next file on failure
  fi

  echo "Successfully split $file into $1-line parts."
done
