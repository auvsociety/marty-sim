#!/bin/bash

# Configuration
txt_file="/home/invictus/ros_ws/src/marty-sim/data/simu.txt"          # Path to your text file with OBJ filenames
source_dir="/home/invictus/ros_ws/src/marty-sim/data/meshes"    # Directory containing your 100 OBJ files
target_dir="/home/invictus/ros_ws/src/marty-sim/data/temp_meshes"     # Directory where selected files will be moved

# Create target directory if it doesn't exist
mkdir -p "$target_dir"

# Read the text file line by line and process each file
while IFS= read -r filename; do
    # Remove any leading/trailing whitespace (including CR/LF issues)
    filename_clean=$(echo "$filename" | xargs)
    
    # Check if the file exists in source directory
    if [ -f "$source_dir/$filename_clean" ]; then
        # To MOVE files:
        mv "$source_dir/$filename_clean" "$target_dir/"
        
        # To COPY files instead, comment the above line and use:
        # cp "$source_dir/$filename_clean" "$target_dir/"
        
        echo "Processed: $filename_clean"
    else
        echo "File not found: $filename_clean"
    fi
done < "$txt_file"

echo "Operation completed. Selected files are in: $target_dir"

