#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./create_sdf_from_urdf_xacro.sh filename"
    exit 1
fi

FILENAME="$1"

# Extract the base filename and extensions
BASE="${FILENAME%%.*}"
NAME="${FILENAME%.*}"
EXT="${FILENAME##*.}"

#echo "Processing file: $FILENAME"
#echo "Base: $BASE"
#echo "Name: $NAME"
#echo "Extension: $EXT"

echo "Converting $BASE.urdf.xacro to $BASE.urdf..."
ros2 run xacro xacro "$BASE.urdf.xacro" > "$BASE.urdf"

echo "Converting $BASE.urdf to $BASE.sdf..."
gz sdf -p "$BASE.urdf" > "$BASE.sdf"

echo "Conversion complete!"