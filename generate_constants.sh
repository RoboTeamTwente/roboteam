#!/usr/bin/env sh

# Move to the directory of the script
cd "$(dirname "$0")"

if [ $# -ne 2 ]
  then
    printf "Illegal number of parameters!"
fi

# First arg:
# src/constants.cpp
# Second arg:
# include/roboteam_utils/constants.h 

# Create directories if they don't exist
mkdir -p $(dirname $1)
mkdir -p $(dirname $2)

source /opt/ros/melodic/setup.bash

rosrun roboteam_utils constants_generator header < String\ Constants > $2
rosrun roboteam_utils constants_generator < String\ Constants > $1
