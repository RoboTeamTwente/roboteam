#!/usr/bin/env bash

if [ "$#" -ne 3 ]; then
    printf "Usage: rosrun roboteam_utils drive_to_point.sh ROBOT_ID X Y\n"
    exit 1
fi

rosrun roboteam_tactics TestX GoToPos bool:avoidRobots=true int:ROBOT_ID=$1 double:xGoal=$2 double:yGoal=$3
