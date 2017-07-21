#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    printf "Usage: rosrun roboteam_utils square_demo.sh ROBOT_ID\n"
    exit 1
fi

rosrun roboteam_tactics TestX rtt_ewoud/DemoSquare int:ROBOT_ID=$1
