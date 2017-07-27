#!/usr/bin/env bash

if [ "$#" -ne 2 ]; then
    printf "Usage: rosrun roboteam_utils circle_demo.sh ROBOT_ID_1 ROBOT_ID_2\n"
    exit 1
fi

rosrun roboteam_tactics TestX rtt_ewoud/DemoCircleSmall ROBOT_ID=$1 &
rosrun roboteam_tactics TestX rtt_ewoud/DemoCircleBig ROBOT_ID=$2 &
wait
