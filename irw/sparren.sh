#!/usr/bin/env bash

if [ "$#" -ne 2 ]; then
    printf "Usage: rosrun roboteam_utils sparren.sh ATTACKER_ROBOT_ID KEEPER_ROBOT_ID\n"
    exit 1
fi

rosrun roboteam_utils just_attacker.sh $1 &
rosrun roboteam_utils just_keeper.sh $2 &
wait

