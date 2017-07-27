#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    printf "Usage: rosrun roboteam_utils just_keeper.sh KEEPER_ROBOT_ID\n"
    exit 1
fi

rosrun roboteam_tactics TestX rtt_bob/BasicKeeperTree int:ROBOT_ID=$1
