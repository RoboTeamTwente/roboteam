#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    printf "Usage: rosrun roboteam_utils just_attacker.sh ATTACKER_ROBOT_ID\n"
    exit 1
fi

rosrun roboteam_tactics TestX rtt_bob/DemoAttacker int:ROBOT_ID=$1

