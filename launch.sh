#!/bin/bash

WS=""

if [[ $# -eq 0 ]] ; then
    WS="$(pwd)/../.."
else 
    WS=$1
fi

pushd $WS
catkin_make >/dev/null 2>/dev/null
source devel/setup.sh
popd

shutdown() {
    PGID=$(ps -o pgid= $$ | grep -o [0-9]*)
    setsid kill -- -$PGID
    exit 0
}

trap "shutdown" SIGINT SIGTERM

(cd $WS/src/NavSim; java -jar ./NavSim.jar) & grsim & rqt -s roboteam_sim --force-discover & roslaunch $WS/src/roboteam_utils/all.launch &
wait
