#!/bin/bash

WS=""

if [[ $# -eq 0 ]] ; then
    WS="$(pwd)/../.."
else 
    WS=$1
fi

pushd $WS
catkin_make
source devel/setup.sh
popd

pushd $WS/src/NavSim
java -jar ./NavSim.jar &
popd

grsim & rqt -s roboteam_sim --force-discover & roslaunch $WS/src/roboteam_utils/all.launch && fg
