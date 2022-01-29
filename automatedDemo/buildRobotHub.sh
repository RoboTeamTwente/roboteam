#!/bin/sh

num_of_cores=7

echo "Making RobotHub with $num_of_cores threads"
make -C build/ roboteam_robothub -j$num_of_cores
