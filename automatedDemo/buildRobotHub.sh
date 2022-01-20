#!/bin/sh

echo "Configuring Cmake"
cmake -B build -S ../

num_of_cores=3

echo "Making RobotHub"
make -C build/ roboteam_robothub -j$num_of_cores
