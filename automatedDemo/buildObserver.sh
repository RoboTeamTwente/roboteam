#!/bin/sh

echo "Configuring Cmake"
cmake -B build -S ../

num_of_cores=3

echo "Making Observer"
make -C build/ roboteam_observer -j$num_of_cores
