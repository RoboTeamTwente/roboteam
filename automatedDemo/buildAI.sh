#!/bin/sh

echo "Configuring Cmake"
cmake -B build -S ../

num_of_cores=3

echo "Making AI"
make -C build/ roboteam_ai -j$num_of_cores
