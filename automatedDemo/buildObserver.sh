#!/bin/sh

num_of_cores=7

echo "Making Observer with $num_of_cores threads"
make -C build/ roboteam_observer -j$num_of_cores

