#!/bin/sh

num_of_cores=7

echo "Making AI with $num_of_cores threads"
make -C build/ roboteam_ai -j$num_of_cores
