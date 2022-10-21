#!/bin/bash

num_of_cores=$(nproc)
num_of_cores=$(expr $num_of_cores / 2)

echo "Building AI with $num_of_cores cores"

cmake --build cmake-build-debug --target roboteam_observer -- -j $num_of_cores

cmake --build cmake-build-debug --target roboteam_ai -- -j $num_of_cores

cmake --build cmake-build-debug --target roboteam_robothub -- -j $num_of_cores
