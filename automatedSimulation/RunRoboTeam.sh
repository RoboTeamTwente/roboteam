#!/bin/bash
num_of_cores=9

cd ..

echo "Building Observer with $num_of_cores threads"
/snap/clion/204/bin/cmake/linux/bin/cmake --build cmake-build-debug --target roboteam_observer -- -j $num_of_cores

echo "Building Robothub with $num_of_cores threads"
/snap/clion/204/bin/cmake/linux/bin/cmake --build cmake-build-debug --target roboteam_robothub -- -j $num_of_cores

echo "Building AI with $num_of_cores threads"
/snap/clion/204/bin/cmake/linux/bin/cmake --build cmake-build-debug --target roboteam_ai -- -j $num_of_cores

echo "starting Observer"
./cmake-build-debug/roboteam_world/roboteam_observer &

echo "starting Robothub"
./cmake-build-debug/roboteam_robothub/roboteam_robothub && fg &

echo "Starting primary AI"
./cmake-build-debug/roboteam_ai/roboteam_ai 0 &

echo "Starting secondary AI"
./cmake-build-debug/roboteam_ai/roboteam_ai 1
