#!/bin/bash
num_of_cores=9

echo "starting Observer"
./cmake-build-debug/roboteam_world/roboteam_observer &

echo "starting Robothub"
./cmake-build-debug/roboteam_robothub/roboteam_robothub && fg &

echo "Starting primary AI"
./cmake-build-debug/roboteam_ai/roboteam_ai 0 &

echo "Starting secondary AI"
./cmake-build-debug/roboteam_ai/roboteam_ai 1
