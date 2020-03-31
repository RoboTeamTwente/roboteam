#!/bin/bash

# Cd into the suite
cd /opt/roboteam/roboteam_suite
# Make sure every branch is on master, except for roboteam proto
cd roboteam_ai
git checkout master

cd ../roboteam_world
git checkout master

cd ../roboteam_utils
git checkout master

cd ../roboteam_robothub
git checkout master

cd ..
# Pull each
git submodule foreach git pull
# Cd into correct repo
cd roboteam_proto
# Checkout branch name
git checkout "$1"
# Cd back and create + cd into build
cd ..
mkdir build
cd build
# Cmake generate build files
cmake -DCMAKE_BUILD_TYPE=Debug -G "CodeBlocks - Ninja" ..
# Cmake build
cmake --build . --target proto_tests -- -j $(nproc)
echo "Finished compiling"
# Run tests, xvfb-run simulates an xserver
cd roboteam_proto
xvfb-run ./proto_tests