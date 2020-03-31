#!/bin/bash

# Cmake build
cmake --build . --target world_tests -- -j $(nproc)
# Run tests, xvfb-run simulates an xserver
cd roboteam_world
xvfb-run ./world_tests