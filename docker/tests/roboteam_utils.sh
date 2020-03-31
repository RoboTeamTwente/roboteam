#!/bin/bash

# Cmake build
cmake --build . --target utils_tests -- -j $(nproc)
# Run tests, xvfb-run simulates an xserver
cd roboteam_utils
xvfb-run ./utils_tests