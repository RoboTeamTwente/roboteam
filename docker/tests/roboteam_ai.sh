#!/bin/bash

# Cmake build
cmake --build . --target ai_tests -- -j $(nproc)
# Run tests, xvfb-run simulates an xserver
cd roboteam_ai
xvfb-run ./ai_tests