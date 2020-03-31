#!/bin/bash

# Cmake build
cmake --build . --target proto_tests -- -j $(nproc)
# Run tests, xvfb-run simulates an xserver
cd roboteam_proto
xvfb-run ./proto_tests