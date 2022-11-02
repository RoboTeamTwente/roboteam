#!/bin/bash
cd ../../ssl-autorefs
./build_all.sh
./run_erforce.sh &
./run_tigers.sh -a
