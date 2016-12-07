#!/usr/bin/env sh

rosrun roboteam_utils constants_generator header < String\ Constants > include/roboteam_utils/constants.h
rosrun roboteam_utils constants_generator < String\ Constants > src/constants.cpp
