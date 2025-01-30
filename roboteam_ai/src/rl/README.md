# Nova AI 

RL implementation to swap out the play system. Current goal is to make the role division dynamic.

## Features

- Feature 1
- Feature 2
- Feature 3

## Usage


## Explanation .py scripts
- GetState.py gets a combined state and contains 2 functions, one to get the ball position and one to get robot position
- sentActionCommand sends a command using proto to the legacy AI system
- teleportBall.py to tp the ball to a location we can define in our environment


## High-level RL explanation

# Loop
Agent receives a state in the form of where all the robots are, if we are dribbling and where the ball is.


Every loop the game needs to be reset. This means resetting:
- Time
- Current stage
- 

Also when a ball placement happens, we need to extract the designated position and teleport the ball there for faster simulation.


