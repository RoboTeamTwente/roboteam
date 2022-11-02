#!/bin/bash

./buildRoboTeam.sh

echo "Starting Game Controller"
gnome-terminal -e ./RunGameController.sh

sleep 5

echo "Starting Auto Referee"
gnome-terminal -e ./RunAutoReferee.sh

sleep 5

echo "Starting RoboTeam"
gnome-terminal -e ./RunRoboTeam.sh

sleep 5

echo "Starting grSim"
gnome-terminal -e ./RunGrSim.sh

echo "Opening Game Controller UI"
xdg-open http://localhost:8081/
