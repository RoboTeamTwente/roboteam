#!/bin/bash


./bin/buildRoboTeam.sh

echo "Starting Game Controller"
gnome-terminal -e ./bin/RunGameController.sh

sleep 5

echo "Starting Auto Referee"
gnome-terminal -e ./bin/RunAutoReferee.sh

sleep 5

echo "Starting RoboTeam"
gnome-terminal -e ./bin/RunRoboTeam.sh

sleep 5

echo "Starting grSim"
gnome-terminal -e ./bin/RunGrSim.sh

echo "Opening Game Controller UI"
xdg-open http://localhost:8081/
