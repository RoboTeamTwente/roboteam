# Docker setup for RTT
## Why docker?
Docker can create a virtual environment with pre installed depencendies. This makes sure everyone with this virtual environment can build or run our code.

For a simulation tournament (2022/04), every team had to put their program in a docker image, so that their image could run on the server of the people that hosted the tournament (So less delay between our program and the simulator).

## Structure
The description of the environment is located in the Dockerfile inside the `docker` folder. The file is divided in two targets: "development" and "release".

Release target is only intended as a GitHub Action target, its purpose is to create the Docker image containing runnable binaries that can be pushed into a Registry. If you really want to build it locally you can do it by first building the development target, then compile sources with it and then build the release target. Note that release target contains only RoboTeamTwente binaries and not external ones like erforce simulator or erforce autoref.
Development target is instead intended as a normal usage target.

Inside `docker` folder there are three subdirectories: `builder` `game` and `simulator`.
Every folder contains a convenient docker compose file:
    - builder is used by lazy people like me to build sources without building and then directly executing command on a container
    - simulator is used to start every service needed to start a simulation (apart from some external software, see [Normal Usage](#normal-usage)
    - game is used to start roboteam software in real game mode

Docker composes use volumes because this way it's possible to compile sources and start up just built services without rebuilding any container.

## How to use?
First things first: for convenience create an ENV VAR containing the path to this repo (change rc file accordingly to your shell). Make sure to be in the repo root folder (should be one level up from the folder in which this README is) and then:
```
echo ROBOTEAM_REPO=$(pwd)/ >> ~/.bashrc
source ~/.bashrc
```

Init submodules:
```
cd $ROBOTEAM_REPO
git submodule update --init --recursive
```

Every subproject is meant to be executed as a service using its own container from a docker-compose.
Every RTT subproject (service) runs in its own 'rtt-build-env' container, this way that all dependencies and requirements are satisfied. 
Subproject containers start their own software from `build/release/bin` folder mounted as a volume.

### Normal usage
1) Make your edits on RTT sources
2) Spin-up "builder" compose
3) Spin-up "simulator"/"game" compose
4) (Game-only) Download, install and run ssl-vision
4) Download and run ssl-game-controller (https://github.com/RoboCup-SSL/ssl-game-controller/releases)

As simple as that.

1. Whenever you want to build all RTT services (normal operation) you simply need to run:
```
cd $ROBOTEAM_REPO/docker/builder
docker-compose up
```
2. Start services:
```
cd $ROBOTEAM_REPO/docker/<simulator / game>
docker-compose up <-d>
```
When you want to stop services:
```
docker compose down
```

Note: you may want to use `docker logs -t <container_name>` to see command output of a specific container (service).
Example: `docker logs -t RTT_roboteam_primary_ai`

#### TODO
https://lemariva.com/blog/2020/10/vscode-c-development-and-debugging-containers

### Docker geeks
#### Build environment
You can manually build the 'build environment' container doing:
```
docker build -t rtt-build-env .
```

Spin-up the just built container:
```
docker run -itd --name rtt-build-env -h rtt-build-env -v $ROBOTEAM_REPO:/home/roboteamtwente/roboteam rtt-build-env
```

Then, you can compile sources using `build.sh` script located in the repo root folder from inside the container.
First, attach to a contanier shell: 
```
docker exec -it -u roboteamtwente -w /home/roboteamtwente/roboteam rtt-build-env /bin/bash
```
then you can execute build script (or manually compiler whatever you want):
```
./build.sh
```

Remember that you can stop/start 'rtt-build-env' container simply using:
```
docker <start/stop> rtt-build-env
```

Note: avoid tagging the "run" stage container with the same name "rtt-build-env", otherwise when you will start the "build" compose you won't be able to compile sources (because the compose will be re-using the container with that name).
#### Manually starting a service
You can start a service by simply executing the binary from the rtt-build-env container.
Example:
Log into a container shell:
```
docker exec -it -u roboteamtwente -w /home/roboteamtwente/ -v <abs_path_to_build_folder>/:/home/roboteamtwente/ rtt-build-env /bin/bash
```
execute roboteam_ai:
```
./release/bin/roboteam_ai --primary_ai
```

