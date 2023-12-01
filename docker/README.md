# Docker setup for RTT
## Why docker?
Docker can create a virtual environment with pre installed depencendies. This makes sure everyone with this virtual environment can build or run our code.

For a simulation tournament (2022/04), every team had to put their program in a docker image, so that their image could run on the server of the people that hosted the tournament (So less delay between our program and the simulator).

## Structure
The description of the environment is located in the Dockerfile inside the `docker` folder. The file is divided in two targets: "development" and "release".

**Release target is only intended as a GitHub Action target**, its purpose is to create the Docker image containing runnable binaries that can be pushed into a Registry. If you really want to build it locally you can do it by first building the development target, then compile sources with it and then build the release target. Note that release target contains only RoboTeamTwente binaries and not external ones like erforce simulator or erforce autoref, it does not contain RTT autoref either.
Development target is instead intended as a normal usage target.

Inside `docker` folder there are three subdirectories: `builder` and `runner`.
Every folder contains a convenient docker compose file:
- builder is used by lazy people like me to build sources without building and then directly executing command on a container.
Builder contains two (default and `diff`) profiles: one is used to simply build current source while the second is used to build both current sources and and an old commit of the current repo, this is used to start `diff` compose to compare current ai with an old version to evaluate performances after changes.
- runner is used to start every service needed to start a simulation (with current ai vs current ai or current ai vs and old version) or a game (game needs also some external software, see [Normal Usage](#normal-usage)).
This is again done through docker compose profiles specifying `simulation`, `diff` or `game` profile.

NOTE: Docker composes use volumes because this way it's possible to compile sources and start up just built services without rebuilding any container.

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

Then, install prerequisite: install Docker https://docs.docker.com/engine/install/, if you want you can add yourself to docker group in order to run Docker command without invoking root user https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user and finally install docker compose.

Every subproject is meant to be executed as a service using its own container from a docker compose.
Every RTT subproject (service) runs in its own container (based on the same roboteamtwente image), this way that all dependencies and requirements are satisfied. 
Subproject containers start their own software from `build/release/bin` folder mounted as a volume.

### Basestation
In order to access Basestation tty from inside Docker container we first need to set a persistent name to the tty device:
```
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", ATTRS{serial}==\"*\", SYMLINK+=\"rtt-basestation\"" | sudo tee -a /etc/udev/rules.d/99-rtt-basestation.rules
```

Add yourself to the dialout group:
```
sudo usermod -a -G dialout $USER
```
Finally, reboot your pc to apply.

### Normal/development usage
1) Make your edits on RTT sources
2) Spin-up "builder" compose
3) Spin-up "runner" compose
4) (Game-only) Download, install and run ssl-vision
5) (Game-only) Configure udev rule for Basestation, see [Basestation](#basestation)
5) You can now reach game controller UI on localhost:8081 and roboteam interface on localhost:8080, connect to primary_ai (port 12676) or secondary_ai (port 12677).
6) If you started "game" open interface and select "basestation"

As simple as that.

1. Whenever you want to build all RTT services (normal operation) you simply need to run:
```
cd $ROBOTEAM_REPO/docker/builder
docker compose up
```
or, if you want to build diff (compare current ai with a previous version):
```
COMMIT=<commit> docker compose --profile diff up
```
NOTE: if you dont't specify COMMIT=<commit>, the default will be origin/main~1

2. Start services:
```
cd $ROBOTEAM_REPO/docker/runner
docker compose --profile <simulator / diff / game> up -d
```
When you want to stop services:
```
docker compose --profile <simulator / diff /game> down
```

Note: you may want to use `docker logs -t <container_name>` to see output of a container (service).
Example: `docker logs -t RTT_roboteam_primary_ai`

NOTE: If you notice USB errors when running "game" compose, do a `docker compose --profile game down`, detach and re-attach basestation and the start game again.

### Release usage
Pull release image:
```
docker pull tollsimy/roboteam-rl:latest
```
Run the image:
```
docker run -itd --name rtt-release-env -h rtt-release-env tollsimy/roboteam-rl:latest
```

#### TODO
https://lemariva.com/blog/2020/10/vscode-c-development-and-debugging-containers

### Docker geeks
We highly discourage executing command under this section unless you know what you are doing and for some reason you need to edit dockerfiles or composes structure.

Note: remember that our "composes" download image from Dockerhub if not available locally, thus, if you want to test with a new image you need to build the new image and tag it with the same name (`tollsimy/roboteam-rl:development` or `tollsimy/roboteam-rl:latest`).
#### Development environment
You can manually build the 'development environment' container executing:
```
docker build -t tollsimy/roboteam-rl:development --target development .
```

Spin-up the just built container:
```
docker run -itd --name rtt-build-env -h rtt-build-env -v $ROBOTEAM_REPO:/home/tollsimy/roboteam-rl tollsimy/roboteam-rl:development
```

Then, you can compile sources using `build.sh` script located in the repo root folder from inside the container.
First, attach to a contanier shell: 
```
docker exec -it -u roboteamtwente -w /home/tollsimy/roboteam-rl rtt-build-env /bin/bash
```
then you can execute build script (or manually compiler whatever you want):
```
./build.sh
```

Remember that you can stop/start 'rtt-build-env' container simply using:
```
docker <start/stop> rtt-build-env
```

#### Release environment
Note: **if you really want to build the release container** (highly discouraged) you must first build the development container as explained in the previous section, then you need to compile sources using `build.sh` script and lastly you can build the container.

Note that the release container can be downloaded from Dockerhub (highly recommended) by a simple:
`docker pull tollsimy/roboteam-rl:latest`.

In order to build the release container, you must call the `docker build` command from parent folder, otherwise docker is not able to copy files from outside context:
```
cd ../
docker build -t tollsimy/roboteam-rl:latest --target release -f ./docker/Dockerfile .
```

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

