FROM ubuntu:22.04
RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim git cmake pkg-config python3 python3-pip ccache

RUN git clone https://github.com/RoboTeamTwente/roboteam.git --recursive
WORKDIR /roboteam
RUN ./install_UBUNTU_20_04.sh
RUN mkdir build 
WORKDIR /roboteam/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Debug && make -j$(nproc) roboteam_observer roboteam_ai roboteam_world


# Step 1: Build
# $ docker build . -t rtt
#
### https://stackoverflow.com/questions/25281992/alternatives-to-ssh-x11-forwarding-for-docker-containers
# Step 2: Allow X11 access
# $ xhost local:
# This will give local VMs acces to the X11-server, which is needed to do GUI stuff
#
# Step 3: create the container
# $ docker run -it -v /tmp/.X11-unix/ -e DISPLAY=$DISPLAY --net=host --name rtt rtt
