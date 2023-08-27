# ========================== #
#        INSTRUCTIONS        #
# ========================== #
#
#	==== QUICKLY BOOT
#     $ docker run --init --rm --net=host roboteamtwente/roboteam:latest ./roboteam_robothub
#	  $ docker run --init --rm --net=host roboteamtwente/roboteam:latest ./roboteam_observer
#     $ xhost local:
#	  $ docker run --init --rm --net=host -v /tmp/.X11-unix/ -e DISPLAY=$DISPLAY rtt-release /roboteam_ai
#
#
#
#   ==== DO IT YOURSELF
#   Step 1.1: Build
#     $ docker build . --target development -t rtt-development
#     $ docker build . --target release -t rtt-release
#
#   Step 2: Allow X11 access
#     $ xhost local:
#   This will give local VMs acces to the X11-server, which is needed to do GUI stuff
#   https://stackoverflow.com/questions/25281992/alternatives-to-ssh-x11-forwarding-for-docker-containers
#
#   Step 3.1: Create a development container, to which you can e.g. hook into with VSCode
#     $ docker run -it -v /tmp/.X11-unix/ -e DISPLAY=$DISPLAY --net=host --name rtt rtt-development
#	OR
#	Step 3.2: Start any executable 
#	  $ docker run --init --rm --net=host rtt-release /roboteam_observer
#	  $ docker run --init --rm --net=host -v /tmp/.X11-unix/ -e DISPLAY=$DISPLAY rtt-release /roboteam_ai
#	  $ docker run --init --rm --net=host rtt-release /roboteam_robothub
#
# ========================== #



# ========================== #
#   DEVELOPMENT CONTAINER    #
# ========================== #
FROM ubuntu:22.04 AS development
RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim git cmake pkg-config python3 python3-pip ccache

RUN git clone https://github.com/RoboTeamTwente/roboteam.git --recursive
WORKDIR /roboteam
RUN ./install_UBUNTU_22_04.sh
RUN mkdir build 
WORKDIR /roboteam/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) roboteam_observer roboteam_ai roboteam_robothub
WORKDIR /roboteam



# ========================== #
#     RELEASE CONTAINER      #
# ========================== #
FROM ubuntu:22.04 AS release
COPY --from=development /roboteam/build/release/bin/roboteam_observer         /roboteam_observer
COPY --from=development /roboteam/build/release/bin/roboteam_ai               /roboteam_ai
COPY --from=development /roboteam/build/release/bin/roboteam_robothub         /roboteam_robothub

COPY --from=development /usr/lib/x86_64-linux-gnu                             /usr/lib/x86_64-linux-gnu
COPY --from=development /usr/share/fonts                                      /usr/share/fonts
COPY --from=development /roboteam/build/release/lib/libzmqpp.so               /usr/lib/libzmqpp.so
COPY --from=development /roboteam/build/release/lib/libNFParam.so             /usr/lib/libNFParam.so
COPY --from=development /usr/local/lib/libprotobuf.so.30.0.6                  /usr/lib/libprotobuf.so.30

# The entire /usr/lib/x86_64-linux-gnu folder is cloned since it's too much of a hassle to figure out what is and isn't needed
# TODO figure out how to exlude large files such as /usr/lib/x86_64-linux-gnu/libLLVM* (200+MB)
# Can't use .dockerignore for this https://github.com/moby/moby/issues/33923
