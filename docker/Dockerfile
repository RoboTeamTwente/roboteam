# RTT Build Environment
# The purpose of this container is to have a stable, quick and reliable environment
# to build RTT software.
#
# =========================================================================== #
#
# Manually build the development container:
#   docker build -t tollsimy/roboteam-rl:development --target development .
#
# Spin-up the development container:
#   docker run -itd --name rtt-build-env -h rtt-build-env -v <abs_path_to_the_repo_root>/:/home/tollsimy/roboteam-rl tollsimy/roboteam-rl:development
#
# Start/Stop container:
#   docker <start/stop> rtt-build-env
#
# Attach to the container: 
#   docker exec -it -w /home/roboteamtwente rtt-build-env /bin/bash
#
# =========================================================================== #
#

# Development target
FROM ubuntu:23.04 AS development

ENV TZ="Europe/Rome"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USERNAME=roboteamtwente

RUN apt update && apt install -y \
    apt-utils bash cmake make libtool \
    clang ninja-build autoconf automake build-essential \
    python3 python3-pip git ccache sudo \
    curl unzip wget zip nodejs npm libsdl2-dev libjemalloc-dev \
    openssl qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5svg5-dev \
    libqt5charts5-dev qtdeclarative5-dev libzmq5 libzmq5-dev libusb-1.0-0 libusb-1.0-0-dev \
    libeigen3-dev libgtest-dev libudev-dev libtbb-dev libboost-all-dev
RUN npm install -g yarn

WORKDIR /root/protobuf
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v3.20.3/protobuf-cpp-3.20.3.zip -O protobuf.zip && unzip protobuf.zip && rm protobuf.zip
WORKDIR /root/protobuf/protobuf-3.20.3
RUN ./configure && make -j$(nproc) && make check -j$(nproc) && make install -j$(nproc) && ldconfig $(pwd) && cd .. && rm -rf /root/protobuf/protobuf-3.20.3

WORKDIR /home
RUN wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-cxx11-abi-shared-with-deps-2.2.0.dev20231030%2Bcpu.zip -O libtorch.zip && unzip libtorch.zip && rm libtorch.zip
WORKDIR /home/libtorch/lib
RUN ldconfig $(pwd)

ENV HOME=/home/$USERNAME
ENV USER=$USERNAME

# Remove existing ubuntu user, otherwise new user
# will have uid and gid 1001:1001 instead of 1000:1000
# and permissions of files in mounted volumes won't match
RUN deluser --remove-home ubuntu

RUN useradd -m $USER && \
    usermod -aG sudo $USER && echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER

USER $USER
WORKDIR $HOME

# Release target
FROM ubuntu:23.04 AS release

ENV TZ="Europe/Rome"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USERNAME=roboteamtwente
ENV HOME=/home/$USERNAME
ENV USER=$USERNAME

RUN apt update && apt install -y \
    curl unzip wget zip sudo nodejs npm libsdl2-dev libjemalloc-dev \
    openssl qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5svg5-dev \
    libqt5charts5-dev qtdeclarative5-dev libzmq5 libzmq5-dev libusb-1.0-0 \
    libeigen3-dev libgtest-dev libudev-dev libtbb-dev apt-utils libboost-all-dev
RUN npm install -g yarn

COPY --from=development --chown=root:root /usr/local/lib/libprotobuf.so.31 /usr/local/lib/libprotobuf.so.31
COPY --from=development --chown=root:root /usr/local/bin/protoc /usr/local/bin/protoc
COPY --from=development --chown=root:root /home/libtorch/lib /home/libtorch/lib
WORKDIR /home/libtorch/lib
RUN ldconfig $(pwd)

# Remove existing ubuntu user, otherwise new user
# will have uid and gid 1001:1001 instead of 1000:1000
# and permissions of files in mounted volumes won't match
RUN deluser --remove-home ubuntu

RUN useradd -m $USER && \
    usermod -aG sudo $USER && echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER

USER $USER
WORKDIR $HOME

# Add roboteam libraries to LD path
ENV LD_LIBRARY_PATH=$HOME/lib/

# Copy just build binaries (roboteam software) from volume to container home
# Note: cannot COPY from outside context, build from parent folder
COPY --chown=$USER:$USER ../build/release/ $HOME/