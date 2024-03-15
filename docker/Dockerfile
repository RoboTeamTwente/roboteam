# RTT Build Environment
# The purpose of this container is to have a stable, quick and reliable environment
# to build RTT software.
#
# =========================================================================== #
#
# Manually build the development container:
#   docker build -t roboteamtwente/roboteam:development --target development .
#
# Spin-up the development container:
#   docker run -itd --name rtt-build-env -h rtt-build-env -v <abs_path_to_the_repo_root>/:/home/roboteamtwente/roboteam roboteamtwente/roboteam:development
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
FROM alpine:3.18 AS development

ENV TZ="Europe/Rome"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USERNAME=roboteamtwente

RUN apk add --no-cache bash build-base cmake make musl-dev libtool \
    clang ninja-build autoconf automake pkgconfig \
    python3 py3-pip git ccache sudo \
    libzmq zeromq-dev libzmq-static eigen-dev gtest-dev \
    libtbb-dev boost-dev curl unzip wget zlib-dev zip nodejs npm \
    openssl-dev sdl2-dev libudev-zero libudev-zero-dev libusb libusb-dev \
    qt5-qtbase-dev qt5-qtcharts-dev qt5-qtdeclarative-dev qt5-qtsvg-dev
RUN npm install -g yarn

WORKDIR /root/protobuf
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v3.20.3/protobuf-cpp-3.20.3.zip -O protobuf.zip && unzip protobuf.zip && rm protobuf.zip
WORKDIR /root/protobuf/protobuf-3.20.3
RUN ./configure && make -j$(nproc) && make check -j$(nproc) && make install && ldconfig . && cd .. && rm -rf /root/protobuf/protobuf-3.20.3

ENV HOME=/home/$USERNAME
ENV USER=$USERNAME

RUN adduser -D $USER && \
    echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER

USER $USER
WORKDIR $HOME

# Release target
FROM alpine:3.18 AS release

ENV TZ="Europe/Rome"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USERNAME=roboteamtwente
ENV HOME=/home/$USERNAME
ENV USER=$USERNAME

RUN apk add --no-cache \
    libtool libzmq libzmq-static libudev-zero libusb \
    zeromq-dev eigen-dev gtest-dev \
    libtbb-dev boost-dev zlib-dev \
    openssl-dev sdl2-dev  \
    qt5-qtbase-dev qt5-qtcharts-dev qt5-qtdeclarative-dev qt5-qtsvg-dev \
    sudo nodejs npm
RUN npm install -g yarn
RUN npm install chart.js
RUN npm install vue-socket.io socket.io socket.io-client
RUN npm install axios

COPY --from=development --chown=root:root /usr/local/lib/libprotobuf.so.31 /usr/local/lib/libprotobuf.so.31
COPY --from=development --chown=root:root /usr/local/bin/protoc /usr/local/bin/protoc

RUN adduser -D $USER && \
    echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER

USER $USER
WORKDIR $HOME

# Add roboteam libraries to LD path
ENV LD_LIBRARY_PATH=$HOME/lib/

# Copy just build binaries (roboteam software) from volume to container home
# Note: cannot COPY from outside context, build from parent folder
COPY --chown=$USER:$USER ../build/release/ $HOME/
