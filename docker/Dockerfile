FROM ubuntu:latest

LABEL Maintainer Roboteam <roboteamtwente@gmail.com>

# Set environment variables / arguments
ARG DEBIAN_FRONTEND=noninteractive

# Update lists
RUN /bin/bash -c "apt-get update"

# Upgrade packages, -y marks everything "yes"
RUN /bin/bash -c "apt-get dist-upgrade -y" 

# Install dependencies
RUN /bin/bash -c "apt-get -y install \
    ccache \
    clang \
    xvfb \
    ninja-build \
    autoconf \
    automake \
    libtool \
    libtbb-dev \
    apt-utils \
    libboost-all-dev \
    libeigen3-dev \
    curl \
    make \
    g++ \
    unzip \
    cmake \
    libqt5charts5-dev \
    libsdl2-dev \
    qtdeclarative5-dev \
    git"

# Nlohmann-json
RUN /bin/bash -c "mkdir -p /usr/include/nlohmann && \
    curl https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp > /usr/include/nlohmann/json.hpp \
    "
 
# Setup protobuf
RUN /bin/bash -c "git clone https://github.com/protocolbuffers/protobuf.git /opt/protobuf &&\
    cd /opt/protobuf &&\
    git submodule update --init --recursive &&\
    ./autogen.sh &&\
    ./configure &&\
    make -j $(nproc)&&\
    make install &&\
    ldconfig"

# Setup gtest
RUN /bin/bash -c "git clone https://github.com/google/googletest.git /opt/googletest &&\
    cd /opt/googletest &&\
    mkdir install &&\
    cd install &&\
    cmake .. &&\
    make -j $(nproc)&&\
    make install"

# Clone the repos and 
RUN /bin/bash -c "git clone https://github.com/RoboTeamTwente/roboteam_suite /opt/roboteam/roboteam_suite --recurse-submodules"

# checkout development on everything
RUN "cd /opt/roboteam/roboteam_suite &&\
    python ./checkout_branch.py development\
    "

# Set ccache symbolic link
RUN mkdir -p /lib/ccache
# Set symbolic link
RUN ln -s /usr/lib/ccache /lib/ccache/bin
# Create ccache folder
RUN mkdir -p /opt/.ccache
# Add tests to the container.
RUN mkdir -p /opt/robotests
# Copy over tests folder
COPY tests /opt/robotests
# Copy over divider
COPY divider.sh /divider.sh
# Set the entrypoint for when your docker container runs
ENTRYPOINT [ "/divider.sh" ]