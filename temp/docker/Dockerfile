FROM robocupssl/ubuntu-vnc:latest

LABEL Maintainer Roboteam <roboteamtwente@gmail.com>

# Some settings need root privileges
USER root

# Update lists
RUN /bin/bash -c "apt-get update -y; apt-get upgrade -y"

# Install dependencies
RUN /bin/bash -c "apt-get -y install \
    ccache \
    libzmq3-dev \
    clang \
    clang-format \
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
    git \
    nano"

# Install sublime text
RUN /bin/bash -c "wget -q 'https://download.sublimetext.com/sublime-text_build-3211_amd64.deb' -O sublime.deb"
RUN /bin/bash -c "apt-get install ./sublime.deb"

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

# Copy the clone script
COPY clone.sh /clone.sh

# Switch back to the default user
USER default