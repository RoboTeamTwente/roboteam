#!/bin/bash

echo Updating repositories
apt-get -y update
apt-get -y upgrade

echo Installing required packaged dependencies
apt-get install -y cmake
apt-get install -y g++
apt-get install -y qt5-default
apt-get install -y libqt5charts5-dev
apt-get install -y libzmq3-dev
apt-get install -y libeigen3-dev
apt-get install -y libgtest-dev
apt-get install -y libudev-dev

echo Installing other useful packages
apt-get install -y clang
apt-get install -y clang-format
apt-get install -y ninja-build
apt-get install -y autoconf
apt-get install -y automake
apt-get install -y libtool
apt-get install -y libtbb-dev
apt-get install -y apt-utils
apt-get install -y libboost-all-dev
apt-get install -y curl
apt-get install -y make
apt-get install -y unzip
apt-get install -y qtdeclarative5-dev

# The default protobuf package is too low of a version for us
echo Installing protobuf
git clone https://github.com/protocolbuffers/protobuf.git /opt/protobuf
cd /opt/protobuf
git checkout 3.19.x
git submodule update --init --recursive
./autogen.sh
./configure
make -j $(nproc)
make install
ldconfig
