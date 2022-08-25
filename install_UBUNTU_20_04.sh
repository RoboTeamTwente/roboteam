#!/bin/bash

echo Updating repositories
apt-get update
apt-get upgrade

echo Installing package dependencies
apt-get install libzmq3-dev -y
apt-get install clang -y
apt-get install clang-format -y
apt-get install ninja-build -y
apt-get install autoconf -y
apt-get install automake -y
apt-get install libtool -y
apt-get install libtbb-dev -y
apt-get install apt-utils -y
apt-get install libboost-all-dev -y
apt-get install libeigen3-dev -y
apt-get install curl -y
apt-get install make -y
apt-get install g++ -y
apt-get install unzip -y
apt-get install cmake -y
apt-get install libqt5charts5-dev -y
apt-get install libsdl2-dev -y
apt-get install qtdeclarative5-dev -y

# The default protobuf package is too low of a version for us
echo Installing protobuf
git clone https://github.com/protocolbuffers/protobuf.git /opt/protobuf
cd /opt/protobuf
git submodule update --init --recursive
./autogen.sh
./configure
make -j $(nproc)
make install
ldconfig

echo Installing gtest
git clone https://github.com/google/googletest.git /opt/googletest
cd /opt/googletest
mkdir install
cd install
cmake ..
make -j $(nproc)
make install