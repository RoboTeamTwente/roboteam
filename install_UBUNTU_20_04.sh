#!/bin/bash
install_directory=$(pwd)

echo Updating repositories
apt-get -y update
apt-get -y upgrade

echo Installing required packaged dependencies
apt-get install -y cmake
apt-get install -y g++
apt-get install -y qt5-default
apt-get install -y libqt5charts5-dev
apt-get install -y libsdl2-dev
apt-get install -y libzmq3-dev
apt-get install -y libeigen3-dev
apt-get install -y libgtest-dev

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

echo Installing roboteam
cd $install_directory
if [ -d "build/" ] 
then
echo build directory found
cd build
else
echo build directory not found: making directory
mkdir build && cd build
fi
cmake ..
make -j$(nproc)

echo Installing grSim
cd $install_directory/..
if [ -d "grSim/" ]
then
echo grSim already installed: updating
cd grSim/
git pull
else
echo grSim not installed: installing
git clone https://github.com/RoboCup-SSL/grSim.git
cd grSim/
fi
if [ -d "build/" ] 
then
echo build directory found
cd build
else
echo build directory not found: making directory
mkdir build && cd build
fi
cmake ..
make -j$(nproc)

echo Installing autorefs
cd $install_directory/..
if [ -d "ssl-autorefs/" ]
then
echo autorefs already installed: updating
cd ssl-autorefs/
./updateAll.sh
else
echo autorefs not installed: installing
git clone https://github.com/RoboCup-SSL/ssl-autorefs.git
cd ssl-autorefs
git clone https://github.com/TIGERs-Mannheim/AutoReferee.git tigers
git clone https://github.com/robotics-erlangen/autoref.git erforce
./installDeps.sh
fi
./buildAll.sh

