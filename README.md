# roboteam_suite
All relevant roboteamtwente repositories in one.


# See also

Running with camera's and real robots:
SSL-Vision (https://github.com/RoboCup-SSL/ssl-vision)

Working with a referee
SSL-Game-controller (https://github.com/RoboCup-SSL/ssl-game-controller)
SSL-vision-client (https://github.com/RoboCup-SSL/ssl-vision-client)

Working with the grSim simulator
grSim (https://github.com/RoboTeamTwente/grSim) 


# Usage
Easiest way to run things is to make some compound executables in CLion that boot everything you need. Make sure you have grsim or ssl-vision running and are listening to the right ports.


running one team:
```
./kalman_world
./roboteam_ai
./roboteam_robothub 
```
running two teams:
```
./kalman_world
./roboteam_ai
./roboteam_robothub 
./roboteam_ai 1
./roboteam_robothub 1
```


# Installation
## List of dependencies

- CMake 3.10
- Protobuf 3.9.1 (https://developers.google.com/protocol-buffers/)
- ZeroMQ 4.2.5 (https://zeromq.org/)
- zmqpp 4.1.2 (https://github.com/zeromq/zmqpp) 
- Armadillo 9.6 (http://arma.sourceforge.net/)


## Installation on Linux (tested on Ubuntu 18.04 Bionic Beaver)

### Install protobuf 3.9.1
```
$ sudo apt-get install autoconf automake libtool curl make g++ unzip

$ git clone https://github.com/protocolbuffers/protobuf.git
$ cd protobuf
$ git submodule update --init --recursive
$ ./autogen.sh

$ ./configure
$ make
$ make check
$ sudo make install
$ sudo ldconfig # refresh shared library cache.
```

### Install ZMQ
```
$ sudo apt-get install libzmq3-dev libzmqpp-dev
```

### install Armadillo
```
sudo apt-get install libarmadillo-dev
```

## installation on macOS (tested on macOS 10.14 Mojave)
Make sure you already have the following:
- CMake (3.10 or higher)
- HomeBrew

```
$ brew install zmq zmqpp armadillo protobuf
```
## Code Style Formatting
To be consistent in the code style, we use clang-format. You can use clang-format to automatically format your code during every commit and you can use clang-format in CLion to format all code files in a given folder.

### Commit Formatting
Run install.sh from the cloned 'formatting' folder in this repository. This will make sure that the code is formatted during every commit from then on. 

### CLion Formatting
1. Open CLion and go to File->Settings->Tools->External Tools
2. Click on Add and fill in without quotes: 
- Name: 'clang-format'
- Program: The location on your computer to the formatting/format-all.sh file of this repository.
- Working directory: '$FilePath$'
3. Click on OK, OK
4. If you now right click a folder and go to External Tools then you can click on clang-format which will format the entire folder (not this unfortunately does not yet work on single files).

