# roboteam_suite

This repo contains:

- Roboteam AI, which makes decisions on what the robots should do based on the vision data, and feedback data (received from robots) which it receives from Roboteam World and Roboteam Robothub respectively.
- Roboteam World, which interprets vision data (received from GRSim, or from a physical camera)
- Roboteam Robothub, used to send commands to the robots (either through GRSim or the base station)
- Roboteam Utils, which contains a lot of helper functions, primarily geometrical in nature, such as Lines, Triangles, etc. These are used in other repo's, such as Roboteam AI and Roboteam World
- Roboteam Proto, which contains the protocol used to send and receive messages over TCP/UDP

In order to test this code, we use the GRSim simulator developed by the SSL Team Parsian. However, due to differences in the control mechanism of the robot (specifically, theta vs omega control), this software requires the [RoboTeam fork of GRSim](https://github.com/RoboTeamTwente/grSim). Installation instructions can be found in the readme of that repo.

Please note:
This software is tested on Ubuntu 20.04 LTS, and the installation instructions are written with that distro/OS in mind. However, other distro's of Linux are also likely to be able to run this software, albeit with some modifications.

## Usage

First, clone this repository. We recommend using an SSH key if you are able to do so, as this immediately authorizes your future pushes to the repositories.

```bash
$ mkdir -p ~/roboteam
$ git clone --recursive https://github.com/RoboTeamTwente/roboteam_suite.git ~/roboteam/roboteam_suite
# This might actually create the directory, but might aswel create it manually
```

Now, you can open the roboteam_suite project with an IDE of your choice. We recommed CLion, as it has a lot of very helpful features and you can get a student license for it.

Make sure you have CMake installed. Go to the CMakeLists.txt and click on it. CLion will generate a pop-up saying "project files not loaded" in blue in the top right corner. Clicking it will have CMake load the project. This means it generates instructions on how the files should be compiled in order to produce the executables specified in the CMakeLists.txt file. You can now compile the targets!

The easiest way to run the application is to make some [compound executables](https://www.jetbrains.com/help/clion/run-debug-configuration.html#config-folders) in CLion that boot everything you need. To run 1 team in GRSim, the simulator, you will need to add roboteam_ai, roboteam_world, and roboteam_robothub.
Make sure you have GRSim or ssl-vision running and are listening to the right ports. In general, the GRSim vision multicast port should be set to 10006.

### Auxiliary Software

It is recommended to use the game controller to control the gamestates. You can do so by downloading the latest binary release of the [SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller) repo, and executing it. For example, when downloaded to the Downloads folder:

```bash
~/Downloads/ssl-game-controller_v1.2.0_linux_amd64
```

You might need to [chmod](https://www.howtoforge.com/tutorial/linux-chmod-command/) the file to make it executable. The AI client should now listen to the commands sent by the GameController.

Also, make sure you have installed GRSim using the instructions found on the [RoboTeam fork of GRSim](https://github.com/RoboTeamTwente/grSim). Now execute the grsim binary from the command line. This usually looks like this:

```bash
/directory_you_cloned_grsim/grSim/bin/grsim
```

### Making the executables

Make sure the vision multicast port is set to 10006

#### One Team

Make the compound executable shown below:
![run_one_team](https://github.com/RoboTeamTwente/roboteam_suite/blob/RobotJesse-patch-1/readme_images/run_two_teams.png?raw=true)
Then run this compound executable and run GRSim.

#### Two Teams

Follow the steps used for the other compound executable, only now add 2 extra targets: roboteam_ai_1 and roboteam_robothub_1, as seen in the screenshot, to the compound. While creating the new CMake Targets, be sure to include a "1" in the program arguments, as seen in the image.
![run_two_teams](https://github.com/RoboTeamTwente/roboteam_suite/blob/RobotJesse-patch-1/readme_images/run_two_teams.png?raw=true)

## Installation

### List of dependencies

- [CMake 3.16](https://cmake.org/)
- [Protobuf 3.9.1](https://developers.google.com/protocol-buffers/)
- [ZeroMQ 4.2.5](https://zeromq.org/)
- [zmqpp 4.1.2](https://github.com/zeromq/zmqpp)
- [Eigen3 3.3.7-2](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [QT5](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)
- [QT5Charts](https://doc.qt.io/qt-5/qtcharts-index.html)
- [Ninja](http://ninja-build.org)
- [CCache](https://ccache.dev)
- [Google Test and Google Mock](https://github.com/google/googletest)
- [nlohmann-json](https://github.com/nlohmann/json)

### Installation of Dependencies (tested on Ubuntu 20.04)

Any command preceded by `#` indicates that it has to be ran using `sudo`.
Any command peceded by `$` can be ran as your user.

#### Dependencies in aptitude

Install these by running the following:

```bash
# apt -y update
# apt -y dist-upgrade
# apt -y install \
    ccache \
    clang \
    clang-format \
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
    git
```

#### Install nlohmann-json

```bash
$ sudo -s /bin/bash -c "mkdir -p /usr/include/nlohmann && \
    curl https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp > /usr/include/nlohmann/json.hpp"
```

#### Install protobuf 3.9.1

```bash
$ sudo /bin/bash -c "git clone https://github.com/protocolbuffers/protobuf.git /opt/protobuf &&\
    cd /opt/protobuf &&\
    git submodule update --init --recursive &&\
    ./autogen.sh &&\
    ./configure &&\
    make -j $(nproc)&&\
    make install &&\
    ldconfig"
```

### Install Google Test

```bash
$ sudo -s /bin/bash -c "git clone https://github.com/google/googletest.git /opt/googletest &&\
    cd /opt/googletest &&\
    mkdir install &&\
    cd install &&\
    cmake .. &&\
    make -j $(nproc)&&\
    make install"
```

### Code Style Formatting
To be consistent in the code style, we use clang-format. You can use clang-format to automatically format your code during every commit and you can use clang-format in CLion to format all code files in a given folder.

You can install the pre-commit hooks by doing the following

```bash
$ cd ~/roboteam/roboteam_suite
$ ./formatting/install.sh
# Should give output:
clang-format has been properly set up
```

#### CLion Formatting

- Open CLion and go to File->Settings->Tools->External Tools
- Click on Add and fill in without quotes: 
  - Name: 'clang-format'
  - Program: The location on your computer to the formatting/format-all.sh file of this repository.
  - Working directory: '$FilePath$'
- Click on OK, OK
- If you now right click a folder in your Project Tree and go to External Tools then you can click on clang-format which will format the entire folder (not this unfortunately does not yet work on single files).

## See also

### VSCode

To run roboteam code within VSCode, which John thinks is nicer, but nobody else agrees with :(, you can always ask him for help on setting up.

Running with camera's and real robots:

- [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision)

Working with a referee

- [SSL-Game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
- [SSL-vision-client](https://github.com/RoboCup-SSL/ssl-vision-client)

Working with the grSim simulator

- [grSim](https://github.com/RoboTeamTwente/grSim)
