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


# Usage
First, clone this repository. We recommend using an SSH key if you are able to do so, as this immediately authorizes your future pushes to the repositories. 
```
git clone --recurse-submodule git@github.com:RoboTeamTwente/roboteam_suite.git
```
Now, you can open the roboteam_suite project with an IDE of your choice. We recommed CLion, as it has a lot of very helpful features and you can get a student license for it. 

Make sure you have CMake installed. Go to the CMakeLists.txt and click on it. CLion will generate a pop-up saying "project files not loaded" in blue in the top right corner. Clicking it will have CMake load the project. This means it generates instructions on how the files should be compiled in order to produce the executables specified in the CMakeLists.txt file. You can now compile the targets!

The easiest way to run the application is to make some [compound executables](https://www.jetbrains.com/help/clion/run-debug-configuration.html#config-folders) in CLion that boot everything you need. To run 1 team in GRSim, the simulator, you will need to add roboteam_ai, roboteam_world, and roboteam_robothub. 
Make sure you have GRSim or ssl-vision running and are listening to the right ports. In general, the GRSim vision multicast port should be set to 10006.

## Auxiliary Software
It is recommended to use the game controller to control the gamestates. You can do so by downloading the latest binary release of the [SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller) repo, and executing it. For example, when downloaded to the Downloads folder:
```
~/Downloads/ssl-game-controller_v1.2.0_linux_amd64
```
You might need to [chmod](https://www.howtoforge.com/tutorial/linux-chmod-command/) the file to make it executable. The AI client should now listen to the commands sent by the GameController. 

Also, make sure you have installed GRSim using the instructions found on the [RoboTeam fork of GRSim](https://github.com/RoboTeamTwente/grSim). Now execute the grsim binary from the command line. This usually looks like this:
```
/directory_you_cloned_grsim/grSim/bin/grsim
```
## Making the executables: 

Make sure the vision multicast port is set to 10006

### One Team: 
Make the compound executable shown below:
<img src="https://github.com/RoboTeamTwente/roboteam_suite/blob/RobotJesse-patch-1/readme_images/run_two_teams.png?raw=true" width="800" height="500"><br>
Then run this compound executable and run GRSim. 

### Two Teams:
Follow the steps used for the other compound executable, only now add 2 extra targets: roboteam_ai_1 and roboteam_robothub_1, as seen in the screenshot, to the compound. While creating the new CMake Targets, be sure to include a "1" in the program arguments, as seen in the image.
<img src="https://github.com/RoboTeamTwente/roboteam_suite/blob/RobotJesse-patch-1/readme_images/run_two_teams.png?raw=true" width="800" height="500"><br>



# Installation
## List of dependencies

- [CMake 3.16](https://cmake.org/)
- [Protobuf 3.9.1](https://developers.google.com/protocol-buffers/)
- [ZeroMQ 4.2.5](https://zeromq.org/)
- [Eigen3 3.3.7-2](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [QT5](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)
- [QT5Charts](https://doc.qt.io/qt-5/qtcharts-index.html)
- [Google Test and Google Mock](https://github.com/google/googletest)
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
$ sudo apt-get install libzmq3-dev
```
### Install Eigen3
```
$ sudo apt install libeigen3-dev
```

### Install QT5
```
$ sudo apt-get install qt5-default
```

### Install QT5-Charts
```
$ sudo apt install libqt5charts5-dev 
```
### Install Google Test
If you want to run tests, please install and build gtest using the following instructions:
`sudo apt-get install lcov gcovr` <br>
`sudo apt install libgtest-dev cmake` <br>
`cd /usr/src/gtest`<br>
`sudo cmake CMakeLists.txt`<br>
`sudo make`<br>
`sudo cp *.a /usr/lib`<br>
`cd /usr/src/gmock`<br>
`sudo cmake CMakeLists.txt`<br>
`sudo make`<br>
`sudo cp *.a /usr/lib`<br>

(https://stackoverflow.com/questions/28869319/cannot-find-lgtest-when-setting-up-google-test) <br>
\* You can follow similar steps for google mock, should you need it

## Installation on macOS (tested on macOS 11.4 Big Sur)
Make sure you already have the following:
- XCode
- XCode Command Line Tools (Run XCode once, then `xcode-select --install`)
- Homebrew

```
$ brew install cmake zmq armadillo libsodium
$ brew install protobuf --HEAD     # As of 3.17.3, protobuf is is broken and neeeds to be compiled from HEAD
```
## Code Style Formatting
To be consistent in the code style, we use clang-format. You can use clang-format to automatically format your code during every commit and you can use clang-format in CLion to format all code files in a given folder.

### Commit Formatting
1. Open a terminal and run 'sudo apt-get install clang-format'
2. Run install.sh in the cloned 'formatting' folder with as working directory the cloned roboteam_suite folder. 
This will make sure that the code is formatted during every commit from then on. 

### CLion Formatting
1. Open CLion and go to File->Settings->Tools->External Tools
2. Click on Add and fill in without quotes: 
- Name: 'clang-format'
- Program: The location on your computer to the formatting/format-all.sh file of this repository.
- Working directory: '$FilePath$'
3. Click on OK, OK
4. If you now right click a folder in your Project Tree and go to External Tools then you can click on clang-format which will format the entire folder (not this unfortunately does not yet work on single files).

## Optional Libraries:
### Install Pagmo
Pagmo is an optimization library. It has some nice features to explore. 
Before installing make sure you have TBB, the intel threading library, installed via apt
```
$ sudo apt install libtbb-dev
$ git clone https://github.com/esa/pagmo2.git
$ cd /path/to/pagmo
$ mkdir build
$ cd build

$ cmake ../
$ cmake --build .
$ cmake --build . --target install
```


# See also

Running with camera's and real robots:
SSL-Vision (https://github.com/RoboCup-SSL/ssl-vision)

Working with a referee
[SSL-Game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
[SSL-vision-client](https://github.com/RoboCup-SSL/ssl-vision-client)

Working with the grSim simulator
[grSim](https://github.com/RoboTeamTwente/grSim) 
