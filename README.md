# RoboTeam

[TOC]

## Overview
This software is tested on Ubuntu 22.04 LTS, and the installation instructions are written with that distro/OS in mind. However, other distro's of Linux are also likely to be able to run this software, albeit with some modifications.

This repository contains the following projects:

- `RoboTeam World`. It is responsible from receiving and combining all sources of information into one coherent world state, which can be used by `RoboTeam AI`. These sources include data from either a simulator or SSL-Vision, and feedback from our robots (WIP). Note: The executable is called `roboteam_observer`*. 
- `RoboTeam AI`. The brain of RoboTeam. It makes decisions on what robots should do based on data received from `RoboTeam World`.
- `RoboTeam Robothub`. Responsible for the communication with a simulator or the basestation (and thus the robots). `RoboTeam AI` sends its commands here, to be forwarded.
- `RoboTeam Utils`. It contains a lot of helper functions, primarily geometrical in nature, such as Lines, Triangles, etc. These are used in other repo's, such as RoboTeam AI and RoboTeam World
- `RoboTeam Networking`. This repository contains all our `Protobuf` messages, and classes for publishing and subscribing to `ZMQ` channels. It lets `RoboTeam World`, `RoboTeam AI`, and `RoboTeam Robothub` communicate with eachother. 

\* `RoboTeam World` its executable is named `roboteam_observer`, since `RoboTeam AI` also contains a class named `World`, which caused confusion.



## Dependencies
All dependencies are listed in `install_dependencies_ubuntu22-04.sh`. Simply run this file to install all of them. Additionally, it installs other useful packages.

## IDEs
### VSCode
You can install VSCode from the [website](https://code.visualstudio.com/download) or via `apt` or `snap`. We recommend to install the [`C/C++ Extension Pack`](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack) extension, which will provide you with IntelliSense (code completion), debugging, code browsing, and CMake support. Other neat extensions are [`GitLens`](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens), [`Git Graph`](https://marketplace.visualstudio.com/items?itemName=mhutchie.git-graph), [`GoogleTest Adapter`](https://marketplace.visualstudio.com/items?itemName=DavidSchuldenfrei.gtest-adapter), and [`Copilot`](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot). If you want to develop from within a container, install [`Remote Development`](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack).

### Clion
To install [CLion](https://www.jetbrains.com/clion/), you need a [student license](https://www.jetbrains.com/community/education/#students/).

Make sure you have the dependencies installed. Within CLion, go to the CMakeLists.txt and click on it. CLion will generate a pop-up saying "project files not loaded" in blue in the top right corner. Clicking it will have CMake load the project. This means it generates instructions on how the files should be compiled in order to produce the executables specified in the CMakeLists.txt file. You can now compile the targets!

## Running a game
![alt text](readme_images/rtt_software_stack.png)

To run a game, you will need to boot up `RoboTeam AI`, `RoboTeam World`, and `RoboTeam Robothub`. The executables are named `roboteam_ai`, `roboteam_observer`, and `roboteam_robothub` respectively. Next to that, either boot up a simulator or SSL-Vision.
 
## SSL Software / Simulators
* (`SSL Vision`)[https://github.com/RoboCup-SSL/ssl-vision]
* (`SSL Game Controller`)[https://github.com/RoboCup-SSL/ssl-game-controller]
* (`GrSim simulator`)[https://github.com/RoboCup-SSL/grSim]
* (`ER-Force simulator`)[https://github.com/robotics-erlangen/framework#simulator-cli]
* RTT simulator
* (`Other SSL software`)[https://github.com/RoboCup-SSL]

It is recommended to use the game controller to control the gamestates. You can do so by downloading the latest binary release of the [SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller) repo, and executing it. For example, when downloaded to the Downloads folder:

```bash
~/Downloads/ssl-game-controller_v1.2.0_linux_amd64
```

You might need to [chmod](https://www.howtoforge.com/tutorial/linux-chmod-command/) the file to make it executable. The AI client should now listen to the commands sent by the GameController.

Also, make sure you have installed GRSim using the instructions found on the [RoboTeam fork of GRSim](https://github.com/RoboTeamTwente/grSim). Now execute the grsim binary from the command line. This usually looks like this:

```bash
/directory_you_cloned_grsim/grSim/bin/grsim
```






Running with camera's and real robots:

- [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision)

Working with a referee

- [SSL-Game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
- [SSL-vision-client](https://github.com/RoboCup-SSL/ssl-vision-client)

Working with the grSim simulator

- [grSim](https://github.com/RoboTeamTwente/grSim)






## Installation Ubuntu
All dependencies are listed in `install_dependencies_ubuntu22-04.sh`. Simply run the script to install all of these, and some useful other packages.

## Installation on macOS (tested on macOS 12.2.1 Monterey)
Make sure you already have the following:
- XCode
- XCode Command Line Tools (Run XCode once, then `xcode-select --install`)
- Homebrew

Install remaining dependencies
```
$ brew install cmake zmq armadillo libsodium qt@5 protobuf llvm
```

Create a sparse (bundle) disk image with a case-sensitive FS to hold the source code
```
$ hdiutil create -size 2g -volname rbtt-workspace -layout GPTSPUD -fs "Case-sensitive APFS" -type SPARSEBUNDLE -nospotlight ~/Desktop/rbtt-workspace
$ hdiutil attach ~/Desktop/rbtt-workspace
$ cd /Volumes/rbtt_workspace/
$ git clone git@github.com:RoboTeamTwente/roboteam_suite.git
$ cd roboteam_suite/
$ git submodule update --init --recursive
$ git submodule foreach git checkout development
```

Select a newer clang to be used to compile RBTT software

1. Open CLion
2. Go to Preferences -> Build, Execution, Deployment -> Toolchains -> Default
3. Change C Compiler to:  `/usr/local/Cellar/llvm/13.0.1/bin/clang`
4. Change C++ Compiler to: `/usr/local/Cellar/llvm/13.0.1/bin/clang++`

Notes:
* C(++) Compiler paths may differ based on installed version. Verify the version with `$ brew info llvm`
* Toolchains may vary if you changed the default CLion settings. If you want to deviate from this readme, ensure the compiler selected in the toolchain used to compile RBTT software has support for C++20 



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

### Tracy Profiler
Tracy is quite cool(and lightweight) profiler, that can help you analyze the performance of your code.

To enable Tracy
- Define `TRACY_ENABLE` macro (e.g. by passing -DTRACY_ENABLE=1 to cmake)
- Add ZoneScoped/ZoneScopedN etc. to scope you want to evaluate
- Compile Tracy Server and run
  - Information is in the tracy [docs](https://github.com/wolfpld/tracy)
- Run AI


