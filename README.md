# RoboTeam

## Overview
This repository contains the following projects:

- `RoboTeam World`. It is responsible from receiving and combining all sources of information into one coherent world state, which can be used by `RoboTeam AI`. These sources include data from either a simulator or SSL-Vision, and feedback from our robots (WIP). Note: The executable is called `roboteam_observer`*. 
- `RoboTeam AI`. The brain of RoboTeam. It makes decisions on what robots should do based on data received from `RoboTeam World`.
- `RoboTeam Robothub`. Responsible for the communication with a simulator or the basestation (and thus the robots). `RoboTeam AI` sends its commands here, to be forwarded.
- `RoboTeam Utils`. It contains a lot of helper functions, primarily geometrical in nature, such as Lines, Triangles, etc. These are used in other repo's, such as RoboTeam AI and RoboTeam World
- `RoboTeam Networking`. This repository contains all our `Protobuf` messages, and classes for publishing and subscribing to `ZMQ` channels. It lets `RoboTeam World`, `RoboTeam AI`, and `RoboTeam Robothub` communicate with eachother. 

\* `RoboTeam World` its executable is named `roboteam_observer`, since `RoboTeam AI` also contains a class named `World`, which caused confusion.

## How to
Refer to `docker/README.md` for usage of our software.

## Libtorch
Note: there are plenty of issues when trying to link libtorch in an Alpine Linux container because of GLIBC version and musl. This is the reason why we switched to and Ubuntu23-based Docker image.

The libtorch version used is libtorch cpu: libtorch-cxx11-abi-shared-with-deps.

## Development
### IDEs
#### VSCode
You can install VSCode from the [website](https://code.visualstudio.com/download) or via `apt` or `snap`. We recommend to install the [`C/C++ Extension Pack`](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack) extension, which will provide you with IntelliSense (code completion), debugging, code browsing, and CMake support. Other neat extensions are [`GitLens`](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens), [`Git Graph`](https://marketplace.visualstudio.com/items?itemName=mhutchie.git-graph), [`GoogleTest Adapter`](https://marketplace.visualstudio.com/items?itemName=DavidSchuldenfrei.gtest-adapter), and [`Copilot`](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot). If you want to develop from within a container, install [`Remote Development`](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack).

#### Clion
To install [CLion](https://www.jetbrains.com/clion/), you need a [student license](https://www.jetbrains.com/community/education/#students/).

Make sure you have the dependencies installed. Within CLion, go to the CMakeLists.txt and click on it. CLion will generate a pop-up saying "project files not loaded" in blue in the top right corner. Clicking it will have CMake load the project. This means it generates instructions on how the files should be compiled in order to produce the executables specified in the CMakeLists.txt file. You can now compile the targets!

#### Code Style Formatting
To be consistent in the code style, we use clang-format. You can use clang-format to automatically format your code during every commit and you can use clang-format in CLion to format all code files in a given folder. Format the code by running `format.sh`
## See also

#### Tracy Profiler
Tracy is quite cool(and lightweight) profiler, that can help you analyze the performance of your code.

To enable Tracy
- Define `TRACY_ENABLE` macro (e.g. by passing -DTRACY_ENABLE=1 to cmake)
- Add ZoneScoped/ZoneScopedN etc. to scope you want to evaluate
- Compile Tracy Server and run
  - Information is in the tracy [docs](https://github.com/wolfpld/tracy)
- Run AI
