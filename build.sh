#! /bin/bash

# Exit on errors
# Note: do not concat commands with && otheriwise won't be catched
set -eo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
ORANGE='\033[0;33m'
RESET='\033[0m'

error() {
    echo -e "${RED}Fatal error on line $1${RESET}" >&2
}

trap 'error $LINENO' ERR

help() {
    echo "Usage: build.sh [-y] [-o / -o <COMMIT>]
                -y : ignore docker enviroment check
                -o : build AI from given commit (or origin/main~1 if not specified) and save output in build_old folder
                -s : skip external

    Example: ./build.sh -o origin/main~3
             ./build.sh -o bedd8e7ed

    './build.sh -h' to print this help msg."
    exit 0
}

OK=0
OLD=0
SKIP=0
COMMIT=origin/main~1

while getopts ":yo:s" opt; do
  case ${opt} in
    y ) OK=1
      ;;
    o ) OLD=1
        COMMIT=${OPTARG}
      ;;
    s ) SKIP=1
      ;;
    \? ) help
      ;;
  esac
done

# Check if the script is being executed from inside container or explicitly
# asked to run it on host environment
if [ -f /.dockerenv ] || [ "$OK" = "1" ];
then
    if [ "$0" == "./build.sh" ];
    then
        if [ $OLD == 0 ];
        then
            echo -e "${GREEN}Checking submodules${RESET}"
            git submodule update --init --recursive
            echo "Done"
            #rm -rf build
            mkdir -p build
            pushd build
                echo -e "${GREEN}Building RoboTeamTwente software${RESET}"
                cmake .. -DCMAKE_BUILD_TYPE=Release
                make -j$(nproc) roboteam_observer roboteam_ai roboteam_robothub
            popd
                echo -e "${GREEN}Installing interface${RESET}"
            pushd roboteam_interface
                yarn install
            popd
            if [ $SKIP == 0 ];
            then
                pushd external
                    echo -e "${GREEN}Building external${RESET}"
                    pushd framework
                        echo -e "${GREEN}Building external/framework${RESET}"
                        mkdir -p build
                        pushd build
                        cmake ..
                        make simulator-cli -j$(nproc)
                        popd
                    popd
                    pushd autoref
                        echo -e "${GREEN}Building external/autoref${RESET}"
                        mkdir -p build
                        pushd build
                        cmake ..
                        make autoref-cli -j$(nproc)
                        popd
                    popd
                popd
            fi
            echo -e "${GREEN}Done, exiting builder..${RESET}"
        else
            echo -e "${GREEN}Checking submodules${RESET}"
            git submodule update --init --recursive
            echo "Done"
            rm -rf build_old # Delete previously built folder, otherwise compose starts with wrong version
            mkdir -p build_old/release
            pushd build_old
                echo -e "${GREEN}Building old RoboTeamTwente AI${RESET}"
                git clone https://github.com/tollsimy/roboteam-rl.git /tmp/roboteam || true # Do not fail if directory already exist
                pushd /tmp/roboteam/
                    echo -e "${GREEN}Building old commit: $(git describe --always --dirty)${RESET}"
                    git reset --hard --recurse-submodule $COMMIT
                    echo -e "${GREEN}Calling 'build.sh' in the old commit repo${RESET}"
                    ./build.sh -s
                popd
                cp -a /tmp/roboteam/build/release/* release/
                # Start with a fresh repo in next build, otherwise problems when moving HEAD
                rm -rf /tmp/roboteam
            popd
            echo -e "${GREEN}Done, exiting builder..${RESET}"
        fi
    else
        echo -e "${RED}E: The script must be called from the root folder${RESET}"
        exit 1
    fi
else
    echo -e "${ORANGE}W: If you really want to build outside the container call the script with -y argument.${RESET}"
    exit 1
fi
