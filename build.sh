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

if [ "$1" == "-y" ];
then
    OK=1
fi

# Check if the script is being executed from inside container or explicitly
# asked to run it on host environment
if [ -f /.dockerenv ] || [ "$OK" = "1" ];
then
    if [ "$0" == "./build.sh" ];
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
        echo -e "${GREEN}Done, exiting builder..${RESET}"
    else
        echo -e "${RED}E: The script must be called from the root folder${RESET}"
        exit 1
    fi
else
    echo -e "${ORANGE}W: If you really want to build outside the container call the script with -y argument.${RESET}"
    exit 1
fi
