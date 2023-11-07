#! /bin/bash

set -e

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
        echo "Checking submodules"
        git submodule update --init --recursive
        echo "Done"
        #rm -rf build
        mkdir -p build
        pushd build
            echo "Building RoboTeamTwente software"
            cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) roboteam_observer roboteam_ai roboteam_robothub
        popd
            echo "Installing interface"
        pushd roboteam_interface
            yarn install
        popd
        pushd external
            echo "Building external"
            pushd framework
                echo "Building external/framework"
                mkdir -p build && pushd build
                cmake .. && make simulator-cli -j$(nproc)
                popd
            popd
            pushd autoref
                echo "Building external/autoref"
                mkdir -p build && pushd build
                cmake .. && make autoref-cli -j$(nproc)
                popd
            popd
        popd
        echo "Done, exiting builder.."
    else
        echo "E: The script must be called from the root folder"
        exit 1
    fi
else
    echo "W: If you really want to build outside the container call the script with -y argument."
    exit 1
fi
