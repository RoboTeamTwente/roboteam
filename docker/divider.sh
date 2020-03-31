#!/bin/bash


# If you want to add a repo, you simply create a file called {repo_name}.sh in tests
# This should just contain the commands to run sequentially for the repo
# Should start with:
#
# cd /opt/roboteam/roboteam_suite
# git submodule foreach git checkout master
# git submodule foreach git pull
# cd {repo_name}
# git checkout "$1"


REPO_NAME=$(echo $2 | tr "/" "\n" | sed -n 2p)
echo "Repo: "
echo $REPO_NAME
echo "Branch: "
echo $1

export CCACHE_DIR=/opt/.ccache
export CXX=/usr/lib/ccache/clang++
export CC=/usr/lib/ccache/clang

# Checks if the test for the repo exists
if [[ ! -f "/opt/robotests/$REPO_NAME.sh" ]]; then
    echo "No such test found."
    exit 1
fi

# Cd into the suite
cd /opt/roboteam/roboteam_suite
# Pull each
git submodule foreach git pull
# cd into correct repo
cd "$REPO_NAME"
# Checkout branch name
git checkout "$1"
# Cd back and create + cd into build
cd ..
mkdir build
cd build
# Cmake generate build files
cmake -DCMAKE_BUILD_TYPE=Debug -G "CodeBlocks - Ninja" .. -DCMAKE_CXX_COMPILER=/usr/lib/ccache/clang++ -DCMAKE_CC_COMPILER=/usr/lib/ccache/clang

# Runs the test for the repo
/bin/bash -c "\"/opt/robotests/$REPO_NAME.sh\" \"$1\""
