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

# Runs the test for the repo
bash "/opt/robotests/$REPO_NAME.sh" "$1"