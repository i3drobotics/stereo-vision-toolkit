# !/bin/bash

# DO NOT USE. WORK IN PROGRESS.

# Set working directory to script directory
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd $SCRIPTPATH

sudo apt-get install libboost-dev=1.66.0