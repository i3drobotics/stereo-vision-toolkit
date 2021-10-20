#!/bin/bash

# exit on command failure
set -e

SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cd $SCRIPT_PATH/../

doxygen_path="/c/Program Files/doxygen/bin/doxygen.exe"
"$doxygen_path"