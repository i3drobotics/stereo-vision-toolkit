#!/bin/bash

BUILD_VERSION=$1
LATEST_VERSION=$2

# modified from: https://stackoverflow.com/a/4025065

# compare version strings
vercomp () {
    if [[ $1 == $2 ]]
    then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            return 2
        fi
    done
    return 0
}

# check arg1 version is greater than arg2 version
vergt () {
    vercomp $1 $2
    res=$?
    # case $? in
    #     0) op='=';;
    #     1) op='>';;
    #     2) op='<';;
    # esac
    if [[ $res != 1 ]]
    then
        echo "ERROR: Build Version must be greater than currently released version."
    else
        echo "Build version is valid."
    fi
}

echo "BUILD_VERSION: $BUILD_VERSION"
echo "LATEST_VERSION: $LATEST_VERSION"
vergt $BUILD_VERSION $LATEST_VERSION