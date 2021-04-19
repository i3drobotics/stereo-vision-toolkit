# !/bin/bash

# DO NOT USE. WORK IN PROGRESS.

# Set working directory to script directory
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd $SCRIPTPATH

# Install dependencies
sudo apt install build-essential checkinstall zlib1g-dev -y

# Download OpenSSL source
sudo wget https://www.openssl.org/source/openssl-1.1.1g.tar.gz

# Extract contents
sudo tar -xf openssl-1.1.1g.tar.gz
cd openssl-1.1.1g

# Build and install OpenSSL
sudo ./config --prefix=/usr/local/ssl --openssldir=/usr/local/ssl shared zlib
sudo make
sudo make test
sudo make install