# !/bin/bash

# DO NOT USE. WORK IN PROGRESS.

# Set working directory to script directory
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd $SCRIPTPATH

# Define OpenCV version
opencv_version=4.5.0

# Define build / install folders
install_folder = $PWD/opencv/install
build_folder = $PWD/opencv/build

# Download and install CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-ubuntu1604.pin
sudo mv cuda-ubuntu1604.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/ /"
sudo apt-get update
sudo apt-get -y install cuda

# Download cudnn 8.0.5. Uses secure redirect link to avoid needing to join developer program 
wget -O libcudnn8_8.0.5.39-1+cuda11.1_amd64.deb https://developer.download.nvidia.com/compute/machine-learning/cudnn/secure/8.0.5/11.1_20201106/Ubuntu16_04-x64/libcudnn8_8.0.5.39-1%2Bcuda11.1_amd64.deb?pqUXol3nvqRYPFtuFMLlzzSZ2g2VZ7GD75M9Ff4BgKykVwGpUWqeZ6Y1pgYl9pLJEs58N4d44r0c72KrkvMvBmtV6-unSh9nAV32sSO1tmGiw-hJ4SaH47GV4vtLPaIs8fvaQpL2mNkL-9J-kZubgcjouZLcohT41gB5JYaaO7Wa54ltxsPYNwUEb3NYVnE6YGN9fcApPPduV8ej5Cj8NuScBvwvWRKcBW94Cq8k37f5Rg
wget -O libcudnn8-dev_8.0.5.39-1+cuda11.1_amd64.deb https://developer.download.nvidia.com/compute/machine-learning/cudnn/secure/8.0.5/11.1_20201106/Ubuntu16_04-x64/libcudnn8-dev_8.0.5.39-1%2Bcuda11.1_amd64.deb?E_1fGc1AOWyn1g1cXF5eBRqCAASR6uXAR49i5QzpMnoZDj4ug06R1WBY2ZdzuiFLxCSu0GKs5DNn7DbiTxosRvAvglGdQ1NOecY_2f7GngVemWOgy_QCRzV6ym77C0T3uKowbgHZ7OHNzjhISoN0DzM3Ic5hXiGdO4O2GRaePFxJrzBB7ABrfUMbY0X00JAEAvX2KqS_nOuk7npARzCW81PeEqzrvWWqh_HRHojOypnQ_N-WeqI
sudo dpkg -i libcudnn8_8.0.5.39-1+cuda11.1_amd64.deb
sudo dpkg -i libcudnn8-dev_8.0.5.39-1+cuda11.1_amd64.deb

# Install OpenCV dependencies
sudo apt install --assume-yes build-essential cmake git pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
sudo apt install libhdf5-dev
sudo apt install --assume-yes libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
sudo apt install --assume-yes libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt install --assume-yes libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
sudo apt install --assume-yes libvorbis-dev libxvidcore-dev v4l-utils

# Clone OpenCV repo
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout tags/$opencv_version
cd ..

# Clone OpenCV contrib repo
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout tags/$opencv_version
cd ..
opencv_contrib_path = $PWD/opencv_contrib/modules

# Create build and install folders
mkdir $build_folder
mkdir $install_folder

# Move to build folder
cd $build_folder

# Build OpenCV
cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=$install_folder \
    -D BUILD_opencv_world=ON \
    -D WITH_CUDA=ON \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_FFMPEG=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=$opencv_contrib_path \
    -D BUILD_opencv_python=OFF \
    -D BUILD_opencv_python3=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_EXAMPLES=OFF ..