# !/bin/bash

# DO NOT USE. WORK IN PROGRESS.

# Set working directory to script directory
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd $SCRIPTPATH

# Useful links
# https://kezunlin.me/post/91842b71/

# Install dependencies
sudo apt-get update
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui 
sudo apt-get install libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common 

sudo apt-get install libpcap-dev
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libopenni2-dev
sudo apt-get install libqhull7 libqhull-dev 

sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev 
sudo apt-get install mono-complete
sudo apt-get install openjdk-8-jdk openjdk-8-jre

sudo apt-get install build libgl1-mesa-dev

# purge previous versions of QT
sudo apt-get purge qt5-default qtcreator 
sudo apt-get purge qt4-designer qt4-dev-tools

# Download QT
wget http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-linux-x64-5.7.0.run
# Install QT
chmod +x qt-opensource-linux-x64-5.7.0.run
./qt-opensource-linux-x64-5.7.0.run

# Download PCL repo
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz

# Download vtk
wget https://www.vtk.org/files/release/8.1/VTK-8.1.0.tar.gz
wget https://www.vtk.org/files/release/8.1/VTKData-8.1.0.tar.gz

# Create vtk build folder
mkdir VTK-8.1.0/build
cmake \
    -D VTK_Group_Qt=ON \
    -D VTK_QT_VERSION=5 \
    -D QT5_DIR=/opt/qt/5.7/gcc_64/lib/cmake/Qt5 \
    -D VTK_RENDERING_BACKEND=OpenGL2 \
    -D BUILD_SHARED_LIBS=ON \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    ..

cd ../..

# Create pcl build folder
pcl_build_folder=$PWD/pcl-1.8.1/build
mkdir $pcl_build_folder
cd $pcl_build_folder

# Build PCl
cmake \
    -D QT_USE_FILE=$pcl_build_folder/use-qt5.cmake \
    -D VTK_DIR=/usr/local/lib/cmake/vtk-8.1 \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_CONFIGURATION_TYPES=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D PCL_SHARED_LIBS=ON \
    -D PCL_QT_VERSION=5 \
    -D PCL_ENABLE_SSE=ON \
    -D Build_visualization=ON \
    -D Build_apps=ON \
    -D Build_examples=OFF ..