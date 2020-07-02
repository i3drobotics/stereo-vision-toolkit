# Stereo Vision Toolkit

[Software Website](https://i3drobotics.github.io/stereo-vision-toolkit/)

## Latest Release
Download the latest release v1.3 [here](https://github.com/i3drobotics/stereo-vision-toolkit/releases/download/v1.3.0/StereoVisionToolkit-1.3.0-Win64.exe)

For detailed instructions see the [User Guide](https://i3drobotics.github.io/stereo-vision-toolkit/app/UserGuide.pdf)

## About

I3DR's stereo vision toolkit is an application provided for testing of stereo cameras and gathering of 3D data.

You can calibrate stereo cameras, acquire images and perform matching and 3D reconstruction. You can save raw stereo video to capture a scene and then replay it in the software to fine tune matching parameters.

SVTK is under active development. At the moment the software only officially supports the i3DR Deimos and Phobos cameras, but in principle any usb camera pair or pre-rectified stereo video of the correct format (side-by-side) will work.

## Roadmap

Currently SVTK is a useful and functional tool for exploring stereo imaging, and allows you to get going with your I3DR stereo camera quickly. There are a number of features/improvements in development including:

- Automated camera calibration (currently in beta)
- Linux support
- ROS support
- Vimba camera support
- Unit tests and other deployment improvements

See [issues](https://github.com/i3drobotics/stereo-vision-toolkit/issues) for details on known bugs and future enhancements

## Stereo matching support

We have included support for two of OpenCV's matchers: the basic block matcher and semi-global block matching. The block matcher will run at over 60fps on a fast CPU (e.g. i5.) SGBM should provide better results, but will run around a factor of five slower.

I3DR's own 3D matching algorithm (I3DRSGM) can be built into this application however the files required are not included directly in this repository. This is because it uses proprietary libraries owned by I3DR. For this reason, these files cannot be open source and are kept in the submodule i3drsgm. However, releases are built with I3DRSGM support. 
Please contact info@i3drobotics.com for requesting a license. 

## Installation

Download and install the windows installer for the latest release [here](https://github.com/i3drobotics/stereo-vision-toolkit/releases/download/v1.3.0/StereoVisionToolkit-1.3.0-Win64.exe).

See the [User Guide](https://i3drobotics.github.io/stereo-vision-toolkit/app/UserGuide.pdf) for details on using the software as well as building from source. 

## Development

This application is under active development and it is likely that in the short term there may be breaking changes to classes, or significant changes to the GUI. Have a look at the [project milestones](https://github.com/i3drobotics/stereo-vision-toolkit/milestones) for information on future improvements.

Feel free to send us pull requests!

## License

This code is provided under the MIT license, which essentially means it's open source, but we require you to add our copyright if you distribute it elsewhere.

## Previous Releases

See [release](https://github.com/i3drobotics/stereo-vision-toolkit/releases) for previous builds. 

## Developer zone
See [User Guide](https://i3drobotics.github.io/stereo-vision-toolkit/app/UserGuide.pdf) for details on building from source.

Documentation can be found [here](https://i3drobotics.github.io/stereo-vision-toolkit/definitions/html/index.html). 

### 3rd Party Content

The project uses OpenCV for image processing, PCL and VTK for point cloud visualisation and hidapi for camera control. PCL requires Boost and Eigen which are included here. These dependencies are provided in accordance with their respective licenses which may be found in the license folder. We also use FontAwesome for icons via QtAwesome, along with QDarkStyle.

There is limited usage of CUDA for certain image processing steps (e.g. rectification).

This repository is used for internal development and so we include both debug and release libraries and DLLs. This makes for quite a large repository for a rather small codebase (around 500MB), so be warned.

### Phobos control
Arduino code for controlling Phobos cameras is provided in src/camera/camera_control.
This is for the serial communication between the arduino and this toolkit. See [issue](https://github.com/i3drobotics/stereo-vision-toolkit/issues/54) for more information.
