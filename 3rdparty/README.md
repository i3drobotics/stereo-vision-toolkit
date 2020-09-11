The project uses OpenCV for image processing, PCL and VTK for point cloud visualisation and hidapi for camera control. PCL requires Boost and Eigen which are included here. These dependencies are provided in accordance with their respective licenses which may be found in the license folder. We also use FontAwesome for icons via QtAwesome, along with QDarkStyle.

Some libraries have been moved to be externally downloaded to reduce the repostiory size. To download these libraries use the '3rdparty.bat' script provided in 'scripts'. This will download the libraries of the correct versions to the correct folders.  
*Note: This process builds boost with the required boost libraries so will take some time to complete.*

```
cd PATH_TO_REPO/scripts
3rdparty.bat
```

There is limited usage of CUDA for certain image processing steps (e.g. rectification).

This repository is used for internal development and so we include both debug and release libraries and DLLs. This makes for quite a large repository for a rather small codebase (around 500MB), so be warned.
