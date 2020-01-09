/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "stereocamerabasler.h"

bool StereoCameraBasler::initCamera(std::string left_camera_name, std::string right_camera_name, int binning) {
  left_camera = new CameraBasler;
  right_camera = new CameraBasler;

  bool res = left_camera->initCamera(left_camera_name,binning);
  res &= right_camera->initCamera(right_camera_name,binning);

  if (res) {
    res &= left_camera->setMaximumResolution();
    res &= right_camera->setMaximumResolution();

    left_camera->setExposure(5);
    right_camera->setExposure(5);

    left_camera->setGain(0);
    right_camera->setGain(0);

    left_camera->getImageSize(image_width, image_height, image_size);
  }else{
      qDebug() << "Failed to setup cameras";
      disconnectCamera();
  }

  connected = res;

  return res;
}

//No paramters sent to initCamera so use default
bool StereoCameraBasler::initCamera(void) { return initCamera("phobos_nuclear_l", "phobos_nuclear_r", 2); }

std::vector<std::string> StereoCameraBasler::listSystems(void){
    // find basler systems connected
    // Initialise Basler Pylon
    Pylon::PylonInitialize();
    // Create an instant camera object with the camera device found first.
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);

    Pylon::CInstantCameraArray cameras(devices.size());
    Pylon::CDeviceInfo info;

    std::vector<std::string> camera_names;

    for (size_t i = 0; i < cameras.GetSize(); ++i)
    {
        cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
        std::string camera_serial = std::string(cameras[i].GetDeviceInfo().GetSerialNumber());
        //camera_names.push_back(std::string(cameras[i].GetDeviceInfo().GetUserDefinedName()));
        camera_names.push_back(camera_serial);
        std::cout << "Camera found... " << cameras[i].GetDeviceInfo().GetSerialNumber() << std::endl;
    }
    return camera_names;
}

bool StereoCameraBasler::autoConnect(void){
    std::vector<std::string> camera_names = listSystems();

    if (camera_names.size() == 2){
        initCamera(camera_names.at(0),camera_names.at(1), 2);
    } else {
        std::cerr << "More than 2 basler cameras found. Cannot auto connect." << std::endl;
    }
    return true;
}

bool StereoCameraBasler::setExposure(double exposure) {
  bool res = left_camera->setExposure(exposure);
  res &= right_camera->setExposure(exposure);

  return res;
}

bool StereoCameraBasler::enableAutoExpose(bool enable){
    bool res = left_camera->setAutoExposure(enable);
    res &= right_camera->setAutoExposure(enable);

    return res;
}

bool StereoCameraBasler::capture() {
  QFuture<bool> left_result =
      QtConcurrent::run(left_camera, &CameraBasler::capture);
  QFuture<bool> right_result =
      QtConcurrent::run(right_camera, &CameraBasler::capture);

  left_result.waitForFinished();
  right_result.waitForFinished();

  if (left_result.result() && right_result.result()) {
    cv::Mat *left_data = left_camera->getImage();
    cv::Mat *right_data = right_camera->getImage();
    left_data->copyTo(left_raw);
    right_data->copyTo(right_raw);

    emit left_captured();
    emit right_captured();

    return true;
  }

  return false;
}

void StereoCameraBasler::disconnectCamera() {
  left_camera->close();
  right_camera->close();
  connected = false;
  emit finished();
  emit disconnected();
}

StereoCameraBasler::~StereoCameraBasler() {
  disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
  disconnectCamera();
  Pylon::PylonTerminate();
}
