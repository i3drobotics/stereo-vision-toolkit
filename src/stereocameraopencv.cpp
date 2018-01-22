/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocameraopencv.h"

bool StereoCameraOpenCV::initCamera(int left_index, int right_index) {
  left_camera = new CameraOpenCV;
  right_camera = new CameraOpenCV;

  bool res = left_camera->initCamera(left_index);
  res &= right_camera->initCamera(right_index);

  if (res) {
    res &= left_camera->setMaximumResolution();
    res &= right_camera->setMaximumResolution();

    left_camera->setExposure(32);
    right_camera->setExposure(32);

    left_camera->setGain(0);
    right_camera->setGain(0);

    left_camera->getImageSize(image_width, image_height, image_size);
  }else{
      disconnectCamera();
  }

  connected = res;

  return res;
}

bool StereoCameraOpenCV::setExposure(double exposure) {
  bool res = left_camera->setExposure(exposure);
  res &= right_camera->setExposure(exposure);

  return res;
}

bool StereoCameraOpenCV::initCamera(void) { return initCamera(0, 1); }

bool StereoCameraOpenCV::capture() {
  QFuture<bool> left_result =
      QtConcurrent::run(left_camera, &CameraOpenCV::capture);
  QFuture<bool> right_result =
      QtConcurrent::run(right_camera, &CameraOpenCV::capture);

  left_result.waitForFinished();
  right_result.waitForFinished();

  if (left_result.result() && right_result.result()) {
    left_raw.data = left_camera->getImage()->data;
    right_raw.data = right_camera->getImage()->data;

    return true;
  }

  return false;
}

void StereoCameraOpenCV::disconnectCamera() {
  left_camera->close();
  right_camera->close();
}

StereoCameraOpenCV::~StereoCameraOpenCV() {
  disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
  disconnectCamera();
  emit finished();
}
