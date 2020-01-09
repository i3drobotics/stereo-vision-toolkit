/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "cameraopencv.h"

CameraOpenCV::CameraOpenCV(QObject *parent) : QObject(parent) {}

void CameraOpenCV::assignThread(QThread *thread) {
  moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

bool CameraOpenCV::isAvailable() { return camera.isOpened(); }

void CameraOpenCV::close() { camera.release(); }

bool CameraOpenCV::initCamera(int index) {
  camera = cv::VideoCapture(index + cv::CAP_DSHOW);

  if (!camera.isOpened()) {
    return false;
  }else{
      qDebug() << "Opened camera" << index;
      return true;
  }
}

void CameraOpenCV::getImageSize(int &image_width, int &image_height,
                                cv::Size &image_size) {
  image_width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
  image_height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
  image_size = cv::Size(image_width, image_height);
}

bool CameraOpenCV::setMaximumResolution(void) {
  bool res = camera.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  res &= camera.set(CV_CAP_PROP_FRAME_HEIGHT, 960);

  return res;
}

bool CameraOpenCV::setFrame16(void) {
  bool res = false;

  // Note this will not work in versions of OpenCV prior to 4.0 due
  // to a bug in the DShow driver.
  res = camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));
  camera.set(CV_CAP_PROP_CONVERT_RGB, false);

  image_format = Y16;

  return res;
}

bool CameraOpenCV::setFrame8(void) {
  bool res = false;

  // Note this will not work in versions of OpenCV prior to 4.0 due
  // to a bug in the DShow driver.
  res = camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
  camera.set(CV_CAP_PROP_CONVERT_RGB, false);

  image_format = Y800;

  return res;
}

bool CameraOpenCV::setExposure(double exposure) {
  return camera.set(CV_CAP_PROP_EXPOSURE, -log2(exposure));
}

bool CameraOpenCV::setGain(double gain) {
  return camera.set(CV_CAP_PROP_GAIN, gain);
}

bool CameraOpenCV::capture(void) {
  bool res = camera.read(image_buffer);

  if (image_buffer.channels() > 1) {
    split(image_buffer, channels);
    image = channels[0];
  } else {
    image = image_buffer;
  }

  if (res) {
    emit captured();
  }
  return res;
}

cv::Mat *CameraOpenCV::getImage(void) { return &image; }

CameraOpenCV::~CameraOpenCV(void) { camera.release(); }
