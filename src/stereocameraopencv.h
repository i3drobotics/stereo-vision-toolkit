/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/


#ifndef STEREOCAMERAOPENCV_H
#define STEREOCAMERAOPENCV_H

#include <abstractstereocamera.h>
#include "cameraopencv.h"

//!  Stereo OpenCV camera control
/*!
  Control of stereo pair of OpenCV cameras and 3D generation
*/

class StereoCameraOpenCV : public AbstractStereoCamera
{
Q_OBJECT

public:
    explicit StereoCameraOpenCV(QObject *parent = 0) :
                AbstractStereoCamera(parent)
                {}
    void setDevices(int left_index, int right_index);
    bool capture();
    void disconnectCamera();
    bool initCamera(int left_index, int right_index);
    bool initCamera(void);
    bool setExposure(double exposure);
    ~StereoCameraOpenCV(void);

private:
    CameraOpenCV *left_camera;
    CameraOpenCV *right_camera;

    bool getImageSize(CameraOpenCV *camera);

    QThread *left_thread;
    QThread *right_thread;
};

#endif //STEREOCAMERAOPENCV_H
