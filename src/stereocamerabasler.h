/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/


#ifndef STEREOCAMERABASLER_H
#define STEREOCAMERABASLER_H

#include <abstractstereocamera.h>
#include "camerabasler.h"

//!  Stereo balser cameras
/*!
  Control of stereo pair of basler cameras and generation of 3D
*/

class StereoCameraBasler : public AbstractStereoCamera
{
Q_OBJECT

public:
    explicit StereoCameraBasler(QObject *parent = 0) :
                AbstractStereoCamera(parent)
                {}
    bool capture();
    void disconnectCamera();
    bool initCamera(std::string left_camera_name, std::string right_camera_name, int binning = 1);
    bool initCamera(void);
    //bool setExposure(double exposure);
    //bool enableAutoExpose(bool enable);
    std::vector<std::string> listSystems();
    bool autoConnect();
    ~StereoCameraBasler(void);

public slots:
    bool setExposure(double exposure);
    bool enableAutoExpose(bool enable);

private:
    CameraBasler *left_camera;
    CameraBasler *right_camera;

    bool getImageSize(CameraBasler *camera);

    QThread *left_thread;
    QThread *right_thread;
};

#endif //STEREOCAMERABASLER_H
