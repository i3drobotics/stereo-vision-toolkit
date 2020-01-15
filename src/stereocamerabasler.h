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
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    bool autoConnect();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double exposure);
    void toggleAutoGain(bool enable);
    void adjustGain(int gain);
    void adjustBinning(int gain);
    ~StereoCameraBasler(void);

public slots:
    bool setExposure(double exposure);
    bool setGain(int gain);
    void setBinning(int val);
    void changeFPS(int fps);
    void enableTrigger(bool enable);
    bool enableAutoExpose(bool enable);
    bool enableAutoGain(bool enable);

private:
    CameraBasler *left_camera;
    CameraBasler *right_camera;

    bool getImageSize(CameraBasler *camera);

    QThread *left_thread;
    QThread *right_thread;
};

#endif //STEREOCAMERABASLER_H
