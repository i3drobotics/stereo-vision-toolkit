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
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double exposure);
    void toggleAutoGain(bool enable);
    void adjustGain(int gain);
    void adjustBinning(int gain);
    void toggleTrigger(bool enable);
    void adjustFPS(int fps);
    void adjustPacketSize(int);

    QFuture<bool> qfuture_left;
    QFuture<bool> qfuture_right;

    QFutureWatcher<bool> qfutureWatcher_left;
    QFutureWatcher<bool> qfutureWatcher_right;

    ~StereoCameraBasler(void);

public slots:
    bool setExposure(double exposure);
    bool setGain(int gain);
    void setBinning(int val);
    void setPacketSize(int val);
    //TODO add packet delay

    void changeFPS(int fps);
    void enableTrigger(bool enable);
    bool enableAutoExpose(bool enable);
    bool enableAutoGain(bool enable);
    void left_finished();
    void right_finished();

private:
    CameraBasler *left_camera;
    CameraBasler *right_camera;

    bool getImageSize(CameraBasler *camera);

    QThread *left_thread;
    QThread *right_thread;
};

#endif //STEREOCAMERABASLER_H
