/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERABASLER_H
#define STEREOCAMERABASLER_H

#include <abstractstereocamera.h>
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include "arduinocommscameracontrol.h"
#include "pylonsupport.h"

//!  Stereo balser cameras
/*!
  Control of stereo pair of basler cameras and generation of 3D
*/

class StereoCameraBasler : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraBasler(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
        Pylon::PylonInitialize();
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();

    ~StereoCameraBasler(void);

public slots:
    // Implimentations of virtual functions from parent class
    bool openCamera();
    bool closeCamera();
    bool captureSingle();
    bool enableCapture(bool enable);
    bool setFPS(int fps);
    bool setExposure(double exposure);
    bool enableAutoExposure(bool enable);
    bool setPacketSize(int);
    bool setPacketDelay(int);
    bool enableTrigger(bool);
    bool enableAutoGain(bool);
    bool setGain(int);
    bool setBinning(int);

    void captureThreaded();
    bool enableFPS(bool enable);

private:
    QFuture<void> future;

    ArduinoCommsCameraControl* camControl;

    bool hardware_triggered = false;

    Pylon::CInstantCameraArray *cameras;
    Pylon::CImageFormatConverter *formatConverter;

    bool grab();
    void getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, int &bitdepth);
};

#endif //STEREOCAMERABASLER_H

