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

//Example of an image event handler.
class CPylonImageEventHandler : public QObject, public Pylon::CImageEventHandler
{
    Q_OBJECT
public:

    CPylonImageEventHandler(){
        formatConverter = new Pylon::CImageFormatConverter();
        formatConverter->OutputPixelFormat = Pylon::PixelType_Mono8;
        formatConverter->OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
    }

    virtual void OnImageGrabbed( Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& ptrGrabResult)
    {
        bool res = PylonSupport::grabImage2mat(ptrGrabResult,formatConverter,image);
        emit image_grabbed(res);
    }

    cv::Mat getImage(void){
        return image;
    }

signals:
    void image_grabbed(bool sucess);

private:
    cv::Mat image;
    Pylon::CImageFormatConverter *formatConverter;
};

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

    bool enableFPS(bool enable);

private slots:
    void leftCaptured(bool success);
    void rightCaptured(bool success);

private:
    QFuture<void> future;

    ArduinoCommsCameraControl* camControl;

    CPylonImageEventHandler* leftImageEventHandler;
    CPylonImageEventHandler* rightImageEventHandler;

    bool grab_event_l = false;
    bool grab_event_r = false;
    bool capture_one = false;

    Pylon::CInstantCameraArray *cameras;
    Pylon::CImageFormatConverter *formatConverter;

    bool grab();
    void getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, int &bitdepth);
};

#endif //STEREOCAMERABASLER_H

