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

// Image event handler
class CStereoCameraBaslerImageEventHandler : public Pylon::CImageEventHandler
{
public:
    CStereoCameraBaslerImageEventHandler(std::string left_serial, std::string right_serial, Pylon::CImageFormatConverter *formatConverter);
    void OnImageGrabbed( Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& ptrGrabResult);
    bool getImagePair(cv::Mat &left, cv::Mat &right, int timeout);

private:
    enum captureSide {CAPTURE_LEFT, CAPTURE_RIGHT};
    std::string left_serial;
    std::string right_serial;
    Pylon::CImageFormatConverter *formatConverter;
    cv::Mat live_left_image;
    cv::Mat live_right_image;
    cv::Mat left_image;
    cv::Mat right_image;
    int timestamp_left = 0;
    int timestamp_right = 0;
    std::mutex mtx;
    captureSide capSide = captureSide::CAPTURE_LEFT;
    bool newFrames = false;
};

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
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystemsQuick(Pylon::CTlFactory* tlFactory);

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
    ArduinoCommsCameraControl* camControl;
    CStereoCameraBaslerImageEventHandler* imageEventHandler;

    bool hardware_triggered = false;
    int lineStatusTimerId;
    int lineStatusTimerDelay = 1000;

    cv::Mat tmp_left_img;
    cv::Mat tmp_right_img;

    Pylon::CInstantCameraArray *cameras;
    Pylon::CImageFormatConverter *formatConverter;

    bool grab();
    void getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, int &bitdepth);
    void enableDeviceLinkThroughputLimit(bool enable);
    void setDeviceLinkThroughput(int value);
    void getLineStatus(int line, bool &status_l, bool &status_r);

protected:
    QFuture<void> future;

    bool getCameraFrame(cv::Mat &cam_left_image, cv::Mat &cam_righ_image);
    void timerEvent(QTimerEvent *event);
};

#endif //STEREOCAMERABASLER_H

