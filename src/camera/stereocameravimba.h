/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/


#ifndef STEREOCAMERAVIMBA_H
#define STEREOCAMERAVIMBA_H

#include <opencv2/opencv.hpp>
#include <abstractstereocamera.h>
#include <VimbaCPP/include/VimbaCPP.h>

#include "ApiController.h"

//!  Stereo vimba cameras
/*!
  Control of stereo pair of vimba cameras and generation of 3D
*/

class StereoCameraVimba : public AbstractStereoCamera
{
Q_OBJECT

public:
    explicit StereoCameraVimba(QObject *parent = 0) :
                AbstractStereoCamera(parent),
                apiController(AVT::VmbAPI::Examples::ApiController())
                {
        apiControllerStatus = apiController.StartUp();
    }
    bool capture();
    void disconnectCamera();
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double val);
    void toggleAutoGain(bool enable);
    void adjustGain(int val);
    void adjustBinning(int val);
    void toggleTrigger(bool enable);
    void adjustFPS(int val);
    void adjustPacketSize(int val){} //NA

    ~StereoCameraVimba(void);

public slots:
    bool setExposure(double val);
    bool setGain(int val);
    void changeFPS(int val);
    void changeBinning(int val);
    void enableTrigger(bool enable);
    bool enableAutoExpose(bool enable);
    bool enableAutoGain(bool enable);

private:
    AVT::VmbAPI::Examples::ApiController apiController;
    AVT::VmbAPI::CameraPtr camera_l, camera_r;

    int m_binning;
    int m_iTrigger;
    int m_fps;

    VmbErrorType apiControllerStatus;

    QFuture<bool> qfuture_capture;

    void setBinning(int val);
    void setFPS(int val);
    void enableFPS(bool enable);
    void setTrigger(bool enable);

    bool grab();

    bool setupCameras(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,int iBinning, int iTrigger, int iFps);

    void getImageSize(int &width, int &height, cv::Size &size);
};

#endif //STEREOCAMERAVIMBA_H
