/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/


#ifndef STEREOCAMERABASLER_H
#define STEREOCAMERABASLER_H

#include <opencv2/opencv.hpp>
#include <abstractstereocamera.h>
#include <pylon/PylonIncludes.h>

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
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double val);
    void toggleAutoGain(bool enable);
    void adjustGain(int val);
    void adjustBinning(int val);
    void adjustPacketSize(int val);
    void toggleTrigger(bool enable);
    void adjustFPS(int val);

    ~StereoCameraBasler(void);

public slots:
    bool setExposure(double val);
    bool setGain(int val);
    void setPacketDelay(int val);
    void changeFPS(int val);
    void changeBinning(int val);
    void changePacketSize(int val);
    void enableTrigger(bool enable);
    bool enableAutoExpose(bool enable);
    bool enableAutoGain(bool enable);

private:

    Pylon::CInstantCameraArray *cameras;
    Pylon::CImageFormatConverter *formatConverter;
    int m_binning;
    int m_iTrigger;
    int m_fps;
    int m_packet_size;

    //bool grab_success = true;

    QFuture<bool> qfuture_capture;

    void setBinning(int val);
    void setPacketSize(int val);
    void setFPS(int val);
    void enableFPS(bool enable);
    void setTrigger(bool enable);

    bool grab();

    bool setupCameras(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,int iBinning, int iTrigger, int iFps, int iPacketSize);

    void getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, cv::Size &size);
};

#endif //STEREOCAMERABASLER_H
