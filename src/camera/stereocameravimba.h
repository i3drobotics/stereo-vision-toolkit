/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERAVIMBA_H
#define STEREOCAMERAVIMBA_H

#include <opencv2/opencv.hpp>
#include <abstractstereocamera.h>
#include <cameravimba.h>

#include <VimbaCPP/Include/VimbaCPP.h>

//!  Stereo vimba cameras
/*!
  Control of stereo pair of vimba cameras and generation of 3D
*/

class StereoCameraVimba  : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraVimba (AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
        AbstractStereoCamera(serial_info, camera_settings, parent), sys(VimbaSystem::GetInstance()){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    ~StereoCameraVimba(void);

public slots:
    // Implimentations of virtual functions from parent class
    bool openCamera();
    bool closeCamera();
    bool captureSingle();
    bool enableCapture(bool enable);
    bool setFPS(int fps);
    bool setExposure(double exposure);
    bool enableAutoExposure(bool enable);
    bool setPacketSize(int){return false;} //NA
    bool setPacketDelay(int){return false;} //NA
    bool enableTrigger(bool enable);
    bool enableAutoGain(bool enable);
    bool setGain(int gain);
    bool setBinning(int binning);

private slots:
    void leftGrabFailed(void);
    void rightGrabFailed(void);
    void leftCaptured(void);
    void rightCaptured(void);
    void checkStereoCapture(void);

private:
    QFuture<void> future;

    CameraVimba* camera_left;
    CameraVimba* camera_right;
    VimbaSystem &sys;

    bool grab_success_l = false;
    bool grab_success_r = false;

    bool grab_finish_l = false;
    bool grab_finish_r = false;

    bool setupCameras(AbstractStereoCamera::StereoCameraSerialInfo CSI_cam_info,int iBinning, bool trigger, int iFps);

signals:
    void start_capture();
    void stop_capture();
};

#endif //STEREOCAMERAVIMBA_H
