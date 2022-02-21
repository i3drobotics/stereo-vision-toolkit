/*
* Copyright I3D Robotics Ltd, 2021
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERATITANIA_H
#define STEREOCAMERATITANIA_H

#include <QObject>
#include <abstractstereocamera.h>
#include <stereocamerabasler.h>
#ifdef WITH_VIMBA
    #include <stereocameravimba.h>
#endif

//!  Stereo Titania camera control
/*!
  Control of Titania stereo camera
*/

// TODO replace this with single titania class
class StereoCameraTitaniaBasler : public StereoCameraBasler
{
Q_OBJECT

public:

    explicit StereoCameraTitaniaBasler(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                StereoCameraBasler(serial_info, camera_settings, parent){
        // flip left camera in x and y as mounted upside down
        stereoCameraSettings_.flip_left_x = true;
        stereoCameraSettings_.flip_left_y = true;
        stereoCameraSettings_.flip_right_x = false;
        stereoCameraSettings_.flip_right_y = false;
        // Correct for alternate mounting of cameras on custom welding Titania
        if (serial_info.i3dr_serial == "746974616e24322"){
            stereoCameraSettings_.flip_left_x = false;
            stereoCameraSettings_.flip_left_y = false;
            stereoCameraSettings_.flip_right_x = true;
            stereoCameraSettings_.flip_right_y = true;
        }
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> autoDetectTitanias(Pylon::CTlFactory* tlFactory);
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystemsQuick(Pylon::CTlFactory* tlFactory);

public slots:
    bool captureSingle();
    void captureThreaded();


private:
    bool getCameraFrame(cv::Mat &cam_left_image, cv::Mat &cam_righ_image);

    ~StereoCameraTitaniaBasler(void);
};

#ifdef WITH_VIMBA
// TODO replace this with single titania class
class StereoCameraTitaniaVimba : public StereoCameraVimba
{
Q_OBJECT

public:

    explicit StereoCameraTitaniaVimba(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                StereoCameraVimba(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();

    ~StereoCameraTitaniaVimba(void);
};
#endif

#endif // STEREOCAMERATITANIA_H
