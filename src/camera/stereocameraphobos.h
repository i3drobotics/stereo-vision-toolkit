/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERAPHOBOS_H
#define STEREOCAMERAPHOBOS_H

#include <QObject>
#include <stereocamerabasler.h>
#include <stereocameratis.h>

//!  Stereo Phobos camera control
/*!
  Control of Phobos stereo camera
*/

// TODO replace this with single phobos class
class StereoCameraPhobosBasler : public StereoCameraBasler
{
Q_OBJECT

public:

    explicit StereoCameraPhobosBasler(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                StereoCameraBasler(serial_info, camera_settings, parent){
        // flip cameras in x and y as mounted upside down
        stereoCameraSettings_.flip_left_x = true;
        stereoCameraSettings_.flip_left_y = true;
        stereoCameraSettings_.flip_right_x = true;
        stereoCameraSettings_.flip_right_y = true;
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystemsQuick(Pylon::CTlFactory* tlFactory);

    ~StereoCameraPhobosBasler(void);
};

// TODO replace this with single phobos class
class StereoCameraPhobosTIS : public StereoCameraTIS
{
Q_OBJECT

public:

    explicit StereoCameraPhobosTIS(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                StereoCameraTIS(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystemsQuick(DShowLib::Grabber* handle);

    ~StereoCameraPhobosTIS(void);
};

#endif // STEREOCAMERAPHOBOS_H
