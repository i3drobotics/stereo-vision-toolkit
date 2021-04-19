/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERADEIMOS_H
#define STEREOCAMERADEIMOS_H

#include <abstractstereocamera.h>
#include <stereocameratara.h>

// Correct for math.h warnings
#define NOMINMAX
//#define _USE_MATH_DEFINES
//#define _MATH_DEFINES_DEFINED
#define _WINSOCKAPI_
#include <windows.h>
#include <dshow.h>
#include <hidapi/hidapi.h>
#include <QThread>

//!  Deimos camera control
/*!
  Control of Deimos stereo camera
  https://i3drobotics.com/deimos
*/

class StereoCameraDeimos : public StereoCameraTara
{
Q_OBJECT

public:

    explicit StereoCameraDeimos(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                StereoCameraTara(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();

    ~StereoCameraDeimos(void);
};

#endif // STEREOCAMERADEIMOS_H
