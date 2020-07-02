/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERASUPPORT_H
#define STEREOCAMERASUPPORT_H

#include <QObject>
#include "stereocameradeimos.h"
#include "stereocamerafromvideo.h"
#include "stereocameraopencv.h"
#include "stereocameratis.h"
#include "stereocamerabasler.h"
#ifdef WITH_VIMBA
    #include "stereocameravimba.h"
#endif

//!  Stereo camera support
/*!
  Support class for using stereo cameras.
*/
class StereoCameraSupport
{

public:
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> getStereoDeviceList(DShowLib::Grabber* handle){

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> all_camera_serial_info;

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> basler_camera_serial_info = StereoCameraBasler::listSystems();

#ifdef WITH_VIMBA
        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> vimba_camera_serial_info = StereoCameraVimba::listSystems();
#endif

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> tis_camera_serial_info = StereoCameraTIS::listSystemsQuick(handle);

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> deimos_camera_serial_info = StereoCameraDeimos::listSystems();

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> usb_camera_serial_info = StereoCameraOpenCV::listSystems();

        all_camera_serial_info.insert( all_camera_serial_info.end(), basler_camera_serial_info.begin(), basler_camera_serial_info.end() );
#ifdef WITH_VIMBA
        all_camera_serial_info.insert( all_camera_serial_info.end(), vimba_camera_serial_info.begin(), vimba_camera_serial_info.end() );
#endif
        all_camera_serial_info.insert( all_camera_serial_info.end(), tis_camera_serial_info.begin(), tis_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), deimos_camera_serial_info.begin(), deimos_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), usb_camera_serial_info.begin(), usb_camera_serial_info.end() );

        return all_camera_serial_info;
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> getStereoDeviceList(){

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> all_camera_serial_info;

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> basler_camera_serial_info = StereoCameraBasler::listSystems();

#ifdef WITH_VIMBA
        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> vimba_camera_serial_info = StereoCameraVimba::listSystems();
#endif

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> tis_camera_serial_info = StereoCameraTIS::listSystems();

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> deimos_camera_serial_info = StereoCameraDeimos::listSystems();

        std::vector<AbstractStereoCamera::StereoCameraSerialInfo> usb_camera_serial_info = StereoCameraOpenCV::listSystems();

        all_camera_serial_info.insert( all_camera_serial_info.end(), basler_camera_serial_info.begin(), basler_camera_serial_info.end() );
#ifdef WITH_VIMBA
        all_camera_serial_info.insert( all_camera_serial_info.end(), vimba_camera_serial_info.begin(), vimba_camera_serial_info.end() );
#endif
        all_camera_serial_info.insert( all_camera_serial_info.end(), tis_camera_serial_info.begin(), tis_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), deimos_camera_serial_info.begin(), deimos_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), usb_camera_serial_info.begin(), usb_camera_serial_info.end() );

        return all_camera_serial_info;
    }

};

#endif // STEREOCAMERASUPPORT_H
