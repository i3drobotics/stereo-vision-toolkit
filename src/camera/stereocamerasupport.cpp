#include "stereocamerasupport.h"

StereoCameraSupport::StereoCameraSupport(){

}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraSupport::getDeviceList(){
    StereoCameraTIS* stereo_cam_tis = new StereoCameraTIS;

    StereoCameraDeimos* stereo_cam_deimos = new StereoCameraDeimos;

    StereoCameraOpenCV* stereo_cam_cv = new StereoCameraOpenCV;

    StereoCameraBasler * stereo_cam_basler = new StereoCameraBasler;

#ifdef WITH_VIMBA
    StereoCameraVimba * stereo_cam_vimba = new StereoCameraVimba;
#endif

    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> all_camera_serial_info;

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> basler_camera_serial_info = stereo_cam_basler->listSystems();

    #ifdef WITH_VIMBA
        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> vimba_camera_serial_info = stereo_cam_vimba->listSystems();
    #endif

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> tis_camera_serial_info = stereo_cam_tis->listSystems();

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> deimos_camera_serial_info = stereo_cam_deimos->listSystems();

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> usb_camera_serial_info = stereo_cam_cv->listSystems();

        all_camera_serial_info.insert( all_camera_serial_info.end(), basler_camera_serial_info.begin(), basler_camera_serial_info.end() );
    #ifdef WITH_VIMBA
        all_camera_serial_info.insert( all_camera_serial_info.end(), vimba_camera_serial_info.begin(), vimba_camera_serial_info.end() );
    #endif
        all_camera_serial_info.insert( all_camera_serial_info.end(), tis_camera_serial_info.begin(), tis_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), deimos_camera_serial_info.begin(), deimos_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), usb_camera_serial_info.begin(), usb_camera_serial_info.end() );

    return all_camera_serial_info;
}

StereoCameraSupport::~StereoCameraSupport(){

}
