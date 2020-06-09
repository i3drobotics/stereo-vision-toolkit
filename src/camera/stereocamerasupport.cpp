#include "stereocamerasupport.h"

StereoCameraSupport::StereoCameraSupport(){

}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraSupport::getDeviceList(bool showGUI = true){
    StereoCameraTIS* stereo_cam_tis = new StereoCameraTIS;
    QThread* tis_thread = new QThread;
    stereo_cam_tis->assignThread(tis_thread);

    StereoCameraDeimos* stereo_cam_deimos = new StereoCameraDeimos;
    QThread* deimos_thread = new QThread;
    stereo_cam_deimos->assignThread(deimos_thread);

    StereoCameraOpenCV* stereo_cam_cv = new StereoCameraOpenCV;
    QThread* cv_thread = new QThread;
    stereo_cam_cv->assignThread(cv_thread);

    StereoCameraBasler * stereo_cam_basler = new StereoCameraBasler;
    QThread* basler_thread = new QThread;
    stereo_cam_basler->assignThread(basler_thread);

#ifdef WITH_VIMBA
    StereoCameraVimba * stereo_cam_vimba = new StereoCameraVimba;
    QThread* vimba_thread = new QThread;
    stereo_cam_vimba->assignThread(vimba_thread);
#endif

    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> all_camera_serial_info;

    if (showGUI){ //TODO enable gui support (needs to be passed widget to run from)
        /*
        QProgressDialog progressSearch("Finding connected devices...", "", 0, 100, this);
        progressSearch.setWindowTitle("SVT");
        progressSearch.setWindowModality(Qt::WindowModal);
        progressSearch.setCancelButton(nullptr);
        progressSearch.setMinimumDuration(0);
        progressSearch.setValue(1);

        progressSearch.setValue(10);
        progressSearch.setLabelText("Searching for basler cameras...");

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> basler_camera_serial_info = stereo_cam_basler->listSystems();
        progressSearch.setValue(30);

    #ifdef WITH_VIMBA
        progressSearch.setLabelText("Searching for vimba cameras...");
        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> vimba_camera_serial_info = stereo_cam_vimba->listSystems();
    #endif
        progressSearch.setValue(40);
        progressSearch.setLabelText("Searching for imaging source cameras...");

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> tis_camera_serial_info = stereo_cam_tis->listSystems();
        progressSearch.setLabelText("Searching for deimos cameras...");
        progressSearch.setValue(50);

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> deimos_camera_serial_info = stereo_cam_deimos->listSystems();
        progressSearch.setLabelText("Searching for usb cameras...");
        progressSearch.setValue(60);

        std::vector<AbstractStereoCamera::stereoCameraSerialInfo> usb_camera_serial_info = stereo_cam_cv->listSystems();
        progressSearch.setLabelText("Processing found devices...");
        progressSearch.setValue(80);

        all_camera_serial_info.insert( all_camera_serial_info.end(), basler_camera_serial_info.begin(), basler_camera_serial_info.end() );
    #ifdef WITH_VIMBA
        all_camera_serial_info.insert( all_camera_serial_info.end(), vimba_camera_serial_info.begin(), vimba_camera_serial_info.end() );
    #endif
        all_camera_serial_info.insert( all_camera_serial_info.end(), tis_camera_serial_info.begin(), tis_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), deimos_camera_serial_info.begin(), deimos_camera_serial_info.end() );
        all_camera_serial_info.insert( all_camera_serial_info.end(), usb_camera_serial_info.begin(), usb_camera_serial_info.end() );

        progressSearch.setLabelText("Process complete");
        progressSearch.setValue(100);
        progressSearch.close();
        QCoreApplication::processEvents();
        */
    } else {

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
    }

    return all_camera_serial_info;
}

StereoCameraSupport::~StereoCameraSupport(){

}
