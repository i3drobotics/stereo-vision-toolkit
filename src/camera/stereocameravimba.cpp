/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocameravimba.h"

bool StereoCameraVimba::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    int binning = stereoCameraSettings_.binning;
    bool trigger;
    if (stereoCameraSettings_.trigger == 1){
        trigger = true;
    } else {
        trigger = false;
    }
    int fps = stereoCameraSettings_.fps;
    double exposure = stereoCameraSettings_.exposure;
    int gain = stereoCameraSettings_.gain;
    bool autoExpose;
    if (stereoCameraSettings_.autoExpose == 1){
        autoExpose = true;
    } else {
        autoExpose = false;
    }
    bool autoGain;
    if (stereoCameraSettings_.autoGain == 1){
        autoGain = true;
    } else {
        autoGain = false;
    }

    bool connected = setupCameras(stereoCameraSerialInfo_,binning,trigger,fps);

    enableAutoGain(autoGain);
    enableAutoExposure(autoExpose);
    setExposure(exposure);
    setGain(gain);

    return connected;
}

bool StereoCameraVimba::closeCamera(){
    if (connected){
        //close vimba camera
        //disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
        VmbErrorType err_l = camera_l->Close();
        VmbErrorType err_r = camera_r->Close();
    }
    connected = false;
    emit disconnected();
    return true;
}

void StereoCameraVimba::getImageSize(int &width, int &height, int &bitdepth)
{
    //get image size
    VmbInt64_t h, w;
    VmbErrorType err_h = apiController_.GetFeatureIntValue(camera_l,"Height",h);
    VmbErrorType err_w = apiController_.GetFeatureIntValue(camera_l,"Width",w);
    if (err_h == VmbErrorSuccess && err_w == VmbErrorSuccess){
        height = (int) h;
        width = (int) w;
        bitdepth = 1; //TODO get bit depth
    } else {
        qDebug() << "Failed to get width / height";
    }
}

bool StereoCameraVimba::setupCameras(AbstractStereoCamera::StereoCameraSerialInfo CSI_cam_info,int iBinning, int iTrigger, int iFps){
    //this->connected = false;
    //disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));

    // find vimba systems connected
    AVT::VmbAPI::CameraPtrVector all_cameras = apiController_.GetCameraList();
    std::string camera_left_serial = CSI_cam_info.left_camera_serial;
    std::string camera_right_serial = CSI_cam_info.right_camera_serial;
    bool cameraLeftFind = false;
    for (size_t i = 0; i < all_cameras.size(); ++i)
    {
        std::string device_serial;
        all_cameras[i]->GetSerialNumber(device_serial);
        if (device_serial == camera_left_serial)
        {
            //all_cameras[i]->GetID(cameraID_l);
            camera_l = all_cameras[i];
            cameraLeftFind = true;
            break;
        }
    }

    if (!cameraLeftFind){
        std::cerr << "Failed to find left camera with serial: " << camera_left_serial << std::endl;
        return false;
    }

    bool cameraRightFind = false;
    for (size_t i = 0; i < all_cameras.size(); ++i)
    {
        std::string device_serial;
        all_cameras[i]->GetSerialNumber(device_serial);
        if (device_serial == camera_right_serial)
        {
            //all_cameras[i]->GetID(cameraID_r);
            camera_r = all_cameras[i];
            cameraRightFind = true;
            break;
        }
    }

    if (!cameraRightFind){
        std::cerr << "Failed to find right camera with serial: " << camera_right_serial << std::endl;
        return false;
    }

    // open cameras
    //VmbErrorType err_l = apiController_.OpenCamera(cameraID_l);
    //VmbErrorType err_r = apiController_.OpenCamera(cameraID_r);
    VmbErrorType err_l = camera_l->Open(VmbAccessModeFull);
    VmbErrorType err_r = camera_r->Open(VmbAccessModeFull);

    if (err_l == VmbErrorSuccess && err_r == VmbErrorSuccess){
        int width, height, bitdepth;
        getImageSize(width,height,bitdepth);
        emit update_size(width, height, bitdepth);

        if (iBinning > 0){
            setBinning(iBinning);
        }
        if (iTrigger >= 0){
            bool trigger;
            if (iTrigger == 0){
                trigger = false;
            } else {
                trigger = true;
            }
            enableTrigger(trigger);
        }

        if (iFps > 0){
            setFPS(iFps);
        }
        return true;
    } else {
        qDebug() << "Failed to open camera";
        return false;
    }
}

bool StereoCameraVimba::captureSingle(){
    bool res = false;
    try
    {
        if (this->connected){
            // get image from cameras
            AVT::VmbAPI::FramePtr pFrame_l, pFrame_r;
            VmbErrorType captureErr_l, captureErr_r;
            VmbFrameStatusType status_l, status_r = VmbFrameStatusIncomplete;
            captureErr_l = camera_l->AcquireSingleImage( pFrame_l, 5000 );
            captureErr_r = camera_r->AcquireSingleImage( pFrame_r, 5000 );
            if ( captureErr_l == VmbErrorSuccess && captureErr_r == VmbErrorSuccess )
            {
                captureErr_l = pFrame_l->GetReceiveStatus( status_l );
                captureErr_r = pFrame_r->GetReceiveStatus( status_r );
                if (     captureErr_l == VmbErrorSuccess
                     && status_l == VmbFrameStatusComplete &&
                         captureErr_r == VmbErrorSuccess
                        && status_r == VmbFrameStatusComplete)
                {
                    VmbPixelFormatType ePixelFormat_l,ePixelFormat_r = VmbPixelFormatMono8;
                    captureErr_l = pFrame_l->GetPixelFormat( ePixelFormat_l );
                    captureErr_r = pFrame_r->GetPixelFormat( ePixelFormat_r );
                    if ( captureErr_l == VmbErrorSuccess)
                    {
                        if (    ( ePixelFormat_l != VmbPixelFormatMono8 )
                                &&  ( ePixelFormat_r != VmbPixelFormatMono8 ))
                        {
                            captureErr_l = VmbErrorInvalidValue;
                            captureErr_r = VmbErrorInvalidValue;
                            qDebug() << "Invalid pixel format";
                            emit error(CAPTURE_ERROR);
                        }
                        else
                        {
                            VmbUint32_t nImageSize = 0;
                            captureErr_l = pFrame_l->GetImageSize( nImageSize );
                            if ( captureErr_l == VmbErrorSuccess )
                            {
                                VmbUint32_t nWidth = 0;
                                captureErr_l = pFrame_l->GetWidth( nWidth );
                                if ( captureErr_l == VmbErrorSuccess )
                                {
                                    VmbUint32_t nHeight = 0;
                                    captureErr_l = pFrame_l->GetHeight( nHeight );
                                    if ( captureErr_l == VmbErrorSuccess )
                                    {
                                        VmbUchar_t *pImage_l,*pImage_r  = NULL;
                                        captureErr_l = pFrame_l->GetImage( pImage_l );
                                        captureErr_r = pFrame_r->GetImage( pImage_r );
                                        if ( captureErr_l == VmbErrorSuccess
                                             &&  captureErr_r == VmbErrorSuccess )
                                        {
                                            cv::Mat image_left_temp = cv::Mat(nHeight, nWidth, CV_8UC1, pImage_l );
                                            cv::Mat image_right_temp = cv::Mat(nHeight, nWidth, CV_8UC1, pImage_r );
                                            image_left_temp.copyTo(left_raw);
                                            image_right_temp.copyTo(right_raw);
                                            res = true;
                                        }
                                    }
                                }
                            }

                        }
                    } else {
                        qDebug() << "Failed to get pixel format";
                        emit error(CAPTURE_ERROR);
                    }
                } else {
                    qDebug() << "Failed to acquire frame or incomplete";
                    emit error(CAPTURE_ERROR);
                }
            } else {
                qDebug() << "Failed to acquire image";
                emit error(CAPTURE_ERROR);
            }
        } else {
            qDebug() << "Camera is not connected or is initalising";
            std::cerr << "Camera is not connected or is initalising" << std::endl;
            res = false;
            emit error(CAPTURE_ERROR);
        }
    }
    catch (...)
    {
        // Error handling.
        //qDebug() << "An exception occurred." << e.GetDescription();
        //std::cerr << "An exception occurred." << std::endl
        //          << e.GetDescription() << std::endl;
        qDebug() << "Failed to grab camera";
        res = false;
        emit error(CAPTURE_ERROR);
    }
    emit captured();
    return res;
}

void StereoCameraVimba::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraVimba::captureSingle);
}

bool StereoCameraVimba::enableCapture(bool enable){
    if (enable){
        //Start capture thread
        connect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = true;
        captureThreaded();
    } else {
        //Stop capture thread
        disconnect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = false;
    }
    return true;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraVimba::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_VIMBA);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    AVT::VmbAPI::Examples::ApiController apiController = AVT::VmbAPI::Examples::ApiController();
    //VmbErrorType apiControllerStatus = apiController.StartUp();

    // find vimba systems connected
    AVT::VmbAPI::CameraPtrVector all_cameras = apiController.GetCameraList();

    std::vector<std::string> connected_camera_names;
    std::vector<std::string> connected_serials;
    //TODO add generic way to recognise i3dr cameras whilst still being
    //able to make sure the correct right and left cameras are selected together
    for (size_t i = 0; i < all_cameras.size(); ++i)
    {
        std::string device_serial;
        VmbInterfaceType interface_type;
        all_cameras[i]->GetSerialNumber(device_serial);
        //TODO get device_name
        all_cameras[i]->GetInterfaceType(interface_type);
        if (interface_type == VmbInterfaceUsb){
            connected_serials.push_back(device_serial);
            //connected_camera_names.push_back(device_name);
        } else {
            qDebug() << "Unsupported vimba class: " << interface_type;
        }
    }

    for (auto& known_serial_info : known_serial_infos) {
        bool left_found = false;
        bool right_found = false;
        std::string left_serial;
        std::string right_serial;
        //find left
        for (auto& connected_serial : connected_serials)
        {
            left_serial = connected_serial;
            if (left_serial == known_serial_info.left_camera_serial){
                left_found = true;
                break;
            }
        }
        if (left_found){
            //find right
            for (auto& connected_serial : connected_serials)
            {
                right_serial = connected_serial;
                if (right_serial == known_serial_info.right_camera_serial){
                    right_found = true;
                    break;
                }
            }
        }
        if (left_found && right_found){ //only add if both cameras found
            connected_serial_infos.push_back(known_serial_info);
        }
    }

    return connected_serial_infos;
}

bool StereoCameraVimba::enableAutoExposure(bool enable) {
    //TODO set auto exposure
    return false;
}

bool StereoCameraVimba::setGain(int val) {
    //TODO set gain
    return false;
}

bool StereoCameraVimba::setBinning(int val){
    //TODO set binning
    return false;
}

bool StereoCameraVimba::setExposure(double exposure) {
    //TODO set exposure
    return false;
}

bool StereoCameraVimba::enableAutoGain(bool enable){
    //TODO toggle auto gain
    return false;
}

bool StereoCameraVimba::setFPS(int fps){
    if (!isCapturing()){
        AVT::VmbAPI::CameraPtr pCamera;
        //apiController_.GetCamera(cameraID_l,pCamera);
        //pCamera->GetFeatureByName("Exposure",)
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

bool StereoCameraVimba::enableTrigger(bool enable){
    //set hardware trigger
    /*
    AVT::VmbAPI::CameraPtr pCamera;
    AVT::VmbAPI::FeaturePtr pFeature;
    apiController.GetCamera(cameraID_l,pCamera);
    VmbErrorType err = pCamera->GetFeatureByName( "TriggerMode", pFeature );
    if (err == VmbErrorSuccess){
        if (enable){
            pFeature->SetValue( "On" );
        } else {
            pFeature->SetValue( "Off" );
        }
    }
    */
    return false;
}

StereoCameraVimba::~StereoCameraVimba(void) {
    if (connected){
        closeCamera();
    }
    apiController_.ShutDown();
}
