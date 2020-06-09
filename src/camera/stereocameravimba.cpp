/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "stereocameravimba.h"

//see: https://www.alliedvision.com/en/products/software/vimba-details.html

bool StereoCameraVimba::initCamera(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings) {

    int binning = inital_camera_settings.binning;
    bool trigger;
    if (inital_camera_settings.trigger == 1){
        trigger = true;
    } else {
        trigger = false;
    }
    int fps = inital_camera_settings.fps;
    double exposure = inital_camera_settings.exposure;
    int gain = inital_camera_settings.gain;
    bool autoExpose;
    if (inital_camera_settings.autoExpose == 1){
        autoExpose = true;
    } else {
        autoExpose = false;
    }
    bool autoGain;
    if (inital_camera_settings.autoGain == 1){
        autoGain = true;
    } else {
        autoGain = false;
    }

    bool res = setupCameras(CSI_cam_info,binning,trigger,fps);

    enableAutoGain(autoGain);
    enableAutoExpose(autoExpose);
    setExposure(exposure);
    setGain(gain);

    this->connected = true;

    return res;
}

bool StereoCameraVimba::setupCameras(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,int iBinning, int iTrigger, int iFps){
    //this->connected = false;
    //disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));

    this->camera_serial_info = CSI_cam_info;
    this->m_binning = iBinning;
    this->m_iTrigger = iTrigger;
    this->m_fps = iFps;

    // find vimba systems connected
    AVT::VmbAPI::CameraPtrVector all_cameras = apiController.GetCameraList();
    std::string camera_left_serial = camera_serial_info.left_camera_serial;
    std::string camera_right_serial = camera_serial_info.right_camera_serial;
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
    //VmbErrorType err_l = apiController.OpenCamera(cameraID_l);
    //VmbErrorType err_r = apiController.OpenCamera(cameraID_r);
    VmbErrorType err_l = camera_l->Open(VmbAccessModeFull);
    VmbErrorType err_r = camera_r->Open(VmbAccessModeFull);

    if (err_l == VmbErrorSuccess && err_r == VmbErrorSuccess){
        getImageSize(image_width,image_height,image_size);
        emit update_size(image_width, image_height, 1);

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
            setTrigger(trigger);
        }

        if (iFps > 0){
            changeFPS(iFps);
        }
        return true;
    } else {
        qDebug() << "Failed to open camera";
        return false;
    }
    //connect(this, SIGNAL(acquired()), this, SLOT(capture()));
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraVimba::listSystems(void){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;

    if (apiControllerStatus != VmbErrorSuccess)
        return connected_serial_infos;

    //get list of known vimba camera serials
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos = loadSerials(AbstractStereoCamera::CAMERA_TYPE_VIMBA);
    
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

void StereoCameraVimba::getImageSize(int &width, int &height, cv::Size &size)
{
    //get image size
    VmbInt64_t h, w;
    VmbErrorType err_h = apiController.GetFeatureIntValue(camera_l,"Height",h);
    VmbErrorType err_w = apiController.GetFeatureIntValue(camera_l,"Width",w);
    if (err_h == VmbErrorSuccess && err_w == VmbErrorSuccess){
        height = (int) h;
        width = (int) w;
        size = cv::Size(width,height);
    } else {
        qDebug() << "Failed to get width / height";
    }
}

void StereoCameraVimba::enableTrigger(bool enable){
    setTrigger(enable);
}

void StereoCameraVimba::changeBinning(int val){
    qDebug() << "Setting binning";
    //TODO close camera before setting binning
    setupCameras(this->camera_serial_info,val,this->m_iTrigger,this->m_fps);
    qDebug() << "Binning updated";
}

void StereoCameraVimba::setFPS(int val){
    //TODO set fps
    AVT::VmbAPI::CameraPtr pCamera;
    //apiController.GetCamera(cameraID_l,pCamera);
    //pCamera->GetFeatureByName("Exposure",)
}

void StereoCameraVimba::enableFPS(bool enable){
    //TODO toggle fps control (hardware trigger off)
}

void StereoCameraVimba::changeFPS(int val){
    // unlimited frame rate if set to 0 (TODO check this is true for vimba)
    if (val>0){
        setFPS(val);
        enableFPS(true);
    } else {
        enableFPS(false);
    }
}

bool StereoCameraVimba::setExposure(double val) {
    //TODO set exposure
    return false;
}

void StereoCameraVimba::setBinning(int val){
    //TODO set binning
}

void StereoCameraVimba::setTrigger(bool enable){
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
}

void StereoCameraVimba::toggleTrigger(bool enable){
    enableTrigger(enable);
}
void StereoCameraVimba::adjustFPS(int val){
    changeFPS(val);
}

void StereoCameraVimba::toggleAutoExpose(bool enable){
    enableAutoExpose(enable);
}

void StereoCameraVimba::adjustExposure(double val){
    setExposure(val);
}

void StereoCameraVimba::toggleAutoGain(bool enable){
    enableAutoGain(enable);
}

void StereoCameraVimba::adjustGain(int val){
    setGain(val);
}

bool StereoCameraVimba::setGain(int val) {
    //TODO set gain
    return false;
}

bool StereoCameraVimba::enableAutoGain(bool enable){
    //TODO toggle auto gain
    return false;
}

bool StereoCameraVimba::enableAutoExpose(bool enable){
    //TODO toggle auto exposure
    return false;
}

void StereoCameraVimba::adjustBinning(int val){
    changeBinning(val);
}

bool StereoCameraVimba::grab(){
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
                    }
                } else {
                    qDebug() << "Failed to acquire frame or incomplete";
                }
            } else {
                qDebug() << "Failed to acquire image";
            }
        } else {
            qDebug() << "Camera is not connected or is initalising";
            std::cerr << "Camera is not connected or is initalising" << std::endl;
            res = false;
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
        this->connected = false;
        //TODO disconnect and reconnect camera
        this->connected = true;
    }

    return res;
}

bool StereoCameraVimba::capture() {
    qfuture_capture = QtConcurrent::run(this, &StereoCameraVimba::grab);
    //qfutureWatcher_capture.setFuture(qfuture_capture);

    qfuture_capture.waitForFinished();
    emit right_captured();
    emit left_captured();

    return true;
}

void StereoCameraVimba::disconnectCamera() {
    if (connected){
        //close vimba camera
        VmbErrorType err_l = camera_l->Close();
        VmbErrorType err_r = camera_r->Close();
    }
    connected = false;
    emit disconnected();
    //emit finished();
}

StereoCameraVimba::~StereoCameraVimba() {
    disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
    disconnectCamera();
    apiController.ShutDown();
}
