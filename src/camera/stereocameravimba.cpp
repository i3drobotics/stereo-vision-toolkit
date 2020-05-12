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

    //TODO setup cameras

    //connect(this, SIGNAL(acquired()), this, SLOT(capture()));
    return false;
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraVimba::listSystems(void){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;
    //TODO get list of connected stereo vimba cameras
    /*
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_USB);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    
    // find basler systems connected
    // Initialise Basler Pylon
    Pylon::PylonInitialize();
    // Create an instant camera object with the camera device found first.
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);

    Pylon::CInstantCameraArray all_cameras(devices.size());
    Pylon::CDeviceInfo info;

    std::string DEVICE_CLASS_GIGE = "BaslerGigE";
    std::string DEVICE_CLASS_USB = "BaslerUsb";

    std::vector<std::string> connected_camera_names;
    std::vector<std::string> connected_serials;
    //TODO add generic way to recognise i3dr cameras whilst still being
    //able to make sure the correct right and left cameras are selected together
    for (size_t i = 0; i < all_cameras.GetSize(); ++i)
    {
        all_cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
        std::string device_class = std::string(all_cameras[i].GetDeviceInfo().GetDeviceClass());
        std::string device_name = std::string(all_cameras[i].GetDeviceInfo().GetUserDefinedName());
        std::string device_serial = std::string(all_cameras[i].GetDeviceInfo().GetSerialNumber()); //TODO needs testing with usb
        if (device_name.find("i3dr") != std::string::npos) {
            if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
                connected_serials.push_back(device_serial);
                connected_camera_names.push_back(device_name);
            } else {
                qDebug() << "Unsupported basler class: " << device_class.c_str();
            }
        } else {
            qDebug() << "Unsupported basler camera with name: " << device_name.c_str();
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
    */

    return connected_serial_infos;
}

void StereoCameraVimba::getImageSize(int &width, int &height, cv::Size &size)
{
    //TODO get image size
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
    //TODO set hardware trigger
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
            //TODO get image from cameras

            //image_left_temp.copyTo(left_raw);
            //image_right_temp.copyTo(right_raw);
                                
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
        //TODO close vimba camera
    }
    connected = false;
    emit finished();
    emit disconnected();
}

StereoCameraVimba::~StereoCameraVimba() {
    disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
    disconnectCamera();
}
