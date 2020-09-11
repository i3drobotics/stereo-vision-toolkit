/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
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

    this->connected = setupCameras(stereoCameraSerialInfo_,binning,trigger,fps);
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
        camera_left->close();
        camera_right->close();
    }
    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraVimba::setupCameras(AbstractStereoCamera::StereoCameraSerialInfo CSI_cam_info,int iBinning, bool trigger, int iFps){
    std::string camera_left_serial = CSI_cam_info.left_camera_serial;
    std::string camera_right_serial = CSI_cam_info.right_camera_serial;

    camera_left = new CameraVimba();
    camera_right = new CameraVimba();

    bool error = camera_left->initCamera(camera_left_serial, iBinning, trigger , iFps);
    error &= camera_right->initCamera(camera_right_serial, iBinning, trigger, iFps);

    if(!error){
        int height, width, bitdepth;
        camera_left->getImageSize(height, width, bitdepth);
        emit update_size(width, height, bitdepth);
    }

    return error;
}

bool StereoCameraVimba::captureSingle(){
    bool left_res = camera_left->capture();
    bool right_res = camera_right->capture();
    bool res = false;

    if(left_res && right_res){
        camera_left->getImage()->copyTo(left_raw);
        camera_right->getImage()->copyTo(right_raw);
        emit captured_success();
        res = true;
    }else{
        emit captured_fail();
        res = false;
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

    using namespace AVT::VmbAPI;

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_VIMBA);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    // NB you need to explicitly state the left hand type, otherwise
    // you'll get weird private constructor errors - don't use auto

    VimbaSystem& system = VimbaSystem::GetInstance();
    CameraPtrVector all_cameras;

    auto err = system.GetCameras( all_cameras );            // Fetch all cameras known to Vimba
    if( err != VmbErrorSuccess ){
        qDebug() << "Could not list cameras. Error code: " << err << "\n";
        return connected_serial_infos;
    }

    qDebug() << "Cameras found: " << all_cameras.size() <<"\n\n";

    if(all_cameras.size() == 0){
        return connected_serial_infos;
    }

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
    return camera_left->enableAutoExposure(enable) &&
           camera_right->enableAutoExposure(enable);
}

bool StereoCameraVimba::setGain(int gain) {
    return camera_left->setGain(gain) &&
           camera_right->setGain(gain);
}

bool StereoCameraVimba::setBinning(int val){
    return camera_left->setBinning(val) &&
           camera_right->setBinning(val);
}

bool StereoCameraVimba::setExposure(double exposure) {
    return camera_left->setExposure(exposure) &&
           camera_right->setExposure(exposure);
}

bool StereoCameraVimba::enableAutoGain(bool enable){
    return camera_left->enableAutoGain(enable) &&
           camera_right->enableAutoGain(enable);
}

bool StereoCameraVimba::setFPS(int fps){
    if (!isCapturing()){

        camera_left->setFPS(fps);
        camera_right->setFPS(fps);

        //TODO this frame rate should be set by the camera's internal calculation
        frame_rate = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

bool StereoCameraVimba::enableTrigger(bool enable){
    return camera_left->enableTrigger(enable) &&
           camera_right->enableTrigger(enable);
}

StereoCameraVimba::~StereoCameraVimba(void) {
    camera_left->close();
    camera_right->close();
}
