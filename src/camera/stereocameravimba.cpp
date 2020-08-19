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

    return connected;
}

bool StereoCameraVimba::closeCamera(){
    camera_left->close();
    camera_right->close();

    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraVimba::setupCameras(AbstractStereoCamera::StereoCameraSerialInfo CSI_cam_info,int iBinning, bool trigger, int iFps){
    std::string camera_left_serial = CSI_cam_info.left_camera_serial;
    std::string camera_right_serial = CSI_cam_info.right_camera_serial;

    camera_left = new CameraVimba();
    camera_right = new CameraVimba();

    QThread* left_camera_thread = new QThread();
    camera_left->assignThread(left_camera_thread);

    QThread* right_camera_thread = new QThread();
    camera_right->assignThread(right_camera_thread);

    bool success = camera_left->initCamera(camera_left_serial, iBinning, trigger , iFps);
    success &= camera_right->initCamera(camera_right_serial, iBinning, trigger, iFps);

    if(success){
        int height, width, bitdepth;
        camera_left->getImageSize(height, width, bitdepth);
        emit update_size(width, height, bitdepth);
    }

    connect(this, SIGNAL(start_capture()), camera_left, SLOT(startCapture()));
    connect(this, SIGNAL(stop_capture()), camera_right, SLOT(startCapture()));
    connect(this, SIGNAL(start_capture()), camera_left, SLOT(stopCapture()));
    connect(this, SIGNAL(stop_capture()), camera_right, SLOT(stopCapture()));

    return success;
}

bool StereoCameraVimba::captureSingle(){

    camera_left->stopCapture();
    camera_right->stopCapture();

    auto future_left = QtConcurrent::run(camera_left, &CameraVimba::capture);
    auto future_right = QtConcurrent::run(camera_right, &CameraVimba::capture);

    future_left.waitForFinished();
    future_right.waitForFinished();

    bool res = false;

    if(future_left.result() && future_right.result()){
        camera_left->getImage(left_raw);
        camera_right->getImage(right_raw);
        emit captured_success();
        res = true;
    }else{
        emit captured_fail();
        qDebug() << "Capture failed";
        res = false;
    }

    emit captured();
    return res;
}

void StereoCameraVimba::leftGrabFailed(){
    grab_finish_l = true;
    grab_success_l = false;
    checkStereoCapture();
}

void StereoCameraVimba::rightGrabFailed(){
    grab_finish_r = true;
    grab_success_r = false;
    checkStereoCapture();
}

void StereoCameraVimba::leftCaptured(){
    grab_finish_l = true;
    grab_success_l = true;
    camera_left->getImage(left_raw);
    checkStereoCapture();
}

void StereoCameraVimba::rightCaptured(){
    grab_finish_r = true;
    grab_success_r = true;
    camera_right->getImage(right_raw);
    checkStereoCapture();
}

void StereoCameraVimba::checkStereoCapture(){
    if (grab_finish_l && grab_finish_r){
        if (grab_success_r && grab_success_l){
            emit captured_success();
        } else {
            emit captured_fail();
            send_error(CAPTURE_ERROR);
        }
        grab_finish_r = false;
        grab_finish_l = false;
        grab_success_r = false;
        grab_success_l = false;

        emit captured();
    }
}

bool StereoCameraVimba::enableCapture(bool enable){
    qDebug() << "Enabling capture";

    if (enable){
        // Connect callbacks
        connect(camera_left, SIGNAL(captured()), this, SLOT(leftCaptured()));
        connect(camera_right, SIGNAL(captured()), this, SLOT(rightCaptured()));
        camera_left->startCapture();
        camera_right->startCapture();
        capturing = true;
    } else {
        // Disconnect callbacks
        disconnect(camera_left, SIGNAL(captured()), this, SLOT(leftCaptured()));
        disconnect(camera_right, SIGNAL(captured()), this, SLOT(rightCaptured()));
        camera_left->stopCapture();
        camera_right->stopCapture();
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

        camera_left->changeFPS(fps);
        camera_right->changeFPS(fps);

        //TODO this frame rate should be set by the camera's internal calculation
        frame_rate = fps;
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
    // Cameras will clean up themselves;
    return;
}
