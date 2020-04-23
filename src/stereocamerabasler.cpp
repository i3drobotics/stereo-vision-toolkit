/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "stereocamerabasler.h"

bool StereoCameraBasler::initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings) {

    this->camera_serial_info = camera_serial_info;
    this->left_camera = new CameraBasler;
    this->right_camera = new CameraBasler;

    int m_binning = inital_camera_settings.binning;
    int m_fps = inital_camera_settings.fps;
    bool m_trigger;
    if (inital_camera_settings.trigger == 1){
        m_trigger = true;
    } else {
        m_trigger = false;
    }

    double exposure = inital_camera_settings.exposure;
    int gain = inital_camera_settings.gain;
    int packetDelay = inital_camera_settings.packetDelay;
    int packetSize = inital_camera_settings.packetSize;
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

    bool res = this->left_camera->initCamera(camera_serial_info.left_camera_serial,m_binning,m_trigger,m_fps,packetSize);
    res &= this->right_camera->initCamera(camera_serial_info.right_camera_serial,m_binning,m_trigger,m_fps,packetSize);

    if (res) {
        this->left_camera->setAutoExposure(autoExpose);
        this->right_camera->setAutoExposure(autoExpose);
        this->left_camera->setExposure(exposure);
        this->right_camera->setExposure(exposure);
        this->left_camera->setAutoGain(autoGain);
        this->right_camera->setAutoGain(autoGain);
        this->left_camera->setGain(gain);
        this->right_camera->setGain(gain);
        this->left_camera->setInterPacketDelay(packetDelay);
        this->right_camera->setInterPacketDelay(0);

        this->left_camera->getImageSize(image_width, image_height, image_size);
        emit update_size(image_width, image_height, 1);
    }else{
        qDebug() << "Failed to setup cameras";
        disconnectCamera();
    }

    connect(&qfutureWatcher_left, SIGNAL(finished()), SLOT(left_finished()));
    connect(&qfutureWatcher_right, SIGNAL(finished()), SLOT(right_finished()));

    this->connected = res;

    return res;
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraBasler::listSystems(void){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_gige = loadSerials("baslergige");
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_usb = loadSerials("baslerusb");
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;
    // find basler systems connected
    // Initialise Basler Pylon
    Pylon::PylonInitialize();
    // Create an instant camera object with the camera device found first.
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);

    Pylon::CInstantCameraArray cameras(devices.size());
    Pylon::CDeviceInfo info;

    std::string DEVICE_CLASS_GIGE = "BaslerGigE";
    std::string DEVICE_CLASS_USB = "BaslerUsb";

    std::vector<std::string> camera_names;
    std::vector<std::string> connected_serials;
    for (size_t i = 0; i < cameras.GetSize(); ++i)
    {
        cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
        std::string device_class = std::string(cameras[i].GetDeviceInfo().GetDeviceClass());
        std::string device_serial = std::string(cameras[i].GetDeviceInfo().GetSerialNumber()); //TODO needs testing with usb
        if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
            connected_serials.push_back(device_serial);
        } else {
            qDebug() << "Unsupported basler class: " << device_class.c_str();
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

void StereoCameraBasler::enableTrigger(bool enable){
    bool res = left_camera->enableTrigger(enable);
    res &= right_camera->enableTrigger(enable);
}

void StereoCameraBasler::changeFPS(int fps){
    bool res = left_camera->changeFPS(fps);
    res &= right_camera->changeFPS(fps);
}

bool StereoCameraBasler::setExposure(double exposure) {
    bool res = left_camera->setExposure(exposure);
    res &= right_camera->setExposure(exposure);

    return res;
}

void StereoCameraBasler::setBinning(int val){
    qDebug() << "Setting binning";
    bool res = left_camera->changeBinning(val);
    res &= right_camera->changeBinning(val);
    this->left_camera->getImageSize(image_width, image_height, image_size);
    emit update_size(image_width, image_height, 1);
    qDebug() << "Binning updated";
}

void StereoCameraBasler::toggleTrigger(bool enable){
    enableTrigger(enable);
}
void StereoCameraBasler::adjustFPS(int fps){
    changeFPS(fps);
}

void StereoCameraBasler::adjustPacketSize(int packetSize){
    setPacketSize(packetSize);
}

void StereoCameraBasler::setPacketSize(int val){
    qDebug() << "Setting packet size";
    bool res = left_camera->changePacketSize(val);
    res &= right_camera->changePacketSize(val);
    qDebug() << "Packet size updated";
}

void StereoCameraBasler::toggleAutoExpose(bool enable){
    enableAutoExpose(enable);
}

void StereoCameraBasler::adjustExposure(double exposure){
    setExposure(exposure);
}

void StereoCameraBasler::toggleAutoGain(bool enable){
    enableAutoGain(enable);
}

void StereoCameraBasler::adjustGain(int gain){
    setGain(gain);
}

bool StereoCameraBasler::setGain(int gain) {
    bool res = left_camera->setGain(gain);
    res &= right_camera->setGain(gain);

    return res;
}

bool StereoCameraBasler::enableAutoGain(bool enable){
    bool res = left_camera->setAutoGain(enable);
    res &= right_camera->setAutoGain(enable);

    return res;
}

bool StereoCameraBasler::enableAutoExpose(bool enable){
    bool res = left_camera->setAutoExposure(enable);
    res &= right_camera->setAutoExposure(enable);

    return res;
}

void StereoCameraBasler::adjustBinning(int binning){
    setBinning(binning);
}

void StereoCameraBasler::left_finished(){
    cv::Mat left;
    if (qfuture_left.result()){
        (left_camera->getImage())->copyTo(left);
    } else {
        qDebug() << "Failed to capture left image";
    }
    left.copyTo(left_raw);
    emit left_captured();
}

void StereoCameraBasler::right_finished(){
    cv::Mat right;
    if (qfuture_right.result()){
        (right_camera->getImage())->copyTo(right);
    } else {
        qDebug() << "Failed to capture rigt image";
    }
    right.copyTo(right_raw);
    emit right_captured();
}

bool StereoCameraBasler::capture() {

    qfuture_left = QtConcurrent::run(left_camera, &CameraBasler::capture);
    qfuture_right = QtConcurrent::run(right_camera, &CameraBasler::capture);

    qfutureWatcher_left.setFuture(qfuture_left);
    qfutureWatcher_right.setFuture(qfuture_right);

    return true;
}

void StereoCameraBasler::disconnectCamera() {
    if (connected){
        left_camera->close();
        right_camera->close();
    }
    connected = false;
    //TODO disconnect future watcher
    emit finished();
    emit disconnected();
}

StereoCameraBasler::~StereoCameraBasler() {
    disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
    disconnectCamera();
    Pylon::PylonTerminate();
}
