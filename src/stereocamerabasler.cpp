/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "stereocamerabasler.h"

bool StereoCameraBasler::initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info) {
    this->camera_serial_info = camera_serial_info;
    this->left_camera = new CameraBasler;
    this->right_camera = new CameraBasler;

    int binning = 2;

    bool res = this->left_camera->initCamera(camera_serial_info.left_camera_serial,binning,false,0);
    res &= this->right_camera->initCamera(camera_serial_info.right_camera_serial,binning,false,0);

    if (res) {
        this->left_camera->setExposure(5);
        this->right_camera->setExposure(5);

        this->left_camera->setGain(0);
        this->right_camera->setGain(0);

        this->left_camera->getImageSize(image_width, image_height, image_size);
        emit update_size(image_width, image_height, 1);
    }else{
        qDebug() << "Failed to setup cameras";
        disconnectCamera();
    }

    this->connected = res;

    return res;
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraBasler::listSystems(void){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos = loadSerials("basler");
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

    std::vector<std::string> camera_names;
    std::vector<std::string> connected_serials;
    for (size_t i = 0; i < cameras.GetSize(); ++i)
    {
        cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
        connected_serials.push_back(std::string(cameras[i].GetDeviceInfo().GetSerialNumber()));
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

bool StereoCameraBasler::autoConnect(void){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> camera_serials = listSystems();

    if (camera_serials.size() >= 1){
        // pick first avaiable camera
        return initCamera(camera_serials.at(0));
    } else {
        return false;
    }
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

bool StereoCameraBasler::capture() {
    //capturing = true;

    QFuture<bool> left_result =
            QtConcurrent::run(left_camera, &CameraBasler::capture);
    QFuture<bool> right_result =
            QtConcurrent::run(right_camera, &CameraBasler::capture);

    left_result.waitForFinished();
    right_result.waitForFinished();

    if (left_result.result() && right_result.result()) {
        cv::Mat *left_data = left_camera->getImage();
        cv::Mat *right_data = right_camera->getImage();
        left_data->copyTo(left_raw);
        right_data->copyTo(right_raw);

        emit left_captured();
        emit right_captured();

        return true;
    } else {
        qDebug() << "Failed to capture stereo pair";
        emit left_captured();
        emit right_captured();
    }

    //capturing = false;

    return false;
}

void StereoCameraBasler::disconnectCamera() {
    if (connected){
        left_camera->close();
        right_camera->close();
    }
    connected = false;
    emit finished();
    emit disconnected();
}

StereoCameraBasler::~StereoCameraBasler() {
    disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
    disconnectCamera();
    Pylon::PylonTerminate();
}
