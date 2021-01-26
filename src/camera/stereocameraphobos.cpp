/*
* Copyright I3D Robotics Ltd, 2021
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameraphobos.h"

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraPhobosBasler::listSystemsQuick(Pylon::CTlFactory* tlFactory){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;
    // find basler systems connected
    // Initialise Basler Pylon
    // Create an instant camera object with the camera device found first.
    QElapsedTimer task_timer;
    task_timer.start();
    //Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    tlFactory->EnumerateDevices(devices);

    qDebug() << "Time to initalise pylon: " << task_timer.elapsed();

    std::string DEVICE_CLASS_GIGE = "BaslerGigE";
    std::string DEVICE_CLASS_USB = "BaslerUsb";

    std::vector<std::string> connected_camera_names;
    std::vector<std::string> connected_serials;
    //TODO add generic way to recognise i3dr cameras whilst still being
    //able to make sure the correct right and left cameras are selected together
    for (size_t i = 0; i < devices.size(); ++i)
    {
        std::string device_class = std::string(devices[i].GetDeviceClass());
        std::string device_name = std::string(devices[i].GetUserDefinedName());
        std::string device_serial = std::string(devices[i].GetSerialNumber());
        if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
            connected_serials.push_back(device_serial);
            connected_camera_names.push_back(device_name);
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

    //Pylon::PylonTerminate();
    return connected_serial_infos;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraPhobosBasler::listSystems(void){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;
    // find basler systems connected
    // Initialise Basler Pylon
    // Create an instant camera object with the camera device found first.
    QElapsedTimer task_timer;
    task_timer.start();
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

    Pylon::DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);

    qDebug() << "Time to initalise pylon: " << task_timer.elapsed();

    std::string DEVICE_CLASS_GIGE = "BaslerGigE";
    std::string DEVICE_CLASS_USB = "BaslerUsb";

    std::vector<std::string> connected_camera_names;
    std::vector<std::string> connected_serials;
    //TODO add generic way to recognise i3dr cameras whilst still being
    //able to make sure the correct right and left cameras are selected together
    for (size_t i = 0; i < devices.size(); ++i)
    {
        std::string device_class = std::string(devices[i].GetDeviceClass());
        std::string device_name = std::string(devices[i].GetUserDefinedName());
        std::string device_serial = std::string(devices[i].GetSerialNumber());
        if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
            connected_serials.push_back(device_serial);
            connected_camera_names.push_back(device_name);
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

    //Pylon::PylonTerminate();
    return connected_serial_infos;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraPhobosTIS::listSystemsQuick(DShowLib::Grabber* handle){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_TIS_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    /* Get number of cameras */
    auto devices = handle->getAvailableVideoCaptureDevices();

    for (auto& known_serial_info : known_serial_infos) {
        bool left_found = false;
        bool right_found = false;
        qint64 left_serial;
        qint64 right_serial;
        //find left
        for (auto& device : *devices) {
            device.getSerialNumber(left_serial);
            if (QString::number(left_serial).toStdString() == known_serial_info.left_camera_serial){
                left_found = true;
                break;
            }
        }
        if (left_found){
            //find right
            for (auto& device : *devices) {
                device.getSerialNumber(right_serial);
                if (QString::number(right_serial).toStdString() == known_serial_info.right_camera_serial){
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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraPhobosTIS::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(AbstractStereoCamera::CAMERA_TYPE_PHOBOS_TIS_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    DShowLib::Grabber handle;
    /* Get number of cameras */
    auto devices = handle.getAvailableVideoCaptureDevices();

    for (auto& known_serial_info : known_serial_infos) {
        bool left_found = false;
        bool right_found = false;
        qint64 left_serial;
        qint64 right_serial;
        //find left
        for (auto& device : *devices) {
            device.getSerialNumber(left_serial);
            if (QString::number(left_serial).toStdString() == known_serial_info.left_camera_serial){
                left_found = true;
                break;
            }
        }
        if (left_found){
            //find right
            for (auto& device : *devices) {
                device.getSerialNumber(right_serial);
                if (QString::number(right_serial).toStdString() == known_serial_info.right_camera_serial){
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

StereoCameraPhobosBasler::~StereoCameraPhobosBasler(void) {
}

StereoCameraPhobosTIS::~StereoCameraPhobosTIS(void) {
}
