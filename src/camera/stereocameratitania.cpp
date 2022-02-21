/*
* Copyright I3D Robotics Ltd, 2021
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameratitania.h"

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraTitaniaBasler::autoDetectTitanias(Pylon::CTlFactory* tlFactory){
    Pylon::DeviceInfoList_t devices;
    tlFactory->EnumerateDevices(devices);

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> foundTitanias;

    // find valid labels titania devices
    Pylon::DeviceInfoList_t valid_titania_devices_left;
    Pylon::DeviceInfoList_t valid_titania_devices_right;
    for (size_t i = 0; i < devices.size(); ++i)
    {
        std::string device_class = std::string(devices[i].GetDeviceClass());
        std::string device_name = std::string(devices[i].GetUserDefinedName());
        std::string device_serial = std::string(devices[i].GetSerialNumber());

        QString device_name_qstr = QString::fromStdString(device_name);
        if (device_name_qstr.contains("I3DRTitania")){
            QStringList device_name_qstrlist = device_name_qstr.split('_');
            if (device_name_qstrlist.size() == 3){
                if (device_name_qstrlist.at(2) == "l"){
                    valid_titania_devices_left.push_back(devices[i]);
                }
                else if (device_name_qstrlist.at(2) == "r"){
                    valid_titania_devices_right.push_back(devices[i]);
                }
                else {
                    qDebug() << "Detected I3DR Titania with malformed device name: " << device_name.c_str();
                }
            } else {
                qDebug() << "Detected I3DR Titania with malformed device name: " << device_name.c_str();
            }
        }
    }
    
    for (auto& valid_titania_device_left : valid_titania_devices_left)
    {
        for (auto& valid_titania_device_right : valid_titania_devices_right)
        {
            std::string left_device_name = std::string(valid_titania_device_left.GetUserDefinedName());
            std::string right_device_name = std::string(valid_titania_device_right.GetUserDefinedName());
            QString left_device_name_qstr = QString::fromStdString(left_device_name);
            QString right_device_name_qstr = QString::fromStdString(right_device_name);
            QStringList left_device_name_qstrlist = left_device_name_qstr.split('_');
            QStringList right_device_name_qstrlist = right_device_name_qstr.split('_');
            if (left_device_name_qstrlist.at(1) == right_device_name_qstrlist.at(1)){
                AbstractStereoCamera::StereoCameraSerialInfo camera_info = {
                    std::string(valid_titania_device_left.GetSerialNumber()),
                    std::string(valid_titania_device_right.GetSerialNumber()),
                    AbstractStereoCamera::StereoCameraType::CAMERA_TYPE_TITANIA_BASLER_USB,
                    left_device_name_qstrlist.at(1).toStdString()
                };
                foundTitanias.push_back(camera_info);
            }
        }
    }

    return foundTitanias;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraTitaniaBasler::listSystemsQuick(Pylon::CTlFactory* tlFactory){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    // auto detect first
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> auto_serial_infos = autoDetectTitanias(tlFactory);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos = auto_serial_infos;
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
    for (auto& device : devices)
    {
        std::string device_class = std::string(device.GetDeviceClass());
        std::string device_name = std::string(device.GetUserDefinedName());
        std::string device_serial = std::string(device.GetSerialNumber());
        if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
            // only add if doesn't already exist from auto detection
            bool found_in_auto = false;
            for (auto& auto_serial_info : auto_serial_infos)
            {
                if (device_serial == auto_serial_info.left_camera_serial || device_serial == auto_serial_info.right_camera_serial){
                    found_in_auto = true;
                    break;
                }
            }
            if (!found_in_auto){
                connected_serials.push_back(device_serial);
                connected_camera_names.push_back(device_name);
            }
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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraTitaniaBasler::listSystems(void){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB);
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

bool StereoCameraTitaniaBasler::getCameraFrame(cv::Mat &cam_left_image, cv::Mat &cam_right_image){
    cv::Mat left_tmp, right_tmp;
    bool res = StereoCameraBasler::getCameraFrame(left_tmp,right_tmp);
    if (res){
        // Rotate right camera as it is mounted upside down
        //cv::flip(left_tmp,left_tmp,0);
        //cv::flip(left_tmp,left_tmp,1);
        //cv::flip(right_tmp,right_tmp,0);
        //cv::flip(right_tmp,right_tmp,1);
        cam_left_image = left_tmp.clone();
        cam_right_image = right_tmp.clone();
    }
    return res;
}

bool StereoCameraTitaniaBasler::captureSingle(){
    cv::Mat left_tmp, right_tmp;
    bool res = getCameraFrame(left_tmp,right_tmp);
    if (!res){
        send_error(CAPTURE_ERROR);
        emit captured_fail();
    } else {
        left_raw = left_tmp.clone();
        right_raw = right_tmp.clone();
        emit captured_success();
    }
    emit captured();
    return res;
}

void StereoCameraTitaniaBasler::captureThreaded() {
    future = QtConcurrent::run(this, &StereoCameraTitaniaBasler::captureSingle);
}

#ifdef WITH_VIMBA
std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraTitaniaVimba::listSystems(){

    using namespace AVT::VmbAPI;

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_TITANIA_VIMBA_USB);
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
#endif

StereoCameraTitaniaBasler::~StereoCameraTitaniaBasler(void) {
}

#ifdef WITH_VIMBA
StereoCameraTitaniaVimba::~StereoCameraTitaniaVimba(void) {
}
#endif
