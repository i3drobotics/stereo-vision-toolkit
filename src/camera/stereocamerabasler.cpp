/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocamerabasler.h"

//see: https://github.com/basler/pypylon/blob/master/samples/grabmultiplecameras.py

bool StereoCameraBasler::openCamera(){
    camControl = new ArduinoCommsCameraControl();
    std::vector<QSerialPortInfo> serial_device_list = camControl->getSerialDevices();
    if (serial_device_list.size() <= 0){
        qDebug() << "Failed to find serial device for hardware control.";
        //return false;
    } else {
        has_trigger_fps_control = camControl->open(serial_device_list.at(0),115200);
    }

    int binning = stereoCameraSettings_.binning;
    bool trigger;
    if (stereoCameraSettings_.trigger == 1){
        trigger = true;
    } else {
        trigger = false;
    }
    hardware_triggered = trigger;
    int fps = stereoCameraSettings_.fps;;
    double exposure = stereoCameraSettings_.exposure;
    int gain = stereoCameraSettings_.gain;
    int packet_size = stereoCameraSettings_.packetSize;
    int packet_delay = stereoCameraSettings_.packetDelay;
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

    try
    {

        // Create an instant camera object with the camera device found first.
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

        Pylon::DeviceInfoList_t devices;
        tlFactory.EnumerateDevices(devices);

        this->cameras = new Pylon::CInstantCameraArray(2);
        Pylon::CInstantCameraArray all_cameras(devices.size());

        std::string camera_left_serial = stereoCameraSerialInfo_.left_camera_serial;
        std::string camera_right_serial = stereoCameraSerialInfo_.right_camera_serial;

        bool cameraLeftFind = false;
        for (size_t i = 0; i < all_cameras.GetSize(); ++i)
        {
            all_cameras[i].Attach(tlFactory.CreateDevice(devices[i]));

            if (all_cameras[i].GetDeviceInfo().GetSerialNumber() == camera_left_serial.c_str())
            {
                cameras->operator[](0).Attach(tlFactory.CreateDevice(devices[i]));
                cameraLeftFind = true;
                break;
            }
        }

        if (!cameraLeftFind){
            std::cerr << "Failed to find left camera with serial: " << camera_left_serial << std::endl;
            return false;
        }

        bool cameraRightFind = false;
        for (size_t i = 0; i < all_cameras.GetSize(); ++i)
        {
            all_cameras[i].Attach(tlFactory.CreateDevice(devices[i]));

            if (all_cameras[i].GetDeviceInfo().GetSerialNumber() == camera_right_serial.c_str())
            {
                cameras->operator[](1).Attach(tlFactory.CreateDevice(devices[i]));
                cameraRightFind = true;
                break;
            }
        }

        if (!cameraRightFind){
            std::cerr << "Failed to find right camera with serial: " << camera_right_serial << std::endl;
            return false;
        }

        // Set device link throughput to fix data collision
        enableDeviceLinkThroughputLimit(true);
        setDeviceLinkThroughput(100000000);

        //check open
        bool connected = true;
        try{
            for (size_t i = 0; i < cameras->GetSize(); ++i)
            {
                cameras->operator[](i).Open();
                connected &= cameras->operator[](i).IsOpen();
            }
        }
        catch (const Pylon::GenericException &e)
        {
            // Error handling.
            std::cerr << "An exception occurred whilst opening camera." << std::endl
                      << e.GetDescription() << std::endl;
            connected = false;
        }

        if (!connected){
            return connected;
        }

        getImageSize(cameras->operator[](0),image_width,image_height,image_bitdepth);
        emit update_size(image_width, image_height, image_bitdepth);

        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            //cameras->operator[](i).MaxNumBuffer = 1;
            //cameras->operator[](i).OutputQueueSize = 1;
            //cameras->operator[](i).ClearBufferModeEnable();
        }

        formatConverter = new Pylon::CImageFormatConverter();

        //formatConverter->OutputPixelFormat = Pylon::PixelType_Mono8;
        formatConverter->OutputPixelFormat = Pylon::PixelType_BGR8packed;
        formatConverter->OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst setting up cameras." << std::endl
                  << e.GetDescription() << std::endl;
        connected = false;
        return connected;
    }

    // Set device parameters
    setBinning(binning);
    enableTrigger(trigger);
    setPacketSize(packet_size);
    setFPS(fps);

    enableAutoGain(autoGain);
    enableAutoExposure(autoExpose);
    setExposure(exposure);
    setGain(gain);
    setPacketDelay(packet_delay);

    //lineStatusTimerId = startTimer(lineStatusTimerDelay);

    connected = true;
    return connected;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraBasler::listSystemsQuick(Pylon::CTlFactory* tlFactory){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_USB);
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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraBasler::listSystems(void){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_USB);
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

void StereoCameraBasler::timerEvent(QTimerEvent *event)
{
    if(event->timerId() == lineStatusTimerId) {
        bool status_l, status_r;
        getLineStatus(2,status_l,status_r);
        //TODO enable this
        //qDebug() << "Line Status.. (" << status_l << ", " << status_r << ")";
    }
}

void StereoCameraBasler::getLineStatus(int line, bool &status_l, bool &status_r){
    try
    {
        std::string line_str = "Line" + std::to_string(line); //LineX
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](0).Open();
            Pylon::CEnumParameter(cameras->operator[](0).GetNodeMap().GetNode("LineSelector")).FromString(line_str.c_str());
            //get device line status
            if (i == 0){
                status_l = Pylon::CBooleanParameter(cameras->operator[](0).GetNodeMap(), "LineStatus").GetValue();;
            } else if (i == 1){
                status_r = Pylon::CBooleanParameter(cameras->operator[](0).GetNodeMap(), "LineStatus").GetValue();;
            }
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst getting device line status." << std::endl
                    << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, int &bitdepth)
{
    try
    {
        camera.Open();
        width = Pylon::CIntegerParameter(camera.GetNodeMap(), "Width").GetValue();
        height = Pylon::CIntegerParameter(camera.GetNodeMap(), "Height").GetValue();
        bitdepth = 1; //TODO get bitdepth
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst getting camera image size." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::enableDeviceLinkThroughputLimit(bool enable){
    try
    {
        std::string mode = "Off";
        if (enable){
            mode = "On";
        }
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            //Set device link throughput mode
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "DeviceLinkThroughputLimitMode").FromString(mode.c_str());
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst setting device link throughput parameter." << std::endl
                    << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::setDeviceLinkThroughput(int value){
    try
    {
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            //Set device link throughput limit
            Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "DeviceLinkThroughputLimit").SetValue(value);
        } 
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst setting device link throughput parameter." << std::endl
                    << e.GetDescription() << std::endl;
    }
}

bool StereoCameraBasler::setPacketSize(int packetSize)
{
    try
    {
        //Apply only to left camera as this should be the delay betwen left and right
        cameras->operator[](0).Open();
        Pylon::CIntegerParameter(cameras->operator[](0).GetNodeMap(), "GevSCPSPacketSize").SetValue(packetSize);
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::setPacketDelay(int interPacketDelay)
{
    try
    {
        //Apply only to left camera as this should be the delay betwen left and right
        if (interPacketDelay >= 0){
            cameras->operator[](0).Open();
            Pylon::CIntegerParameter(cameras->operator[](0).GetNodeMap(), "GevSCPD").SetValue(interPacketDelay);
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}


bool StereoCameraBasler::setFPS(int val){
    if (hardware_triggered){
        qDebug() << "Setting hardware fps..." << val;
        if (!camControl->isConnected()){
            std::vector<QSerialPortInfo> serial_device_list = camControl->getSerialDevices();
            if (serial_device_list.size() <= 0){
                qDebug() << "Failed to find serial device for hardware control.";
            } else {
                camControl->open(serial_device_list.at(0),115200);
            }
            if (camControl->isConnected()){
                camControl->updateFPS(val);
            } else {
                qDebug() << "Failed to open connection to camera control device";
                // TODO remove this when camcontrol consistently working
                frame_rate = val;
                return false;
            }
        } else {
            camControl->updateFPS(val);
            frame_rate = val;
            return true;
        }
    } else {
        try
        {
            qDebug() << "Setting software fps..." << val;
            float fps_f = (float)val;
            for (size_t i = 0; i < cameras->GetSize(); ++i)
            {
                cameras->operator[](i).Open();
                if (stereoCameraSettings_.isGige){
                    Pylon::CFloatParameter(cameras->operator[](i).GetNodeMap(), "AcquisitionFrameRateAbs").SetValue(fps_f);
                } else {
                    Pylon::CFloatParameter(cameras->operator[](i).GetNodeMap(), "AcquisitionFrameRate").SetValue(fps_f);
                }
            }
            frame_rate = val;
            return true;
        }
        catch (const Pylon::GenericException &e)
        {
            // Error handling.
            std::cerr << "An exception occurred." << std::endl
                      << e.GetDescription() << std::endl;
            return false;
        }
    }
    return false;
}

bool StereoCameraBasler::enableFPS(bool enable){
    try
    {
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            qDebug() << "AcquisitionFrameRateEnable: " << enable;
            cameras->operator[](i).Open();
            Pylon::CBooleanParameter(cameras->operator[](i).GetNodeMap(), "AcquisitionFrameRateEnable").SetValue(enable);
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::setExposure(double val) {
    try
    {
        // convert from seconds to milliseconds
        int exposure_i = val * 10000;
        qDebug() << "Setting exposure..." << exposure_i;
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            if (stereoCameraSettings_.isGige){
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "ExposureTimeRaw").SetValue(exposure_i);
            } else {
                Pylon::CFloatParameter(cameras->operator[](i).GetNodeMap(), "ExposureTime").SetValue(exposure_i);
            }
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::setBinning(int val){
    try
    {
        if (val >= 1){
            qDebug() << "Setting binning... " << val;
            for (size_t i = 0; i < cameras->GetSize(); ++i)
            {
                cameras->operator[](i).Open();
                Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "BinningHorizontalMode").FromString("Average");
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "BinningHorizontal").SetValue(val);
                Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "BinningVerticalMode").FromString("Average");
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "BinningVertical").SetValue(val);
            }
        }

        getImageSize(cameras->operator[](0),image_width,image_height,image_bitdepth);
        emit update_size(image_width, image_height, image_bitdepth);
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::enableTrigger(bool enable){
    enableFPS(!enable);
    hardware_triggered = enable;
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "On";
        }
        qDebug() << "Enable hardware trigger: " << enable_str.c_str();
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "TriggerSelector").SetValue("FrameStart");
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "TriggerSource").SetValue("Line1");
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "TriggerMode").SetValue(enable_str.c_str());
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::setGain(int val) {
    try
    {
        int gain_i = val;
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            if (stereoCameraSettings_.isGige){
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "GainRaw").SetValue(gain_i);
            } else {
                Pylon::CFloatParameter(cameras->operator[](i).GetNodeMap(), "Gain").SetValue(gain_i);
            }
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::enableAutoGain(bool enable){
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "Continuous";
        }
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "GainAuto").FromString(enable_str.c_str());
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::enableAutoExposure(bool enable){
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "Continuous";
        }
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "ExposureAuto").FromString(enable_str.c_str());
        }
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
}

bool StereoCameraBasler::getCameraFrame(cv::Mat &cam_left_image, cv::Mat &cam_right_image){
    // set maximum timeout of capure to 2xfps
    int max_timeout = 2*(1000*(1.0f/(float)frame_rate));
    if (max_timeout > 1000){
        max_timeout = 1000;
    }
    bool res = false;
    try
    {
        if (this->connected){
            if (capturing){
                if (cameras->operator[](0).IsGrabbing() && cameras->operator[](1).IsGrabbing())
                {
                    Pylon::CGrabResultPtr ptrGrabResult_left,ptrGrabResult_right;
                    cameras->operator[](0).RetrieveResult(max_timeout, ptrGrabResult_left, Pylon::TimeoutHandling_ThrowException);
                    cameras->operator[](1).RetrieveResult(max_timeout, ptrGrabResult_right, Pylon::TimeoutHandling_ThrowException);

                    bool res_l = PylonSupport::grabImage2mat(ptrGrabResult_left,formatConverter,cam_left_image);
                    bool res_r = PylonSupport::grabImage2mat(ptrGrabResult_right,formatConverter,cam_right_image);

                    ptrGrabResult_left.Release();
                    ptrGrabResult_right.Release();

                    res = res_l && res_r;
                    if (!res){
                        if (capturing){
                            qDebug() << "Failed to convert grab result to mat";
                        } else {
                            qDebug() << "Failed to convert grab result to mat but not grabbing so will just ignore this frame.";
                        }
                    }
                } else {
                    res = false;
                    qDebug() << "Camera is not grabbing. Should use cameras->StartGrabbing()";
                }
            } else {
                qDebug() << "Camera isn't grabbing images. Will grab one";
                Pylon::CGrabResultPtr ptrGrabResult_left,ptrGrabResult_right;
                cameras->operator[](0).GrabOne( max_timeout, ptrGrabResult_left);
                cameras->operator[](1).GrabOne( max_timeout, ptrGrabResult_right);

                bool res_l = PylonSupport::grabImage2mat(ptrGrabResult_left,formatConverter,cam_left_image);
                bool res_r = PylonSupport::grabImage2mat(ptrGrabResult_right,formatConverter,cam_right_image);

                ptrGrabResult_left.Release();
                ptrGrabResult_right.Release();

                res = res_l && res_r;
                if (!res){
                    qDebug() << "Failed to convert grab result to mat";
                }
            }
        } else {
            qDebug() << "Camera is not connected or is initalising";
            std::cerr << "Camera is not connected or is initalising" << std::endl;
            res = false;
            closeCamera();
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        qDebug() << "An exception occurred." << e.GetDescription();
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        res = false;
    }
    return res;
}

bool StereoCameraBasler::captureSingle(){
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

void StereoCameraBasler::captureThreaded() {
    future = QtConcurrent::run(this, &StereoCameraBasler::captureSingle);
}

bool StereoCameraBasler::enableCapture(bool enable){
    if (enable){
        //Start capture thread
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImageOnly);
            //cameras->operator[](i).StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_OneByOne);
        }
        //TODO replace this with pylon callback
        connect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = true;
        captureThreaded();
    } else {
        //Stop capture thread
        capturing = false;
        disconnect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        cameras->StopGrabbing();
        cameras->operator[](0).StopGrabbing();
        cameras->operator[](1).StopGrabbing();
    }
    return true;
}

//TODO add pylon disconnect callback

bool StereoCameraBasler::closeCamera() {
    if (connected){
        cameras->StopGrabbing();
        cameras->operator[](0).StopGrabbing();
        cameras->operator[](1).StopGrabbing();
        cameras->Close();
        cameras->operator[](0).Close();
        cameras->operator[](1).Close();
        camControl->close();
    }
    connected = false;
    emit disconnected();
    return true;
}

StereoCameraBasler::~StereoCameraBasler() {
    killTimer(lineStatusTimerId);
}
