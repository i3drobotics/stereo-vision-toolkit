/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "stereocamerabasler.h"

//see: https://github.com/basler/pypylon/blob/master/samples/grabmultiplecameras.py

bool StereoCameraBasler::initCamera(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings) {
    Pylon::PylonInitialize();
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
    int packet_size = inital_camera_settings.packetSize;
    int packet_delay = inital_camera_settings.packetDelay;
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

    bool res = setupCameras(CSI_cam_info,binning,trigger,fps,packet_size);

    enableAutoGain(autoGain);
    enableAutoExpose(autoExpose);
    setExposure(exposure);
    setGain(gain);
    if (packet_delay >= 0){
        setPacketDelay(packet_delay);
    }

    this->connected = true;

    return res;
}

bool StereoCameraBasler::setupCameras(AbstractStereoCamera::stereoCameraSerialInfo CSI_cam_info,int iBinning, int iTrigger, int iFps, int iPacketSize){
    //this->connected = false;
    //disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));

    this->camera_serial_info = CSI_cam_info;
    this->m_binning = iBinning;
    this->m_iTrigger = iTrigger;
    this->m_fps = iFps;
    this->m_packet_size = iPacketSize;

    try
    {

        // Create an instant camera object with the camera device found first.
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

        Pylon::DeviceInfoList_t devices;
        tlFactory.EnumerateDevices(devices);

        this->cameras = new Pylon::CInstantCameraArray(2);
        Pylon::CInstantCameraArray all_cameras(devices.size());

        std::string camera_left_serial = camera_serial_info.left_camera_serial;
        std::string camera_right_serial = camera_serial_info.right_camera_serial;

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

        getImageSize(cameras->operator[](0),image_width,image_height,image_size);
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
        if (iPacketSize >= 0){
            setPacketSize(iPacketSize);
        }
        if (iFps > 0){
            changeFPS(iFps);
        }

        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).MaxNumBuffer = 1;
            cameras->operator[](i).OutputQueueSize = cameras->operator[](i).MaxNumBuffer.GetValue();
        }

        formatConverter = new Pylon::CImageFormatConverter();

        formatConverter->OutputPixelFormat = Pylon::PixelType_Mono8;
        formatConverter->OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;

        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImages);
        }

        //cameras->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImages);

        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst setting up cameras." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }

    //connect(this, SIGNAL(acquired()), this, SLOT(capture()));
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraBasler::listSystems(void){
    Pylon::PylonInitialize();
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_gige = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos_usb = loadSerials(AbstractStereoCamera::CAMERA_TYPE_BASLER_USB);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos;
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_gige.begin(), known_serial_infos_gige.end() );
    known_serial_infos.insert( known_serial_infos.end(), known_serial_infos_usb.begin(), known_serial_infos_usb.end() );

    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;
    // find basler systems connected
    // Initialise Basler Pylon
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
        std::string device_serial = std::string(all_cameras[i].GetDeviceInfo().GetSerialNumber());
        //if (device_name.find("i3dr") != std::string::npos) {
        if (device_class == DEVICE_CLASS_GIGE || device_class == DEVICE_CLASS_USB){
            connected_serials.push_back(device_serial);
            connected_camera_names.push_back(device_name);
        } else {
            qDebug() << "Unsupported basler class: " << device_class.c_str();
        }
        //} else {
        //    qDebug() << "Unsupported basler camera with name: " << device_name.c_str();
        //}
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

    Pylon::PylonTerminate();
    return connected_serial_infos;
}

void StereoCameraBasler::getImageSize(Pylon::CInstantCamera &camera, int &width, int &height, cv::Size &size)
{
    try
    {
        camera.Open();
        width = Pylon::CIntegerParameter(camera.GetNodeMap(), "Width").GetValue();
        height = Pylon::CIntegerParameter(camera.GetNodeMap(), "Height").GetValue();
        size = cv::Size(width,height);
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst getting camera image size." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::enableTrigger(bool enable){
    setTrigger(enable);
}

void StereoCameraBasler::changeBinning(int val){
    //TODO fix this
    qDebug() << "Setting binning";
    cameras->StopGrabbing();
    cameras->Close();
    setupCameras(this->camera_serial_info,val,this->m_iTrigger,this->m_fps,this->m_packet_size);
    qDebug() << "Binning updated";
}

void StereoCameraBasler::changePacketSize(int packetSize){
    //TODO test this
    qDebug() << "Setting packet size";
    cameras->StopGrabbing();
    cameras->Close();
    setupCameras(this->camera_serial_info,this->m_binning,this->m_iTrigger,this->m_fps,packetSize);
    qDebug() << "Packet size updated";
}

void StereoCameraBasler::setPacketSize(int packetSize)
{
    try
    {
        if (packetSize >= 220){
            for (size_t i = 0; i < cameras->GetSize(); ++i)
            {
                cameras->operator[](i).Open();
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "GevSCPSPacketSize").SetValue(packetSize);
            }
        }
        this->m_packet_size = packetSize;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::setPacketDelay(int interPacketDelay)
{
    try
    {
        if (interPacketDelay >= 0){
            cameras->operator[](0).Open();
            Pylon::CIntegerParameter(cameras->operator[](0).GetNodeMap(), "GevSCPD").SetValue(interPacketDelay);
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}


void StereoCameraBasler::setFPS(int val){
    try
    {
        float fps_f = (float)val;
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CFloatParameter(cameras->operator[](i).GetNodeMap(), "AcquisitionFrameRateAbs").SetValue(fps_f);
        }
        this->m_fps = val;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::enableFPS(bool enable){
    try
    {
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CBooleanParameter(cameras->operator[](i).GetNodeMap(), "AcquisitionFrameRateEnable").SetValue(enable);
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::changeFPS(int val){
    // unlimited frame rate if set to 0
    if (val>0){
        setFPS(val);
        enableFPS(true);
    } else {
        enableFPS(false);
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
            Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "ExposureTimeRaw").SetValue(exposure_i);
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

void StereoCameraBasler::setBinning(int val){
    try
    {
        if (val >= 1){
            for (size_t i = 0; i < cameras->GetSize(); ++i)
            {
                cameras->operator[](i).Open();
                Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "BinningHorizontalMode").FromString("Average");
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "BinningHorizontal").SetValue(val);
                Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "BinningVerticalMode").FromString("Average");
                Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "BinningVertical").SetValue(val);
            }
        }

        this->m_binning = val;

        getImageSize(cameras->operator[](0),image_width,image_height,image_size);
        emit update_size(image_width, image_height, 1);
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::setTrigger(bool enable){
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "On";
        }
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CEnumParameter(cameras->operator[](i).GetNodeMap(), "TriggerMode").FromString(enable_str.c_str());
        }
        int iTrigger;
        if (enable){
            iTrigger = 1;
        } else {
            iTrigger = 0;
        }
        this->m_iTrigger = iTrigger;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

void StereoCameraBasler::toggleTrigger(bool enable){
    enableFPS(!enable);
    enableTrigger(enable);
}
void StereoCameraBasler::adjustFPS(int val){
    changeFPS(val);
}

void StereoCameraBasler::toggleAutoExpose(bool enable){
    enableAutoExpose(enable);
}

void StereoCameraBasler::adjustExposure(double val){
    setExposure(val);
}

void StereoCameraBasler::toggleAutoGain(bool enable){
    enableAutoGain(enable);
}

void StereoCameraBasler::adjustGain(int val){
    setGain(val);
}

bool StereoCameraBasler::setGain(int val) {
    try
    {
        int gain_i = val;
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            cameras->operator[](i).Open();
            Pylon::CIntegerParameter(cameras->operator[](i).GetNodeMap(), "GainRaw").SetValue(gain_i);
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

bool StereoCameraBasler::enableAutoExpose(bool enable){
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

void StereoCameraBasler::adjustBinning(int val){
    changeBinning(val);
}

void StereoCameraBasler::adjustPacketSize(int val){
    changePacketSize(val);
}

bool StereoCameraBasler::grab(){
    int max_timeout = 1100;
    bool res = false;
    try
    {
        if (this->connected){
            //if (cameras->IsGrabbing())
            if (cameras->operator[](0).IsGrabbing() && cameras->operator[](1).IsGrabbing())
            {
                Pylon::CGrabResultPtr ptrGrabResult_left,ptrGrabResult_right;
                cameras->operator[](0).RetrieveResult(max_timeout, ptrGrabResult_left, Pylon::TimeoutHandling_ThrowException);
                cameras->operator[](1).RetrieveResult(max_timeout, ptrGrabResult_right, Pylon::TimeoutHandling_ThrowException);

                if (ptrGrabResult_left == NULL || ptrGrabResult_right == NULL){
                    qDebug() << "Camera grab pointer is null";
                    res = false;
                } else {
                    if (ptrGrabResult_left->GrabSucceeded() && ptrGrabResult_right->GrabSucceeded())
                    {
                        int frameCols_l = ptrGrabResult_left->GetWidth();
                        int frameRows_l = ptrGrabResult_left->GetHeight();
                        int frameCols_r = ptrGrabResult_right->GetWidth();
                        int frameRows_r = ptrGrabResult_right->GetHeight();

                        if (frameCols_l == 0 || frameRows_l == 0 || frameCols_r == 0 || frameRows_r == 0){
                            qDebug() << "Image buffer size is incorrect";
                            res = false;
                        } else {
                            Pylon::CPylonImage pylonImage_left, pylonImage_right;
                            formatConverter->Convert(pylonImage_left, ptrGrabResult_left);
                            formatConverter->Convert(pylonImage_right, ptrGrabResult_right);
                            cv::Mat image_left_temp = cv::Mat(frameRows_l, frameCols_l, CV_8UC1, static_cast<uchar *>(pylonImage_left.GetBuffer()));
                            cv::Mat image_right_temp = cv::Mat(frameRows_r, frameCols_r, CV_8UC1, static_cast<uchar *>(pylonImage_right.GetBuffer()));

                            if (image_left_temp.cols == 0 || image_left_temp.rows == 0 || image_right_temp.cols == 0 || image_right_temp.rows == 0){
                                qDebug() << "Image result buffer size is incorrect";
                                res = false;

                            } else if ((image_left_temp.cols != image_right_temp.cols) || (image_left_temp.rows != image_right_temp.rows)){
                                qDebug() << "Left and right images are different sizes but the MUST be equal for stereo matching";
                                res = false;
                            } else {
                                image_left_temp.copyTo(left_raw);
                                image_right_temp.copyTo(right_raw);
                                res = true;
                            }
                            pylonImage_left.Release();
                            pylonImage_right.Release();
                        }
                    }
                    else
                    {
                        qDebug() << "Failed to grab left image.";
                        qDebug() << "Error: " << ptrGrabResult_left->GetErrorCode() << " " << ptrGrabResult_left->GetErrorDescription();
                        std::cerr << "Failed to grab left image." << std::endl;
                        std::cerr << "Error: " << ptrGrabResult_left->GetErrorCode() << " " << ptrGrabResult_left->GetErrorDescription() << std::endl;

                        qDebug() << "Failed to grab right image.";
                        qDebug() << "Error: " << ptrGrabResult_right->GetErrorCode() << " " << ptrGrabResult_right->GetErrorDescription();
                        std::cerr << "Failed to grab right image." << std::endl;
                        std::cerr << "Error: " << ptrGrabResult_right->GetErrorCode() << " " << ptrGrabResult_right->GetErrorDescription() << std::endl;
                        res = false;
                    }
                    ptrGrabResult_left.Release();
                    ptrGrabResult_right.Release();
                }
            }
            else
            {
                qDebug() << "Camera isn't grabbing images.";
                std::cerr << "Camera isn't grabbing images." << std::endl;
                //cameras->StartGrabbing();
                res = false;
            }
        } else {
            qDebug() << "Camera is not connected or is initalising";
            std::cerr << "Camera is not connected or is initalising" << std::endl;
            res = false;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        qDebug() << "An exception occurred." << e.GetDescription();
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        res = false;
        this->connected = false;
        cameras->StopGrabbing();
        cameras->Close();
        setupCameras(this->camera_serial_info,this->m_binning,this->m_iTrigger,this->m_fps,this->m_packet_size);
        this->connected = true;
    }

    return res;
}

bool StereoCameraBasler::capture() {
    qfuture_capture = QtConcurrent::run(this, &StereoCameraBasler::grab);
    //qfutureWatcher_capture.setFuture(qfuture_capture);

    qfuture_capture.waitForFinished();
    emit right_captured();
    emit left_captured();

    return true;
}

void StereoCameraBasler::disconnectCamera() {
    if (connected){
        cameras->StopGrabbing();
        cameras->operator[](0).StopGrabbing();
        cameras->operator[](1).StopGrabbing();
        cameras->Close();
        cameras->operator[](0).Close();
        cameras->operator[](1).Close();
    }
    connected = false;
    //emit finished();
    emit disconnected();
    Pylon::PylonTerminate();
}

StereoCameraBasler::~StereoCameraBasler() {
    disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
    disconnectCamera();
    Pylon::PylonTerminate();
}
