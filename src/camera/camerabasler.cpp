/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#include "camerabasler.h"

CameraBasler::CameraBasler(QObject *parent) : QObject(parent)
{
    Pylon::PylonInitialize();
}

void CameraBasler::assignThread(QThread *thread)
{
    moveToThread(thread);
    connect(this, SIGNAL(finished()), thread, SLOT(quit()));
    connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

bool CameraBasler::isAvailable() { return camera->IsGrabbing(); }

void CameraBasler::close() {
    try
    {
        if (connected){
            camera->StopGrabbing();
            camera->Close();
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
    }
    connected = false;
}

bool CameraBasler::initCamera(std::string camera_serial,int binning, bool trigger, int fps, int packet_size)
{
    this->camera_serial = camera_serial;
    this->binning = binning;
    this->trigger = trigger;
    this->fps = fps;
    this->packet_size = packet_size;
    connected = false;
    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();

        Pylon::DeviceInfoList_t devices;
        tlFactory.EnumerateDevices(devices);

        Pylon::CInstantCameraArray cameras(devices.size());
        Pylon::CDeviceInfo info;

        bool cameraFind = false;
        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach(tlFactory.CreateDevice(devices[i]));

            if (cameras[i].GetDeviceInfo().GetSerialNumber() == camera_serial.c_str())
            {
                info = cameras[i].GetDeviceInfo();
                cameraFind = true;
                break;
            }
        }

        if (!cameraFind){
            std::cerr << "Failed to find camera with name: " << camera_serial << std::endl;
            return false;
        }

        camera_device = Pylon::CTlFactory::GetInstance().CreateDevice(info);
        camera = new Pylon::CInstantCamera(camera_device);

        setBinning(binning);
        setTrigger(trigger);
        setPacketSize(packet_size);
        changeFPS(fps);

        // Print the model name of the camera.
        std::cout << "Using device " << camera->GetDeviceInfo().GetModelName() << std::endl;

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 16.
        camera->MaxNumBuffer = 16;
        // The size of the output queue can be adjusted.
        // When using this strategy the OutputQueueSize parameter can be changed during grabbing.
        //camera->OutputQueueSize = camera->MaxNumBuffer.GetValue();

        formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;
        formatConverter.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera->StartGrabbing();

        if (camera->IsGrabbing())
        {
            Pylon::CGrabResultPtr ptrGrabResult;
            camera->RetrieveResult(max_timeout, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            if (ptrGrabResult != NULL){
                if (ptrGrabResult->GrabSucceeded())
                {
                    connected = true;
                    return true;
                }
            } else {
                qDebug() << "Camera grab pointer is null";
            }
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        return false;
    }
    return false;
}

void CameraBasler::getImageSize(int &image_width, int &image_height,
                                cv::Size &image_size)
{
    try
    {
        camera->Open();
        image_width = Pylon::CIntegerParameter(camera->GetNodeMap(), "Width").GetValue();
        image_height = Pylon::CIntegerParameter(camera->GetNodeMap(), "Height").GetValue();
        image_size = cv::Size(image_height,image_width);
        //camera->Close();
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred whilst getting camera image size." << std::endl
                  << e.GetDescription() << std::endl;
    }
}

bool CameraBasler::setPTP(bool enable){
    try
    {
        camera->Open();
        Pylon::CFloatParameter(camera->GetNodeMap(), "GevIEEE1588").SetValue(enable);
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

bool CameraBasler::setMaximumResolution(void)
{
    //TODO: set resolution
    return true;
}

bool CameraBasler::setFrame16(void)
{
    //TODO: set camera to 16bit
    return true;
}

bool CameraBasler::setFrame8(void)
{
    //TODO: set camera to 8bit
    return true;
}

bool CameraBasler::changeFPS(int fps){
    bool res = true;
    // unlimited frame rate if set to 0
    if (fps>0){
        res &= setFPS(fps);
        res &= enableFPS(true);
        this->fps = fps;
    } else {
        res &= enableFPS(false);
        this->fps = fps;
    }
    return res;
}

bool CameraBasler::setFPS(int fps){
    try
    {
        float fps_f = (float)fps;
        camera->Open();
        Pylon::CFloatParameter(camera->GetNodeMap(), "AcquisitionFrameRateAbs").SetValue(fps_f);
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

bool CameraBasler::enableFPS(bool enable){
    try
    {
        camera->Open();
        Pylon::CBooleanParameter(camera->GetNodeMap(), "AcquisitionFrameRateEnable").SetValue(enable);
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

bool CameraBasler::enableTrigger(bool enable){
    close();
    return (initCamera(this->camera_serial,this->binning,enable,this->fps,this->packet_size));
}

bool CameraBasler::changeBinning(int binning){
    close();
    return (initCamera(this->camera_serial,binning,this->trigger,this->fps,this->packet_size));
}

bool CameraBasler::changePacketSize(int packetSize){
    close();
    return (initCamera(this->camera_serial,binning,this->trigger,this->fps,packetSize));
}

bool CameraBasler::setTrigger(bool enable){
    bool res = false;
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "On";
        }
        camera->Open();
        Pylon::CEnumParameter(camera->GetNodeMap(), "TriggerMode").FromString(enable_str.c_str());
        res = true;
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        res = false;
    }

    return res;
}

bool CameraBasler::setPacketSize(int packetSize)
{
    try
    {
        camera->Open();
        Pylon::CIntegerParameter(camera->GetNodeMap(), "GevSCPSPacketSize").SetValue(packetSize);
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

bool CameraBasler::setInterPacketDelay(int interPacketDelay)
{
    try
    {
        // convert from seconds to milliseconds
        camera->Open();
        Pylon::CIntegerParameter(camera->GetNodeMap(), "GevSCPD").SetValue(interPacketDelay);
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

bool CameraBasler::setBinning(int binning)
{
    try
    {
        // convert from seconds to milliseconds
        camera->Open();
        Pylon::CEnumParameter(camera->GetNodeMap(), "BinningHorizontalMode").FromString("Average");
        Pylon::CIntegerParameter(camera->GetNodeMap(), "BinningHorizontal").SetValue(binning);
        Pylon::CEnumParameter(camera->GetNodeMap(), "BinningVerticalMode").FromString("Average");
        Pylon::CIntegerParameter(camera->GetNodeMap(), "BinningVertical").SetValue(binning);
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

bool CameraBasler::setExposure(double exposure)
{
    try
    {
        // convert from seconds to milliseconds
        int exposure_i = exposure * 10000;
        qDebug() << "Setting exposure..." << exposure_i;
        camera->Open();
        Pylon::CIntegerParameter(camera->GetNodeMap(), "ExposureTimeRaw").SetValue(exposure_i);
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

bool CameraBasler::setAutoExposure(bool enable)
{
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "Continuous";
        }
        camera->Open();
        Pylon::CEnumParameter(camera->GetNodeMap(), "ExposureAuto").FromString(enable_str.c_str());
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

bool CameraBasler::setAutoGain(bool enable)
{
    try
    {
        std::string enable_str = "Off";
        if (enable){
            enable_str = "Continuous";
        }
        camera->Open();
        Pylon::CEnumParameter(camera->GetNodeMap(), "GainAuto").FromString(enable_str.c_str());
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

bool CameraBasler::setGain(int gain)
{
    try
    {
        int gain_i = gain;
        camera->Open();
        Pylon::CIntegerParameter(camera->GetNodeMap(), "GainRaw").SetValue(gain_i);
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

bool CameraBasler::capture(void)
{
    bool res = false;
    try
    {
        if (connected){
            if (camera->IsGrabbing())
            {
                Pylon::CGrabResultPtr ptrGrabResult;
                camera->RetrieveResult(max_timeout, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
                if (ptrGrabResult == NULL){
                    qDebug() << "Camera grab pointer is null";
                    res = false;
                } else {
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        int frameCols = ptrGrabResult->GetWidth();
                        int frameRows = ptrGrabResult->GetHeight();

                        if (frameCols == 0 || frameRows == 0){
                            qDebug() << "Image buffer size is incorrect";
                            qDebug() << "cols: " << frameCols << " rows: " << frameRows;
                            res = false;
                        } else {
                            Pylon::CPylonImage pylonImage;
                            formatConverter.Convert(pylonImage, ptrGrabResult);
                            cv::Mat image_temp = cv::Mat(frameRows, frameCols, CV_8UC1, static_cast<uchar *>(pylonImage.GetBuffer()));
                            image_temp.copyTo(image);
                            if (image.cols == 0 || image.rows == 0){
                                qDebug() << "Image result buffer size is incorrect";
                                qDebug() << "cols: " << image.cols << " rows: " << image.rows;
                                res = false;
                            } else {
                                res = true;
                            }
                            pylonImage.Release();
                        }
                    }
                    else
                    {
                        qDebug() << "Failed to grab image.";
                        qDebug() << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();
                        std::cerr << "Failed to grab image." << std::endl;
                        std::cerr << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
                        res = false;
                    }
                    ptrGrabResult.Release();
                }
            }
            else
            {
                qDebug() << "Camera isn't grabbing images.";
                std::cerr << "Camera isn't grabbing images." << std::endl;
                res = false;
            }
        } else {
            qDebug() << "Camera is null";
            std::cerr << "Camera is null" << std::endl;
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
    }
    emit captured();
    return res;
}

cv::Mat *CameraBasler::getImage(void) {
    if (image.cols == 0 || image.rows == 0){
        qDebug() << "Global image result buffer size is incorrect";
        qDebug() << "cols: " << image.cols << " rows: " << image.rows;
    }
    return &image;
}

CameraBasler::~CameraBasler(void)
{
    close();
    Pylon::PylonTerminate();
}
