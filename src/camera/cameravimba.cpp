#include "cameravimba.h"

CameraVimba::CameraVimba(QObject *parent) : QObject(parent), system(VimbaSystem::GetInstance())
{
}


void CameraVimba::assignThread(QThread *thread)
{
    moveToThread(thread);
    connect(this, SIGNAL(finished()), thread, SLOT(quit()));
    connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void CameraVimba::close() {

    if (connected){
        camera->EndCapture();
        camera->Close();
    }

    connected = false;
}

bool CameraVimba::initCamera(std::string camera_serial,int binning, bool trigger, int fps)
{
    this->camera_serial = camera_serial;
    this->binning = binning;
    this->trigger = trigger;
    this->fps = fps;
    connected = false;

    // Create an instant camera object with the camera device found first.
    CameraPtrVector cameras;
    system.GetCameras(cameras);

    bool cameraFind = false;
    for (CameraPtr trial_camera : cameras){
        std::string serial;
        trial_camera->GetSerialNumber(serial);

        if(serial == camera_serial){
            cameraFind = true;
            camera = trial_camera;
            break;
        }
    }

    if (!cameraFind){
        std::cerr << "Failed to find camera with name: " << camera_serial << std::endl;
        return false;
    }

    // Print the model name of the camera.
    std::string model;
    camera->GetModel(model);
    qDebug() << "Using device " << model.c_str();

    auto error = camera->Open(VmbAccessModeFull);

    if (error == VmbErrorSuccess){

        //setBinning(binning);
        //enableTrigger(trigger);
        //changeFPS(fps);

        qDebug() << "Attempting to start capture";

        auto capture_status = camera->StartCapture();

        if(capture_status == VmbErrorSuccess){
            connected = true;
        }else{
            if(capture_status == VmbErrorInvalidAccess){
                qDebug() << "Access error";
                camera->Close();
            }
            qDebug() << "Failed to start capture";
        }

    } else {
        qDebug() << "Failed to open camera";
        connected = false;
    }

    return connected;
}

VmbError_t CameraVimba::getStringFeature(std::string name, std::string res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return error;
}

VmbError_t CameraVimba::getIntFeature(std::string name, VmbInt64_t res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return error;
}

VmbError_t CameraVimba::getBoolFeature(std::string name, bool res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return res;
}

VmbError_t CameraVimba::getDoubleFeature(std::string name, double res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return res;
}

void CameraVimba::getImageSize(int &width, int &height, int &bitdepth)
{
    //get image size
    int h = 0, w = 0;
    auto err_h = getIntFeature("Height", h);
    auto err_w = getIntFeature("Width", w);

    std::string format;
    auto err_fmt = getStringFeature("PixelSize", format);

    if (err_h == VmbErrorSuccess && err_w == VmbErrorSuccess && err_fmt == VmbErrorSuccess){
        height = h;
        width = w;
        if(format == "Bpp8")
            bitdepth = 8;
        else if(format == "Bpp16")
            bitdepth = 16;
    } else {
        qDebug() << "Failed to get width / height";
    }
}

bool CameraVimba::changeFPS(int fps){
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

bool CameraVimba::setFPS(int fps){
    FeaturePtr feature;
    camera->GetFeatureByName("AcquisitionFrameRate", feature);
    auto error = feature->SetValue(static_cast<double>(fps));
    return error == VmbErrorSuccess;
}

bool CameraVimba::enableFPS(bool enable){
    FeaturePtr feature;
    camera->GetFeatureByName("AcquisitionFrameRateEnable", feature);
    auto error = feature->SetValue(enable);
    return error == VmbErrorSuccess;
}

bool CameraVimba::enableTrigger(bool enable){
    FeaturePtr feature;
    camera->GetFeatureByName("TriggerMode", feature);
    VmbError_t error;

    if(enable)
        error = feature->SetValue("On");
    else
        error = feature->SetValue("Off");

    return error == VmbErrorSuccess;
}

bool CameraVimba::changeBinning(int binning){
    close();
    return (initCamera(this->camera_serial,binning,this->trigger,this->fps));
}


bool CameraVimba::setBinning(int binning)
{
    FeaturePtr feature;
    bool success = true;
    camera->GetFeatureByName("BinningHorizontal", feature);
    // Camera doesn't support binning
    if(feature.get() == nullptr) return false;

    success &= (feature->SetValue(binning) == VmbErrorSuccess);
    camera->GetFeatureByName("BinningVertical", feature);
    success &= (feature->SetValue(binning) == VmbErrorSuccess);

    return success;
}

bool CameraVimba::setExposure(double exposure)
{
    FeaturePtr feature;
    camera.get()->GetFeatureByName("ExposureTime", feature);
    auto error = feature->SetValue(1000.0*exposure); // microseconds

    return error == VmbErrorSuccess;
}

bool CameraVimba::enableAutoExposure(bool enable)
{
    std::string exposure_mode = "Off";
    if(enable) exposure_mode = "Continuous";

    FeaturePtr feature;

    camera->GetFeatureByName("ExposureAuto", feature);
    auto error = feature->SetValue(exposure_mode.c_str());

    return error == VmbErrorSuccess;
}

bool CameraVimba::enableAutoGain(bool enable)
{

    std::string gain_mode = "Off";
    if(enable) gain_mode = "Continuous";

    FeaturePtr feature;
    camera->GetFeatureByName("GainAuto", feature);
    auto error = feature->SetValue(gain_mode.c_str());

    return error == VmbErrorSuccess;
}

bool CameraVimba::setGain(int gain)
{
    FeaturePtr feature;
    camera.get()->GetFeatureByName("Gain", feature);
    auto error = feature->SetValue(gain);

    return error == VmbErrorSuccess;
}

bool CameraVimba::capture(void)
{
    bool res = false;
    if (connected){
        // get image from cameras
        AVT::VmbAPI::FramePtr pFrame;
        VmbErrorType capture_err;
        VmbFrameStatusType status = VmbFrameStatusIncomplete;

        capture_err = camera->AcquireSingleImage( pFrame, 5000 );

        if ( capture_err == VmbErrorSuccess)
        {
            capture_err = pFrame->GetReceiveStatus( status );
            if (     capture_err == VmbErrorSuccess
                 && status == VmbFrameStatusComplete)
            {
                VmbPixelFormatType ePixelFormat = VmbPixelFormatMono8;
                capture_err = pFrame->GetPixelFormat( ePixelFormat );
                if ( capture_err == VmbErrorSuccess)
                {
                    if (    ( ePixelFormat != VmbPixelFormatMono8 ))
                    {
                        capture_err = VmbErrorInvalidValue;
                        qDebug() << "Invalid pixel format";
                        res = false;
                    }
                    else
                    {
                        VmbUint32_t nImageSize = 0;
                        capture_err = pFrame->GetImageSize( nImageSize );
                        if ( capture_err == VmbErrorSuccess )
                        {
                            VmbUint32_t nWidth = 0;
                            capture_err = pFrame->GetWidth( nWidth );
                            if ( capture_err == VmbErrorSuccess )
                            {
                                VmbUint32_t nHeight = 0;
                                capture_err = pFrame->GetHeight( nHeight );
                                if ( capture_err == VmbErrorSuccess )
                                {
                                    VmbUchar_t *pImage = nullptr;
                                    capture_err = pFrame->GetImage( pImage );
                                    if ( capture_err == VmbErrorSuccess)
                                    {
                                        cv::Mat image_temp = cv::Mat(nHeight, nWidth, CV_8UC1, pImage );
                                        image_temp.copyTo(image);
                                        res = true;
                                    }
                                }
                            }
                        }

                    }
                } else {
                    qDebug() << "Failed to get pixel format";
                    res = false;
                }
            } else {
                qDebug() << "Failed to acquire frame or incomplete";
                res = false;
            }
        } else {
            qDebug() << "Failed to acquire image";
            res = false;
        }
    } else {
        qDebug() << "Camera is not connected or is initalising";
        std::cerr << "Camera is not connected or is initalising" << std::endl;
        res = false;
    }

    return res;
}

cv::Mat *CameraVimba::getImage(void) {
    if (image.cols == 0 || image.rows == 0){
        qDebug() << "Global image result buffer size is incorrect";
        qDebug() << "cols: " << image.cols << " rows: " << image.rows;
    }
    return &image;
}

CameraVimba::~CameraVimba(void)
{
    close();
}
