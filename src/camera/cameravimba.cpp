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

    qDebug() << "Closing camera";

    stopCapture();
    camera->Close();

    connected = false;
}

bool CameraVimba::initCamera(std::string camera_serial,int binning, bool trigger, int fps)
{
    this->camera_serial = camera_serial;
    this->binning = binning;
    this->trigger = trigger;
    this->fps = fps;
    connected = false;

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

        bool success = changeFPS(fps);
        if(!success)
            qDebug() << "Failed to set FPS";

        // Currently binning is not supported
        //bool error = setBinning(binning);
        success = enableTrigger(trigger);
        if(!success)
            qDebug() << "Failed to set trigger mode";

        frame_observer = shared_ptr<FrameObserver>(new FrameObserver(camera));
        connected = true;

    } else {
        qDebug() << "Failed to open camera";
        connected = false;
    }

    return connected;
}

void CameraVimba::startCapture(void){

    if(capturing) return;

    auto res = camera->StartContinuousImageAcquisition(15, frame_observer);

    if(res != VmbErrorSuccess){
        qDebug() << "Couldn't start capture!" << res;
    }else{
        connect(frame_observer.get(), SIGNAL( frameReady(int) ), this, SLOT( onFrame(int) ) );
        capturing = true;
    }
}

void CameraVimba::stopCapture(void){
    if(!capturing) return;

    camera->StopContinuousImageAcquisition();
    disconnect(frame_observer.get(), SIGNAL( frameReady(int) ), this, SLOT( onFrame(int) ) );
    capturing = false;
}

void CameraVimba::onFrame(int status){
    if(!connected){
        return;
     }

    // Pick up frame
    FramePtr pFrame = frame_observer->getFrame();
    if( pFrame == nullptr){
        qDebug() << "frame pointer is NULL, late frame ready message";
        return;
    }

    // See if it is not corrupt
    if( status == VmbFrameStatusComplete ){
        VmbUchar_t *pBuffer;
        VmbErrorType err = pFrame.get()->GetImage(pBuffer);
        if( VmbErrorSuccess != err )
        {
            qDebug() << "Failed to get image pointer";
            return;
        }

        VmbPixelFormatType ePixelFormat = VmbPixelFormatMono8;
        err = pFrame->GetPixelFormat( ePixelFormat );
        if ( err != VmbErrorSuccess)
        {
            qDebug() << "Failed to get pixel format";
            return;
        }

        if (( ePixelFormat != VmbPixelFormatMono8 ))
        {
            qDebug() << "Invalid pixel format";
            return;
        }

        VmbUint32_t nImageSize = 0;
        err = pFrame->GetImageSize( nImageSize );
        if ( err != VmbErrorSuccess )
        {
            qDebug() << "Failed to get image size";
            return;
        }

        VmbUint32_t nWidth = 0;
        err = pFrame->GetWidth( nWidth );
        if ( err != VmbErrorSuccess ){
            qDebug() << "Failed to get image width";
            return;
        }

        VmbUint32_t nHeight = 0;
        err = pFrame->GetHeight( nHeight );
        if ( err != VmbErrorSuccess )
        {
            qDebug() << "Failed to get image height";
            return;
        }

        VmbUchar_t *pImage = nullptr;
        err = pFrame->GetImage( pImage );
        if ( err == VmbErrorSuccess)
        {
            frame_mutex.lock();
            cv::Mat image_temp = cv::Mat(static_cast<int>(nHeight), static_cast<int>(nWidth), CV_8UC1, pImage );
            image_temp.copyTo(image);
            frame_mutex.unlock();

        }

        camera->QueueFrame(pFrame);
    }

    emit captured();
}

VmbError_t CameraVimba::getStringFeature(std::string name, std::string &res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return error;
}

VmbError_t CameraVimba::getIntFeature(std::string name, long long &res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         VmbInt64_t out;
         feature->GetValue(out);
         res = static_cast<int>(out);
    }

    return error;
}

VmbError_t CameraVimba::getBoolFeature(std::string name, bool &res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return error;
}

VmbError_t CameraVimba::getDoubleFeature(std::string name, double &res){

    FeaturePtr feature;
    auto error = camera->GetFeatureByName(name.c_str(), feature);
    if(error == VmbErrorSuccess){
         feature->GetValue(res);
    }

    return error;
}

void CameraVimba::getImageSize(int &width, int &height, int &bitdepth)
{
    //get image size
    long long h = 0, w = 0;
    auto err_h = getIntFeature("Height", h);
    auto err_w = getIntFeature("Width", w);

    std::string format;
    auto err_fmt = getStringFeature("PixelSize", format);

    if (err_h == VmbErrorSuccess && err_w == VmbErrorSuccess && err_fmt == VmbErrorSuccess){
        height = static_cast<int>(h);
        width = static_cast<int>(w);
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
    bool capture_state = capturing;

    if(capture_state){
        stopCapture();
    }
    // unlimited frame rate if set to 0
    if (fps > 0){
        // Order is important!
        res &= enableFPS(true);
        if(!res)
            qDebug() << "Failed to enable FPS";
        res &= setFPS(fps);
        if(!res)
            qDebug() << "Failed to set FPS";

        this->fps = fps;
    } else {
        res &= enableFPS(false);
        this->fps = fps;
    }
    if(capture_state){
        startCapture();
    }
    return res;
}

bool CameraVimba::setFPS(int fps){
    FeaturePtr feature;
    qDebug() << "Setting fps: " << fps;
    camera->GetFeatureByName("AcquisitionFrameRate", feature);
    auto error = feature->SetValue(static_cast<double>(fps));
    return error == VmbErrorSuccess;
}

bool CameraVimba::enableFPS(bool enable){
    if (enable){
        qDebug() << "Enabling fps";
    } else {
        qDebug() << "Disabling fps";
    }

    FeaturePtr feature;
    camera->GetFeatureByName("AcquisitionFrameRateEnable", feature);
    auto error = feature->SetValue(enable);

    bool state;
    feature->GetValue(state);

    return (error == VmbErrorSuccess) && (state == enable);
}

bool CameraVimba::enableTrigger(bool enable){
    FeaturePtr feature;
    VmbError_t error;
    std::string state;
    std::string requested_trigger_mode;
    std::string requested_line_selector;
    std::string requested_line_mode;
    std::string requested_trigger_source;
    std::string requested_trigger_selector;
    bool success = true;

    if(enable){
        qDebug() << "Enabling trigger";
        requested_trigger_mode = "On";
        requested_trigger_source = "Line0";
        requested_trigger_selector = "FrameStart";
        requested_line_selector = "Line0";
        requested_line_mode = "Input";
    } else {
        qDebug() << "Disabling trigger";
        requested_trigger_mode = "Off";
        requested_trigger_source = "Software";
        requested_trigger_selector = "FrameStart";
        requested_line_selector = "Line0";
        requested_line_mode = "Input";
    }

    if (enable){
        /*
        camera->GetFeatureByName("LineSelector", feature);
        error = feature->SetValue(requested_line_selector.c_str());
        feature->GetValue(state);
        success &= (error == VmbErrorSuccess) && (state == requested_line_selector);

        if (!success){
            qDebug() << "Failed to set line selector";
            return success;
        }

        camera->GetFeatureByName("LineMode", feature);
        error = feature->SetValue(requested_line_mode.c_str());
        feature->GetValue(state);
        success &= (error == VmbErrorSuccess) && (state == requested_line_mode);

        if (!success){
            qDebug() << "Failed to set line mode";
            return success;
        }
        */
    }

    camera->GetFeatureByName("TriggerSelector", feature);
    error = feature->SetValue(requested_trigger_selector.c_str());
    feature->GetValue(state);
    success &= (error == VmbErrorSuccess) && (state == requested_trigger_selector);

    if (!success){
        qDebug() << "Failed to set trigger selector";
        return success;
    }

    camera->GetFeatureByName("TriggerSource", feature);
    error = feature->SetValue(requested_trigger_source.c_str());
    feature->GetValue(state);
    success &= (error == VmbErrorSuccess) && (state == requested_trigger_source);

    if (!success){
        qDebug() << "Failed to set trigger source";
        return success;
    }

    camera->GetFeatureByName("TriggerMode", feature);
    error = feature->SetValue(requested_trigger_mode.c_str());
    feature->GetValue(state);
    success &= (error == VmbErrorSuccess) && (state == requested_trigger_mode);

    if (!success){
        qDebug() << "Failed to set trigger mode";
        qDebug() << state.c_str();
        return success;
    }

    return success;
}

bool CameraVimba::changeBinning(int binning){
    close();
    return (initCamera(this->camera_serial,binning,this->trigger,this->fps));
}

double CameraVimba::getFPS(void){
    FeaturePtr feature;
    double fps;
    auto success = getDoubleFeature("AcquisitionFrameRate", fps);
    if(success != VmbErrorSuccess){
        fps = -1;
    }
    return fps;
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
    int fps = static_cast<int>(getFPS());
    camera.get()->GetFeatureByName("ExposureTime", feature);
    double exposure_scaled = exposure * 1000;
    qDebug() << "Setting exposure to: " << exposure_scaled;
    qDebug() << "Vimba exposure to: " << exposure_scaled;
    auto error = feature->SetValue(exposure_scaled); // microseconds

    if(error != VmbErrorSuccess){
        qDebug() << "Failed to set exposure";
    }else{
        /*
        if(fps < 30 && 1.0/(1000.0*exposure) < 1.0/30){
            changeFPS(30);
        }
        */
    }

    return error == VmbErrorSuccess;
}

bool CameraVimba::enableAutoExposure(bool enable)
{
    std::string exposure_mode = "Off";
    if(enable) exposure_mode = "Continuous";

    FeaturePtr feature;

    if (enable){
        qDebug() << "Enabling auto exposure";
    } else {
        qDebug() << "Disabling auto exposure";
    }

    camera->GetFeatureByName("ExposureAuto", feature);
    auto error = feature->SetValue(exposure_mode.c_str());

    return error == VmbErrorSuccess;
}

bool CameraVimba::enableAutoGain(bool enable)
{

    if (enable){
        qDebug() << "Enabling auto gain";
    } else {
        qDebug() << "Disabling auto gain";
    }

    std::string gain_mode = "Off";
    if(enable) gain_mode = "Continuous";

    FeaturePtr feature;
    camera->GetFeatureByName("GainAuto", feature);
    auto error = feature->SetValue(gain_mode.c_str());

    return error == VmbErrorSuccess;
}

bool CameraVimba::setGain(int gain)
{

    enableAutoGain(false);

    FeaturePtr feature;
    camera.get()->GetFeatureByName("Gain", feature);

    qDebug() << "Setting gain to: " << gain;

    double min_gain, max_gain;
    feature->GetRange(min_gain, max_gain);

    qDebug() << "Vimba gain to: " << max_gain*static_cast<double>(gain)/100.0;

    auto error = feature->SetValue(max_gain*static_cast<double>(gain)/100.0);

    return error == VmbErrorSuccess;
}

bool CameraVimba::capture(void)
{
    bool res = false;
    QElapsedTimer timer;
    timer.start();

    if (connected){
        // get image from cameras
        AVT::VmbAPI::FramePtr pFrame;
        VmbErrorType capture_err;
        VmbFrameStatusType status = VmbFrameStatusIncomplete;

        //qDebug() << "Init: " << timer.nsecsElapsed() / 1e9;

        capture_err = camera->AcquireSingleImage( pFrame, 5000 );
        //timer.start();

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
            if(capture_err == VmbErrorTimeout && trigger){
                qDebug() << "Waiting for trigger?";
            }
            qDebug() << "Failed to acquire image";
            res = false;
        }
    } else {
        qDebug() << "Camera is not connected or is initalising";
        std::cerr << "Camera is not connected or is initalising" << std::endl;
        res = false;
    }

     qDebug() << "Capture time: " << timer.nsecsElapsed() / 1e9;

    return res;
}

bool CameraVimba::getImage(cv::Mat &out) {
    cv::Mat temp;
    frame_mutex.lock();
    if (image.cols == 0 || image.rows == 0){
        qDebug() << "Global image result buffer size is incorrect";
        qDebug() << "cols: " << image.cols << " rows: " << image.rows;
        return false;
    }
    image.copyTo(out);
    frame_mutex.unlock();
    return true;
}

CameraVimba::~CameraVimba(void)
{
    close();
    emit finished();
}
