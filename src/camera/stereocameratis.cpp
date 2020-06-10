#include "stereocameratis.h"

bool StereoCameraTIS::initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings){
    //TODO use inital camera settings input

    this->camera_serial_info = camera_serial_info;
    auto left_camera_tmp = new CameraImagingSource();
    auto right_camera_tmp = new CameraImagingSource();

    std::string left_serial = camera_serial_info.left_camera_serial;
    std::string right_serial = camera_serial_info.right_camera_serial;

    if (left_camera_tmp->open(left_serial) && right_camera_tmp->open(right_serial)) {
        return setCameras(inital_camera_settings,left_camera_tmp,right_camera_tmp,new Listener(), new Listener());
    } else {
        return false;
    }
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraTIS::listSystems(){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TIS);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;
    /* Get number of cameras */
    DShowLib::InitLibrary();

    DShowLib::Grabber handle;

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

/*
bool StereoCameraTIS::autoConnect(){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> camera_serials = listSystems();

    if (camera_serials.size() > 0){

        auto left_camera_tmp = new CameraImagingSource();
        auto right_camera_tmp = new CameraImagingSource();

        std::string left_serial = camera_serials.at(0).left_camera_serial;
        std::string right_serial = camera_serials.at(0).right_camera_serial;

        if (left_camera_tmp->open(left_serial) && right_camera_tmp->open(right_serial)) {
            assert(left_camera_tmp->getSerial() == left_serial);
            assert(right_camera_tmp->getSerial() == right_serial);

            setCameras(left_camera_tmp, right_camera_tmp,new Listener(),new Listener());
            return true;
        }
    }
    return false;
}
*/

QList<qint64> StereoCameraTIS::load_serials(QString filename) {
    QList<qint64> serials;

    QFile inputFile(filename);
    if (inputFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputFile);
        while (!in.atEnd()) {
            QStringList line = in.readLine().split(' ');

            if(line.size() == 3){
                qint64 serial = line.at(0).toLongLong();
                int width = line.at(1).toInt();
                int height = line.at(2).toInt();

                if (serial) {
                    serials.append(serial);
                    widths.append(width);
                    heights.append(height);
                    qDebug() << "Looking for Phobos camera serial " << (qint64)serial;
                }
            }
        }
        inputFile.close();
    } else {
        qDebug() << "Couldn't open Phobos camera serials file " << filename;
    }

    return serials;
}

bool StereoCameraTIS::setCameras(AbstractStereoCamera::stereoCameraSettings inital_camera_settings, CameraImagingSource *camera_left, CameraImagingSource *camera_right, Listener *listener_left, Listener *listener_right){
    this->left_camera = camera_left;
    this->right_camera = camera_right;
    this->left_listener = listener_left;
    this->right_listener = listener_right;

    setup_cameras(inital_camera_settings);

    return true;
}

void StereoCameraTIS::setup_cameras(AbstractStereoCamera::stereoCameraSettings inital_camera_settings){

    QThread* left_camera_thread = new QThread();
    left_camera->assignThread(left_camera_thread);

    QThread* right_camera_thread = new QThread();
    right_camera->assignThread(right_camera_thread);

    image_height = left_camera->height;
    image_width = left_camera->width;

    image_size = cv::Size(image_width, image_height);
    emit update_size(image_width, image_height, 1);

    double exposure = inital_camera_settings.exposure;
    int gain = inital_camera_settings.gain;
    int fps = inital_camera_settings.fps;
    bool trigger;
    if (inital_camera_settings.trigger == 1){
        trigger = true;
    } else {
        trigger = false;
    }
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

    left_camera->setup(image_width, image_height, fps);
    right_camera->setup(image_width, image_height, fps);

    left_raw.create(image_size, CV_8UC1);
    right_raw.create(image_size, CV_8UC1);

    left_listener->setOutputBuffer(left_raw.data);
    right_listener->setOutputBuffer(right_raw.data);

    left_camera->setListener((DShowLib::GrabberListener*)  left_listener);
    right_camera->setListener((DShowLib::GrabberListener*) right_listener);

    assert(left_camera->getListener() != nullptr);
    assert(right_camera->getListener() != nullptr);

    connect((Listener*) left_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(leftCaptured()));
    connect((Listener*) right_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(rightCaptured()));

    connect(left_camera, SIGNAL(grabError()), this, SLOT(leftGrabFailed()));
    connect(right_camera, SIGNAL(grabError()), this, SLOT(rightGrabFailed()));

    connect(this, SIGNAL(start_capture(void)), left_camera, SLOT(startCapture(void)));
    connect(this, SIGNAL(start_capture(void)), right_camera, SLOT(startCapture(void)));

    connect(this, SIGNAL(stereo_grab(void)), left_camera, SLOT(grabImage(void)));
    connect(this, SIGNAL(stereo_grab(void)), right_camera, SLOT(grabImage(void)));

    left_camera->setTrigger(trigger);
    right_camera->setTrigger(trigger);

    left_camera->enableAutoExposure(autoExpose);
    right_camera->enableAutoExposure(autoExpose);

    left_camera->changeExposure(exposure);
    right_camera->changeExposure(exposure);

    left_camera->enableAutoGain(autoGain);
    right_camera->enableAutoGain(autoGain);

    left_camera->changeGain(gain);
    right_camera->changeGain(gain);

    connected = true;

    emit start_capture();
}

void StereoCameraTIS::leftGrabFailed(){
    grab_success_l = false;
    //connected = false;
    emit register_left_capture();
    //disconnectCamera();
}

void StereoCameraTIS::rightGrabFailed(){
    grab_success_r = false;
    //connected = false;
    emit register_right_capture();
    //disconnectCamera();
}

void StereoCameraTIS::leftCaptured(){
    grab_success_l = true;
    emit register_left_capture();
}

void StereoCameraTIS::rightCaptured(){
    grab_success_r = true;
    emit register_right_capture();
}

void StereoCameraTIS::loadLeftSettings(){
    left_camera->showProperties();
}

void StereoCameraTIS::loadRightSettings(){
    right_camera->showProperties();
}

void StereoCameraTIS::enableTrigger(bool enable){
    if (connected){
        left_camera->setTrigger(enable);
        right_camera->setTrigger(enable);
    }
}

void StereoCameraTIS::changeFPS(int fps){
    left_camera->setFrameRate((double)fps);
    right_camera->setFrameRate((double)fps);
}

void StereoCameraTIS::toggleTrigger(bool enable){
    enableTrigger(enable);
}
void StereoCameraTIS::adjustFPS(int fps){
    changeFPS(fps);
}

void StereoCameraTIS::toggleAutoExpose(bool enable){
    enableAutoExpose(enable);
}

void StereoCameraTIS::enableAutoExpose(bool enable){
    left_camera->enableAutoExposure(enable);
    right_camera->enableAutoExposure(enable);
}

void StereoCameraTIS::setExposure(double exposure){
    left_camera->changeExposure(exposure);
    right_camera->changeExposure(exposure);
}

void StereoCameraTIS::adjustExposure(double exposure){
    setExposure(exposure);
}

void StereoCameraTIS::toggleAutoGain(bool enable){
    enableAutoGain(enable);
}

void StereoCameraTIS::adjustGain(int gain){
    setGain(gain);
}

void StereoCameraTIS::setGain(int gain) {
    left_camera->changeGain(gain);
    right_camera->changeGain(gain);
}

void StereoCameraTIS::enableAutoGain(bool enable){
    left_camera->enableAutoGain(enable);
    right_camera->enableAutoGain(enable);
}

bool StereoCameraTIS::capture(){

    frametimer.restart();

    capturing = true;
    captured_stereo = false;

    /* Asynchronous grab */
    emit stereo_grab();

    capturing = false;

    if (!grab_success_l || !grab_success_r){
        emit register_right_capture();
        emit register_left_capture();
    }

    return grab_success_l && grab_success_r;
}

void StereoCameraTIS::disconnectCamera(){
    if (connected){
        disconnect((Listener*) left_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(leftCaptured()));
        disconnect((Listener*) right_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(rightCaptured()));

        disconnect(left_camera, SIGNAL(grabError()), this, SLOT(leftGrabFailed()));
        disconnect(right_camera, SIGNAL(grabError()), this, SLOT(rightGrabFailed()));

        disconnect(this, SIGNAL(start_capture(void)), left_camera, SLOT(startCapture(void)));
        disconnect(this, SIGNAL(start_capture(void)), right_camera, SLOT(startCapture(void)));

        disconnect(this, SIGNAL(stereo_grab(void)), left_camera, SLOT(grabImage(void)));
        disconnect(this, SIGNAL(stereo_grab(void)), right_camera, SLOT(grabImage(void)));

        //disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));

        left_camera->close();
        right_camera->close();
    }
    connected = false;

    emit disconnected();
}

StereoCameraTIS::~StereoCameraTIS(){
}
