/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocameratis.h"

bool StereoCameraTIS::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    auto left_camera_tmp = new CameraImagingSource();
    auto right_camera_tmp = new CameraImagingSource();

    std::string left_serial = stereoCameraSerialInfo_.left_camera_serial;
    std::string right_serial = stereoCameraSerialInfo_.right_camera_serial;

    if (left_camera_tmp->open(left_serial) && right_camera_tmp->open(right_serial)) {
        connected = setCameras(stereoCameraSettings_,left_camera_tmp,right_camera_tmp,new Listener(), new Listener());
    } else {
        connected = false;
    }
    return connected;
}

bool StereoCameraTIS::closeCamera(){
    if (connected){
        disconnect((Listener*) left_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(leftCaptured()));
        disconnect((Listener*) right_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(rightCaptured()));

        disconnect(left_camera, SIGNAL(grabError()), this, SLOT(leftGrabFailed()));
        disconnect(right_camera, SIGNAL(grabError()), this, SLOT(rightGrabFailed()));

        disconnect(left_listener, SIGNAL(deviceDisconnected(void)), this, SLOT(closeCamera(void)));
        disconnect(right_listener, SIGNAL(deviceDisconnected(void)), this, SLOT(closeCamera(void)));

        disconnect(this, SIGNAL(start_capture(void)), left_camera, SLOT(startCapture(void)));
        disconnect(this, SIGNAL(start_capture(void)), right_camera, SLOT(startCapture(void)));

        disconnect(this, SIGNAL(stop_capture(void)), left_camera, SLOT(stopCapture(void)));
        disconnect(this, SIGNAL(stop_capture(void)), right_camera, SLOT(stopCapture(void)));

        disconnect(this, SIGNAL(stereo_grab(void)), left_camera, SLOT(grabImage(void)));
        disconnect(this, SIGNAL(stereo_grab(void)), right_camera, SLOT(grabImage(void)));

        left_camera->close();
        right_camera->close();
    }
    connected = false;

    emit disconnected();
    return false;
}

bool StereoCameraTIS::captureSingle(){
    if (!capturing){
        left_camera->startCapture();
        right_camera->startCapture();
        left_camera->grabSingle();
        right_camera->grabSingle();
    } else {
        emit stereo_grab();
    }
    return true;
}

void StereoCameraTIS::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraTIS::captureSingle);
}

bool StereoCameraTIS::enableCapture(bool enable){
    if (enable){
        //Start capture thread
        emit start_capture();
        connect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = true;
        captureThreaded();
    } else {
        //Stop capture thread
        disconnect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = false;
        emit stop_capture();
    }
    return true;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraTIS::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(AbstractStereoCamera::CAMERA_TYPE_TIS);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;
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

bool StereoCameraTIS::setCameras(AbstractStereoCamera::StereoCameraSettings inital_camera_settings, CameraImagingSource *camera_left, CameraImagingSource *camera_right, Listener *listener_left, Listener *listener_right){
    this->left_camera = camera_left;
    this->right_camera = camera_right;
    this->left_listener = listener_left;
    this->right_listener = listener_right;

    setup_cameras(inital_camera_settings);

    return true;
}

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

void StereoCameraTIS::setup_cameras(AbstractStereoCamera::StereoCameraSettings inital_camera_settings){

    QThread* left_camera_thread = new QThread();
    left_camera->assignThread(left_camera_thread);

    QThread* right_camera_thread = new QThread();
    right_camera->assignThread(right_camera_thread);

    image_height = left_camera->height;
    image_width = left_camera->width;
    image_bitdepth = 1; //TODO get bit depth

    emit update_size(image_width, image_height, image_bitdepth);

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

    left_raw.create(getSize(), CV_8UC1);
    right_raw.create(getSize(), CV_8UC1);

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

    connect(left_listener, SIGNAL(deviceDisconnected(void)), this, SLOT(closeCamera(void)));
    connect(right_listener, SIGNAL(deviceDisconnected(void)), this, SLOT(closeCamera(void)));

    connect(this, SIGNAL(start_capture(void)), left_camera, SLOT(startCapture(void)));
    connect(this, SIGNAL(start_capture(void)), right_camera, SLOT(startCapture(void)));

    connect(this, SIGNAL(stop_capture(void)), left_camera, SLOT(stopCapture(void)));
    connect(this, SIGNAL(stop_capture(void)), right_camera, SLOT(stopCapture(void)));

    connect(this, SIGNAL(stereo_grab(void)), left_camera, SLOT(grabImage(void)));
    connect(this, SIGNAL(stereo_grab(void)), right_camera, SLOT(grabImage(void)));

    enableTrigger(trigger);
    enableAutoExposure(autoExpose);
    setExposure(exposure);
    enableAutoGain(autoGain);
    setGain(gain);

    connected = true;
}

void StereoCameraTIS::leftGrabFailed(){
    grab_finish_l = true;
    grab_success_l = false;
    checkStereoCapture();
}

void StereoCameraTIS::rightGrabFailed(){
    grab_finish_r = true;
    grab_success_r = false;
    checkStereoCapture();
}

void StereoCameraTIS::leftCaptured(){
    grab_finish_l = true;
    grab_success_l = true;
    checkStereoCapture();
}

void StereoCameraTIS::rightCaptured(){
    grab_finish_r = true;
    grab_success_r = true;
    checkStereoCapture();
}

void StereoCameraTIS::checkStereoCapture(){
    if (grab_finish_l && grab_finish_r){
        if (grab_success_r && grab_success_l){
            emit captured_success();
        } else {
            emit captured_fail();
            send_error(CAPTURE_ERROR);
        }
        grab_finish_r = false;
        grab_finish_l = false;
        grab_success_r = false;
        grab_success_l = false;
        emit captured();
    }
}

void StereoCameraTIS::loadLeftSettings(){
    left_camera->showProperties();
}

void StereoCameraTIS::loadRightSettings(){
    right_camera->showProperties();
}

bool StereoCameraTIS::enableTrigger(bool enable){
    left_camera->setTrigger(enable);
    right_camera->setTrigger(enable);
    return true;
}

bool StereoCameraTIS::enableAutoExposure(bool enable) {
    left_camera->enableAutoExposure(enable);
    right_camera->enableAutoExposure(enable);
    return true;
}

bool StereoCameraTIS::enableAutoGain(bool enable){
    left_camera->enableAutoGain(enable);
    right_camera->enableAutoGain(enable);
    return true;
}

bool StereoCameraTIS::setGain(int gain) {
    left_camera->changeGain(gain);
    right_camera->changeGain(gain);
    return true;
}

bool StereoCameraTIS::setExposure(double exposure) {
    left_camera->changeExposure(exposure);
    right_camera->changeExposure(exposure);
    return true;
}

bool StereoCameraTIS::setFPS(int fps){
    if (!isCapturing()){
        double fps_d = fps;
        left_camera->setFrameRate(fps_d);
        right_camera->setFrameRate(fps_d);
        frame_rate = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

StereoCameraTIS::~StereoCameraTIS(void) {
    if (connected){
        closeCamera();
    }
}
