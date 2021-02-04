/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocamerafromimage.h"

bool StereoCameraFromImage::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    int fps = stereoCameraSettings_.fps;
    std::string fname_l = stereoCameraSerialInfo_.left_camera_serial;
    std::string fname_r = stereoCameraSerialInfo_.right_camera_serial;

    left_raw = cv::imread(fname_l,cv::IMREAD_COLOR);
    right_raw = cv::imread(fname_r,cv::IMREAD_COLOR);

    //TODO check image files exist

    image_height = left_raw.size().height;
    image_width = left_raw.size().width;
    image_bitdepth = 1; //TODO get bit depth
    emit update_size(image_width, image_height, image_bitdepth);

    setFPS(fps);

    connected = true;
    return connected;
}

bool StereoCameraFromImage::closeCamera(){
    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraFromImage::captureSingle(){

    // Simulate frame rate
    double delay_needed = (1000.0/(video_fps+1)) - frame_timer.elapsed();
    if(delay_needed > 0){
        QThread::msleep(delay_needed);
    }

    frame_timer.restart();

    //send_error(CAPTURE_ERROR);
    //emit captured_fail();
    emit captured_success();
    emit captured();
    return true;
}

void StereoCameraFromImage::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraFromImage::captureSingle);
}

bool StereoCameraFromImage::enableCapture(bool enable){
    if (enable){
        //Start capture thread
        connect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = true;
        captureThreaded();
    } else {
        //Stop capture thread
        disconnect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = false;
    }
    return true;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraFromImage::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;
    return connected_serial_infos;
}

bool StereoCameraFromImage::setFPS(int fps){
    if (!isCapturing()){
        frame_rate = fps;
        video_fps = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

StereoCameraFromImage::~StereoCameraFromImage(void) {
    if (connected){
        closeCamera();
    }
}
