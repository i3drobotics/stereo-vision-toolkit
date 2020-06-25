/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocamerafromvideo.h"

bool StereoCameraFromVideo::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    int fps = stereoCameraSettings_.fps;
    std::string fname = stereoCameraSerialInfo_.filename;

    stream = cv::VideoCapture(fname);

    if (stream.isOpened()) {
        image_height = stream.get(CV_CAP_PROP_FRAME_HEIGHT);
        image_width = stream.get(CV_CAP_PROP_FRAME_WIDTH) / 2;
        image_bitdepth = 1; //TODO get bit depth
        emit update_size(image_width, image_height, image_bitdepth);

        number_frames = stream.get(CV_CAP_PROP_FRAME_COUNT);

        setFPS(fps);

        connected = true;
    } else {
        connected = false;
    }
    return connected;
}

void StereoCameraFromVideo::setPosition(int position){
    stream.set(CV_CAP_PROP_POS_FRAMES, (0.01*number_frames) * position);
}

bool StereoCameraFromVideo::closeCamera(){
    if (connected){
        if (stream.isOpened()) {
            stream.release();
        }
    }
    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraFromVideo::captureSingle(){
    int current_frame = stream.get(CV_CAP_PROP_POS_FRAMES);
    if (current_frame >= number_frames ){
        setPosition(0);
    }

    bool res = stream.read(image_buffer);

    if (res) {
        // Simulate frame rate
        double delay_needed = (1000.0/(video_fps+1)) - frame_timer.elapsed();
        if(delay_needed > 0){
            QThread::msleep(delay_needed);
        }
        emit videoPosition(100*((float) stream.get(CV_CAP_PROP_POS_FRAMES))/number_frames);
        if(image_buffer.channels() == 3)
            cv::cvtColor(image_buffer, image_buffer, CV_RGB2GRAY);

        cv::Mat(image_buffer,
                cv::Rect(0, 0, image_buffer.cols / 2, image_buffer.rows))
                .copyTo(left_raw);
        cv::Mat(image_buffer, cv::Rect(image_buffer.cols / 2, 0,
                                       image_buffer.cols / 2, image_buffer.rows))
                .copyTo(right_raw);

        frame_timer.restart();
    } else {
        emit error(CAPTURE_ERROR);
    }
    emit captured();
    return res;
}

void StereoCameraFromVideo::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraFromVideo::captureSingle);
}

bool StereoCameraFromVideo::enableCapture(bool enable){
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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraFromVideo::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;
    return connected_serial_infos;
}

bool StereoCameraFromVideo::setFPS(int fps){
    if (!isCapturing()){
        video_fps = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

StereoCameraFromVideo::~StereoCameraFromVideo(void) {
    if (connected){
        closeCamera();
    }
}
