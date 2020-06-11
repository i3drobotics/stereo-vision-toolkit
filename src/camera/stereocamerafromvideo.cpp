/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocamerafromvideo.h"

bool StereoCameraFromVideo::initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings) {
    //TODO use inital camera settings input

    std::string fname = camera_info.filename;
    stream = cv::VideoCapture(fname);

    if (stream.isOpened()) {
        stream_valid = true;
        stream_file = fname;
    }

    image_height = stream.get(CV_CAP_PROP_FRAME_HEIGHT);
    image_width = stream.get(CV_CAP_PROP_FRAME_WIDTH) / 2;
    image_bitdepth = 1; //TODO get bit depth
    emit update_size(image_width, image_height, image_bitdepth);

    frame_rate = inital_camera_settings.fps;

    number_frames = stream.get(CV_CAP_PROP_FRAME_COUNT);

    connected = true;

    frame_timer.restart();
    return stream.isOpened();
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraFromVideo::listSystems(){
    return std::vector<AbstractStereoCamera::stereoCameraSerialInfo>();
}

void StereoCameraFromVideo::setPosition(int position){

    stream.set(CV_CAP_PROP_POS_FRAMES, (0.01*number_frames) * position);
}

bool StereoCameraFromVideo::enableAutoExpose(bool enable) {
    return enable;
}

void StereoCameraFromVideo::disconnectCamera(){
    if (connected){
        stream.release();
    }
    connected = false;
    emit disconnected();
}

void StereoCameraFromVideo::adjustFPS(int val){
    frame_rate = val;
}

bool StereoCameraFromVideo::capture() {
    bool res = stream.read(image_buffer);

    if (res) {
        // Simulate frame rate

        double delay_needed = (1000.0/(frame_rate+1)) - frame_timer.elapsed();

        if(delay_needed > 0){
            QThread::msleep(delay_needed);
        }

        //if(!isMatching())
        //    QThread::msleep(1000 / frame_rate);

        emit videoPosition(100*((float) stream.get(CV_CAP_PROP_POS_FRAMES))/number_frames);

        if(image_buffer.channels() == 3)
            cv::cvtColor(image_buffer, image_buffer, CV_RGB2GRAY);

        cv::Mat(image_buffer,
                cv::Rect(0, 0, image_buffer.cols / 2, image_buffer.rows))
                .copyTo(left_raw);
        cv::Mat(image_buffer, cv::Rect(image_buffer.cols / 2, 0,
                                       image_buffer.cols / 2, image_buffer.rows))
                .copyTo(right_raw);

        emit captured();

        frame_timer.restart();

    } else {
        cv::Mat left, right;
        left.copyTo(left_raw);
        right.copyTo(right_raw);
        // Loop video
        if (stream_valid) {
            stream = cv::VideoCapture(stream_file);
        } else {
            qDebug() << "Capture fail";
        }
    }

    emit left_captured();
    emit right_captured();

    return res;
}
