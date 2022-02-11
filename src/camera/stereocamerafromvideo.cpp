/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocamerafromvideo.h"

cv::Mat StereoCameraFromVideo::extractFirstFrame(){
    if (isConnected()){
        qDebug() << "Cannot extract first frame when already connected";
        return cv::Mat();
    }
    std::string fname = stereoCameraSerialInfo_.left_camera_serial;
    stream = cv::VideoCapture(fname);

    cv::Mat firstFrame;
    stream.read(firstFrame);

    if (stream.isOpened()) {
        stream.release();
    }

    return firstFrame;
}

StereoCameraFromVideo::StereoVideoType StereoCameraFromVideo::detectVideoType(){
    cv::Mat firstFrame = extractFirstFrame();
    if (firstFrame.empty()){
        qDebug() << "Could not extract first frame for video type detection";
        return StereoVideoType::UNKNOWN_VIDEO;
    }
    if (firstFrame.channels() == 1){
        return StereoVideoType::CONCAT_VIDEO;
    }

    if (firstFrame.channels() != 3){
        qDebug() << "Could not detect type version, invalid number of channels";
        return StereoVideoType::UNKNOWN_VIDEO;
    }

    std::string fname = stereoCameraSerialInfo_.left_camera_serial;
    // remove extension from filename
    std::string filename;
    size_t lastdot = fname.find_last_of(".");
    if (lastdot == std::string::npos){
        qDebug() << "File is missing extension";
        return StereoVideoType::UNKNOWN_VIDEO;
    }
    filename = fname.substr(0, lastdot);
    std::string last_6 = filename.substr(filename.size() - 6);
    qDebug() << last_6.c_str();
    // rg stereo video must have 'srgvid' before extension in filename
    if (last_6 == "srgvid"){
        return StereoVideoType::RG_VIDEO;
    }
    // concat stereo video must have 'sctvid' before extension in filename
    if (last_6 == "sctvid"){
        return StereoVideoType::CONCAT_VIDEO;
    }
    // if it doesn't assume it's rg stereo video as this is the default
    qDebug() << "srgvid or sctvid missing from filename. Assuming Stereo RG video.";
    return StereoVideoType::RG_VIDEO;
}

bool StereoCameraFromVideo::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    video_type = detectVideoType();
    if (video_type == StereoVideoType::UNKNOWN_VIDEO){
        connected = false;
        return false;
    }
    int fps = stereoCameraSettings_.fps;
    std::string fname = stereoCameraSerialInfo_.left_camera_serial;

    stream = cv::VideoCapture(fname);

    if (stream.isOpened()) {
        image_height = stream.get(cv::CAP_PROP_FRAME_HEIGHT);
        if (video_type == StereoVideoType::CONCAT_VIDEO){
            image_width = stream.get(cv::CAP_PROP_FRAME_WIDTH) / 2;
        } else if (video_type == StereoVideoType::RG_VIDEO){
            image_width = stream.get(cv::CAP_PROP_FRAME_WIDTH);
        } else {
            connected = false;
            return false;
        }
        
        image_bitdepth = 1; //TODO get bit depth
        emit update_size(image_width, image_height, image_bitdepth);

        number_frames = stream.get(cv::CAP_PROP_FRAME_COUNT);

        setFPS(fps);

        connected = true;
    } else {
        connected = false;
    }
    return connected;
}

void StereoCameraFromVideo::setPosition(int position){
    stream.set(cv::CAP_PROP_POS_FRAMES, (0.01*number_frames) * position);
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
    int current_frame = stream.get(cv::CAP_PROP_POS_FRAMES);
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
        emit videoPosition(100*((float) stream.get(cv::CAP_PROP_POS_FRAMES))/number_frames);
        //if(image_buffer.channels() == 3)
        //    cv::cvtColor(image_buffer, image_buffer, cv::COLOR_RGB2GRAY);

        if (video_type == StereoVideoType::CONCAT_VIDEO){
            cv::Mat(image_buffer,
                cv::Rect(0, 0, image_buffer.cols / 2, image_buffer.rows))
                .copyTo(left_raw);
            cv::Mat(image_buffer, cv::Rect(image_buffer.cols / 2, 0,
                                        image_buffer.cols / 2, image_buffer.rows))
                    .copyTo(right_raw);
        } else if (video_type == StereoVideoType::RG_VIDEO){
            cv::Mat bgr[3];
            cv::split(image_buffer, bgr);
            bgr[1].copyTo(left_raw);
            bgr[2].copyTo(right_raw);
        } else {
            res = false;
        }

        frame_timer.restart();
    }
    if (!res){
        send_error(CAPTURE_ERROR);
        emit captured_fail();
    } else {
        emit captured_success();
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
        frame_rate = fps;
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
