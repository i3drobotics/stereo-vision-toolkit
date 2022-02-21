/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "abstractstereomatcher.h"

AbstractStereoMatcher::AbstractStereoMatcher(QObject *parent)
    : QObject(parent) {
}

void AbstractStereoMatcher::assignThread(QThread *thread) {
    this->moveToThread(thread);
    connect(this, SIGNAL(finished()), thread, SLOT(quit()));
    connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void AbstractStereoMatcher::stopThread(){
    emit finished();
}

void AbstractStereoMatcher::convertImages(cv::Mat left_img, cv::Mat right_img, cv::Mat& left_bgr_conv_img, cv::Mat& left_conv_img, cv::Mat& right_conv_img) {
    cv::Mat right_tmp, left_tmp, left_bgr_tmp;

    if (left_img.type() == CV_8UC1){
        left_img.copyTo(left_bgr_tmp);
        left_img.copyTo(left_tmp);
    } else if (left_img.type() == CV_8UC3){
        left_img.copyTo(left_bgr_tmp);
        cv::cvtColor(left_bgr_tmp,left_tmp,cv::COLOR_BGR2GRAY);
        left_tmp.convertTo(left_tmp, CV_8UC1);
    } else {
        std::cerr << "Unknown image type in left image for stereo matching: " << left_img.type() << ". Expected CV_8U1 or CV_8UC3" << std::endl;
        left_img.copyTo(left_bgr_tmp);
        left_img.copyTo(left_tmp);
    }

    if (right_img.type() == CV_8UC1){
        right_img.copyTo(right_tmp);
    } else if (right_img.type() == CV_8UC3){
        cv::cvtColor(right_img,right_tmp,cv::COLOR_BGR2GRAY);
        right_tmp.convertTo(right_tmp, CV_8UC1);
    } else {
        std::cerr << "Unknown image type in right image for stereo matching: " << right_img.type() << ". Expected CV_8U1 or CV_8UC3" << std::endl;
        right_img.copyTo(right_tmp);
    }

    if (downsample_factor != 1){
        cv::resize(right_tmp,right_tmp,cv::Size(),downsample_factor,downsample_factor);
        cv::resize(left_bgr_tmp,left_bgr_tmp,cv::Size(),downsample_factor,downsample_factor);
        cv::resize(left_tmp,left_tmp,cv::Size(),downsample_factor,downsample_factor);
    }

    left_conv_img = left_tmp.clone();
    left_bgr_conv_img = left_bgr_tmp.clone();
    right_conv_img = right_tmp.clone();
}

void AbstractStereoMatcher::getDisparity(cv::Mat &dst) {
    disparity_buffer.copyTo(dst);
    return;
}

void AbstractStereoMatcher::getDisparity16(cv::Mat &dst) {
    disparity16.copyTo(dst);
}

void AbstractStereoMatcher::getDisparityRange(int &val) {
    val = disparity_range;
}

void AbstractStereoMatcher::getMinDisparity(int &val) {
    val = min_disparity;
}

void AbstractStereoMatcher::checkLRConsistencyFull(double threshold){
    //TODO access if this is needed
    //NOTE: Removed to try and not use ximageproc
    /*
    backwardMatch();

    cv::Mat difference = disparity_rl - disparity_lr;

    for(int i=0; i < static_cast<int>(difference.total()); i++){
        if( abs(difference.at<float>(i)) < threshold) continue;
        else disparity_lr.at<float>(i) = min_disparity;
    }
    */
}

bool AbstractStereoMatcher::match(cv::Mat left_img, cv::Mat right_img) {
    QElapsedTimer timer;
    timer.restart();

    convertImages(left_img,right_img,left_bgr,left,right);
    //required to check if size has changed
    //I3DRSGM needs to regenerate its matcher if so
    if (this->image_size != left.size()){
        this->image_size = left.size();
        sizeChangedThisFrame = true;
    } else {
        sizeChangedThisFrame = false;
    }
    bool valid = forwardMatch(left,right);
    if (valid){
        // qDebug() << 1/(timer.elapsed() / 1e3);

        //checkLRConsistencyFull(5);

        disparity_lr.convertTo(disparity_buffer, CV_32F);
        disparity16 = disparity_lr / 16;
        return true;
    } else {
        qDebug() << "Stereo match failed";
        return false;
    }
}

AbstractStereoMatcher::~AbstractStereoMatcher() { emit finished(); }
