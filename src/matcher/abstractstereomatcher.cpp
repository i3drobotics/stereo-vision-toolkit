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

void AbstractStereoMatcher::convertImages(cv::Mat left_img, cv::Mat right_img, cv::Mat& left_bgr_conv_img, cv::Mat& left_conv_img, cv::Mat& right_conv_img) {
    cv::Mat right_tmp, left_tmp, left_bgr_tmp;

    if (left_img.type() == CV_8UC3){
        left_img.copyTo(left_bgr_tmp);
        cv::cvtColor(left_bgr_tmp,left_tmp,cv::COLOR_BGR2GRAY);
        left_tmp.convertTo(left_tmp, CV_8UC1);
    } else {
        left_img.copyTo(left_bgr_tmp);
        left_img.copyTo(left_tmp);
    }
    if (right_img.type() == CV_8UC3){
        cv::cvtColor(right_img,right_tmp,cv::COLOR_BGR2GRAY);
        right_tmp.convertTo(right_tmp, CV_8UC1);
    } else {
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

void AbstractStereoMatcher::saveDisparity(QString filename) {
    cv::Mat disparity_output;

    disparity16.copyTo(disparity_output);

    cv::imwrite(filename.toStdString(), disparity_output);

    return;
}

void AbstractStereoMatcher::saveDisparityColormap(QString filename) {
    cv::Mat disparity_output;

    //generate normalised colormap for saving
    disparity2colormap(disparity16,disparity_output);

    cv::imwrite(filename.toStdString(), disparity_output);

    return;
}

void AbstractStereoMatcher::checkLRConsistencyFull(double threshold){
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

void AbstractStereoMatcher::normaliseDisparity(cv::Mat inDisparity, cv::Mat &outNormalisedDisparity){
    cv::Mat disparity_thresh;

    inDisparity.copyTo(disparity_thresh);

    double min_disp, max_disp;
    getMinMaxDisparity(disparity_thresh, min_disp, max_disp);

    for (int i = 0; i < disparity_thresh.rows; i++)
    {
        for (int j = 0; j < disparity_thresh.cols; j++)
        {
            float d = disparity_thresh.at<float>(i, j);
            if (d > max_disp || d < min_disp){
                disparity_thresh.at<float>(i, j) = 0;
            }
        }
    }

    disparity_thresh.convertTo(disparity_thresh, CV_8U);

    cv::normalize(disparity_thresh, disparity_thresh, 0, 255, cv::NORM_MINMAX);

    outNormalisedDisparity = disparity_thresh;
}

void AbstractStereoMatcher::getMinMaxDisparity(cv::Mat inDisparity, double &min_disp, double &max_disp){
    min_disp = 10000;
    max_disp = 0;

    for (int i = 0; i < inDisparity.rows; i++)
    {
        for (int j = 0; j < inDisparity.cols; j++)
        {
            float d = inDisparity.at<float>(i, j);
            if (d < 10000){
                float d = inDisparity.at<float>(i, j);
                if (d < min_disp){
                    min_disp = d;
                }
                if (d > max_disp){
                    max_disp = d;
                }
            }
        }
    }
}

void AbstractStereoMatcher::disparity2colormap(cv::Mat inDisparity, cv::Mat &outColormap){
    // normalise disparity
    cv::Mat normDisp;
    normaliseDisparity(inDisparity, normDisp);

    // apply colormap
    int colourmap_index = 2;
    cv::Mat colormap;
    cv::applyColorMap(normDisp, colormap, colourmap_index);

    // remove blue background (black becomes blue after colormap)
    for (int x = 0; x < colormap.cols; x++) {
        for (int y = 0; y < colormap.rows; y++) {
            if (colormap.at<cv::Vec3b>(y, x)[2] == 0) {
                colormap.at<cv::Vec3b>(y, x)[0] = 0;
                colormap.at<cv::Vec3b>(y, x)[1] = 0;
                colormap.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    outColormap = colormap;
}

void AbstractStereoMatcher::calcDepth(cv::Mat inDisparity, cv::Mat &outDepth){
    //TODO impliment this
}

void AbstractStereoMatcher::calcPointCloud(cv::Mat inDepth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPoints){
    //TODO impliment this
}

AbstractStereoMatcher::~AbstractStereoMatcher() { emit finished(); }
