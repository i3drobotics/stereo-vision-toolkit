/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "abstractstereomatcher.h"

AbstractStereoMatcher::AbstractStereoMatcher(QObject *parent,
                                             cv::Size img_size)
    : QObject(parent) {
  this->image_size = img_size;
  cv::Mat(image_size, CV_32F).copyTo(disparity_buffer);
}

void AbstractStereoMatcher::assignThread(QThread *thread) {
  this->moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

void AbstractStereoMatcher::setImages(cv::Mat *left_img, cv::Mat *right_img) {
  this->left = left_img;
  this->right = right_img;
}

void AbstractStereoMatcher::getDisparity(cv::Mat &dst) {
  disparity_buffer.copyTo(dst);
  return;
}

void AbstractStereoMatcher::getDisparityRange(int &val) {
  val = disparity_range;
}

void AbstractStereoMatcher::getMinDisparity(int &val) {
  val = min_disparity;
}

void AbstractStereoMatcher::saveDisparity(QString filename) {
  cv::Mat disparity_output;

  //disparity_lr.convertTo(disparity_output, CV_16SC1);
  //disparity_output += min_disparity*16;
  //disparity_output.convertTo(disparity_output, CV_16UC1);
  disparity_lr.copyTo(disparity_output);
  disparity_output /= 16;

  cv::imwrite(filename.toStdString(), disparity_output);

  return;
}

void AbstractStereoMatcher::checkLRConsistencyFull(double threshold){

    backwardMatch();

    cv::Mat difference = disparity_rl - disparity_lr;

    for(int i=0; i < static_cast<int>(difference.total()); i++){
       if( abs(difference.at<float>(i)) < threshold) continue;
       else disparity_lr.at<float>(i) = min_disparity;
    }
}

void AbstractStereoMatcher::match() {
  QElapsedTimer timer;
  timer.restart();
  forwardMatch();
  // qDebug() << 1/(timer.elapsed() / 1e3);

  //checkLRConsistencyFull(5);

  disparity_lr.convertTo(disparity_buffer, CV_32F);
}

AbstractStereoMatcher::~AbstractStereoMatcher() { emit finished(); }
