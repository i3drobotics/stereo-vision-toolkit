/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "matcheropencvblock.h"

void MatcherOpenCVBlock::init(void) {
  try {
    matcher = cv::StereoBM::load<cv::StereoBM>(QCoreApplication::applicationDirPath().toStdString()+"/params/stereo_bm_params.xml");
  } catch (cv::Exception& e) {
    matcher = cv::StereoBM::create(64, 9);
    setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
    qDebug() << "Error loading block matching parameters" << e.msg.c_str();
  }

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity) {
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range) {
  if ((disparity_range + min_disparity) > image_size.width) return;

  if ((disparity_range > 0) && (disparity_range % 16 == 0)) {
    this->disparity_range = disparity_range;
    matcher->setNumDisparities(disparity_range);
  }
}

void MatcherOpenCVBlock::setBlockSize(int block_size) {
  this->block_size = block_size;
  matcher->setBlockSize(block_size);
}

void MatcherOpenCVBlock::setDisp12MaxDiff(int diff) {
  matcher->setDisp12MaxDiff(diff);
}

void MatcherOpenCVBlock::setPrefilterType(int type) {
  if (type > 0) matcher->setPreFilterType(type);
}

void MatcherOpenCVBlock::setPrefilterSize(int size) {
  matcher->setPreFilterSize(size);
}

void MatcherOpenCVBlock::setPrefilterCap(int cap) {
  matcher->setPreFilterCap(cap);
}

void MatcherOpenCVBlock::setTextureThreshold(int threshold) {
  matcher->setTextureThreshold(threshold);
}

void MatcherOpenCVBlock::setUniquenessRatio(int ratio) {
  matcher->setUniquenessRatio(ratio);
}

void MatcherOpenCVBlock::setSpeckleFilterWindow(int window) {
  matcher->setSpeckleWindowSize(window);
}

void MatcherOpenCVBlock::setSpeckleFilterRange(int range) {
  matcher->setSpeckleRange(range);
}

int MatcherOpenCVBlock::getErrorDisparity(void){
    return min_disparity - 1;
}

void MatcherOpenCVBlock::forwardMatch() {
  matcher->setMinDisparity(min_disparity);

  try {
    matcher->compute(*left, *right, disparity_lr);

    if(wls_filter){
        backwardMatch();
        cv::Mat disparity_filter;
        auto wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(disparity_lr,*left,disparity_filter,disparity_rl);

        disparity_filter.convertTo(disparity_lr, CV_32F);
    }else{
        disparity_lr.convertTo(disparity_lr, CV_32F);
    }

  } catch (...) {
    qDebug() << "Error in OpenCV block match parameters";
  }
}

void MatcherOpenCVBlock::backwardMatch() {
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    right_matcher->compute(*right, *left, disparity_rl);
}

void MatcherOpenCVBlock::saveParams() {
  matcher->save(QCoreApplication::applicationDirPath().toStdString()+ "/params/stereo_bm_params.xml");
}
