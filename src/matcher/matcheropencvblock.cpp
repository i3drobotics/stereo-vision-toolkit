/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "matcheropencvblock.h"

void MatcherOpenCVBlock::init(void) {

    QString matcher_parameters = QStandardPaths::AppConfigLocation+"/stereo_bm_params.xml";
    if(QFile(matcher_parameters).exists()){
        try {
            matcher = cv::StereoBM::load<cv::StereoBM>(matcher_parameters.toStdString());
                  } catch (cv::Exception& e) {
            qDebug() << "Error loading block matching parameters" << e.msg.c_str();
            setupDefaultMatcher();
        }
    }else{
        setupDefaultMatcher();
    }
}

void MatcherOpenCVBlock::setupDefaultMatcher(void){
    matcher = cv::StereoBM::create(64, 9);
    setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity) {
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range) {
  //if ((disparity_range + min_disparity) > image_size.width) return;

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

bool MatcherOpenCVBlock::forwardMatch(cv::Mat left_img, cv::Mat right_img) {

  matcher->setMinDisparity(min_disparity);

  try {
      if (left_img.type() == CV_8UC1 && right_img.type() == CV_8UC1){
        matcher->compute(left_img, right_img, disparity_lr);
        //NOTE: Removed to try and not use ximageproc
        /*
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
        */
       disparity_lr.convertTo(disparity_lr, CV_32F);
       return true;
      } else {
          qDebug() << "Invalid image type for stereo matcher. MUST be CV_8UC1.";
          qDebug() << "Left image type index: " << left_img.type();
          qDebug() << "Right image type index: " << right_img.type();
          return false;
      }

  } catch (...) {
    qDebug() << "Error in OpenCV block match parameters";
    return false;
  }
}

bool MatcherOpenCVBlock::backwardMatch(cv::Mat left_img, cv::Mat right_img) {
    //NOTE: Removed to try and not use ximageproc
    //auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    //right_matcher->compute(*right, *left, disparity_rl);
    return false;
}

void MatcherOpenCVBlock::saveParams() {
  matcher->save(QCoreApplication::applicationDirPath().toStdString()+ "/params/stereo_bm_params.xml");
}
