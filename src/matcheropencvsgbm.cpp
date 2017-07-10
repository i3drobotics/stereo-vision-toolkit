#include "matcheropencvsgbm.h"

void MatcherOpenCVSGBM::init(void) {
  try {
    matcher =
        cv::StereoSGBM::load<cv::StereoSGBM>("./params/stereo_sgbm_params.xml");
  } catch (cv::Exception& e) {
    matcher = cv::StereoSGBM::create(0, 64, 9);
    setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
    qDebug() << "Error loading SGBM matching parameters: " << e.msg.c_str();
  }

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);

  qDebug() << "Loaded OpenCV SGBM";
}

void MatcherOpenCVSGBM::setMinDisparity(int min_disparity) {
    matcher->setMinDisparity(min_disparity);
    this->min_disparity = min_disparity;
}

void MatcherOpenCVSGBM::setDisparityRange(int disparity_range) {
  if ((disparity_range + min_disparity) > image_size.width) return;

  if ((disparity_range > 0) && (disparity_range % 16 == 0)) {
    this->disparity_range = disparity_range;
    matcher->setNumDisparities(disparity_range);
  }
}

void MatcherOpenCVSGBM::setBlockSize(int block_size) {
  this->block_size = block_size;
  matcher->setBlockSize(block_size);
}

void MatcherOpenCVSGBM::setDisp12MaxDiff(int diff) {
  matcher->setDisp12MaxDiff(diff);
}

void MatcherOpenCVSGBM::setUniquenessRatio(int ratio) {
  matcher->setUniquenessRatio(ratio);
}

void MatcherOpenCVSGBM::setSpeckleFilterWindow(int window) {
  matcher->setSpeckleWindowSize(window);
}

void MatcherOpenCVSGBM::setSpeckleFilterRange(int range) {
  matcher->setSpeckleRange(range);
}

int MatcherOpenCVSGBM::getErrorDisparity(void){
    return min_disparity - 1;
}

void MatcherOpenCVSGBM::forwardMatch() {
  matcher->setMinDisparity(min_disparity);
  try {
    matcher->compute(*left, *right, disparity_lr);

    disparity_lr.convertTo(disparity_lr, CV_32F);

  } catch (...) {
    qDebug() << "Error in SGBM match parameters";
  }
}

void MatcherOpenCVSGBM::backwardMatch() {
  matcher->setMinDisparity(-(min_disparity + disparity_range));
  matcher->compute(*right, *left, disparity_rl);
}

void MatcherOpenCVSGBM::saveParams() {
  matcher->save("./params/stereo_sgbm_params.xml");
}
