/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "matcheropencvblock.h"

void MatcherOpenCVBlock::init(void) {

    QString matcher_parameters = QStandardPaths::AppConfigLocation+"/stereo_bm_params.xml";
    if(QFile(matcher_parameters).exists()){
        try {
#ifdef WITH_CUDA
            qDebug() << "Unable load GPU StereoBM parameters from file";
            setupDefaultMatcher();
            //TODO find correct method to load cuda stereobm from file
            //gpu_matcher = cv::cuda::StereoBM::load<cv::cuda::StereoBM>(matcher_parameters.toStdString());
            matcher = cv::StereoBM::load<cv::StereoBM>(matcher_parameters.toStdString());
#else
            matcher = cv::StereoBM::load<cv::StereoBM>(matcher_parameters.toStdString());
#endif
        } catch (cv::Exception& e) {
            qDebug() << "Error loading block matching parameters" << e.msg.c_str();
            setupDefaultMatcher();
        }
    }else{
        setupDefaultMatcher();
    }
}

void MatcherOpenCVBlock::setupDefaultMatcher(void){
#ifdef WITH_CUDA
    matcher = cv::StereoBM::create(64, 9);
    try
    {
        gpu_matcher = cv::cuda::createStereoBM(64, 9);
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        std::cout << "Disabling GPU CUDA functions" << std::endl;
        useGPU = false;
        gpu_matcher = nullptr;
    }
#else
    matcher = cv::StereoBM::create(64, 9);
#endif
    setUniquenessRatio(15);
#ifdef WITH_CUDA
    gpu_matcher->setDisp12MaxDiff(-1);
#endif
    matcher->setDisp12MaxDiff(-1);
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity) {
#ifdef WITH_CUDA
    if (useGPU)
        gpu_matcher->setMinDisparity(min_disparity);
#endif
    matcher->setMinDisparity(min_disparity);
    this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range) {
  //if ((disparity_range + min_disparity) > image_size.width) return;

  if ((disparity_range > 0) && (disparity_range % 16 == 0)) {
    this->disparity_range = disparity_range;
    matcher->setNumDisparities(disparity_range);
#ifdef WITH_CUDA
    if (useGPU)
        gpu_matcher->setNumDisparities(disparity_range);
#endif
  }
}

void MatcherOpenCVBlock::setBlockSize(int block_size) {
  this->block_size = block_size;
  matcher->setBlockSize(block_size);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setBlockSize(block_size);
#endif
}

void MatcherOpenCVBlock::setDisp12MaxDiff(int diff) {
  matcher->setDisp12MaxDiff(diff);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setDisp12MaxDiff(diff);
#endif
}

void MatcherOpenCVBlock::setPrefilterType(int type) {
  if (type > 0){
      matcher->setPreFilterType(type);
#ifdef WITH_CUDA
      if (useGPU)
        gpu_matcher->setPreFilterType(type);
#endif
  }
}

void MatcherOpenCVBlock::setPrefilterSize(int size) {
  matcher->setPreFilterSize(size);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setPreFilterSize(size);
#endif
}

void MatcherOpenCVBlock::setPrefilterCap(int cap) {
  matcher->setPreFilterCap(cap);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setPreFilterCap(cap);
#endif
}

void MatcherOpenCVBlock::setTextureThreshold(int threshold) {
  matcher->setTextureThreshold(threshold);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setTextureThreshold(threshold);
#endif
}

void MatcherOpenCVBlock::setUniquenessRatio(int ratio) {
  matcher->setUniquenessRatio(ratio);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setUniquenessRatio(ratio);
#endif
}

void MatcherOpenCVBlock::setSpeckleFilterWindow(int window) {
  matcher->setSpeckleWindowSize(window);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setSpeckleWindowSize(window);
#endif
}

void MatcherOpenCVBlock::setSpeckleFilterRange(int range) {
  matcher->setSpeckleRange(range);
#ifdef WITH_CUDA
  if (useGPU)
    gpu_matcher->setSpeckleRange(range);
#endif
}

void MatcherOpenCVBlock::setWLSFilterEnabled(bool enable) {
    wls_filter = enable;
}

#ifdef WITH_CUDA
void MatcherOpenCVBlock::setGPUEnabled(bool enable) {
    if (enable){
      if (gpu_matcher != nullptr){
        gpu_matcher->setMinDisparity(matcher->getMinDisparity());
        gpu_matcher->setNumDisparities(matcher->getNumDisparities());
        gpu_matcher->setBlockSize(matcher->getBlockSize());
        gpu_matcher->setDisp12MaxDiff(matcher->getDisp12MaxDiff());
        gpu_matcher->setPreFilterType(matcher->getPreFilterType());
        gpu_matcher->setPreFilterSize(matcher->getPreFilterSize());
        gpu_matcher->setTextureThreshold(matcher->getTextureThreshold());
        gpu_matcher->setUniquenessRatio(matcher->getUniquenessRatio());
        gpu_matcher->setSpeckleWindowSize(matcher->getSpeckleWindowSize());
        gpu_matcher->setSpeckleRange(matcher->getSpeckleRange());
      }
    }
    useGPU = enable;
}
#endif

int MatcherOpenCVBlock::getErrorDisparity(void){
    return min_disparity - 1;
}

bool MatcherOpenCVBlock::forwardMatch(cv::Mat left_img, cv::Mat right_img) {

  matcher->setMinDisparity(min_disparity);

  try {
      if (left_img.type() == CV_8UC1 && right_img.type() == CV_8UC1){
#ifdef WITH_CUDA
        if (useGPU){
            cv::cuda::GpuMat cuda_left, cuda_right, cuda_disp;
            cuda_left.upload(left_img);
            cuda_right.upload(right_img);
            gpu_matcher->compute(cuda_left, cuda_right, cuda_disp);
            cuda_disp.download(disparity_lr);
        } else {
            matcher->compute(left_img, right_img, disparity_lr);
        }
#else
        matcher->compute(left_img, right_img, disparity_lr);
#endif
        if(wls_filter){
#ifdef WITH_OPENCV_CONTRIB
            backwardMatch(left_img,right_img);
            cv::Mat disparity_filter;
            auto wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
            wls_filter->setLambda(wls_lambda);
            wls_filter->setSigmaColor(wls_sigma);
            wls_filter->filter(disparity_lr,left_img,disparity_filter,disparity_rl);

            disparity_filter.convertTo(disparity_lr, CV_32F);
#else
            disparity_lr.convertTo(disparity_lr, CV_32F);
#endif
        }else{
            disparity_lr.convertTo(disparity_lr, CV_32F);
        }
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
#ifdef WITH_OPENCV_CONTRIB
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    right_matcher->compute(right_img, left_img, disparity_rl);
    return true;
#else
    return false;
#endif
}

void MatcherOpenCVBlock::saveParams() {
  matcher->save(QCoreApplication::applicationDirPath().toStdString()+ "/params/stereo_bm_params.xml");
}
