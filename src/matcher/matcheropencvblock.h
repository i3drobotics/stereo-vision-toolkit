/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef MATCHEROPENCVBLOCK_H
#define MATCHEROPENCVBLOCK_H

#include <abstractstereomatcher.h>
#ifdef WITH_OPENCV_CONTRIB
    #include <opencv2/ximgproc.hpp>
#endif
#ifdef WITH_CUDA
    #include <opencv2/cudastereo.hpp>
#endif
#include <QFile>

//!  Matcher OpenCV Block
/*!
  Stereo matcher using OpenCV's Block Matching algorithm
*/

class MatcherOpenCVBlock : public AbstractStereoMatcher {
    Q_OBJECT
public:
    explicit MatcherOpenCVBlock(QObject *parent = 0)
        : AbstractStereoMatcher(parent) {
        init();
    }

    bool isLicenseValid(){return true;}

public slots:
    void setMinDisparity(int min_disparity);
    void setDisparityRange(int disparity_range);
    void setBlockSize(int block_size);
    void setDisp12MaxDiff(int diff);
    void setPrefilterType(int type);
    void setPrefilterSize(int size);
    void setPrefilterCap(int cap);
    void setTextureThreshold(int threshold);
    void setUniquenessRatio(int ratio);
    void setSpeckleFilterWindow(int window);
    void setSpeckleFilterRange(int range);
    void setWLSFilterEnabled(bool enable);
    void setGPUEnabled(bool enable);
    int getErrorDisparity(void);

    int getMinDisparity(){return matcher->getMinDisparity();}
    int getDisparityRange(){return matcher->getNumDisparities();}
    int getBlockSize(){return matcher->getBlockSize();}
    int getDisp12MaxDiff(){return matcher->getDisp12MaxDiff();}
    int getPrefilterType(){return matcher->getPreFilterType();}
    int getPrefilterSize(){return matcher->getPreFilterSize();}
    int getPrefilterCap(){return matcher->getPreFilterCap();}
    int getTextureThreshold(){return matcher->getTextureThreshold();}
    int getUniquenessRatio(){return matcher->getUniquenessRatio();}
    int getSpeckleFilterWindow(){return matcher->getSpeckleWindowSize();}
    int getSpeckleFilterRange(){return matcher->getSpeckleRange();}
    bool isWLSFilterEnabled(){return wls_filter;}
    bool isGPUEnabled(){return useGPU;}

    void saveParams();

    bool forwardMatch(cv::Mat left_img, cv::Mat right_img);
    bool backwardMatch(cv::Mat left_img, cv::Mat right_img);

private:
#ifdef WITH_CUDA
    cv::Ptr<cv::cuda::StereoBM> gpu_matcher;
    cv::Ptr<cv::StereoBM> matcher;
#else
    cv::Ptr<cv::StereoBM> matcher;
#endif

    void init(void);
    void setupDefaultMatcher(void);

    bool wls_filter = false;
    double wls_lambda = 8000;
    double wls_sigma = 1.5;

    bool useGPU = false;
};

#endif  // MATCHEROPENCVBLOCK_H
