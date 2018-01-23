/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MATCHEROPENCVSGBM_H
#define MATCHEROPENCVSGBM_H

#include <abstractstereomatcher.h>
#include <QFile>

class MatcherOpenCVSGBM : public AbstractStereoMatcher
{
    Q_OBJECT
   public:
    explicit MatcherOpenCVSGBM(QObject *parent = 0,
                                cv::Size image_size = cv::Size(0, 0))
        : AbstractStereoMatcher(parent, image_size) {
      init();
    }

   public slots:
    void setMinDisparity(int min_disparity);
    void setDisparityRange(int disparity_range);
    void setBlockSize(int block_size);
    void setDisp12MaxDiff(int diff);
    void setUniquenessRatio(int ratio);
    void setSpeckleFilterWindow(int window);
    void setSpeckleFilterRange(int range);
    int getErrorDisparity(void);

    void saveParams();

    void forwardMatch(void);
    void backwardMatch(void);


    int getMinDisparity(){return matcher->getMinDisparity();}
    int getDisparityRange(){return matcher->getNumDisparities();}
    int getBlockSize(){return matcher->getBlockSize();}
    int getDisp12MaxDiff(){return matcher->getDisp12MaxDiff();}
    int getPrefilterCap(){return matcher->getPreFilterCap();}
    int getUniquenessRatio(){return matcher->getUniquenessRatio();}
    int getSpeckleFilterWindow(){return matcher->getSpeckleWindowSize();}
    int getSpeckleFilterRange(){return matcher->getSpeckleRange();}

   private:
    cv::Ptr<cv::StereoSGBM> matcher;
    void init(void);
    void setupDefaultMatcher(void);

    bool wls_filter = true;
    double wls_lambda = 8000;
    double wls_sigma = 1.5;
};

#endif // MATCHEROPENCVSGBM_H
