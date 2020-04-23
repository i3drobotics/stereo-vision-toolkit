/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MATCHERWIDGETOPENCVSGBM_H
#define MATCHERWIDGETOPENCVSGBM_H

#include <matcherwidget.h>
#include <matcheropencvsgbm.h>
#include <opencv2/opencv.hpp>

//!  OpenCV's SGBM QT Widget
/*!
  QT widget for OpenCV's SGBM controls
*/

namespace Ui {
class MatcherWidgetOpenCVSGBM;
}

class MatcherWidgetOpenCVSGBM : public MatcherWidget {
  Q_OBJECT

 signals:
   void minDisparity(int);
   void disparityRange(int);
   void blockSize(int);
   void speckleWindow(int);
   void speckleRange(int);
   void consistency(int);
   void uniquenessRatio(int);
   void saveClicked();


 public:
   explicit MatcherWidgetOpenCVSGBM(QWidget *parent = 0, cv::Size image_size =cv::Size(0,0));
  ~MatcherWidgetOpenCVSGBM();
   void setImageWidth(int width);

public slots:
   void updateDisparityRange(int);
   void updateBlockSize(int);
   void updateMinDisparity(int);
   void updateConsistency(int consistency);
   void updateUniquenessRatio(int ratio);
   void updateSpeckleRange(int range);
   void updateSpeckleWindow(int window);
   void enableSpeckleFilter(bool enable);
   void enableConsistency(bool enable);

   AbstractStereoMatcher* getMatcher(void);

   void onSaveClicked();

 private:
  Ui::MatcherWidgetOpenCVSGBM *ui;
  MatcherOpenCVSGBM* matcher;
  int block_size;
  int min_disparity;
  int disparity_range;
  int image_width = 640;

};

#endif // MATCHERWIDGETOPENCVSGBM_H
