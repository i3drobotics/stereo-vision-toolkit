/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef MATCHERWIDGETOPENCVSGBM_H
#define MATCHERWIDGETOPENCVSGBM_H

#include <matcherwidget.h>
#include <matcheropencvsgbm.h>
#include <opencv2/opencv.hpp>

namespace Ui {
class MatcherWidgetOpenCVSGBM;
}

//!  OpenCV's SGBM QT Widget
/*!
  QT widget for OpenCV's SGBM controls
*/

class MatcherWidgetOpenCVSGBM : public MatcherWidget {
  Q_OBJECT

 signals:
   void minDisparity(int);
   void disparityRange(int);
   void blockSize(int);
   void speckleWindow(int);
   void speckleRange(int);
   void uniquenessRatio(int);
   void saveClicked();


 public:
   explicit MatcherWidgetOpenCVSGBM(QWidget *parent = 0);
  ~MatcherWidgetOpenCVSGBM();
   void setImageWidth(int width);

public slots:
   void updateDisparityRange(int);
   void updateBlockSize(int);
   void updateMinDisparity(int);
   void updateUniquenessRatio(int ratio);
   void updateSpeckleRange(int range);
   void updateSpeckleWindow(int window);
   void enableExtendDisparity(bool enable);
   void enableNegativeDisparity(bool enable);
   void enableSpeckleFilter(bool enable);

   AbstractStereoMatcher* getMatcher(void);

   void onSaveClicked();

 private:
  Ui::MatcherWidgetOpenCVSGBM *ui;
  MatcherOpenCVSGBM* matcher;
  int block_size;
  int min_disparity;
  int disparity_range;
  int image_width = 640;
  bool negative_disparity = true;

};

#endif // MATCHERWIDGETOPENCVSGBM_H
