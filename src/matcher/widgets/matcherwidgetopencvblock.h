/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef MATCHERWIDGETOPENCVBLOCK_H
#define MATCHERWIDGETOPENCVBLOCK_H

#include <matcherwidget.h>
#include <matcheropencvblock.h>
#include <opencv2/opencv.hpp>

namespace Ui {
class MatcherWidgetOpenCVBlock;
}

//!  OpenCV's Block Matcher QT Widget
/*!
  QT widget for OpenCV's Block Matcher controls
*/

class MatcherWidgetOpenCVBlock : public MatcherWidget {
  Q_OBJECT

 signals:
   void minDisparity(int);
   void disparityRange(int);
   void blockSize(int);
   void textureThreshold(int);
   void speckleWindow(int);
   void speckleRange(int);
   void uniquenessRatio(int);
   void saveClicked();


 public:
   explicit MatcherWidgetOpenCVBlock(QWidget *parent = 0);
  ~MatcherWidgetOpenCVBlock();
   void setImageWidth(int width);

public slots:
   void updateDisparityRange(int);
   void updateBlockSize(int);
   void updateMinDisparity(int);

   void updateTextureThreshold(int threshold);
   void updateUniquenessRatio(int ratio);
   void updateSpeckleRange(int range);
   void updateSpeckleWindow(int window);
   void enableExtendDisparity(bool enable);
   void enableNegativeDisparity(bool enable);
   void enableSpeckleFilter(bool enable);
   void enableWLSFilter(bool enable);

   AbstractStereoMatcher* getMatcher(void);

   void onSaveClicked();

 private:
  Ui::MatcherWidgetOpenCVBlock *ui;
  MatcherOpenCVBlock* matcher;
  int block_size;
  int min_disparity;
  int disparity_range;
  int image_width = 640;
  bool negative_disparity = true;

};

#endif  // MATCHERWIDGETOPENCVBLOCK_H
