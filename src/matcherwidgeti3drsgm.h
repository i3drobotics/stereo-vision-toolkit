/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MATCHERWIDGETI3DRSGM_H
#define MATCHERWIDGETI3DRSGM_H

#include <matcherwidget.h>
#include <qmatcheri3drsgm.h>

//!  OpenCV's Block Matcher QT Widget
/*!
  QT widget for OpenCV's Block Matcher controls
*/

namespace Ui {
class MatcherWidgetI3DRSGM;
}

class MatcherWidgetI3DRSGM : public MatcherWidget {
  Q_OBJECT

 signals:
   void minDisparity(int);
   void disparityRange(int);
   void blockSize(int);
   void pyramidLevel(int);
   void interpolate(bool);
   void saveClicked();

 public:
   explicit MatcherWidgetI3DRSGM(QWidget *parent = 0, cv::Size image_size =cv::Size(0,0));
  ~MatcherWidgetI3DRSGM();
   void setImageWidth(int width);

public slots:
   void updateDisparityRange(int);
   void updateBlockSize(int);
   void updateMinDisparity(int);
   void updatePyramidLevel(int cap);
   void enableInterpolatation(bool enable);

   AbstractStereoMatcher* getMatcher(void);

   void onSaveClicked();

 private:
  Ui::MatcherWidgetI3DRSGM *ui;
  QMatcherI3DRSGM* matcher;
  int block_size;
  int min_disparity;
  int disparity_range;
  int image_width = 640;

};

#endif  // MATCHERWIDGETI3DRSGM_H
