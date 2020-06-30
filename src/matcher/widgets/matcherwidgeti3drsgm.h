/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MATCHERWIDGETI3DRSGM_H
#define MATCHERWIDGETI3DRSGM_H

#include <matcherwidget.h>
#include <matcheri3drsgm.h>

namespace Ui {
class MatcherWidgetI3DRSGM;
}

//!  I3DR's SGM QT Widget
/*!
  QT widget for I3DR's SGM controls
*/

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
   explicit MatcherWidgetI3DRSGM(QWidget *parent = 0);
  ~MatcherWidgetI3DRSGM();
   void setImageWidth(int width);

public slots:
   void updateDisparityRange(int);
   void updateBlockSize(int);
   void updateMinDisparity(int);
   void updatePyramidLevel(int cap);
   void enableExtendDisparity(bool enable);
   void enableNegativeDisparity(bool enable);
   void enableInterpolatation(bool enable);

   AbstractStereoMatcher* getMatcher(void);

   void onSaveClicked();

 private:
  Ui::MatcherWidgetI3DRSGM *ui;
  MatcherI3DRSGM* matcher;
  int block_size;
  int min_disparity;
  int disparity_range;
  int image_width = 640;
  bool negative_disparity = true;

};

#endif  // MATCHERWIDGETI3DRSGM_H
