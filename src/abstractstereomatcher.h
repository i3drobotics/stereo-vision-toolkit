/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef ABSTRACTSTEREOMATCHER_H
#define ABSTRACTSTEREOMATCHER_H

#include <QObject>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

class AbstractStereoMatcher : public QObject {
  Q_OBJECT
 public:
  explicit AbstractStereoMatcher(QObject* parent = 0, cv::Size image_size = cv::Size(0,0));
  ~AbstractStereoMatcher(void);

   cv::Mat disparity_lr;

 signals:
   void finished();

 public slots:
  void setImages(cv::Mat* left, cv::Mat* right);
  virtual void forwardMatch() = 0;
  virtual void backwardMatch() = 0;
  virtual void init(void) = 0;
  virtual int getErrorDisparity(void) = 0;
  void assignThread(QThread *thread);
  void getDisparity(cv::Mat &dst);
  void saveDisparity(QString filename);
  void checkLRConsistencyFull(double threshold);
  cv::Mat *getLeftImage(void){return left;}
  cv::Mat *getRighttImage(void){return right;}

  virtual void match();

 protected:
  cv::Mat *left;
  cv::Mat *right;

  cv::Mat disparity_buffer;
  cv::Mat disparity_rl;

  cv::Mat disparity_scale;

  cv::Size image_size;

  int min_disparity = 0;
  int disparity_range = 64;
  int block_size = 9;
};

#endif  // ABSTRACTSTEREOMATCHER_H
