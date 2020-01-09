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
#include <QStandardPaths>

//!  Stereo matcher base class
/*!
  An abstract class to match stereo images. This class should not be used directly,
  instead you should subclass it to add new matchers. The class provides a common interface for image
  matching including threaded execution, forward/back matching and disparity map saving.
*/
class AbstractStereoMatcher : public QObject {
  Q_OBJECT
 public:

  explicit AbstractStereoMatcher(QObject* parent = 0, cv::Size image_size = cv::Size(0,0));
  ~AbstractStereoMatcher(void);

   cv::Mat disparity_lr;

 signals:
   //! Emitted when a stereo pair has been matched.
   void finished();

 public slots:
   //!  Set images for matching
   /*!
   * @param[in] left Left image
   * @param[in] right Right image
   */
  void setImages(cv::Mat* left, cv::Mat* right);

  //! Perform a match with the left image as the reference. This is normally what you want.
  virtual void forwardMatch() = 0;

  //! Perform a match with the right image as the reference.
  virtual void backwardMatch() = 0;

  //! Setup the matcher.
  virtual void init(void) = 0;

  //! Returns what value is assigned to pixels with invalid disparities (e.g. -10000)
  virtual int getErrorDisparity(void) = 0;

  //!  Move matcher to a new thread
  /*!
  * @param[in] thread Pointer to thread
  */
  void assignThread(QThread *thread);

  //!  Get the disparity map
  /*!
  * @param[out] dst Output Mat
  */
  void getDisparity(cv::Mat &dst);

  //!  Save the disparity map
  /*!
  * @param[in] filename Output filename
  */
  void saveDisparity(QString filename);

  //!  Perform a full left-right consistency check (experimental)
  /*!
   * \brief checkLRConsistencyFull
   * @param[in] threshold Disparity threshold
   */
  void checkLRConsistencyFull(double threshold);

  //!  Get a pointer to the left image
  cv::Mat *getLeftImage(void){return left;}

  //!  Get a pointer to the right image
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
