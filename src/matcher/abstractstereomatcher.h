/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef ABSTRACTSTEREOMATCHER_H
#define ABSTRACTSTEREOMATCHER_H

#include <QObject>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <QStandardPaths>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cvsupport.hpp"

//!  Stereo matcher base class
/*!
  An abstract class to match stereo images. This class should not be used directly,
  instead you should subclass it to add new matchers. The class provides a common interface for image
  matching including threaded execution, forward/back matching and disparity map saving.
*/
class AbstractStereoMatcher : public QObject {
  Q_OBJECT
 public:

  explicit AbstractStereoMatcher(QObject* parent = 0);
  ~AbstractStereoMatcher(void);

   cv::Mat disparity_lr;

   //! Returns if a valid license was found to use the matcher
   virtual bool isLicenseValid(void) = 0;

 signals:
   //! Emitted when a stereo pair has been matched.
   void finished();

 public slots:

  //! Setup the matcher.
  virtual void init(void) = 0;

  //! Returns what value is assigned to pixels with invalid disparities (e.g. -10000)
  virtual int getErrorDisparity(void) = 0;

  //!  Move matcher to a new thread
  /*!
  * @param[in] thread Pointer to thread
  */
  void assignThread(QThread *thread);

  void stopThread();

  //!  Get the disparity map
  /*!
  * @param[out] dst Output Mat
  */
  void getDisparity(cv::Mat &dst);

  //! Get disparity map after removing multiply factor from matching (16)
  /*!
  * @param[out] dst Output Mat
  */
  void getDisparity16(cv::Mat &dst);

  //!  Get the min disparity value
  void getMinDisparity(int &val);

  //!  Get the min disparity value
  void getDisparityRange(int &val);

  //!  Perform a full left-right consistency check (experimental)
  /*!
   * \brief checkLRConsistencyFull
   * @param[in] threshold Disparity threshold
   */
  void checkLRConsistencyFull(double threshold);

  //!  Get left image used in stereo match
  cv::Mat getLeftImage(void){return left;}

  //! Get left image in colour
  //! Actual matching is done is grayscale but can be useful for
  //! future processing such as point clouds to store original colour image
  cv::Mat getLeftBGRImage(void){return left_bgr;}

  //!  Get right image used in stereo match
  cv::Mat getRightImage(void){return right;}

  void setDownsampleFactor(int factor){ downsample_factor=factor; };

  bool match(cv::Mat left_img, cv::Mat right_img);

private:
  //!  Set images for matching
  /*!
  * @param[in] left_img left image
  * @param[in] right_img right image
  * @param[out] left_bgr_conv_img left image converted to bgr
  * @param[out] left_conv_img left image converted
  * @param[out] right_conv_img right image converted
  */
 void convertImages(cv::Mat left_img, cv::Mat right_img, cv::Mat& left_bgr_conv_img, cv::Mat& left_conv_img, cv::Mat& right_conv_img);

 protected:

    //! Perform a match with the left image as the reference. This is normally what you want.
    virtual bool forwardMatch(cv::Mat left_img, cv::Mat right_img) = 0;

    //! Perform a match with the right image as the reference.
    virtual bool backwardMatch(cv::Mat left_img, cv::Mat right_img) = 0;

  cv::Mat left;
  cv::Mat right;
  cv::Mat left_bgr;

  cv::Mat disparity_buffer;
  cv::Mat disparity_rl;

  cv::Mat disparity16;

  cv::Size image_size;

  bool sizeChangedThisFrame = false;

  int min_disparity = 0;
  int disparity_range = 64;
  int block_size = 9;
  int downsample_factor = 1;
};

#endif  // ABSTRACTSTEREOMATCHER_H
