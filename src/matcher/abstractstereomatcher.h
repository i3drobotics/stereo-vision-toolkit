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
//#include <opencv2/ximgproc.hpp>
#include <QStandardPaths>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

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

  //!  Save the disparity map
  /*!
  * @param[in] filename Output filename
  */
  void saveDisparity(QString filename);

  //!  Save the disparity map as normalised colormap
  /*!
  * @param[in] filename Output filename
  */
  void saveDisparityColormap(QString filename);

  //!  Perform a full left-right consistency check (experimental)
  /*!
   * \brief checkLRConsistencyFull
   * @param[in] threshold Disparity threshold
   */
  void checkLRConsistencyFull(double threshold);

  //!  Get a pointer to the left image
  cv::Mat getLeftImage(void){return left;}

  //!  Get a pointer to the right image
  cv::Mat getRightImage(void){return right;}

  void setDownsampleFactor(int factor){ downsample_factor=factor; };

  bool match(cv::Mat left_img, cv::Mat right_img);

  void normaliseDisparity(cv::Mat inDisparity, cv::Mat &outNormalisedDisparity);

  void getMinMaxDisparity(cv::Mat inDisparity, double &min_disp, double &max_disp);

  void disparity2colormap(cv::Mat inDisparity, cv::Mat &outColormap);

  void calcDepth(cv::Mat inDisparity, cv::Mat &outDepth);

  void calcPointCloud(cv::Mat inDepth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPoints);

private:
  //!  Set images for matching
  /*!
  * \param left Left image
  * \param right Right image
  */
 void convertImages(cv::Mat left_img, cv::Mat right_img, cv::Mat& left_conv_img, cv::Mat& right_conv_img);

 protected:

    //! Perform a match with the left image as the reference. This is normally what you want.
    virtual bool forwardMatch(cv::Mat left_img, cv::Mat right_img) = 0;

    //! Perform a match with the right image as the reference.
    virtual bool backwardMatch(cv::Mat left_img, cv::Mat right_img) = 0;

  cv::Mat left;
  cv::Mat right;

  cv::Mat disparity_buffer;
  cv::Mat disparity_rl;

  cv::Mat disparity16;

  cv::Size image_size;

  int min_disparity = 0;
  int disparity_range = 64;
  int block_size = 9;
  int downsample_factor = 1;
};

#endif  // ABSTRACTSTEREOMATCHER_H
