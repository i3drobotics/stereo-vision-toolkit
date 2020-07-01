/*
 * Copyright I3D Robotics Ltd, 2017
 * Author: Josh Veitch-Michaelis
 */

#ifndef CALIBRATECONFIRMDIALOG_H
#define CALIBRATECONFIRMDIALOG_H

#include <QDialog>
#include <opencv2/opencv.hpp>

namespace Ui {
class CalibrateConfirmDialog;
}

//!  Calibration confirmation dialog
/*!
  A dialog view to display camera calibration results and progress.
*/
class CalibrateConfirmDialog : public QDialog {
  Q_OBJECT

 public:
  explicit CalibrateConfirmDialog(QWidget *parent = 0);
  ~CalibrateConfirmDialog();

  //! Display/update the calibration parameters for the left camera
  /*!
   * \param camera_matrix Camera intrinsic matrix
   * \param distortion Distortion coefficients
   * \param rms RMS reprojection error
  */
  void updateLeft(const cv::Mat &camera_matrix, const cv::Mat &distortion,
             const double &rms);

  //! Display/update the calibration parameters for the right camera
  /*!
   * \param camera_matrix Camera intrinsic matrix
   * \param distortion Distortion coefficients
   * \param rms RMS reprojection error
  */
  void updateRight(const cv::Mat &camera_matrix, const cv::Mat &distortion,
                   const double &rms);

  //! Display/update the calibration parameters for the stereo system
  /*!
   * \param Q Q matrix (see OpenCV documentation)
   * \param rms RMS reprojection error
  */
  void updateStereo(const cv::Mat Q, const double &rms);

  //! Set number of images
  /*!
   * \param number_images set number of images to update progress
  */
  void setNumberImages(int number_images);

public slots:
  //! Update the number of left images processed
  /*! Called during calibration process
   * \param number_done Number of images processed
  */
  void updateLeftProgress(int number_done);

  //! Update the number of right images processed
  /*! Called during calibration process
   * \param number_done Number of images processed
  */
  void updateRightProgress(int number_done);

private:
  //! QT UI dialog
  Ui::CalibrateConfirmDialog *ui;
};

#endif  // CALIBRATECONFIRMDIALOG_H
