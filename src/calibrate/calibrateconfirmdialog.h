/*
 * Copyright I3D Robotics Ltd, 2017
 * Author: Josh Veitch-Michaelis
 */

#ifndef CALIBRATECONFIRMDIALOG_H
#define CALIBRATECONFIRMDIALOG_H

#include <QDialog>
#include <opencv2/opencv.hpp>

namespace Ui {
class calibrateconfirmdialog;
}

//!  Calibration confirmation dialog
/*!
  A dialog view to display camera calibration results and progress.
*/
class calibrateconfirmdialog : public QDialog {
  Q_OBJECT

 public:
  explicit calibrateconfirmdialog(QWidget *parent = 0);
  ~calibrateconfirmdialog();

  //! Display/update the calibration parameters for the left camera
    /*!
   * @\param[in] camera_matrix Camera intrinsic matrix
   * @\param[in] distortion Distortion coefficients
   * @\param[in] rms RMS reprojection error
   */
  void updateLeft(const cv::Mat &camera_matrix, const cv::Mat &distortion,
             const double &rms);

  //! Display/update the calibration parameters for the right camera
    /*!
   * @\param[in] camera_matrix Camera intrinsic matrix
   * @\param[in] distortion Distortion coefficients
   * @\param[in] rms RMS reprojection error
   */
  void updateRight(const cv::Mat &camera_matrix, const cv::Mat &distortion,
                   const double &rms);

  //! Display/update the calibration parameters for the stereo system
    /*!
   * @\param[in] Q Q matrix (see OpenCV documentation)
   * @\param[in] rms RMS reprojection error
   */
  void updateStereo(const cv::Mat Q, const double &rms);

  //! Display/update the number of images used
  void setNumberImages(int number_images);

public slots:
  //! Update the number of left images processed
  /*!
   * \brief Called during calibration process
   * @\param[in] number_done Number of images processed
   */
  void updateLeftProgress(int number_done);

  //! Update the number of right images processed
  /*!
   * \brief Called during calibration process
   * @\param[in] number_done Number of images processed
   */
  void updateRightProgress(int number_done);

private:
  Ui::calibrateconfirmdialog *ui;
};

#endif  // CALIBRATECONFIRMDIALOG_H
