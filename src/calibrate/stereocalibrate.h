/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H

#include <abstractstereocamera.h>
#include <chessboard.h>
#include <calibrateconfirmdialog.h>
#include <QLabel>
#include <QtCore>
#include <QDir>
#include <QProgressDialog>
#include <algorithm>
#include <fstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

//!  Stereo calibration
/*!
  Calibrate stereo cameras
*/

class StereoCalibrate : public QObject {
  Q_OBJECT
  Q_PROPERTY(CalibrationStatus status NOTIFY doneCalibration)

 public:
  explicit StereoCalibrate(QObject* parent = 0,
                            AbstractStereoCamera* stereo_camera = 0);

  enum YamlFileFormat { CV_FILESTORAGE_YAML, ROS_PERCEPTION_YAML };
  enum class CalibrationStatus { SUCCESS, FAILED, ABORTED, INVALID_NUM_OF_IMAGES, INVALID_NUM_VALID_IMAGES };
  Q_ENUM(CalibrationStatus)

  cv::Mat left_camera_matrix;
  cv::Mat left_distortion;
  cv::Mat left_r_vecs;
  cv::Mat left_t_vecs;
  double left_rms_error;

  cv::Mat right_camera_matrix;
  cv::Mat right_distortion;
  cv::Mat right_r_vecs;
  cv::Mat right_t_vecs;
  double right_rms_error;

  cv::Mat stereo_r;
  cv::Mat stereo_t;
  cv::Mat stereo_e;
  cv::Mat stereo_f;
  cv::Mat stereo_q;
  double stereo_rms_error;

  cv::Mat left_rectification_x;
  cv::Mat left_rectification_y;
  cv::Mat right_rectification_x;
  cv::Mat right_rectification_y;

  bool success = false;
  QDir output_folder = QCoreApplication::applicationDirPath() + "/params/";

 private:

  QLabel* left_view;
  QLabel* right_view;
  CalibrateConfirmDialog* cal_dialog;
  AbstractStereoCamera* stereo_camera;
  std::vector<Chessboard*> board_orientations;
  std::vector<cv::Mat> left_images;
  std::vector<cv::Mat> right_images;
  cv::Size pattern_size = cv::Size(8, 6);

  std::vector<std::vector<cv::Point2f> > left_image_points;
  std::vector<std::vector<cv::Point2f> > right_image_points;
  std::vector<bool> left_valid;
  std::vector<bool> right_valid;

  cv::Mat R1, R2, P1, P2;

  bool save_ros = false;

  std::vector<cv::Point3f> pattern_points;

  int total_poses = 0;
  int current_pose = 0;
  int total_images = 0;

  double singleCameraCalibration(
      std::vector<cv::Mat>& images,
      std::vector<std::vector<cv::Point2f> >& imagePoints,
      std::vector<bool>& valid, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
      cv::Mat& rvecs, cv::Mat& tvecs, int cornerFlags = 0, int stereoFlags = 0);
  double stereoCameraCalibration(
      int stereoFlags = cv::CALIB_USE_INTRINSIC_GUESS);

  bool calibrating_left = true;
  bool calibrating_right = false;

  void finishedCalibration(CalibrationStatus success);
  bool findCorners(cv::Mat image, std::vector<cv::Point2f>& corners, int flags);

  cv::Mat left_image_overlay;
  cv::Mat right_image_overlay;

  cv::Size image_size;

 public slots:
  void abortCalibration();
  void setDisplays(QLabel* left, QLabel* right);
  void setBoardOrientations(std::vector<Chessboard*>& orientations);
  void updateViews(void);
  void startCalibration(void);
  void checkImages(void);
  void loadBoardPoses(std::string fname);
  bool imageValid(void);
  void overlayImage(cv::Mat& image, Chessboard* board = 0, bool found = false, bool valid = false);
  void setImages(QList<cv::Mat> left, QList<cv::Mat> right);
  void setPattern(cv::Size size, double squareSize);
  void setImageSize(cv::Size size);
  void setSaveROS(bool save){save_ros = save;}
  void setOutputPath(QString path);
  void overlayArrow(cv::Mat& image, std::vector<cv::Point2f>& points,
                    cv::Point2f offset, cv::Scalar colour, int thickness = 3);
  CalibrationStatus jointCalibration(void);
  bool outputYaml(YamlFileFormat yaml_file_format, QString filename, QString camera_name, cv::Size image_size, cv::Mat camera_matrix, cv::Mat dist_coeffs, cv::Mat P, cv::Mat R);

 signals:
  void doneCalibration(CalibrationStatus);
  void imageProgress(int, int);
  void requestImage(void);
  void chessboardFound(Chessboard* board);
  void done_image(int i);
};

#endif  // STEREOCALIBRATE_H
