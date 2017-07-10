/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H

#include <abstractstereocamera.h>
#include <chessboard.h>
#include <QLabel>
#include <QtCore>
#include <algorithm>
#include <fstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class StereoCalibrate : public QObject {
  Q_OBJECT
 public:
  explicit StereoCalibrate(QObject* parent = 0,
                            AbstractStereoCamera* stereoCamera = 0);

  cv::Mat leftCameraMatrix;
  cv::Mat leftDistCoeffs;
  cv::Mat leftRvecs;
  cv::Mat leftTvecs;
  double leftRMSError;

  cv::Mat rightCameraMatrix;
  cv::Mat rightDistCoeffs;
  cv::Mat rightRvecs;
  cv::Mat rightTvecs;
  double rightRMSError;

  cv::Mat stereoR;
  cv::Mat stereoT;
  cv::Mat stereoE;
  cv::Mat stereoF;
  cv::Mat stereoQ;

  cv::Mat leftRectmapX;
  cv::Mat leftRectmapY;
  cv::Mat rightRectmapX;
  cv::Mat rightRectmapY;

 private:
  QLabel* leftView;
  QLabel* rightView;
  AbstractStereoCamera* stereoCamera;
  std::vector<Chessboard*> boardOrientations;
  std::vector<cv::Mat> leftImages;
  std::vector<cv::Mat> rightImages;
  cv::Size patternsize = cv::Size(8, 6);

  std::vector<std::vector<cv::Point2f> > leftImagePoints;
  std::vector<std::vector<cv::Point2f> > rightImagePoints;
  std::vector<bool> leftValid;
  std::vector<bool> rightValid;

  std::vector<cv::Point3f> patternPoints;

  int totalPoses = 0;
  int currentPose = 0;
  int totalImages = 0;

  double singleCameraCalibration(
      std::vector<cv::Mat>& images,
      std::vector<std::vector<cv::Point2f> >& imagePoints,
      std::vector<bool>& valid, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
      cv::Mat& rvecs, cv::Mat& tvecs, int cornerFlags = 0, int stereoFlags = 0);
  double stereoCameraCalibration(
      int stereoFlags = cv::CALIB_USE_INTRINSIC_GUESS);
  void jointCalibration(void);
  bool calibratingLeft = true;
  bool calibratingRight = false;

  void finishedCalibration();
  bool findCorners(cv::Mat image, std::vector<cv::Point2f>& corners, int flags);

  cv::Mat leftImageOverlay;
  cv::Mat rightImageOverlay;

  cv::Size imageSize;

 public slots:
  void abortCalibration();
  void setDisplays(QLabel* left, QLabel* right);
  void setBoardOrientations(std::vector<Chessboard*>& orientations);
  void updateViews(void);
  void startCalibration(void);
  void checkImages(void);
  void loadBoardPoses(std::string fname);
  bool imageValid(void);
  void overlayImage(cv::Mat& image, Chessboard* board = 0, bool found = false);
  void fromImages(QList<QString> left, QList<QString> right);
  void setPattern(cv::Size size, double squareSize);
  void setImageSize(cv::Size size);
  void overlayArrow(cv::Mat& image, std::vector<cv::Point2f>& points,
                    cv::Point2f offset, CvScalar colour, int thickness = 3);

 signals:
  void doneCalibration(bool);
  void imageProgress(int, int);
  void requestImage(void);
  void chessboardFound(Chessboard* board);
};

#endif  // STEREOCALIBRATE_H
