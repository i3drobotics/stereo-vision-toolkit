/*
 * Copyright I3D Robotics Ltd, 2017
 * Author: Josh Veitch-Michaelis
 */

#ifndef CALIBRATIONCHECKERBOARD_H
#define CALIBRATIONCHECKERBOARD_H

#include <QDebug>
#include <QObject>
#include <opencv2/opencv.hpp>

class Chessboard : public QObject {
  Q_OBJECT
 public:
  explicit Chessboard(QObject *parent = 0, cv::Size pattern = cv::Size(0, 0),
                      cv::Size imsize = cv::Size(0, 0));
  double minHt = -0.1;
  double maxHt = 0.1;
  double minVt = -0.1;
  double maxVt = 0.1;
  double minArea = 0.4;
  double maxArea = 1;

  double leftMargin = -1;
  double rightMargin = -1;
  double topMargin = -1;
  double bottomMargin = -1;

  bool valid = false;

  double horizontalTilt;
  double verticalTilt;
  double leftLength;
  double rightLength;
  double topLength;
  double bottomLength;
  bool leftOutOfBounds = true;
  bool rightOutOfBounds = true;
  bool topOutOfBounds = true;
  bool bottomOutOfBounds = true;
  bool horizontalTiltUnder = true;
  bool horizontalTiltOver = true;
  bool verticalTiltUnder = true;
  bool verticalTiltOver = true;
  bool boardAreaUnder;
  bool boardAreaOver;

  std::vector<cv::Point2f> boardPoints;
  std::vector<cv::Point2f> topPoints;
  std::vector<cv::Point2f> leftPoints;
  std::vector<cv::Point2f> rightPoints;
  std::vector<cv::Point2f> bottomPoints;
  std::vector<cv::Point2f> vertices;

  void setTemplate(std::vector<cv::Point2f> contour);
  bool checkAgainstTemplate();

  double getArea() { return board_area; }

 public slots:

  void setHorizontalTilt(double minHt, double maxHt);
  void setVerticalTilt(double minVt, double maxVt);
  void setBoardArea(double minArea, double maxArea);
  void setBoardMargins(double leftMargin, double rightMargin, double topMargin,
                       double bottomMargin);

  bool check(std::vector<cv::Point2f> &boardPoints);
  bool isValid(void);

 signals:
  void gotTilts(double horizontal, double vertical);
  void gotArea(double area);

 private:
  cv::Size imSize;
  cv::Size pattern;

  std::vector<cv::Point2f> template_contour;

  double template_area = DBL_MAX;
  double board_area = DBL_MAX;
  double fill_factor = 0.8;

  void getMargins();
  bool checkMargins();
  void getTilts();
  bool checkTilts();
  bool checkArea();
};

#endif  // CALIBRATIONCHECKERBOARD_H
