/*
 * Copyright I3D Robotics Ltd, 2017
 * Author: Josh Veitch-Michaelis
 */

#ifndef CALIBRATIONCHECKERBOARD_H
#define CALIBRATIONCHECKERBOARD_H

#include <QDebug>
#include <QObject>
#include <opencv2/opencv.hpp>

/**
 * @brief Class for chessboard detection
 * @brief Detection and refinement of chessboard (also known as checkerboard) detection using openCV
 */
class Chessboard : public QObject {
  Q_OBJECT
 public:
 /**
  * @brief Construct a new Chessboard object
  * 
  * @param parent 
  * @param pattern 
  * @param imsize 
  */
  explicit Chessboard(QObject *parent = 0, cv::Size pattern = cv::Size(0, 0),
                      cv::Size imsize = cv::Size(0, 0));
  double min_horizontal_tilt = -0.1;
  double max_horizontal_tilt = 0.1;
  double min_vertical_tilt = -0.1;
  double max_vertical_tilt = 0.1;
  double min_area = 0.4;
  double max_area = 1;

  double left_margin = -1;
  double right_margin = -1;
  double top_margin = -1;
  double bottom_margin = -1;

  bool valid = false;

  double horizontal_tilt;
  double vertical_tilt;
  double left_length;
  double right_length;
  double top_length;
  double bottom_length;
  bool left_out_of_bounds = true;
  bool right_out_of_bounds = true;
  bool top_out_of_bounds = true;
  bool bottom_out_of_bounds = true;
  bool horizontal_tilt_under = true;
  bool horizontal_tilt_over = true;
  bool vertical_tilt_under = true;
  bool vertical_tilt_over = true;
  bool board_area_under;
  bool board_area_over;

  std::vector<cv::Point2f> board_points;
  std::vector<cv::Point2f> top_points;
  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  std::vector<cv::Point2f> bottom_points;
  std::vector<cv::Point2f> vertices;
  std::vector<cv::Point2i> template_contour;

  void setTemplate(std::vector<cv::Point2i> contour);
  bool checkAgainstTemplate();

  double getArea() { return board_area; }

 public slots:

  void setHorizontalTilt(double min_horizontal_tilt, double max_horizontal_tilt);
  void setVerticalTilt(double min_vertical_tilt, double max_vertical_tilt);
  void setBoardArea(double min_area, double max_area);
  void setBoardMargins(double left_margin, double right_margin, double top_margin,
                       double bottom_margin);

  bool check(std::vector<cv::Point2f> &board_points);
  bool isValid(void);

 signals:
  void gotTilts(double horizontal, double vertical);
  void gotArea(double area);

 private:
  cv::Size image_size;
  cv::Size pattern;

  double template_area = DBL_MAX;
  double board_area = DBL_MAX;
  double fill_factor = 0.90;

  void getMargins();
  bool checkMargins();
  void getTilts();
  bool checkTilts();
  bool checkArea();
};

#endif  // CALIBRATIONCHECKERBOARD_H
