/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "chessboard.h"

Chessboard::Chessboard(QObject *parent, cv::Size pattern, cv::Size imsize)
    : QObject(parent) {
  this->image_size = imsize;
  this->pattern = pattern;
}

void Chessboard::setHorizontalTilt(double minHt, double maxHt) {
  assert(minHt < maxHt);

  this->min_horizontal_tilt = minHt;
  this->max_horizontal_tilt = maxHt;
}

void Chessboard::setVerticalTilt(double minVt, double maxVt) {
  assert(minVt < maxVt);

  this->min_vertical_tilt = minVt;
  this->max_vertical_tilt = maxVt;
}

void Chessboard::setBoardArea(double minArea, double maxArea) {
  assert(minArea < maxArea);

  this->min_area = minArea;
  this->max_area = maxArea;
}

void Chessboard::setTemplate(std::vector<cv::Point2i> contour){
    template_contour = contour;
    template_area = cv::contourArea(contour);
}

void Chessboard::setBoardMargins(double leftMargin, double rightMargin,
                                  double topMargin, double bottomMargin) {
  this->left_margin = leftMargin;

  if (this->right_margin > 0) {
    this->right_margin = this->image_size.width - rightMargin;
  }
  this->top_margin = this->image_size.height - topMargin;

  if (this->bottom_margin > 0) {
    this->bottom_margin = this->image_size.height - bottomMargin;
  }
}

void Chessboard::getMargins() {
  top_points.clear();
  bottom_points.clear();
  left_points.clear();
  right_points.clear();
  vertices.clear();

  /* Check top */
  for (int i = 0; i < pattern.width; i++) {
    top_points.push_back(board_points.at(i));
  }

  /* Check bottom */
  for (int i = pattern.area() - pattern.width; i < pattern.area(); i++) {
    bottom_points.push_back(board_points.at(i));
  }

  /* Check left */
  for (int i = 0; i < pattern.height; i++) {
    left_points.push_back(board_points.at(i * pattern.width));
  }

  /* Check right */
  for (int i = 0; i < pattern.height; i++) {
    right_points.push_back(
        board_points.at((pattern.width - 1) + i * pattern.width));
  }

  vertices.push_back(top_points.front());
  vertices.push_back(top_points.back());
  vertices.push_back(bottom_points.back());
  vertices.push_back(bottom_points.front());
}

bool Chessboard::checkMargins(void) {
  left_out_of_bounds = false;
  right_out_of_bounds = false;
  top_out_of_bounds = false;
  bottom_out_of_bounds = false;

  if (left_margin > 0) {
    for (cv::Point2f point : left_points) {
      if (point.x > left_margin) {
        left_out_of_bounds = true;
        return false;
      }
    }
  }

  if (top_margin > 0) {
    for (cv::Point2f point : top_points) {
      if (point.y > top_margin) {
        top_out_of_bounds = true;
        return false;
      }
    }
  }

  if (bottom_margin > 0) {
    for (cv::Point2f point : bottom_points) {
      if (point.y < (this->image_size.height - bottom_margin)) {
        bottom_out_of_bounds = true;
        return false;
      }
    }
  }

  if (right_margin > 0) {
    for (cv::Point2f point : right_points) {
      if (point.x < (this->image_size.width - right_margin)) {
        right_out_of_bounds = true;
        return false;
      }
    }
  }

  return true;
}

void Chessboard::getTilts() {
  left_length = cv::norm(left_points.front() - left_points.back());
  right_length = cv::norm(right_points.front() - right_points.back());
  top_length = cv::norm(top_points.front() - top_points.back());
  bottom_length = cv::norm(bottom_points.front() - bottom_points.back());

  emit gotTilts(horizontal_tilt, vertical_tilt);

  if (left_length > right_length)
    horizontal_tilt = left_length / right_length - 1;
  else
    horizontal_tilt = -right_length / left_length + 1;

  if (top_length > bottom_length)
    vertical_tilt = top_length / bottom_length - 1;
  else
    vertical_tilt = -bottom_length / top_length + 1;
}

bool Chessboard::checkAgainstTemplate(){

    bool measure_distance = false;

    // Check if the points are within the template bounding box
    for(auto &vertex : vertices){
        if( cv::pointPolygonTest(template_contour, vertex, measure_distance) <= 0)
            return false;
    }

    // Check if the board area fills enough of the bounding box
    if(board_area < (template_area * fill_factor)) return false;
    if(board_area > template_area) return false;

    return true;
}

bool Chessboard::checkTilts() {
  horizontal_tilt_under = false;
  horizontal_tilt_over = false;
  vertical_tilt_under = false;
  vertical_tilt_over = false;

  if (horizontal_tilt < min_horizontal_tilt) {
    horizontal_tilt_under = true;
    return false;
  } else if (horizontal_tilt > max_horizontal_tilt) {
    horizontal_tilt_over = true;
    return false;
  }

  if (vertical_tilt < min_vertical_tilt) {
    vertical_tilt_under = true;
    return false;
  } else if (vertical_tilt > max_vertical_tilt) {
    vertical_tilt_over = true;
    return false;
  }

  return true;
}

bool Chessboard::checkArea() {
  board_area_under = false;
  board_area_over = false;

  if (board_area < min_area) {
    board_area_under = true;
    return false;
  } else if (board_area > max_area) {
    board_area_over = true;
    return false;
  } else {
    return true;
  }
}

bool Chessboard::check(std::vector<cv::Point2f> &detected_points) {
  board_points = detected_points;
  bool res = false;

  getMargins();

  board_area = cv::contourArea(vertices);
  emit gotArea(board_area);

  res = checkAgainstTemplate();

  valid = res;

  return res;
}

bool Chessboard::isValid(void) { return this->valid; }
