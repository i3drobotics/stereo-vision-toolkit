#include "chessboard.h"

Chessboard::Chessboard(QObject *parent, cv::Size pattern, cv::Size imsize)
    : QObject(parent) {
  this->imSize = imsize;
  this->pattern = pattern;
}

void Chessboard::setHorizontalTilt(double minHt, double maxHt) {
  assert(minHt < maxHt);

  this->minHt = minHt;
  this->maxHt = maxHt;
}

void Chessboard::setVerticalTilt(double minVt, double maxVt) {
  assert(minVt < maxVt);

  this->minVt = minVt;
  this->maxVt = maxVt;
}

void Chessboard::setBoardArea(double minArea, double maxArea) {
  assert(minArea < maxArea);

  this->minArea = minArea;
  this->maxArea = maxArea;
}

void Chessboard::setTemplate(std::vector<cv::Point2f> contour){
    template_contour = contour;
    template_area = cv::contourArea(contour);
}

void Chessboard::setBoardMargins(double leftMargin, double rightMargin,
                                  double topMargin, double bottomMargin) {
  this->leftMargin = leftMargin;

  if (this->rightMargin > 0) {
    this->rightMargin = this->imSize.width - rightMargin;
  }
  this->topMargin = this->imSize.height - topMargin;

  if (this->bottomMargin > 0) {
    this->bottomMargin = this->imSize.height - bottomMargin;
  }
}

void Chessboard::getMargins() {
  topPoints.clear();
  bottomPoints.clear();
  leftPoints.clear();
  rightPoints.clear();
  vertices.clear();

  /* Check top */
  for (int i = 0; i < pattern.width; i++) {
    topPoints.push_back(boardPoints.at(i));
  }

  /* Check bottom */
  for (int i = pattern.area() - pattern.width; i < pattern.area(); i++) {
    bottomPoints.push_back(boardPoints.at(i));
  }

  /* Check left */
  for (int i = 0; i < pattern.height; i++) {
    leftPoints.push_back(boardPoints.at(i * pattern.width));
  }

  /* Check right */
  for (int i = 0; i < pattern.height; i++) {
    rightPoints.push_back(
        boardPoints.at((pattern.width - 1) + i * pattern.width));
  }

  vertices.push_back(topPoints.front());
  vertices.push_back(topPoints.back());
  vertices.push_back(bottomPoints.back());
  vertices.push_back(bottomPoints.front());
}

bool Chessboard::checkMargins(void) {
  leftOutOfBounds = false;
  rightOutOfBounds = false;
  topOutOfBounds = false;
  bottomOutOfBounds = false;

  if (leftMargin > 0) {
    for (cv::Point2f point : leftPoints) {
      if (point.x > leftMargin) {
        leftOutOfBounds = true;
        return false;
      }
    }
  }

  if (topMargin > 0) {
    for (cv::Point2f point : topPoints) {
      if (point.y > topMargin) {
        topOutOfBounds = true;
        return false;
      }
    }
  }

  if (bottomMargin > 0) {
    for (cv::Point2f point : bottomPoints) {
      if (point.y < (this->imSize.height - bottomMargin)) {
        bottomOutOfBounds = true;
        return false;
      }
    }
  }

  if (rightMargin > 0) {
    for (cv::Point2f point : rightPoints) {
      if (point.x < (this->imSize.width - rightMargin)) {
        rightOutOfBounds = true;
        return false;
      }
    }
  }

  return true;
}

void Chessboard::getTilts() {
  leftLength = cv::norm(leftPoints.front() - leftPoints.back());
  rightLength = cv::norm(rightPoints.front() - rightPoints.back());
  topLength = cv::norm(topPoints.front() - topPoints.back());
  bottomLength = cv::norm(bottomPoints.front() - bottomPoints.back());

  emit gotTilts(horizontalTilt, verticalTilt);

  if (leftLength > rightLength)
    horizontalTilt = leftLength / rightLength - 1;
  else
    horizontalTilt = -rightLength / leftLength + 1;

  if (topLength > bottomLength)
    verticalTilt = topLength / bottomLength - 1;
  else
    verticalTilt = -bottomLength / topLength + 1;
}

bool Chessboard::checkAgainstTemplate(){

    bool measure_distance = false;

    // Check if the points are within the template bounding box
    for(auto &vertex : vertices){
        if( ! cv::pointPolygonTest(template_contour, vertex, measure_distance))
            return false;
    }

    // Check if the board area fills enough of the bounding box
    if(board_area < template_area * fill_factor) return false;

    return true;
}

bool Chessboard::checkTilts() {
  horizontalTiltUnder = false;
  horizontalTiltOver = false;
  verticalTiltUnder = false;
  verticalTiltOver = false;

  if (horizontalTilt < minHt) {
    horizontalTiltUnder = true;
    return false;
  } else if (horizontalTilt > maxHt) {
    horizontalTiltOver = true;
    return false;
  }

  if (verticalTilt < minVt) {
    verticalTiltUnder = true;
    return false;
  } else if (verticalTilt > maxVt) {
    verticalTiltOver = true;
    return false;
  }

  return true;
}

bool Chessboard::checkArea() {
  boardAreaUnder = false;
  boardAreaOver = false;

  if (board_area < minArea) {
    boardAreaUnder = true;
    return false;
  } else if (board_area > maxArea) {
    boardAreaOver = true;
    return false;
  } else {
    return true;
  }
}

bool Chessboard::check(std::vector<cv::Point2f> &boardPoints) {
  boardPoints = boardPoints;
  bool res = false;

  board_area = cv::contourArea(vertices);
  emit gotArea(board_area);

  checkAgainstTemplate();

  valid = res;

  return res;
}

bool Chessboard::isValid(void) { return this->valid; }
