#include "stereocalibrate.h"

StereoCalibrate::StereoCalibrate(QObject* parent,
                                   AbstractStereoCamera* stereoCamera)
    : QObject(parent) {
  this->stereoCamera = stereoCamera;

  if (stereoCamera) {
    imageSize = cv::Size(stereoCamera->getWidth(), stereoCamera->getHeight());
  }
}

void StereoCalibrate::setPattern(cv::Size size, double squareSize) {
  patternsize = size;

  patternPoints.clear();

  for (int j = 0; j < patternsize.height; j++) {
    for (int i = 0; i < patternsize.width; i++) {
      patternPoints.push_back(cv::Point3f(i * squareSize, j * squareSize, 0));
    }
  }
}

void StereoCalibrate::setImageSize(cv::Size size) { this->imageSize = size; }

void StereoCalibrate::abortCalibration() {
  disconnect(this->stereoCamera, SIGNAL(acquired()), this, SLOT(checkImages()));
  disconnect(this, SIGNAL(requestImage()), this->stereoCamera,
             SLOT(singleShot()));
  emit doneCalibration(false);
}

void StereoCalibrate::finishedCalibration() {
  disconnect(this->stereoCamera, SIGNAL(acquired()), this, SLOT(checkImages()));
  disconnect(this, SIGNAL(requestImage()), this->stereoCamera,
             SLOT(singleShot()));
  emit doneCalibration(true);
}

void StereoCalibrate::setDisplays(QLabel* left, QLabel* right) {
  this->leftView = left;
  this->rightView = right;
}

void StereoCalibrate::setBoardOrientations(
    std::vector<Chessboard*>& orientations) {
  this->boardOrientations = orientations;
}

void StereoCalibrate::startCalibration(void) {

  imageProgress(0, totalPoses);

  totalImages = 0;

  leftImages.clear();
  rightImages.clear();

  connect(this->stereoCamera, SIGNAL(acquired()), this, SLOT(checkImages()));
  connect(this, SIGNAL(requestImage()), this->stereoCamera, SLOT(singleShot()));
  this->stereoCamera->singleShot();
}

void StereoCalibrate::checkImages(void) {
  if (currentPose == totalPoses) {
    if (calibratingLeft) {
      calibratingLeft = false;

      currentPose = 0;
      imageProgress(currentPose, totalPoses);

      calibratingRight = true;
    } else if (calibratingRight) {
      calibratingRight = false;
    } else {
      jointCalibration();
    }
  } else {
    if (imageValid()) {
      currentPose++;

      imageProgress(currentPose, totalPoses);
      cv::Mat image;

      std::string fname;

      // We need to save both left and right images for every view
      stereoCamera->getLeftImage(image);
      leftImages.push_back(image);
      fname = QString("calibration_l_%1.png").arg(totalImages).toStdString();
      cv::imwrite(fname, image);

      stereoCamera->getRightImage(image);
      rightImages.push_back(image);
      fname = QString("calibration_r_%1.png").arg(totalImages).toStdString();
      cv::imwrite(fname, image);

      totalImages++;
    }
  }

  emit requestImage();
  return;
}

void StereoCalibrate::jointCalibration(void) {
  assert(leftImages.size() != 0);
  assert(rightImages.size() != 0);
  assert(leftImagePoints.size() == rightImagePoints.size());

  setImageSize(leftImages.at(0).size());

  int cornerFlags = cv::CALIB_CB_ADAPTIVE_THRESH +
                    cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;

  /* Left Camera */
  leftValid.clear();
  leftRMSError = singleCameraCalibration(leftImages, leftImagePoints, leftValid,
                                         leftCameraMatrix, leftDistCoeffs,
                                         leftRvecs, leftTvecs, cornerFlags);

  cv::FileStorage leftIntrinsicFS("left_calibration.xml",
                                  cv::FileStorage::WRITE);
  leftIntrinsicFS << "cameraMatrix" << leftCameraMatrix;
  leftIntrinsicFS << "distCoeffs" << leftDistCoeffs;
  leftIntrinsicFS << "rms_error" << leftRMSError;
  leftIntrinsicFS.release();

  qDebug() << "Left RMS reprojection error: " << leftRMSError;

  /* Right Camera */
  rightValid.clear();
  rightRMSError = singleCameraCalibration(
      rightImages, rightImagePoints, rightValid, rightCameraMatrix,
      rightDistCoeffs, rightRvecs, rightTvecs, cornerFlags);

  cv::FileStorage rightIntrinsicFS("right_calibration.xml",
                                   cv::FileStorage::WRITE);
  rightIntrinsicFS << "cameraMatrix" << rightCameraMatrix;
  rightIntrinsicFS << "distCoeffs" << rightDistCoeffs;
  rightIntrinsicFS << "rms_error" << rightRMSError;
  rightIntrinsicFS.release();

  qDebug() << "Right RMS reprojection error: " << rightRMSError;

  qDebug() << "Stereo RMS reprojection error: " << stereoCameraCalibration();

  finishedCalibration();
}

double StereoCalibrate::stereoCameraCalibration(int stereoFlags) {

  /* Check valid images */
  std::vector<std::vector<cv::Point2f> > leftImagePointsMask;
  std::vector<std::vector<cv::Point2f> > rightImagePointsMask;
  std::vector<std::vector<cv::Point3f> > objectPointsMask;

  /* Mask image arrays */
  for (int i = 0; i < leftValid.size(); i++) {
    if (leftValid[i] && rightValid[i]) {
      leftImagePointsMask.push_back(leftImagePoints[i]);
      rightImagePointsMask.push_back(rightImagePoints[i]);
      objectPointsMask.push_back(patternPoints);
    }
  }

  assert(leftImagePointsMask.size() > 6);
  assert(rightImagePointsMask.size() > 6);

  double stereoRes = cv::stereoCalibrate(
      objectPointsMask, leftImagePointsMask, rightImagePointsMask,
      leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs,
      imageSize, stereoR, stereoT, stereoE, stereoF, stereoFlags);

  cv::Mat R1, R2, P1, P2;

  cv::stereoRectify(leftCameraMatrix, leftDistCoeffs, rightCameraMatrix,
                    rightDistCoeffs, imageSize, stereoR, stereoT, R1, R2, P1,
                    P2, stereoQ, 0);

  cv::initUndistortRectifyMap(leftCameraMatrix, leftDistCoeffs, R1, P1,
                              imageSize, CV_32FC1, leftRectmapX, leftRectmapY);
  cv::initUndistortRectifyMap(rightCameraMatrix, rightDistCoeffs, R2, P2,
                              imageSize, CV_32FC1, rightRectmapX,
                              rightRectmapY);

  cv::FileStorage stereoFS("stereo_calibration.xml", cv::FileStorage::WRITE);
  stereoFS << "R" << stereoR;
  stereoFS << "T" << stereoT;
  stereoFS << "Q" << stereoQ;
  stereoFS << "E" << stereoE;
  stereoFS << "F" << stereoF;
  stereoFS << "rms_error" << stereoRes;
  stereoFS.release();

  cv::FileStorage leftRectFs("left_rectification.xml", cv::FileStorage::WRITE);
  leftRectFs << "x" << leftRectmapX;
  leftRectFs << "y" << leftRectmapY;
  leftRectFs.release();

  cv::FileStorage rightRectFS("right_rectification.xml",
                              cv::FileStorage::WRITE);
  rightRectFS << "x" << rightRectmapX;
  rightRectFS << "y" << rightRectmapY;
  rightRectFS.release();

  return stereoRes;
}

void StereoCalibrate::fromImages(QList<QString> left, QList<QString> right) {
  leftImages.clear();
  rightImages.clear();

  for (auto& fname : left) {
    cv::Mat im = cv::imread(fname.toStdString(), cv::IMREAD_GRAYSCALE);
    if (im.total() > 0) leftImages.push_back(im);
  }

  for (auto& fname : right) {
    cv::Mat im = cv::imread(fname.toStdString(), cv::IMREAD_GRAYSCALE);
    if (im.total() > 0) rightImages.push_back(im);
  }

  jointCalibration();
}

double StereoCalibrate::singleCameraCalibration(
    std::vector<cv::Mat>& images,
    std::vector<std::vector<cv::Point2f> >& imagePoints,
    std::vector<bool>& valid, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
    cv::Mat& rvecs, cv::Mat& tvecs, int cornerFlags, int stereoFlags) {
  std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<cv::Point2f> corners;
  double res;
  int i = 0;

  imagePoints.clear();

  std::vector<std::vector<cv::Point2f> > validImagePoints;

  for (cv::Mat image : images) {
    if (findCorners(image, corners, cornerFlags)) {
      imagePoints.push_back(corners);
      validImagePoints.push_back(corners);
      objectPoints.push_back(patternPoints);
      valid.push_back(true);
    } else {
      valid.push_back(false);
      corners.clear();
      imagePoints.push_back(corners);
    }

    i++;
  }

  assert(objectPoints.size() == validImagePoints.size());

  res =
      cv::calibrateCamera(objectPoints, validImagePoints, imageSize,
                          cameraMatrix, distCoeffs, rvecs, tvecs, stereoFlags);

  return res;
}

bool StereoCalibrate::findCorners(cv::Mat image,
                                   std::vector<cv::Point2f>& corners,
                                   int flags) {
  bool found = cv::findChessboardCorners(image, patternsize, corners, flags);

  if (found) {
    cv::cornerSubPix(
        image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    return true;
  }

  return false;
}

void StereoCalibrate::overlayImage(cv::Mat& image, Chessboard* board,
                                    bool found) {
  CvScalar red(255, 0, 0, 255);
  CvScalar green(0, 255, 0, 255);
  CvScalar overlayColour = red;

  if (board->leftMargin > 0) {
    cv::line(image, cv::Point2f(board->leftMargin, 0),
             cv::Point2f(board->leftMargin, image.rows), overlayColour, 4);
  }

  if (board->rightMargin > 0) {
    cv::line(image, cv::Point2f(image.cols - board->rightMargin, 0),
             cv::Point2f(image.cols - board->rightMargin, image.rows),
             overlayColour, 4);
  }

  if (board->topMargin > 0) {
    cv::line(image, cv::Point2f(0, board->topMargin),
             cv::Point2f(image.cols, board->topMargin), overlayColour, 4);
  }

  if (board->bottomMargin > 0) {
    cv::line(image, cv::Point2f(0, image.rows - board->bottomMargin),
             cv::Point2f(image.cols, image.rows - board->bottomMargin),
             overlayColour, 4);
  }

  cv::drawChessboardCorners(image, patternsize, board->boardPoints, found);

  if (found) {

    // Draw board outline
    cv::line(image, board->vertices[0], board->vertices[1], green, 3);
    cv::line(image, board->vertices[1], board->vertices[2], green, 3);
    cv::line(image, board->vertices[2], board->vertices[3], green, 3);
    cv::line(image, board->vertices[3], board->vertices[0], green, 3);

    std::vector<cv::Point> ellipse_points;
/*
    if (board->horizontalTiltOver) {

        auto left_margin_middle = (board->leftPoints.front()+board->leftPoints.back())/2;

        cv::ellipse2Poly(left_margin_middle + cv::Point2f(50,0) , cv::Size(100, 50), 0, 180, 10, 10, ellipse_points);
        cv::polylines(image, ellipse_points, false, red, 5);
        cv::line(image, ellipse_points.back(), ellipse_points.back()+cv::Point(20,0), red, 5);
        cv::line(image, ellipse_points.back(), ellipse_points.back()+cv::Point(0,-20), red, 5);
    } else if (board->horizontalTiltUnder) {
        cv::ellipse2Poly((board->leftPoints[0]+board->leftPoints[1])/2, cv::Size(100, 50), 0, -180, -10, 10, ellipse_points);
        cv::polylines(image, ellipse_points, false, red, 5);
        cv::line(image, ellipse_points.back(), ellipse_points.back()+cv::Point(-20,0), red, 5);
        cv::line(image, ellipse_points.back(), ellipse_points.back()+cv::Point(0,-20), red, 5);
    }

    if (board->verticalTiltOver) {
    } else if (board->verticalTiltUnder) {
    }

*/
    if (board->boardAreaOver) {
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[0]},
                   cv::Point2f(25, 25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[1]},
                   cv::Point2f(-25, 25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[2]},
                   cv::Point2f(-25, -25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[3]},
                   cv::Point2f(25, -25), red, 5);
    } else if (board->boardAreaUnder) {
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[0]},
                   cv::Point2f(-25, -25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[1]},
                   cv::Point2f(25, -25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[2]},
                   cv::Point2f(25, 25), red, 5);
      overlayArrow(image, std::vector<cv::Point2f>{board->vertices[3]},
                   cv::Point2f(-25, 25), red, 5);
    }

    if (board->leftOutOfBounds) {
      overlayArrow(image, board->leftPoints, cv::Point2f(-50, 0), red);
    }

    if (board->rightOutOfBounds) {
      overlayArrow(image, board->rightPoints, cv::Point2f(50, 0), red);
    }

    if (board->topOutOfBounds) {
      overlayArrow(image, board->topPoints, cv::Point2f(0, -50), red);
    }

    if (board->bottomOutOfBounds) {
      overlayArrow(image, board->bottomPoints, cv::Point2f(0, 50), red);
    }
  }
}

void StereoCalibrate::overlayArrow(cv::Mat& image,
                                    std::vector<cv::Point2f>& points,
                                    cv::Point2f offset, CvScalar colour,
                                    int thickness) {
  cv::Mat mean_;
  cv::reduce(points, mean_, 1, CV_REDUCE_AVG);
  cv::Point2f centrepoint(mean_.at<float>(0, 0), mean_.at<float>(0, 1));

  qDebug() << centrepoint.x << centrepoint.y;

  auto end = centrepoint + offset;
  auto start = centrepoint;

  cv::arrowedLine(image, start, end, colour, 3);
  cv::line(image, points.front(), points.back(), colour, thickness);
}

bool StereoCalibrate::imageValid() {
  std::vector<cv::Point2f> corners;

  int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
              cv::CALIB_CB_FAST_CHECK;
  bool valid = false;
  bool found_left = false;
  bool found_right = false;

  cv::cvtColor(stereoCamera->getLeftImage(), leftImageOverlay,
               cv::COLOR_GRAY2RGB);
  cv::cvtColor(stereoCamera->getRightImage(), rightImageOverlay,
               cv::COLOR_GRAY2RGB);

  if (calibratingLeft) {
    found_left = findCorners(stereoCamera->getLeftImage(), corners, flags);

    if (found_left) {
      valid = this->boardOrientations[currentPose]->check(corners);
      emit chessboardFound(this->boardOrientations[currentPose]);
    }

    overlayImage(leftImageOverlay, this->boardOrientations[currentPose],
                 found_left);

  } else {
    found_right = findCorners(stereoCamera->getRightImage(), corners, flags);

    if (found_right) {
      valid = this->boardOrientations[currentPose]->check(corners);
      emit chessboardFound(this->boardOrientations[currentPose]);
    }

    overlayImage(rightImageOverlay, this->boardOrientations[currentPose],
                 found_right);
  }

  updateViews();

  return valid;
}

void StereoCalibrate::loadBoardPoses(std::string fname) {
  std::ifstream posefile(fname, std::ifstream::in);

  if (posefile.good()) {
    std::string line;
    totalPoses = 0;

    while (std::getline(posefile, line)) {
      std::istringstream iss(line);
      Chessboard* board = new Chessboard(this, patternsize, imageSize);


      // Load the corners of the pattern pose
      double x, y;
      std::vector<cv::Point2f> vertices;

      for(int i=0; i < 4; i++){
        iss >> x;
        iss >> y;

        vertices.push_back(cv::Point2f(x, y));
      }

      // We need a closed contour
      vertices.push_back(vertices.at(0));

      board->setTemplate(vertices);

      /*if (!(iss >> board->minHt >> board->maxHt >> board->minVt >>
            board->maxVt >> board->minArea >> board->maxArea >>
            board->leftMargin >> board->topMargin >> board->rightMargin >>
            board->bottomMargin)) break;
      */

      boardOrientations.push_back(board);
      totalPoses++;
    }
    currentPose = 0;
  } else {
    abortCalibration();
  }
}

void StereoCalibrate::updateViews(void) {
  cv::Mat output_buffer;
  leftImageOverlay.copyTo(output_buffer);

  QImage im_left(output_buffer.data, stereoCamera->getWidth(),
                 stereoCamera->getHeight(), QImage::Format_RGB888);
  QPixmap pmap_left = QPixmap::fromImage(im_left);
  this->leftView->setPixmap(
      pmap_left.scaled(this->leftView->size(), Qt::KeepAspectRatio));

  rightImageOverlay.copyTo(output_buffer);

  QImage im_right(output_buffer.data, stereoCamera->getWidth(),
                  stereoCamera->getHeight(), QImage::Format_RGB888);
  QPixmap pmap_right = QPixmap::fromImage(im_right);
  this->rightView->setPixmap(
      pmap_right.scaled(this->rightView->size(), Qt::KeepAspectRatio));
}
