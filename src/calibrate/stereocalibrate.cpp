/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocalibrate.h"

StereoCalibrate::StereoCalibrate(QObject* parent,
                                 AbstractStereoCamera* stereoCamera)
    : QObject(parent) {
    this->stereo_camera = stereoCamera;
    cal_dialog = new CalibrateConfirmDialog();

    if (stereoCamera) {
        image_size = cv::Size(stereoCamera->getWidth(), stereoCamera->getHeight());
    }
}

void StereoCalibrate::setOutputPath(QString path){
    output_folder = QDir::cleanPath(path);
}

void StereoCalibrate::setPattern(cv::Size size, double squareSize) {
    pattern_size = size;

    pattern_points.clear();

    for (int j = 0; j < pattern_size.height; j++) {
        for (int i = 0; i < pattern_size.width; i++) {
            pattern_points.push_back(cv::Point3f(i * squareSize, j * squareSize, 0));
        }
    }
}

void StereoCalibrate::setImageSize(cv::Size size) { this->image_size = size; }

void StereoCalibrate::abortCalibration() {
    if (stereo_camera) {
        disconnect(stereo_camera, SIGNAL(acquired()), this, SLOT(checkImages()));
        disconnect(this, SIGNAL(requestImage()), stereo_camera, SLOT(singleShot()));
    }
    emit doneCalibration(CalibrationStatus::ABORTED);
}

void StereoCalibrate::finishedCalibration(CalibrationStatus success) {
    if (stereo_camera) {
        disconnect(stereo_camera, SIGNAL(acquired()), this, SLOT(checkImages()));
        disconnect(this, SIGNAL(requestImage()), stereo_camera, SLOT(singleShot()));
    }
    emit doneCalibration(success);
}

void StereoCalibrate::setDisplays(QLabel* left, QLabel* right) {
    this->left_view = left;
    this->right_view = right;
}

void StereoCalibrate::setBoardOrientations(
        std::vector<Chessboard*>& orientations) {
    this->board_orientations = orientations;
}

void StereoCalibrate::startCalibration(void) {
    success = false;
    imageProgress(0, total_poses);

    total_images = 0;

    left_images.clear();
    right_images.clear();

    connect(this->stereo_camera, SIGNAL(acquired()), this, SLOT(checkImages()));
    connect(this, SIGNAL(requestImage()), this->stereo_camera,
            SLOT(singleShot()));

    //this->stereo_camera->singleShot();
}

void StereoCalibrate::checkImages(void) {
    if (current_pose == total_poses) {
        if (calibrating_left) {
            calibrating_left = false;

            current_pose = 0;
            imageProgress(current_pose, total_poses);

            calibrating_right = true;
        } else if (calibrating_right) {
            calibrating_right = false;
        } else {
            jointCalibration();
        }
    } else {
        if (imageValid()) {
            current_pose++;

            imageProgress(current_pose, total_poses);
            cv::Mat image;

            std::string fname;

            // We need to save both left and right images for every view
            stereo_camera->getLeftImage(image);
            left_images.push_back(image);
            fname = QString("calibration_l_%1.png").arg(total_images).toStdString();
            cv::imwrite(fname, image);

            stereo_camera->getRightImage(image);
            right_images.push_back(image);
            fname = QString("calibration_r_%1.png").arg(total_images).toStdString();
            cv::imwrite(fname, image);

            total_images++;
        }
    }

    emit requestImage();
    return;
}

StereoCalibrate::CalibrationStatus StereoCalibrate::jointCalibration(void) {
    assert(left_images.size() != 0);
    assert(right_images.size() != 0);
    assert(left_image_points.size() == right_image_points.size());

    setImageSize(left_images.at(0).size());

    int cornerFlags = cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE;

    cal_dialog->setNumberImages(left_images.size());
    cal_dialog->show();
    qApp->processEvents();
    QMessageBox alert;
    qDebug() << "Running left calibration.";
    connect(this, SIGNAL(done_image(int)), cal_dialog, SLOT(updateLeftProgress(int)));

    /* Left Camera */
    left_valid.clear();
    left_rms_error = singleCameraCalibration(
                left_images, left_image_points, left_valid, left_camera_matrix,
                left_distortion, left_r_vecs, left_t_vecs, cornerFlags);

    if(left_rms_error > 0){
        cal_dialog->updateLeft(left_camera_matrix, left_distortion, left_rms_error);
        qDebug() << "Left RMS reprojection error: " << left_rms_error;
    }else{
        alert.setText("Left camera calibration failed.");
        alert.exec();
        qDebug() << "Left camera calibration failed.";
        cal_dialog->close();
        finishedCalibration(CalibrationStatus::FAILED);
        return CalibrationStatus::FAILED;
    }

    if (left_valid.size() >= 5){

        qDebug() << "Running right calibration.";

        disconnect(this, SIGNAL(done_image(int)),  cal_dialog, SLOT(updateLeftProgress(int)));
        connect(this, SIGNAL(done_image(int)), cal_dialog, SLOT(updateRightProgress(int)));

        /* Right Camera */
        right_valid.clear();
        right_rms_error = singleCameraCalibration(
                    right_images, right_image_points, right_valid, right_camera_matrix,
                    right_distortion, right_r_vecs, right_t_vecs, cornerFlags);

        if(right_rms_error > 0){
            cal_dialog->updateRight(right_camera_matrix, right_distortion, right_rms_error);
            qDebug() << "Right RMS reprojection error: " << left_rms_error;
        }else{
            alert.setText("Right camera calibration failed.");
            alert.exec();
            qDebug() << "Right camera calibration failed.";
            cal_dialog->close();
            finishedCalibration(CalibrationStatus::FAILED);
            return CalibrationStatus::FAILED;
        }

        disconnect(this, SIGNAL(done_image(int)), cal_dialog, SLOT(updateRightProgress(int)));
        qApp->processEvents(); // Otherwise we seem to go too fast for the GUI to keep up
    } else {
        qDebug() << "Need at least 5 valid left image for calibration. Found: " << left_valid.size();
        cal_dialog->close();
        finishedCalibration(CalibrationStatus::INVALID_NUM_VALID_IMAGES);
        return CalibrationStatus::INVALID_NUM_VALID_IMAGES;
    }

    if (left_valid.size() >= 5 && left_valid.size() >= 5){
        // check at least 5 images pairs were found valid
        int valid_count = 0;
        auto iter_r_valid = right_valid.begin();
        auto iter_l_valid = left_valid.begin();
        while(iter_r_valid != right_valid.end() || iter_l_valid != left_valid.end())
        {
            if (*iter_r_valid && *iter_l_valid){
                valid_count++;
            }
            if(iter_r_valid != right_valid.end())
            {
                ++iter_r_valid;
            }
            if(iter_l_valid != left_valid.end())
            {
                ++iter_l_valid;
            }
        }

        if (valid_count >= 5){
            try
            {
                stereo_rms_error = stereoCameraCalibration();
            }
            catch( cv::Exception& e )
            {
                const char* err_msg = e.what();
                std::cout << "exception caught: " << err_msg << std::endl;
                stereo_rms_error = 0;
                cal_dialog->close();
                finishedCalibration(CalibrationStatus::FAILED);
                return CalibrationStatus::FAILED;
            }
        } else {
            stereo_rms_error = 0;
            qDebug() << "Need at least 5 valid image pairs for calibration. Found: " << valid_count;
            cal_dialog->close();
            finishedCalibration(CalibrationStatus::INVALID_NUM_VALID_IMAGES);
            return CalibrationStatus::INVALID_NUM_VALID_IMAGES;
        }
    } else {
        stereo_rms_error = 0;
        qDebug() << "Need at least 5 valid images for calibration. Found: l - " << left_valid.size() << " r - " << right_valid.size();
        cal_dialog->close();
        finishedCalibration(CalibrationStatus::INVALID_NUM_VALID_IMAGES);
        return CalibrationStatus::INVALID_NUM_VALID_IMAGES;
    }

    if(stereo_rms_error > 0){
        qDebug() << "Stereo RMS reprojection error: " << stereo_rms_error;

        cal_dialog->updateStereo(stereo_q, stereo_rms_error);

        qDebug() << "Saving calibration files... ";

        qDebug() << "Saving camera calibration files... ";
        cv::FileStorage leftIntrinsicFS(output_folder.absoluteFilePath("left_calibration.xml").toStdString(),
                                        cv::FileStorage::WRITE);
        leftIntrinsicFS << "cameraMatrix" << left_camera_matrix;
        leftIntrinsicFS << "distCoeffs" << left_distortion;
        leftIntrinsicFS << "rms_error" << left_rms_error;
        leftIntrinsicFS.release();

        cv::FileStorage rightIntrinsicFS(output_folder.absoluteFilePath("right_calibration.xml").toStdString(),
                                         cv::FileStorage::WRITE);
        rightIntrinsicFS << "cameraMatrix" << right_camera_matrix;
        rightIntrinsicFS << "distCoeffs" << right_distortion;
        rightIntrinsicFS << "rms_error" << right_rms_error;
        rightIntrinsicFS.release();

        qDebug() << "Saving stereo calibration files... ";
        cv::FileStorage stereoFS(output_folder.absoluteFilePath("stereo_calibration.xml").toStdString(), cv::FileStorage::WRITE);
        stereoFS << "R" << stereo_r;
        stereoFS << "T" << stereo_t;
        stereoFS << "Q" << stereo_q;
        stereoFS << "E" << stereo_e;
        stereoFS << "F" << stereo_f;
        stereoFS << "rms_error" << stereo_rms_error;
        stereoFS.release();

        qDebug() << "Saving rectification files... ";
        cv::FileStorage leftRectFs(output_folder.absoluteFilePath("left_rectification.xml").toStdString(), cv::FileStorage::WRITE);
        leftRectFs << "x" << left_rectification_x;
        leftRectFs << "y" << left_rectification_y;
        leftRectFs.release();

        cv::FileStorage rightRectFS(output_folder.absoluteFilePath("right_rectification.xml").toStdString(),
                                    cv::FileStorage::WRITE);
        rightRectFS << "x" << right_rectification_x;
        rightRectFS << "y" << right_rectification_y;
        rightRectFS.release();

        if(save_ros){
            qDebug() << "Saving yaml files... ";
            outputYaml(YamlFileFormat::ROS_PERCEPTION_YAML,output_folder.absoluteFilePath("left.yaml"), "leftCamera", left_images.front().size(), left_camera_matrix, left_distortion, P1, R1);
            outputYaml(YamlFileFormat::ROS_PERCEPTION_YAML,output_folder.absoluteFilePath("right.yaml"), "rightCamera", right_images.front().size(), right_camera_matrix, right_distortion, P2, R2);
        }

        //alert.setText(QString("Written calibration files to: %1").arg(output_folder.absolutePath()));
        //alert.exec();
        finishedCalibration(CalibrationStatus::SUCCESS);

        return CalibrationStatus::SUCCESS;
    }else{
        //alert.setText("Stereo camera calibration failed.");
        //alert.exec();
        qDebug() << "Stereo camera calibration failed.";
        finishedCalibration(CalibrationStatus::FAILED);
        cal_dialog->close();

        return CalibrationStatus::FAILED;
    }
}

double StereoCalibrate::stereoCameraCalibration(int stereoFlags) {
    /* Check valid images */
    std::vector<std::vector<cv::Point2f> > leftImagePointsMask;
    std::vector<std::vector<cv::Point2f> > rightImagePointsMask;
    std::vector<std::vector<cv::Point3f> > objectPointsMask;

    /* Mask image arrays */
    for (int i = 0; i < left_valid.size(); i++) {
        if (left_valid[i] && right_valid[i]) {
            leftImagePointsMask.push_back(left_image_points[i]);
            rightImagePointsMask.push_back(right_image_points[i]);
            objectPointsMask.push_back(pattern_points);
        }
    }

    assert(leftImagePointsMask.size() >= 5);
    assert(rightImagePointsMask.size() >= 5);

    double stereoRes = cv::stereoCalibrate(
                objectPointsMask, leftImagePointsMask, rightImagePointsMask,
                left_camera_matrix, left_distortion, right_camera_matrix,
                right_distortion, image_size, stereo_r, stereo_t, stereo_e, stereo_f,
                stereoFlags);

    cv::stereoRectify(left_camera_matrix, left_distortion, right_camera_matrix,
                      right_distortion, image_size, stereo_r, stereo_t, R1, R2,
                      P1, P2, stereo_q, 0);

    cv::initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1,
                                image_size, CV_32FC1, left_rectification_x,
                                left_rectification_y);
    cv::initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2,
                                image_size, CV_32FC1, right_rectification_x,
                                right_rectification_y);

    return stereoRes;
}

void StereoCalibrate::setImages(QList<cv::Mat> left, QList<cv::Mat> right) {
    left_images.clear();
    right_images.clear();

    for (auto& l : left) {
        cv::Mat im;
        l.copyTo(im);
        if (im.total() > 0) left_images.push_back(im);
    }

    for (auto& r : right) {
        cv::Mat im;
        r.copyTo(im);
        if (im.total() > 0) right_images.push_back(im);
    }

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
        //QFuture<bool> findCorners_t = QtConcurrent::run(this, &StereoCalibrate::findCorners, image, corners, cornerFlags);
        //findCorners_t.waitForFinished();
        //bool success = findCorners_t.result();
        //TODO run this in a thread to stop locking out the GUI
        bool success = findCorners(image,corners,cornerFlags);
        if (success) {
            imagePoints.push_back(corners);
            validImagePoints.push_back(corners);
            objectPoints.push_back(pattern_points);
            valid.push_back(true);
            qDebug() << "Image " << i << " valid.";
        } else {
            valid.push_back(false);
            corners.clear();
            imagePoints.push_back(corners);
            qDebug() << "Image " << i << " invalid.";
            //TODO if x percentage of the images are invalid then cancel the process
        }

        i++;

        emit done_image(i);

        // Allow the GUI to repaint between images
        qApp->processEvents();
    }

    assert(objectPoints.size() == validImagePoints.size());

    try{
        res = cv::calibrateCamera(objectPoints, validImagePoints, image_size,
                                  cameraMatrix, distCoeffs, rvecs, tvecs, stereoFlags);
    }catch(...){
        qDebug() << "Calibration failed";
        res = -1;
    }

    return res;
}

bool StereoCalibrate::findCorners(cv::Mat image,
                                  std::vector<cv::Point2f>& corners,
                                  int flags) {

    //if image is high resolution, find checkerboard on downscaled image
    bool found = false;
    int max_size = 1024;

    if (image.size().width > max_size || image.size().height > max_size){
        qDebug() << "Finding chessboard corners in downscaled image...";
        float scale_amount = (float)max_size/(float)image.size().width;
        qDebug() << "Downscaling image by: " << scale_amount;
        int width = (int)(image.size().width * scale_amount);
        int height = (int)(image.size().height * scale_amount);
        cv::Size dim(width, height);
        qDebug() << "New image size: " << width << "," << height;
        cv::Mat image_downscaled;
        cv::resize(image, image_downscaled, dim, 0, 0, cv::INTER_AREA);
        found = cv::findChessboardCorners(image_downscaled, pattern_size, corners, flags);
        if (found){
            qDebug() << "Found corners in downscaled image.";
            // upscale corners back to correct size
//            for (std::vector<cv::Point2f>::iterator it = corners.begin() ; it != corners.end(); ++it){
//                *it = *it * (1/scale_amount);
//            }
            // Re-run corner detection on larger image
            found = cv::findChessboardCorners(image, pattern_size, corners, flags);
            if (found){
                qDebug() << "Found corners in image.";
            }
        }
    } else {
        qDebug() << "Finding chessboard corners...";
        found = cv::findChessboardCorners(image, pattern_size, corners, flags);
        if (found){
            qDebug() << "Found corners in image.";
        }
    }

    //subpixel refinement
    if (found){
        cv::cornerSubPix(image, corners, pattern_size, cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
    }

    return found;
}

void StereoCalibrate::overlayImage(cv::Mat& image, Chessboard* board,
                                   bool found, bool valid) {
    cv::Scalar red(255, 0, 0, 255);
    cv::Scalar green(0, 255, 0, 255);
    cv::Scalar blue(0, 0, 255, 255);
    cv::Scalar overlayColour = red;

    // cv::drawChessboardCorners(image, pattern_size, board->board_points, found);

    cv::polylines(image, board->template_contour, true, blue, 3);

    if (found) {
        // Draw board outline
        if (valid) {
            cv::line(image, board->vertices[0], board->vertices[1], green, 3);
            cv::line(image, board->vertices[1], board->vertices[2], green, 3);
            cv::line(image, board->vertices[2], board->vertices[3], green, 3);
            cv::line(image, board->vertices[3], board->vertices[0], green, 3);
        } else {
            cv::line(image, board->vertices[0], board->vertices[1], red, 3);
            cv::line(image, board->vertices[1], board->vertices[2], red, 3);
            cv::line(image, board->vertices[2], board->vertices[3], red, 3);
            cv::line(image, board->vertices[3], board->vertices[0], red, 3);
        }
    }
    /*
      std::vector<cv::Point> ellipse_points;

      if (board->horizontalTiltOver) {

          auto left_margin_middle =
     (board->leftPoints.front()+board->leftPoints.back())/2;

          cv::ellipse2Poly(left_margin_middle + cv::Point2f(50,0) ,
     cv::Size(100, 50), 0, 180, 10, 10, ellipse_points); cv::polylines(image,
     ellipse_points, false, red, 5); cv::line(image, ellipse_points.back(),
     ellipse_points.back()+cv::Point(20,0), red, 5); cv::line(image,
     ellipse_points.back(), ellipse_points.back()+cv::Point(0,-20), red, 5); }
     else if (board->horizontalTiltUnder) {
          cv::ellipse2Poly((board->leftPoints[0]+board->leftPoints[1])/2,
     cv::Size(100, 50), 0, -180, -10, 10, ellipse_points); cv::polylines(image,
     ellipse_points, false, red, 5); cv::line(image, ellipse_points.back(),
     ellipse_points.back()+cv::Point(-20,0), red, 5); cv::line(image,
     ellipse_points.back(), ellipse_points.back()+cv::Point(0,-20), red, 5);
      }

      if (board->verticalTiltOver) {
      } else if (board->verticalTiltUnder) {
      }

  */
}

void StereoCalibrate::overlayArrow(cv::Mat& image,
                                   std::vector<cv::Point2f>& points,
                                   cv::Point2f offset, cv::Scalar colour,
                                   int thickness) {
    cv::Mat mean_;
    cv::reduce(points, mean_, 1, cv::REDUCE_AVG);
    cv::Point2f centrepoint(mean_.at<float>(0, 0), mean_.at<float>(0, 1));

    auto end = centrepoint + offset;
    auto start = centrepoint;

    cv::arrowedLine(image, start, end, colour, 3);
    cv::line(image, points.front(), points.back(), colour, thickness);
}

bool StereoCalibrate::imageValid() {
    std::vector<cv::Point2f> corners;

    // int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
    //            cv::CALIB_CB_FAST_CHECK;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE;
    bool valid = false;
    bool found_left = false;
    bool found_right = false;

    cv::cvtColor(stereo_camera->getLeftImage(), left_image_overlay,
                 cv::COLOR_GRAY2RGB);
    cv::cvtColor(stereo_camera->getRightImage(), right_image_overlay,
                 cv::COLOR_GRAY2RGB);

    if (calibrating_left) {
        found_left = findCorners(stereo_camera->getLeftImage(), corners, flags);

        if (found_left) {
            valid = this->board_orientations[current_pose]->check(corners);
            emit chessboardFound(this->board_orientations[current_pose]);
        }

        overlayImage(left_image_overlay, this->board_orientations[current_pose],
                     found_left, valid);

    } else {
        found_right = findCorners(stereo_camera->getRightImage(), corners, flags);

        if (found_right) {
            valid = this->board_orientations[current_pose]->check(corners);
            emit chessboardFound(this->board_orientations[current_pose]);
        }

        overlayImage(right_image_overlay, this->board_orientations[current_pose],
                     found_right, valid);
    }

    updateViews();

    return valid;
}

void StereoCalibrate::loadBoardPoses(std::string fname) {
    std::ifstream posefile(fname, std::ifstream::in);

    if (!posefile.is_open()) {
        qDebug() << "Failed to open pose file" << fname.c_str();
    } else if (posefile.good()) {
        std::string line;
        total_poses = 0;

        while (std::getline(posefile, line)) {
            std::istringstream iss(line);
            Chessboard* board = new Chessboard(this, pattern_size, image_size);

            // Load the corners of the pattern pose
            double x, y;
            std::vector<cv::Point2i> vertices;

            for (int i = 0; i < 4; i++) {
                iss >> x;
                iss >> y;

                vertices.push_back(cv::Point2i(x, y));
            }

            // We need a closed contour
            vertices.push_back(vertices.at(0));

            // Double check the area isn't zero
            assert(cv::contourArea(vertices) > 0);

            board->setTemplate(vertices);

            /*if (!(iss >> board->minHt >> board->maxHt >> board->minVt >>
            board->maxVt >> board->minArea >> board->maxArea >>
            board->leftMargin >> board->topMargin >> board->rightMargin >>
            board->bottomMargin)) break;
      */

            board_orientations.push_back(board);
            total_poses++;
        }
        current_pose = 0;
    } else {
        abortCalibration();
    }
}

void StereoCalibrate::updateViews(void) {
    cv::Mat output_buffer;
    left_image_overlay.copyTo(output_buffer);

    QImage im_left(output_buffer.data, stereo_camera->getWidth(),
                   stereo_camera->getHeight(), QImage::Format_RGB888);
    QPixmap pmap_left = QPixmap::fromImage(im_left);
    this->left_view->setPixmap(
                pmap_left.scaled(this->left_view->size(), Qt::KeepAspectRatio));

    right_image_overlay.copyTo(output_buffer);

    QImage im_right(output_buffer.data, stereo_camera->getWidth(),
                    stereo_camera->getHeight(), QImage::Format_RGB888);
    QPixmap pmap_right = QPixmap::fromImage(im_right);
    this->right_view->setPixmap(
                pmap_right.scaled(this->right_view->size(), Qt::KeepAspectRatio));
}

bool StereoCalibrate::outputYaml(YamlFileFormat yaml_file_format, QString filename, QString camera_name, cv::Size image_size, cv::Mat camera_matrix, cv::Mat dist_coeffs, cv::Mat P, cv::Mat R){

    if (yaml_file_format == YamlFileFormat::CV_FILESTORAGE_YAML){
        cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);

        fs << "image_width" << image_size.width;
        fs << "image_height" << image_size.height;
        fs << "camera_name" << camera_name.toStdString();

        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_model" << "plumb_bob";

        fs << "distortion_coefficients" << dist_coeffs;
        fs << "rectification_matrix" << R;
        fs << "projection_matrix" << P;

        fs.release();
        return true;
    } else if (yaml_file_format == YamlFileFormat::ROS_PERCEPTION_YAML){
        QFile file(filename);
        if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
            QTextStream stream(&file);

            stream << "image_width: " << image_size.width << "\n";
            stream << "image_height: " << image_size.height << "\n";
            stream << "camera_name: " << camera_name << "\n";
            stream << "camera_matrix:\n";
            stream << "  rows: 3\n";
            stream << "  cols: 3\n";
            stream << "  data: [" << camera_matrix.at<double>(0,0) << ", " << camera_matrix.at<double>(0,1) << ", " << camera_matrix.at<double>(0,2) << ", ";
            stream << camera_matrix.at<double>(1,0) << ", " << camera_matrix.at<double>(1,1) << ", " << camera_matrix.at<double>(1,2) << ", ";
            stream << camera_matrix.at<double>(2,0) << ", " << camera_matrix.at<double>(2,1) << ", " << camera_matrix.at<double>(2,2) << "]\n";
            stream << "distortion_model: plumb_bob\n";
            stream << "distortion_coefficients:\n";
            stream << "  rows: 1\n";
            stream << "  cols: 5\n";
            stream << "  data: [" << dist_coeffs.at<double>(0,0) << ", " << dist_coeffs.at<double>(0,1) << ", " << dist_coeffs.at<double>(0,2) << ", ";
            stream << dist_coeffs.at<double>(0,3) << ", " << dist_coeffs.at<double>(0,4) << "]\n";
            stream << "rectification_matrix:\n";
            stream << "  rows: 3\n";
            stream << "  cols: 3\n";
            stream << "  data: [" << R.at<double>(0,0) << ", " << R.at<double>(0,1) << ", " << R.at<double>(0,2) << ", ";
            stream << R.at<double>(1,0) << ", " << R.at<double>(1,1) << ", " << R.at<double>(1,2) << ", ";
            stream << R.at<double>(2,0) << ", " << R.at<double>(2,1) << ", " << R.at<double>(2,2) << "]\n";
            stream << "projection_matrix:\n";
            stream << "  rows: 3\n";
            stream << "  cols: 4\n";
            stream << "  data: [" << P.at<double>(0,0) << ", " << P.at<double>(0,1) << ", " << P.at<double>(0,2) << ", " << P.at<double>(0,3) << ", ";
            stream << P.at<double>(1,0) << ", " << P.at<double>(1,1) << ", " << P.at<double>(1,2) << ", " << P.at<double>(1,3) << ", ";
            stream << P.at<double>(2,0) << ", " << P.at<double>(2,1) << ", " << P.at<double>(2,2) << ", " << P.at<double>(2,3) << "]\n";

            file.close();
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}
