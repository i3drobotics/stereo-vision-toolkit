/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <math.h>
#include <QDialog>
#include <QFileDialog>
#include <QSettings>
#include "stereocalibrate.h"
#include "abstractstereocamera.h"
#include "calibrateconfirmdialog.h"
#include "chessboard.h"

//!  Stereo calibration dialog
/*!
  Dialog for performing stereo calibration
*/

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrationDialog(AbstractStereoCamera* stereo_cam, QWidget *parent = 0);
    int getPatternCols();
    int getPatternRows();
    double getSquareSizeMm();
    bool getSaveROS();
    QList<cv::Mat> getLeftImages(){return left_image_list;}
    QList<cv::Mat> getRightImages(){return right_image_list;}
    QString getOutputPath(void){return output_path;}
    ~CalibrationDialog();

private:
    Ui::CalibrationDialog *ui;
    AbstractStereoCamera *stereo_cam;
    bool running = false;

    QString output_path = "";
    QList<cv::Mat> left_image_list;
    QList<cv::Mat> right_image_list;
    QSettings *settings;

    int p_min_change;
    double skew_min_change;
    int size_min_change;

    int image_width;
    int image_height;

    int x_lowest = -1;
    int x_highest = -1;
    int y_lowest = -1;
    int y_highest = -1;
    double min_y_skew = -1;
    double max_y_skew = -1;
    double min_x_skew = -1;
    double max_x_skew = -1;
    double min_size = -1;
    double max_size = -1;

    double m_scale = 1;

    bool checkImages(cv::Mat left, cv::Mat right);
    void addImages(cv::Mat left, cv::Mat right);
    bool findCorners(cv::Mat image,std::vector<cv::Point2f> &corners);
    cv::RotatedRect getBoardRect(std::vector<cv::Point2f> corners);
    std::vector<cv::Point2f> getBoardOutsideCorners(std::vector<cv::Point2f> corners);
    int getCornerIndex(int row, int col);
    bool checkXYDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect);
    bool checkSizeDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect);
    bool checkSkewDiff(std::vector<cv::Point2f> left_outer_corners, std::vector<cv::Point2f> right_outer_corners); // uses realitive side length for skew measure
    cv::Mat downsampleImage(cv::Mat image);
    void updateProgress();

public slots:
    void calibrate();
    void toggleRun();
    void processImages();
    void selectOutputPath();
    void updateOutputPath(void);

signals:
    void startCalibration();
    void stopCalibration();
};

#endif // CALIBRATIONDIALOG_H
