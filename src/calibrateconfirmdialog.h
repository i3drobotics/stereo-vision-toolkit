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

class calibrateconfirmdialog : public QDialog {
  Q_OBJECT

 public:
  explicit calibrateconfirmdialog(QWidget *parent = 0);
  ~calibrateconfirmdialog();

  void updateLeft(const cv::Mat &camera_matrix, const cv::Mat &distortion,
             const double &rms);
  void updateRight(const cv::Mat &camera_matrix, const cv::Mat &distortion,
                   const double &rms);
  void updateStereo(const cv::Mat Q, const double &rms);

  void setNumberImages(int number_images);
public slots:
  void updateLeftProgress(int number_done);
  void updateRightProgress(int number_done);

private:
  Ui::calibrateconfirmdialog *ui;
};

#endif  // CALIBRATECONFIRMDIALOG_H
