/*
 * Copyright I3D Robotics Ltd, 2017
 * Author: Josh Veitch-Michaelis
 */

#include "calibrateconfirmdialog.h"
#include "ui_calibrateconfirmdialog.h"

calibrateconfirmdialog::calibrateconfirmdialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::calibrateconfirmdialog) {
  ui->setupUi(this);

}

calibrateconfirmdialog::~calibrateconfirmdialog() { delete ui; }

void calibrateconfirmdialog::updateLeft(const cv::Mat &camera_matrix, const cv::Mat &distortion,
                                   const double &rms) {

   double focal_x = camera_matrix.at<double>(0, 0);
   double focal_y = camera_matrix.at<double>(1, 1);
   double cx = camera_matrix.at<double>(0, 2);
   double cy = camera_matrix.at<double>(1, 2);

   ui->left_focal->setText(QString("%1 %2 px").arg(QString::number(focal_x, 'f', 2)).arg(QString::number(focal_y, 'f', 2)));
   ui->left_principle->setText(QString("%1 %2 px").arg(QString::number(cx, 'f', 2)).arg(QString::number(cy, 'f', 2)));

   QString distortion_label;

   for(int i=0; i < distortion.total(); i++){
       distortion_label += QString("%1 ").arg(QString::number(distortion.at<double>(i), 'f', 2));
   }
   ui->left_distortion->setText(distortion_label);

   ui->left_rms->setText(QString("%1 px").arg(QString::number(rms, 'f', 3)));

   ui->left_intrinsic->setEnabled(true);
}

void calibrateconfirmdialog::updateRight(const cv::Mat &camera_matrix, const cv::Mat &distortion,
                                    const double &rms) {

    double focal_x = camera_matrix.at<double>(0, 0);
    double focal_y = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

   ui->right_focal->setText(QString("%1 %2 px").arg(QString::number(focal_x, 'f', 2)).arg(QString::number(focal_y, 'f', 2)));
   ui->right_principle->setText(QString("%1 %2 px").arg(QString::number(cx, 'f', 2)).arg(QString::number(cy, 'f', 2)));

   QString distortion_label;

   for(int i=0; i < distortion.total(); i++){
       distortion_label += QString("%1 ").arg(QString::number(distortion.at<double>(i), 'f', 2));
   }

   ui->right_distortion->setText(distortion_label);

   ui->right_rms->setText(QString("%1 px").arg(QString::number(rms, 'f', 3)));

   ui->right_intrinsic->setEnabled(true);
}

void calibrateconfirmdialog::updateStereo(const cv::Mat Q, const double &rms) {

   auto cx = QString::number(-Q.at<double>(0,3), 'f', 2);
   auto cy = QString::number(-Q.at<double>(1,3), 'f', 2);
   double baseline = -1.0/-Q.at<double>(3,2);

   ui->stereo_focal->setText(QString::number(Q.at<double>(2,3), 'f', 2));
   ui->stereo_principle->setText(QString("%1 %2 px").arg(cx).arg(cy));
   ui->stereo_baseline->setText(QString::number(1000*baseline, 'f', 2));
   ui->stereo_rms->setText(QString("%1 px").arg(QString::number(rms, 'f', 3)));

   ui->stereo_parameters->setEnabled(true);
}

void calibrateconfirmdialog::setNumberImages(int number_images){
    ui->left_progress->setMaximum(number_images);
    ui->right_progress->setMaximum(number_images);
}


void calibrateconfirmdialog::updateLeftProgress(int number_done) {
    ui->left_progress->setValue(number_done);
    update();
}

void calibrateconfirmdialog::updateRightProgress(int number_done) {
    ui->right_progress->setValue(number_done);
    update();
}

