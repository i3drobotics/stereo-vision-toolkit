/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "disparityviewer.h"
#include "ui_disparityviewer.h"

DisparityViewer::DisparityViewer(QWidget *parent)
    : QWidget(parent), ui(new Ui::DisparityViewer) {
  ui->setupUi(this);

  connect(ui->disparityRangeSlider, SIGNAL(sliderMoved(int)),
          ui->disparityRangeSpinbox, SLOT(setValue(int)));
  connect(ui->minDisparitySlider, SIGNAL(sliderMoved(int)),
          ui->minDisparitySpinbox, SLOT(setValue(int)));

  connect(ui->disparityRangeSpinbox, SIGNAL(valueChanged(int)),
          ui->disparityRangeSlider, SLOT(setValue(int)));
  connect(ui->minDisparitySpinbox, SIGNAL(valueChanged(int)),
          ui->minDisparitySlider, SLOT(setValue(int)));

  connect(ui->minDisparitySlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updatePixmapRange()));
  connect(ui->disparityRangeSlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updatePixmapRange()));
  connect(ui->disparityRangeSpinbox, SIGNAL(valueChanged(int)), this,
          SLOT(updatePixmapRange()));
  connect(ui->minDisparitySpinbox, SIGNAL(valueChanged(int)), this,
          SLOT(updatePixmapRange()));

  connect(ui->colourmapComboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(setColourmap(int)));

  ui->colourmapComboBox->setCurrentIndex(colourmap);

  ui->disparityRangeSpinbox->setValue(disparity_range);
  ui->disparityRangeSlider->setValue(disparity_range);
  ui->minDisparitySlider->setValue(min_disparity);
  ui->minDisparitySpinbox->setValue(min_disparity);
}

void DisparityViewer::assignThread(QThread *thread) {
  this->moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

void DisparityViewer::updatePixmapRange(void) {
  min_disparity = ui->minDisparitySlider->value();
  disparity_range = ui->disparityRangeSlider->value();

  ui->minDisparityLabel->setText(
      QString("%1 px").arg(QString::number(min_disparity)));
  ui->maxDisparityLabel->setText(
      QString("%1 px").arg(QString::number(min_disparity+disparity_range)));

  setColourmap(colourmap);

  return;

  if(Q.empty()){
      ui->minDisparityLabel->setText(
          QString("%1 px").arg(QString::number(min_disparity)));
      ui->maxDisparityLabel->setText(
          QString("%1 px").arg(QString::number(min_disparity+disparity_range)));

      return;
  }

  cv::Point3f p_min;
  p_min.x = 0;
  p_min.y = 0;
  p_min.z = min_disparity;

  cv::Point3f p_max;
  p_min.x = 0;
  p_min.y = 0;
  p_min.z = min_disparity+disparity_range;

  std::vector<cv::Point3f> points = {p_min, p_max};
  std::vector<cv::Point3f> output_points = {p_min, p_max};

  cv::perspectiveTransform(points, output_points, Q);

  qDebug() << output_points[0].z << output_points[1].z;
/*
  ui->minDisparityLabel->setText(
      QString("%1 m").arg(QString::number(min_distance, 'g', 2)));
  ui->maxDisparityLabel->setText(
      QString("%1 m").arg(QString::number(max_distance, 'g', 2)));
*/

}

void DisparityViewer::setViewer(QLabel *viewer) { this->viewer = viewer; }

void DisparityViewer::setMatcher(AbstractStereoMatcher *matcher) {
  this->matcher = matcher;
  updatePixmapRange();
  setColourmap(2);
}

void DisparityViewer::setColourmap(int idx) {
  colourmap = idx;

  cv::Mat colourbar(1, 256, CV_8UC1);
  cv::Mat colourbar_vis;

  for (uint8_t i = 0; i < 255; i++) {
    colourbar.at<uint8_t>(i) = (uint8_t)(255 - i);
  }

  cv::applyColorMap(colourbar, colourbar_vis, colourmap);
  cv::cvtColor(colourbar_vis, colourbar_vis, CV_BGR2RGB);
  QImage colourbar_image(colourbar_vis.data, colourbar_vis.cols,
                         colourbar_vis.rows, QImage::Format_RGB888);
  QPixmap colourbar_pmap = QPixmap::fromImage(colourbar_image);

  ui->colourbar->setPixmap(colourbar_pmap.scaled(300, 30));
}

void DisparityViewer::setCalibration(cv::Mat &Q, double baseline, double focal) {
  this->baseline = baseline;
  this->focal = focal;
  Q.copyTo(this->Q);
}

void DisparityViewer::updateDisparityAsync() {
  QtConcurrent::run(this, &DisparityViewer::updateDisparity);
}

void DisparityViewer::updateDisparity() {
  matcher->getDisparity(disparity);

  cv::Mat temp;
  disparity.convertTo(temp, CV_32F);
  temp /= 16.0;

  // Then shift to threshold minimimum displayed disparity
  temp -= min_disparity;
  cv::threshold(temp, temp, disparity_range, 0, cv::THRESH_TRUNC);

  cv::Mat disparity_scale, disparity_vis;

    double scale_factor = 255.0 / (double)disparity_range;

    temp *= scale_factor;
    cv::threshold(temp, temp, 0, 0, cv::THRESH_TOZERO);

    temp.convertTo(disparity_scale, CV_8U);

    // cv::ximgproc::getDisparityVis(temp, disparity_scale, scale_factor);

    cv::applyColorMap(disparity_scale, disparity_vis, colourmap);

    cv::cvtColor(disparity_vis, disparity_vis, CV_BGR2RGB);


    auto left_ptr = matcher->getLeftImage();

    for (int x = 0; x < disparity_scale.cols; x++) {
      for (int y = 0; y < disparity_scale.rows; y++) {
        if (disparity_scale.at<uchar>(y, x) == 255 ||
            disparity_scale.at<uchar>(y, x) == 0 ||
            disparity.at<float>(y, x) == 16 * matcher->getErrorDisparity()) {
          disparity_vis.at<cv::Vec3b>(y, x)[0] = 0;
          disparity_vis.at<cv::Vec3b>(y, x)[1] = 0;
          disparity_vis.at<cv::Vec3b>(y, x)[2] = 0;
        }

        if(left_ptr->empty()) continue;

        uchar image_val = (uchar) left_ptr->at<unsigned char>(y, x);

        if (image_val == 0 ||
            image_val > ui->intensityThresholdSlider->value()) {
          disparity_vis.at<cv::Vec3b>(y, x)[0] = 0;
          disparity_vis.at<cv::Vec3b>(y, x)[1] = 0;
          disparity_vis.at<cv::Vec3b>(y, x)[2] = 0;
        }

      }
    }


    QImage dmap(disparity_vis.data, disparity_vis.cols, disparity_vis.rows,
                QImage::Format_RGB888);

    QPixmap pmap_disparity = QPixmap::fromImage(dmap);

    if(!pmap_disparity.isNull()){
        emit newDisparity(
            pmap_disparity.scaled(viewer->size(), Qt::KeepAspectRatio));
    }

  processing_disparity = false;
}

DisparityViewer::~DisparityViewer() {
  delete ui;
  emit finished();
}
