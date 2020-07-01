/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "disparityviewer.h"
#include "ui_disparityviewer.h"

DisparityViewer::DisparityViewer(QWidget *parent)
    : QWidget(parent), ui(new Ui::DisparityViewer) {
    ui->setupUi(this);

    connect(ui->colourmapComboBox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(setColourmap(int)));

    connect(ui->checkBoxSaveDisparity, SIGNAL(checked(bool)), this, SLOT(saveDisparityChanged(bool)));

    ui->colourmapComboBox->setCurrentIndex(colourmap);
}

void DisparityViewer::saveDisparityChanged(bool enable){
    emit disparitySaveCheckChanged(enable);
}

void DisparityViewer::assignThread(QThread *thread) {
    this->moveToThread(thread);
    connect(this, SIGNAL(finished()), thread, SLOT(quit()));
    connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

/*
void DisparityViewer::updateDisparityRange_spin(int range){
    ui->disparityRangeSpinbox->setValue(range);
    updatePixmapRange();
}

void DisparityViewer::updateDisparityRange_slide(int range){
    ui->disparityRangeSpinbox->setValue(range * 16);
    updatePixmapRange();
}

void DisparityViewer::updatePixmapRange(void) {
  min_disparity = ui->minDisparitySlider->value();
  disparity_range = ui->disparityRangeSlider->value() * 16;

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

}
*/

void DisparityViewer::setViewer(QLabel *viewer) { this->viewer = viewer; }

void DisparityViewer::setMatcher(AbstractStereoMatcher *matcher) {
    this->matcher = matcher;
    //updatePixmapRange();
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

    ui->colourbar->setPixmap(colourbar_pmap.scaled(400, 30));
}

void DisparityViewer::setCalibration(cv::Mat &Q, double baseline, double focal) {
    this->baseline = baseline;
    this->focal = focal;
    Q.copyTo(this->Q);
}

void DisparityViewer::updateDisparityAsync() {
    QtConcurrent::run(this, &DisparityViewer::updateDisparity);
}

float DisparityViewer::genZ(cv::Matx44d Q_, int x_index, int y_index, float d){
    cv::Vec4d homg_pt = Q_ * cv::Vec4d((double)x_index, (double)y_index, (double)d, 1.0);

    //float x = (float)homg_pt[0] / (float)homg_pt[3];
    //float y = (float)homg_pt[1] / (float)homg_pt[3];
    float z = (float)homg_pt[2] / (float)homg_pt[3];
    return z;
}

void DisparityViewer::updateDisparity() {
    cv::Mat disparity;
    matcher->getDisparity16(disparity);
    cv::Mat disparity_thresh;

    disparity.copyTo(disparity_thresh);

    double min_disp = 10000;
    double max_disp = 0;
    double min_depth = 10000;
    double max_depth = 0;

    cv::Matx44d _Q;
    Q.convertTo(_Q, CV_64F);

    for (int i = 0; i < disparity.rows; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            float d = disparity.at<float>(i, j);
            if (d < 10000 && d > 0){
                float d = disparity.at<float>(i, j);
                if (d < min_disp){
                    min_disp = d;
                    max_depth = (double)genZ(_Q,(double)j,(double)i,(double)d);
                }
                if (d > max_disp){
                    max_disp = d;
                    min_depth = (double)genZ(_Q,(double)j,(double)i,(double)d);
                }
            }
        }
    }

    for (int i = 0; i < disparity.rows; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            float d = disparity.at<float>(i, j);
            if (d > max_disp || d < min_disp){
                disparity_thresh.at<float>(i, j) = 0;
            }
        }
    }

    //use depth rather than disparity for more user readable results
    double range_disp = max_disp - min_disp;
    double range_depth = max_depth - min_depth;

    min_disp_ = min_disp;
    max_disp_ = max_disp;
    min_depth_ = min_depth;
    max_depth_ = max_depth;

    ui->minDisparityLabel->setText(
                QString("%1 px").arg(QString::number(min_disp,'g', 4)));
    ui->disparityRangeLabel->setText(
                QString("%1 px").arg(QString::number(range_disp,'g', 4)));
    ui->maxDisparityLabel->setText(
                QString("%1 px").arg(QString::number(max_disp,'g', 4)));

    ui->minDepthLabel->setText(
                QString("%1 m").arg(QString::number(min_depth,'g', 4)));
    ui->depthRangeLabel->setText(
                QString("%1 m").arg(QString::number(range_depth,'g', 4)));
    ui->maxDepthLabel->setText(
                QString("%1 m").arg(QString::number(max_depth,'g', 4)));

    cv::Mat temp;
    disparity_thresh.convertTo(temp, CV_32F);

    cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX);

    // Then shift to threshold minimimum displayed disparity
    //temp -= min_disparity;
    //cv::threshold(temp, temp, range_disp, 0, cv::THRESH_TRUNC);

    cv::Mat disparity_vis, disparity_scale;

    //double scale_factor = 255.0 / (double)range_disp;

    //temp *= scale_factor;
    //cv::threshold(temp, temp, 0, 0, cv::THRESH_TOZERO);

    temp.convertTo(disparity_scale, CV_8U);

    // cv::ximgproc::getDisparityVis(temp, disparity_scale, scale_factor);

    cv::applyColorMap(disparity_scale, disparity_vis, colourmap);

    //colour_disparity = disparity_vis;

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

    colour_disparity = disparity_vis;

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
