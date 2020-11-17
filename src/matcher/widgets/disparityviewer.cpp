/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
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

void DisparityViewer::setViewer(QLabel *viewer) { this->viewer = viewer; }

void DisparityViewer::setMatcher(AbstractStereoMatcher *matcher) {
    this->matcher = matcher;
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
    cv::cvtColor(colourbar_vis, colourbar_vis, cv::COLOR_BGR2RGB);
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

void DisparityViewer::updateDisparity() {
    cv::Mat disparity, disparity_thresh;
    matcher->getDisparity16(disparity);
    disparity.copyTo(disparity_thresh);
    //double d_error = 16 * matcher->getErrorDisparity();

    /*

    double min_disp = 10000;
    double max_disp = 0;
    int min_i = 0;
    int max_i = 0;
    int min_j = 0;
    int max_j = 0;

    double min_depth = 10000;
    double max_depth = 0;

    double d_error = 16 * matcher->getErrorDisparity();

    cv::Matx44d _Q;
    Q.convertTo(_Q, CV_64F);

    for (int i = 0; i < disparity.rows; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            float d = disparity.at<float>(i, j);
            if (d < 10000 && d != d_error){
                float d = disparity.at<float>(i, j);
                if (d < min_disp){
                    //TODO find out why issue with disp < ~3 causing negative w and so negative z
                    if (CVSupport::genZ(_Q,i,j,d) > 0){
                        min_disp = d;
                        min_i = i;
                        min_j = j;
                    }
                }
                if (d > max_disp){
                    max_disp = d;
                    max_i = i;
                    max_j = j;
                }
            }
        }
    }

    min_depth = (double)CVSupport::genZ(_Q,(double)max_i,(double)max_j,(double)max_disp);
    max_depth = (double)CVSupport::genZ(_Q,(double)min_i,(double)min_j,(double)min_disp);
    */

    double min_disp, max_disp, min_depth, max_depth;
    CVSupport::getMinMaxDisparity(disparity_thresh,Q,min_disp,max_disp);
    CVSupport::getMinMaxDepth(disparity_thresh,Q,min_depth,max_depth);
    min_depth_ = min_depth;
    max_depth_ = max_depth;

    //use depth rather than disparity for more user readable results
    double range_disp = max_disp - min_disp;
    double range_depth = max_depth - min_depth;

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

    cv::Mat disparity_vis;
    CVSupport::disparity2colormap(disparity,Q,disparity_vis);

    cv::cvtColor(disparity_vis, disparity_vis, cv::COLOR_BGR2RGB);

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
