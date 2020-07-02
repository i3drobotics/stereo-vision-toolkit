/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef CAMERADISPLAYWIDGET_H
#define CAMERADISPLAYWIDGET_H

#include <QWidget>
#include <QDebug>
#include <QElapsedTimer>
#include "ui_cameradisplaywidget.h"
#include <opencv2/opencv.hpp>
#include "asmopencv.h"

namespace Ui {
class CameraDisplayWidget;
}

//!  Camera display widget
/*!
  QT Widget for displaying camera feed
*/

class CameraDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraDisplayWidget(QWidget *parent = nullptr);
    ~CameraDisplayWidget();

    void updateView(cv::Mat new_image);

    QLabel* getImageDisplay(void){return ui->imageDisplay;}

    float getDownsampleRate(){ return downsample_rate; }

public slots:
    void setSize(int height, int width, int bit_depth);

private:
    Ui::CameraDisplayWidget *ui;

    QVector<QRgb> colourMap;
    int image_width = 0;
    int image_height = 0;

    int max_width = 800;
    int max_height = 600;
    int max_update_rate = 30;
    int min_update_time = 0;
    float downsample_rate = 1;

    QPixmap pixmap;
    QImage *display_image = nullptr;

    QElapsedTimer update_timer;

    bool capturing = true;
    std::vector<unsigned char> displayBuffer;
};

#endif // CAMERADISPLAYWIDGET_H
