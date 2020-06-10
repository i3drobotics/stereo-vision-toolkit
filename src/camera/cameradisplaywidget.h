#ifndef CAMERADISPLAYWIDGET_H
#define CAMERADISPLAYWIDGET_H

#include <QWidget>
#include <QDebug>
#include "ui_cameradisplaywidget.h"
#include <opencv2/opencv.hpp>
#include "asmopencv.h"

//!  Camera display widget
/*!
  QT Widget for displaying camera feed
*/

namespace Ui {
class CameraDisplayWidget;
}

class CameraDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraDisplayWidget(QWidget *parent = nullptr);
    ~CameraDisplayWidget();

    void updateView(cv::Mat new_image){
        cv::Mat downsample_image;
        cv::resize(new_image,downsample_image,cv::Size(),downsample_rate,downsample_rate);
        pixmap = ASM::cvMatToQPixmap(downsample_image);

        if(pixmap.isNull()) return;

        ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));
    }

    QLabel* getImageDisplay(void){return ui->imageDisplay;}

public slots:
    void setSize(int height, int width, int bit_depth);

private:
    Ui::CameraDisplayWidget *ui;

    QVector<QRgb> colourMap;
    int camwidth = 0;
    int camheight = 0;

    int camMaxSize = 480000; //800×600
    int camMaxWidth = 800;
    int camMaxHeight = 600;
    float downsample_rate = 1;

    QPixmap pixmap;
    QImage *display_image = nullptr;

    bool capturing = true;
    std::vector<unsigned char> displayBuffer;
};

#endif // CAMERADISPLAYWIDGET_H
