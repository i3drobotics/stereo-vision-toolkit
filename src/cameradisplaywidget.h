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
        //cv::Mat cvMat = ASM::QPixmapToCvMat( inPixmap, false );
        pixmap = ASM::cvMatToQPixmap(new_image);
        /*
        new_image.copyTo(cv_display_image);
        if (cv_display_image.empty()) return;
        display_image = new QImage((unsigned char*) cv_display_image.data, cv_display_image.cols, cv_display_image.rows,QImage::Format_RGB888);

        if (display_image->isNull()) return;

        pixmap = QPixmap::fromImage(*display_image);
        */

        if(pixmap.isNull()) return;

        ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));
    }

    /*

    void updateView(cv::Mat *new_image) {

        if (new_image->empty()) return;

      if(sizeof(T) != 1){
          int scale = 256 << 8*(sizeof(T) - 1);

          memcpy(displayBuffer.data(), new_image, sizeof(T) * displayBuffer.size());

          for (int i = 0; i < camwidth * camheight; i++) {
            display_image->bits()[i] = static_cast<T>(displayBuffer[i] / scale);
          }

      }else{
          memcpy(display_image->bits(), new_image, sizeof(T) * displayBuffer.size());
      }

      if (display_image->isNull()) return;

      pixmap = QPixmap::fromImage(*display_image);

      if(pixmap.isNull()) return;

      ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));

    }
    */

    void setSettingsCallback(QObject* receiver, const char* slot);

    QLabel* getImageDisplay(void){return ui->imageDisplay;}

public slots:
    void setSize(int height, int width, int bit_depth);

private:
    Ui::CameraDisplayWidget *ui;

    QVector<QRgb> colourMap;
    int camwidth = 0;
    int camheight = 0;

    QPixmap pixmap;
    QImage *display_image = nullptr;

    bool capturing = true;
    std::vector<unsigned char> displayBuffer;
};

#endif // CAMERADISPLAYWIDGET_H
