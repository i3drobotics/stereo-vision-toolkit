#include "cameradisplaywidget.h"
#include "ui_cameradisplaywidget.h"

CameraDisplayWidget::CameraDisplayWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraDisplayWidget)
{
    ui->setupUi(this);

    /* Standard grayscale */
    for (int i = 0; i < 256; i++) colourMap.push_back(qRgb(i, i, i));
}

void CameraDisplayWidget::updateView(cv::Mat new_image){
    cv::Mat downsample_image;
    cv::resize(new_image,downsample_image,cv::Size(),downsample_rate,downsample_rate);
    pixmap = ASM::cvMatToQPixmap(downsample_image);

    if(pixmap.isNull()) return;

    ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));
}

void CameraDisplayWidget::setSize(int width, int height, int bit_depth) {
    int image_size = width * height;

    if (image_size > camMaxSize){
        float width_scale = (float)camMaxWidth/(float)width;
        float height_scale = (float)camMaxHeight/(float)height;

        if (width_scale < height_scale){
            downsample_rate = width_scale;
        } else {
            downsample_rate = height_scale;
        }

        camwidth = width * downsample_rate;
        camheight = height * downsample_rate;
    } else {
        downsample_rate = 1;
        camwidth = width;
        camheight = height;
    }

    displayBuffer.resize(camwidth * camheight * bit_depth);
    display_image = new QImage(camwidth, camheight, QImage::Format_Indexed8);
    display_image->setColorTable(colourMap);
}

CameraDisplayWidget::~CameraDisplayWidget()
{
    delete ui;
    if(display_image) delete display_image;
}
