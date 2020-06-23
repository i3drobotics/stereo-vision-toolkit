#include "cameradisplaywidget.h"
#include "ui_cameradisplaywidget.h"

CameraDisplayWidget::CameraDisplayWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraDisplayWidget)
{
    ui->setupUi(this);

    /* Standard grayscale */
    for (int i = 0; i < 256; i++) colourMap.push_back(qRgb(i, i, i));

    min_update_time = 1000 * (1.0f/(double)max_update_rate);
}

void CameraDisplayWidget::updateView(cv::Mat new_image){
    if (this->isVisible()){
        // check time since last update
        qint64 time_since_update = update_timer.elapsed();
        // for increased performance update time cannot exeed > 30fps
        if (time_since_update > min_update_time){

            pixmap = ASM::cvMatToQPixmap(new_image);

            if(pixmap.isNull()) return;

            ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));
            update_timer.restart();
        }
    }
}

void CameraDisplayWidget::setSize(int width, int height, int bit_depth) {
    int image_size = width * height;
    int max_size = max_width * max_height;

    if (image_size > max_size){
        float width_scale = (float)max_width/(float)width;
        float height_scale = (float)max_height/(float)height;

        if (width_scale < height_scale){
            downsample_rate = width_scale;
        } else {
            downsample_rate = height_scale;
        }

        image_width = width * downsample_rate;
        image_height = height * downsample_rate;
    } else {
        downsample_rate = 1;
        image_width = width;
        image_height = height;
    }

    displayBuffer.resize(image_width * image_height * bit_depth);
    display_image = new QImage(image_width, image_height, QImage::Format_Indexed8);
    display_image->setColorTable(colourMap);
}

CameraDisplayWidget::~CameraDisplayWidget()
{
    delete ui;
    if(display_image) delete display_image;
}
