/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

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

cv::Scalar CameraDisplayWidget::hsv2bgr(cv::Scalar hsv){
    /*
    H(Hue): 0-360 degrees
    S(Saturation): 0-100 percent
    V(Value): 0-100 percent

    R(Red): 0-255
    G(Green): 0-255
    B(Blue): 0-255
    */
    float H = hsv[0];
    float S = hsv[1];
    float V = hsv[2];
    if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
        return cv::Scalar(0,0,0);
    }
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    int R = (r+m)*255;
    int G = (g+m)*255;
    int B = (b+m)*255;
    return cv::Scalar(B,G,R);
}

void CameraDisplayWidget::updateView(cv::Mat new_image){
    if (this->isVisible()){
        // check time since last update
        qint64 time_since_update = update_timer.elapsed();
        // for increased performance update time cannot exeed > 30fps
        if (time_since_update > min_update_time){

            cv::Mat display_image = new_image.clone();
            if (this->show_epipolar_lines){
                // draw epipilar lines on image
                int lineType = cv::LINE_8;
                int numOfLines = 10;
                int lineSpacing = display_image.size().height/numOfLines;
                int thickness = (int)lineSpacing/100.0;
                if (thickness <= 1){ thickness = 1;}
                for (int i = 0; i < numOfLines-1; i++){
                    int line_y = lineSpacing * (i+1);
                    cv::Point start(0,line_y);
                    cv::Point end(display_image.size().width,line_y);
                    int hue = (int)(360.0/numOfLines)*i;
                    cv::Scalar hsv_color(hue, 100, 100);
                    cv::Scalar bgr_color = hsv2bgr(hsv_color);
                    cv::line( display_image,
                        start,
                        end,
                        bgr_color,
                        thickness,
                        lineType);
                }
            }

            pixmap = ASM::cvMatToQPixmap(display_image);

            if(pixmap.isNull()) return;

            ui->imageDisplay->setPixmap(pixmap.scaled(ui->imageDisplay->size(), Qt::KeepAspectRatio));
            update_timer.restart();
        }
    }
}

void CameraDisplayWidget::setSize(int width, int height, int bit_depth) {
    //TODO fix problem with recitifed images resizing incorrectly
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
