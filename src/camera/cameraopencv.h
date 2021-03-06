/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef CAMERAOPENCV_H
#define CAMERAOPENCV_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <opencv2/opencv.hpp>

//!  OpenCV camera control
/*!
  Connecting and controlling a camera using OpenCV
  More info see: https://docs.opencv.org/3.4.1/dd/de7/group__videoio.html
*/

class CameraOpenCV : public QObject
{
    Q_OBJECT
public:
    explicit CameraOpenCV(QObject *parent = 0);
    bool isAvailable();
    void close();
    bool initCamera(int index);
    void assignThread(QThread *thread);
    void getImageSize(int &image_width, int &image_height, cv::Size &image_size);
    bool setFrame16();
    bool setFrame8();
    bool setMaximumResolution();
    bool setExposure(double exposure);
    bool setGain(double gain);
    bool setFPS(double fps);
    ~CameraOpenCV(void);

signals:
    void captured();
    void finished();

private:
    cv::VideoCapture camera;
    cv::Mat image;
    cv::Mat channels[3];
    cv::Mat image_buffer;
    enum format {Y800, Y16};
    format image_format;

public slots:
    bool capture(void);
    cv::Mat* getImage(void);
};

#endif // CAMERAOPENCV_H
