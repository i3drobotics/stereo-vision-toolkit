/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight
*/

#ifndef CAMERABASLER_H
#define CAMERABASLER_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

//!  Basler camera control
/*!
  Connecting and controlling a basler camera using Pylon
  More info see: https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/
*/

class CameraBasler : public QObject
{
    Q_OBJECT
public:
    explicit CameraBasler(QObject *parent = 0);
    bool isAvailable();
    void close();
    bool initCamera(std::string camera_name, int binning=1);
    void assignThread(QThread *thread);
    void getImageSize(int &image_width, int &image_height, cv::Size &image_size);
    bool setFrame16();
    bool setFrame8();
    bool setMaximumResolution();
    bool setExposure(double exposure);
    bool setAutoExposure(bool enable);
    bool setGain(double gain);
    bool setPacketSize(int packetSize);
    bool setInterPacketDelay(int interPacketDelay);
    bool setBinning(int binning);
    ~CameraBasler(void);

signals:
    void captured();
    void finished();

private:
    Pylon::CInstantCamera *camera = nullptr;
    Pylon::IPylonDevice *camera_device = nullptr;
    Pylon::CImageFormatConverter formatConverter;
    cv::Mat image;
    cv::Mat channels[3];
    cv::Mat image_buffer;
    enum format {Y800, Y16};
    format image_format;
    bool connected = false;

public slots:
    bool capture(void);
    cv::Mat* getImage(void);
};

#endif // CAMERABASLER_H
