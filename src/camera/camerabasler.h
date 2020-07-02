/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
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
    bool initCamera(std::string camera_serial, int binning, bool trigger, int fps, int packet_size);
    void assignThread(QThread *thread);
    void getImageSize(int &image_width, int &image_height, cv::Size &image_size);
    bool setFrame16();
    bool setFrame8();
    bool setMaximumResolution();
    bool setPTP(bool enable);
    bool setExposure(double exposure);
    bool setAutoExposure(bool enable);
    bool setAutoGain(bool enable);
    bool setGain(int gain);
    bool setPacketSize(int packetSize);
    bool setInterPacketDelay(int interPacketDelay);
    bool setBinning(int binning);
    bool enableTrigger(bool enable);
    bool setTrigger(bool enable);

    bool changeFPS(int fps);
    bool changeBinning(int binning);
    bool changePacketSize(int packet_size);
    bool setFPS(int fps);
    bool enableFPS(bool enable);

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
    int binning;
    bool trigger;
    int fps;
    int packet_size;
    std::string camera_serial;
    int max_timeout = 2000;

public slots:
    bool capture(void);
    cv::Mat* getImage(void);
};

#endif // CAMERABASLER_H
