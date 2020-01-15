/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCAMERAFROMVIDEO_H
#define STEREOCAMERAFROMVIDEO_H

#include <abstractstereocamera.h>

//!  Stereo video feed
/*!
  Control of stereo video and generation of 3D
*/

class StereoCameraFromVideo : public AbstractStereoCamera
{

    Q_OBJECT

public:
    explicit StereoCameraFromVideo(QObject *parent = 0) :
                AbstractStereoCamera(parent)
                {}
    bool capture();
    void disconnectCamera();
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_info);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    bool autoConnect();
    void toggleAutoExpose(bool enable){};
    void adjustExposure(double exposure){};
    void toggleAutoGain(bool enable){};
    void adjustGain(int gain){};
    void adjustBinning(int gain){};
public slots:
    void setPosition(int position);
    bool enableAutoExpose(bool enable);
private:
    cv::VideoCapture stream;
    cv::Mat image_buffer;
    std::string stream_file;
    bool stream_valid = false;
    double number_frames;
    QElapsedTimer frame_timer;

signals:
    void videoPosition(int);
};


#endif // STEREOCAMERAFROMVIDEO_H
