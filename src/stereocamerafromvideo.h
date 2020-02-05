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
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    void toggleAutoExpose(bool){} //NA
    void adjustExposure(double){} //NA
    void toggleAutoGain(bool){} //NA
    void adjustGain(int){} //NA
    void adjustBinning(int){} //NA
    void toggleTrigger(bool){} //NA
    void adjustFPS(int){} //NA
    void adjustPacketSize(int){}
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
