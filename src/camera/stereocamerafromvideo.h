/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERAFROMVIDEO_H
#define STEREOCAMERAFROMVIDEO_H

#include <abstractstereocamera.h>
#include <QThread>

//!  Stereo video feed
/*!
  Control of stereo video and generation of 3D
*/

class StereoCameraFromVideo : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraFromVideo(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();

    ~StereoCameraFromVideo(void);

public slots:
    // Implimentations of virtual functions from parent class
    bool openCamera();
    bool closeCamera();
    bool captureSingle();
    bool enableCapture(bool enable);
    bool setFPS(int fps);
    bool setExposure(double){return false;} //NA
    bool enableAutoExposure(bool){return false;} //NA
    bool setPacketSize(int){return false;} //NA
    bool setPacketDelay(int){return false;} //NA
    bool enableTrigger(bool){return false;} //NA
    bool enableAutoGain(bool){return false;} //NA
    bool setGain(int){return false;} //NA
    bool setBinning(int){return false;} //NA

    void captureThreaded();
    void setPosition(int position);

signals:
    void videoPosition(int);

private:
    enum StereoVideoType {
        RG_VIDEO = 0,
        CONCAT_VIDEO = 1,
        UNKNOWN_VIDEO = -1
    };

    cv::Mat extractFirstFrame();
    StereoVideoType detectVideoType();

    QFuture<void> future;
    cv::VideoCapture stream;
    cv::Mat image_buffer;
    double number_frames;
    QElapsedTimer frame_timer;
    double video_fps;
    StereoVideoType video_type;
};

#endif // STEREOCAMERAFROMVIDEO_H
