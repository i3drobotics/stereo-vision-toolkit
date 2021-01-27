/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERAFROMIMAGE_H
#define STEREOCAMERAFROMIMAGE_H

#include <abstractstereocamera.h>
#include <QThread>

//!  Stereo image feed
/*!
  Vitual stereo camera from image feed
*/

class StereoCameraFromImage : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraFromImage(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();

    ~StereoCameraFromImage(void);

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

signals:
    void videoPosition(int);

private:
    QFuture<void> future;
    QElapsedTimer frame_timer;
    double video_fps;
};

#endif // STEREOCAMERAFROMIMAGE_H
