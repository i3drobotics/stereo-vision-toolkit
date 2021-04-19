/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERATIS_H
#define STEREOCAMERATIS_H

#include <QObject>
#include <abstractstereocamera.h>
#include <cameraimagingsource.h>

//!  Stereo Imaging Source camera control
/*!
  Control of Imaging Source stereo pair and generation of 3D
*/

class StereoCameraTIS   : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraTIS (AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystemsQuick(DShowLib::Grabber* handle);

    ~StereoCameraTIS(void);

signals:
    void stereo_grab();
    void start_capture();
    void stop_capture();

public slots:
    // Implimentations of virtual functions from parent class
    bool openCamera();
    bool closeCamera();
    bool captureSingle();
    bool enableCapture(bool enable);
    bool setFPS(int fps);
    bool setExposure(double exposure);
    bool enableAutoExposure(bool enable);
    bool setPacketSize(int){return false;} //NA
    bool setPacketDelay(int){return false;} //NA
    bool enableTrigger(bool enable);
    bool enableAutoGain(bool enable);
    bool setGain(int gain);
    bool setBinning(int) {return false;} //NA

    void captureThreaded();
    void loadLeftSettings();
    void loadRightSettings();
    void leftGrabFailed();
    void rightGrabFailed();
    void leftCaptured();
    void rightCaptured();

private:
    bool setCameras(AbstractStereoCamera::StereoCameraSettings inital_camera_settings, CameraImagingSource *camera_left, CameraImagingSource *camera_right, Listener *listener_left, Listener *listener_right);

    void checkStereoCapture();

    CameraImagingSource *left_camera;
    CameraImagingSource *right_camera;

    std::vector<std::unique_ptr<CameraImagingSource>> stereo_cameras;
    std::vector<std::unique_ptr<Listener>> stereo_listeners;

    Listener *left_listener;
    Listener *right_listener;

    QList<qint64> load_serials(QString filename);
    QList<int> widths;
    QList<int> heights;

    bool grab_success_l = false;
    bool grab_success_r = false;

    bool grab_finish_l = false;
    bool grab_finish_r = false;

    void setup_cameras(AbstractStereoCamera::StereoCameraSettings inital_camera_settings);

    QFuture<void> future;
};

#endif // STEREOCAMERATIS_H
