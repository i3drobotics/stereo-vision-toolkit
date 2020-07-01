/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCAMERAOPENCV_H
#define STEREOCAMERAOPENCV_H

#include <abstractstereocamera.h>
// Correct for math.h warnings
#define NOMINMAX
#define _MATH_DEFINES_DEFINED
#include <windows.h>
#include <dshow.h>
#include <hidapi/hidapi.h>
#include <QTimer>
#include <mutex>          // std::mutex

//!  OpenCV camera control
/*!
  Control of opencv camera and generation of 3D
*/

class StereoCameraOpenCV  : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraOpenCV (AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
    }

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems();
    static int usb_index_from_serial(std::string serial);
    static std::string serial_from_usb_index(int index);

    bool setFrameSize(int width, int height);
    bool setFrame16(void);
    void getFrameRate(void);

    void openHID();

    int getExposure();

    ~StereoCameraOpenCV(void);

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
    bool enableTrigger(bool){return false;} //NA
    bool enableAutoGain(bool){return false;} //NA
    bool setGain(int){return false;} //NA
    bool setBinning(int){return false;} //NA

    bool enableHDR(bool enable);
    void captureThreaded();

private:
    static std::string serial_from_device_path(std::string usb_device_path);
    static std::string get_device_path_serial(IMoniker *pMoniker);
    static std::string wchar_to_string(WCHAR * buffer);

    cv::VideoCapture camera_r;
    cv::VideoCapture camera_l;
    cv::Mat image_buffer_r;
    cv::Mat image_buffer_l;

    cv::Mat channels[3];

    hid_device* cam_device_r = NULL;
    hid_device* cam_device_l = NULL;

    double exposure;

    QFuture<void> future;

    bool send_hid(hid_device* cam_device, std::vector<unsigned char> &buffer, size_t command_len);

    qint64 getSerial(void);
};

#endif // STEREOCAMERAOPENCV_H
