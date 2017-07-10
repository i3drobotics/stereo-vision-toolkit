/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCAMERADEIMOS_H
#define STEREOCAMERADEIMOS_H

#include <abstractstereocamera.h>

// All the Windows hoops we have to jump through to check USB devices
#define NOMINMAX
#define _MATH_DEFINES_DEFINED
#include <windows.h>
#include <dshow.h>

#include <QWinEventNotifier>
#include <hidapi/hidapi.h>


enum DEIMOS_COMMAND{
    CAMERA_CONTROL_STEREO = 0x78,
    READFIRMWAREVERSION = 0x40,
    GETCAMERA_UNIQUEID = 0x41,
    GET_EXPOSURE_VALUE = 0x01,
    SET_EXPOSURE_VALUE = 0x02,
    SET_IMU_CONFIG = 0x04,
    CONTROL_IMU_VAL = 0x05,
    SEND_IMU_VAL_BUFF = 0x06,
    IMU_ACC_VAL = 0xFE,
    IMU_GYRO_VAL = 0xFD
};

class StereoCameraDeimos : public AbstractStereoCamera
{
Q_OBJECT

public:
    explicit StereoCameraDeimos(QObject *parent = 0) :
                AbstractStereoCamera(parent)
                {}
    bool capture();
    void disconnectCamera();
    bool initCamera(int devid = -1);
    int findCamera(void);
    bool setFrameSize(int width, int height);
    bool setFrame16(void);
    void getFrameRate(void);

    void openHID();

    int getExposure();

    ~StereoCameraDeimos(void);

public slots:
    bool setExposure(double);
    bool enableAutoExpose(bool enable);

private:
    cv::VideoCapture camera;
    cv::Mat image_buffer;
    cv::Mat channels[3];
    double exposure;

    BSTR device_path;
    hid_device* deimos_device = NULL;
    bool send_hid(std::vector<unsigned char> &buffer, size_t command_len);

    int getSerial(void);

};

std::string wchar_to_string(WCHAR* buffer);

#endif // STEREOCAMERADEIMOS_H
