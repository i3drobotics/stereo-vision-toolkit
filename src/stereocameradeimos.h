/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCAMERADEIMOS_H
#define STEREOCAMERADEIMOS_H

#include <abstractstereocamera.h>

// Correct for math.h warnings
#define NOMINMAX
//#define _MATH_DEFINES_DEFINED
#include <windows.h>
#include <dshow.h>
#include <hidapi/hidapi.h>
#include <QTimer>
#include <mutex>          // std::mutex

//!  Deimos camera control
/*!
  Control of deimos camera and generation of 3D
*/

enum DEIMOS_COMMAND{
    CAMERA_CONTROL_STEREO = 0x78,
    READFIRMWAREVERSION = 0x40,
    GETCAMERA_UNIQUEID = 0x41,
    GET_EXPOSURE_VALUE = 0x01,
    SET_EXPOSURE_VALUE = 0x02,
    SET_IMU_CONFIG = 0x04,
    SET_HDR_MODE_STEREO = 0x0E,
    CONTROL_IMU_VAL = 0x05,
    SEND_IMU_VAL_BUFF = 0x06,
    IMU_ACC_VAL = 0xFE,
    IMU_GYRO_VAL = 0xFD,
    GET_IMU_TEMP_DATA = 0x0D
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
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings);
    bool setFrameSize(int width, int height);
    bool setFrame16(void);
    void getFrameRate(void);

    void openHID();

    int getExposure();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double exposure);
    void adjustPacketSize(int){}
    void toggleAutoGain(bool){} //TODO create auto gain setting function
    void adjustGain(int){} //TODO create gain setting function
    void adjustBinning(int){} //TODO create binning setting function
    void toggleTrigger(bool){} //NA
    void adjustFPS(int fps);

    int usb_index_from_serial(std::string serial);
    std::string serial_from_usb_index(int index);

    ~StereoCameraDeimos(void);

public slots:
    bool setExposure(double exposure_time);
    bool toggleHDR(bool enable);
    bool enableAutoExpose(bool enable);
    double getTemperature(void);
    void changeFPS(int fps);

private:
    cv::VideoCapture camera;
    cv::Mat image_buffer;
    cv::Mat channels[3];
    double exposure;
    QTimer *temperature_timer;

    std::mutex mtx;

    BSTR device_path;
    hid_device* deimos_device = NULL;
    bool send_hid(std::vector<unsigned char> &buffer, size_t command_len);
    std::string serial_from_device_path(std::string usb_device_path);
    std::string get_device_path_serial(IMoniker *pMoniker);

    qint64 getSerial(void);

};

#endif // STEREOCAMERADEIMOS_H
