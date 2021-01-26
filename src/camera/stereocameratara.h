/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef STEREOCAMERATARA_H
#define STEREOCAMERATARA_H

#include <abstractstereocamera.h>

// Correct for math.h warnings
#define NOMINMAX
//#define _USE_MATH_DEFINES
//#define _MATH_DEFINES_DEFINED
#define _WINSOCKAPI_
#include <windows.h>
#include <dshow.h>
#include <hidapi/hidapi.h>
#include <QThread>

//!  Tara camera control
/*!
  Control of tara stereo camera
  https://www.e-consystems.com/3D-USB-stereo-camera.asp
*/

class StereoCameraTara : public AbstractStereoCamera
{
Q_OBJECT

public:

    explicit StereoCameraTara(AbstractStereoCamera::StereoCameraSerialInfo serial_info,
                                AbstractStereoCamera::StereoCameraSettings camera_settings,
                                QObject *parent = 0) :
                AbstractStereoCamera(serial_info, camera_settings, parent){
    }

    enum TARA_COMMAND{
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

    static std::vector<AbstractStereoCamera::StereoCameraSerialInfo> listSystems(std::string friendly_name="See3CAM_Stereo");
    static int usb_index_from_serial(std::string serial);
    static std::string serial_from_usb_index(int index);

    bool setFrameSize(int width, int height);
    bool setFrame16(void);
    void getFrameRate(void);

    void openHID();

    int getExposure();

    ~StereoCameraTara(void);

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

protected:
    static std::string get_device_path_serial(IMoniker *pMoniker);

private:
    static std::string serial_from_device_path(std::string usb_device_path);
    static std::string wchar_to_string(WCHAR * buffer);

    cv::VideoCapture camera;
    cv::Mat image_buffer;
    cv::Mat channels[3];
    double exposure;

    QFuture<void> future;

    hid_device* deimos_device = NULL;

    bool send_hid(std::vector<unsigned char> &buffer, size_t command_len);

    qint64 getSerial(void);
};

#endif // STEREOCAMERATARA_H
