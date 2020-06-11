/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef STEREOCAMERAOPENCV_H
#define STEREOCAMERAOPENCV_H

#include <abstractstereocamera.h>

// Correct for math.h warnings
#define NOMINMAX
//#define _MATH_DEFINES_DEFINED
#include <windows.h>
#include <dshow.h>
#include <hidapi/hidapi.h>
#include <QTimer>
#include <mutex>          // std::mutex

class StereoCameraOpenCV : public AbstractStereoCamera
{
Q_OBJECT

public:
    explicit StereoCameraOpenCV(QObject *parent = 0) :
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

    ~StereoCameraOpenCV(void);

public slots:
    bool setExposure(double exposure_time);
    bool toggleHDR(bool enable);
    bool enableAutoExpose(bool enable);
    double getTemperature(void);
    void changeFPS(int fps);

private:
    cv::VideoCapture camera_r;
    cv::VideoCapture camera_l;
    cv::Mat image_buffer_r;
    cv::Mat image_buffer_l;
    double exposure;

    std::mutex mtx;

    hid_device* cam_device_r = NULL;
    hid_device* cam_device_l = NULL;
    bool send_hid(hid_device* cam_device, std::vector<unsigned char> &buffer, size_t command_len);
    std::string serial_from_device_path(std::string usb_device_path);
    std::string get_device_path_serial(IMoniker *pMoniker);

    std::string wchar_to_string(WCHAR * buffer);

    qint64 getSerial(void);

    void getImageSize(cv::VideoCapture camera, int &width, int &height, int &bitdepth);

};

#endif // STEREOCAMERAOPENCV_H
