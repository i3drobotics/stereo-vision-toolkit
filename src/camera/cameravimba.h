#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <opencv2/opencv.hpp>
#include <VimbaCPP/Include/VimbaCPP.h>

using namespace AVT::VmbAPI;

class CameraVimba : public QObject
{
    Q_OBJECT
public:
    explicit CameraVimba(QObject *parent = nullptr);
    bool isAvailable();
    void close();
    bool initCamera(std::string camera_serial, int binning, bool trigger, int fps);
    void assignThread(QThread *thread);
    void getImageSize(int &image_width, int &image_height, int &bit_depth);
    bool setFrame16();
    bool setFrame8();
    bool setMaximumResolution();
    bool setPTP(bool enable);
    bool setExposure(double exposure);
    bool enableAutoExposure(bool enable);
    bool enableAutoGain(bool enable);
    bool setGain(int gain);
    bool setPacketSize(int packetSize);
    bool setInterPacketDelay(int interPacketDelay);
    bool setBinning(int binning);
    bool enableTrigger(bool enable);

    bool changeFPS(int fps);
    bool changeBinning(int binning);
    bool changePacketSize(int packet_size);
    bool setFPS(int fps);
    bool enableFPS(bool enable);

    ~CameraVimba(void);

signals:
    void captured();
    void finished();

private:
    VimbaSystem &system;
    CameraPtr camera;
    cv::Mat image;
    cv::Mat channels[3];
    cv::Mat image_buffer;
    enum format {Y800, Y16};
    format image_format;
    bool connected = false;
    int binning = 1;
    bool trigger = false;
    int fps = 30;
    std::string camera_serial;
    int max_timeout = 2000;

    VmbError_t getStringFeature(std::string feature, std::string res);
    VmbError_t getBoolFeature(std::string feature, bool res);
    VmbError_t getDoubleFeature(std::string feature, double res);
    VmbError_t getIntFeature(std::string feature, VmbInt64_t res);


public slots:
    bool capture(void);
    cv::Mat* getImage(void);
};

#endif // CAMERAVIMBA_H
