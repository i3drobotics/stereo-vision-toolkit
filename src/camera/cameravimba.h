#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QMutex>
#include <QElapsedTimer>

#include <opencv2/opencv.hpp>
#include <VimbaCPP/Include/VimbaCPP.h>

using namespace AVT::VmbAPI;

// Constructor for the FrameObserver class
class FrameObserver : public QObject, public IFrameObserver{
    Q_OBJECT
public:
    explicit FrameObserver(CameraPtr pCamera) : IFrameObserver(pCamera){}

    // Frame callback notifies about incoming frames
    void FrameReceived(const FramePtr pFrame){
        bool bQueueDirectly = true;
        VmbFrameStatusType eReceiveStatus;
        auto receive_error = pFrame->GetReceiveStatus( eReceiveStatus );
        int num_recievers = receivers(SIGNAL(frameReady(int)));

        if( num_recievers!= 0 &&  receive_error == VmbErrorSuccess ){
            // Lock the frame queue
            frame_mutex.lock();
            frame_queue.push( pFrame );
            // Unlock frame queue
            frame_mutex.unlock();
            // Emit the frame received signal
            emit frameReady(eReceiveStatus);
            bQueueDirectly = false;

        }else{
            m_pCamera->QueueFrame( pFrame );
        }
    }

    FramePtr getFrame(void){
        // Lock the frame queue
        frame_mutex.lock();
        // Pop frame from queue
        FramePtr res;
        if( !frame_queue.empty() )
        {
            res = frame_queue.front();
            frame_queue.pop();
        }
        // Unlock frame queue
        frame_mutex.unlock();
        return res;
    }

    void clearFrameQueue(){
        // Lock the frame queue
        frame_mutex.lock();
        // Clear the frame queue and release the memory
        std::queue<FramePtr> empty;
        std::swap( frame_queue, empty );
        // Unlock the frame queue
        frame_mutex.unlock();
    }

signals:
    void frameReady(int status);

private:
    std::queue<FramePtr> frame_queue;
    QMutex frame_mutex;
};

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
    QMutex frame_mutex;
    bool capturing = false;

    shared_ptr<FrameObserver> frame_observer;
    FramePtrVector frame_buffer;

    VmbError_t getStringFeature(std::string feature, std::string &res);
    VmbError_t getBoolFeature(std::string feature, bool &res);
    VmbError_t getDoubleFeature(std::string feature, double &res);
    VmbError_t getIntFeature(std::string feature, long long &res);
    double getFPS();

public slots:
    bool capture(void);
    bool getImage(cv::Mat &out);

    void startCapture();
    void stopCapture();
private slots:
    void onFrame(int);
};

#endif // CAMERAVIMBA_H
