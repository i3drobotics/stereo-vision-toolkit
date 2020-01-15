#ifndef STEREOCAMERATIS_H
#define STEREOCAMERATIS_H

#include <QObject>
#include <abstractstereocamera.h>
#include <cameraimagingsource.h>

//!  Stereo Imaging Source camera control
/*!
  Control of Imaging Source stereo pair and generation of 3D
*/

class StereoCameraTIS : public AbstractStereoCamera
{

Q_OBJECT

signals:
    void start_capture();
    void stereopair_captured();
    void update_size(int width, int height, int bitdepth);

public slots:
    void loadLeftSettings();
    void loadRightSettings();
    void enableTrigger(bool enable);
    void setExposure(double exposure);
    void setGain(int gain);
    void enableAutoExpose(bool enable);
    void enableAutoGain(bool enable);
    void changeFPS(int fps);

public:
    explicit StereoCameraTIS(QObject *parent = 0) :
                AbstractStereoCamera(parent)
                {}
    bool capture();
    void disconnectCamera();
    bool initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info);
    bool setCameras(CameraImagingSource *camera_left, CameraImagingSource *camera_right, Listener *listener_left, Listener *listener_right);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems();
    void toggleAutoExpose(bool enable);
    void adjustExposure(double exposure);
    void toggleAutoGain(bool enable);
    void adjustGain(int gain);
    void adjustBinning(int gain){}; //TODO create binning setting function
    bool autoConnect();

    ~StereoCameraTIS(void);

private:
    CameraImagingSource *left_camera;
    CameraImagingSource *right_camera;

    std::vector<std::unique_ptr<CameraImagingSource>> stereo_cameras;
    std::vector<std::unique_ptr<Listener>> stereo_listeners;

    Listener *left_listener;
    Listener *right_listener;

    QList<qint64> load_serials(QString filename);
    QList<int> widths;
    QList<int> heights;

    void setup_cameras();
};

#endif // STEREOCAMERATIS_H
