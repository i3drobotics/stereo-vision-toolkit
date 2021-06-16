/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef SVTK_WINDOW_H
#define SVTK_WINDOW_H

#define _USE_MATH_DEFINES

#include "cmath"
#include <QtAwesome.h>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QSpacerItem>
#include <QTimer>
#include <QString>
#include <QDesktopServices>
#include <QUrl>
#include <QStandardItemModel>
#include <QColorDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

#include "aboutdialog.h"

#include "matcherwidgetopencvblock.h"
#include "matcherwidgetopencvsgbm.h"
#ifdef WITH_I3DRSGM
    #include "matcherwidgeti3drsgm.h"
#endif
#include "stereocameratara.h"
#include "stereocameradeimos.h"
#include "stereocamerafromvideo.h"
#include "stereocamerafromimage.h"
#include "stereocameraopencv.h"
#include "stereocameratis.h"
#include "stereocamerabasler.h"
#include "stereocameraphobos.h"
#include "stereocameratitania.h"
#ifdef WITH_VIMBA
    #include "stereocameravimba.h"
#endif
#include "stereocamerasupport.h"
#include "cvsharedmemory.hpp"

#include <disparityviewer.h>
#include "calibratefromimagesdialog.h"
#include "calibrationdialog.h"
#include "stereocalibrate.h"
#include "cameradisplaywidget.h"

#include "detectoropencv.h"
#include "detectorsetupdialog.h"

#include "loadstereoimagepairdialog.h"

#include "paramfile.h"
#ifdef WITH_FERVOR
#include "fvupdater.h"
#endif

using namespace std;
using namespace cv;

namespace Ui {
class SVTKWindow;
}

/**
 * @brief QT Main Window of application
 */
class SVTKWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
    * @brief Construct a new Main Window object
    * 
    * @param parent 
    */
    explicit SVTKWindow(QWidget* parent = nullptr);
    /**
     * @brief Destroy the Main Window object
     * 
     */
    ~SVTKWindow();

private:
    Ui::SVTKWindow* ui;
    QThread* cam_thread;

    int fps_measure_count = 0;
    float fps_measure_total = 0;
    int match_fps_measure_count = 0;
    float match_fps_measure_total = 0;

    int CAMERA_CONNECTION_SUCCESS_EXIT_CODE = 0;
    int CAMERA_CONNECTION_FAILED_EXIT_CODE = -1;
    int CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE = -2;
    int CAMERA_CONNECTION_CANCEL_EXIT_CODE = -3;

    AbstractStereoCamera::StereoCameraSettings default_basler_gige_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_basler_usb_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_tis_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_tara_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_usb_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_video_init_settings;
    AbstractStereoCamera::StereoCameraSettings current_camera_settings;
    AbstractStereoCamera::StereoCameraSettings default_vimba_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_deimos_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_phobos_basler_gige_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_phobos_basler_usb_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_phobos_tis_usb_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_titania_basler_gige_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_titania_basler_usb_init_settings;
    AbstractStereoCamera::StereoCameraSettings default_titania_vimba_usb_init_settings;

    QPixmap pmap_left;
    QPixmap pmap_right;
    QPixmap pmap_disparity;
    QTimer* status_bar_timer;

    QMessageBox* error_msgBox = NULL;

    QFuture<void> qfuture_refreshcameralist;

    //QTimer* frame_timer;
    QTimer* device_list_timer;

    ParamFile* parameters;

    QLabel* frame_counter;
    QLabel* fps_counter;
    QLabel* match_fps_counter;
    QLabel* temp_label;
    QSpacerItem* status_bar_spacer;
    QWidget* status_widget;
    QVariantMap icon_options;
    QtAwesome* awesome;
    DisparityViewer* disparity_view;
    CameraDisplayWidget *left_view;
    CameraDisplayWidget *left_matcher_view;
    CameraDisplayWidget *right_view;
    CameraDisplayWidget *object_detection_display;

    std::vector<QPushButton> deviceListButtons;

    bool cameras_connected = false;
    float measured_fps = 0;
    float measured_match_fps = 0;
    float current_fps = 0;
    int current_binning = 0;
    bool using_gige = false;
    bool first_cloud = true;
    bool first_image = true;

    bool about_dialog_used = false;
    bool calibration_dialog_used = false;
    bool calibration_from_images_dialog_used = false;

    bool autoZ = false;

    AbstractStereoCamera* stereo_cam;
    StereoCalibrate* calibrator;

    QList<MatcherWidget*> matcher_list;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    AboutDialog* about_dialog;
    CalibrationDialog* calibration_dialog;
    CalibrateFromImagesDialog* calibration_images_dialog;
    LoadStereoImagePairDialog* load_stereo_image_pair_dialog;

    QString calibration_directory = "";
    QString save_directory = "";

    QVTKWidget* vtk_widget;

    DShowLib::Grabber* tisgrabber;

    Pylon::CTlFactory* pylonTlFactory;

    DetectorOpenCV* object_detector;
    cvSharedMemory* sharedMemoryInst;
    bool detection_enabled = false;
    bool shared_memory_enabled = false;
    bool detecting = false;
    cv::Mat image_detection;
    cv::Mat image_stream;
    cv::Mat disparity_stream;
    cv::Mat image_detection_rescale;
    QMap<QString, QColor> class_colour_map;
    QMap<QString, bool> class_visible_map;
    QMap<QString, bool> class_filled_map;
    int bounding_box_alpha = 50;

    QSettings *settings = nullptr;

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> current_camera_serial_info_list;
    std::vector<QSignalMapper*>* camera_button_signal_mapper_list;

    void setupMatchers();
    void statusBarInit();
    void resetStatusBar();
    void controlsInit();
    void pointCloudInit();
    void detectionInit();

    void stereoCameraInit(void);
    void stereoCameraInitConnections(void);
    void stereoCameraInitWindow(void);

    bool gigeWarning(int binning,int new_fps=-1);

public slots:
    void stereoCameraRelease(void);
    void updateDisplay(void);
    void updateFrameTime(qint64);
    void updateMatchTime(qint64);
    void updateFrameCount(qint64);
    void enableCapture(bool);
    void singleShotClicked(void);
    void saveSingle(void);
    void displaySaved(bool success);
    void statusMessageTimeout(void);
    void setMatcher(int matcher);
    void updatePointTexture(int index); // 0 = image, 1 = depth

#ifdef WITH_FERVOR
    void downloadUpdateComplete();
    void checkUpdates();
#endif

    void enableAutoZ(bool);

    void disableWindow();
    void enableWindow();

    void error(int error);

    void disableCameraActiveSettings(){toggleCameraActiveSettings(false);}
    void enableCameraActiveSettings(){toggleCameraActiveSettings(true);}
    void toggleCameraActiveSettings(bool enable);

    void disableCameraPassiveSettings(){toggleCameraPassiveSettings(false);}
    void enableCameraPassiveSettings(){toggleCameraPassiveSettings(true);}
    void toggleCameraPassiveSettings(bool enable);

    int openCamera(AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info);
    void cameraDeviceSelected(int index);
    void refreshCameraList(void);
    void refreshCameraListThreaded(void);
    void refreshCameraListGUI(void);
    void startDeviceListTimer(void);
    void stopDeviceListTimer(void);
    void enableRectify(bool enable);
    void enableAutoExpose(bool);
    void enableAutoGain(bool);
    void enableBinning(bool enable);
    void setBinning(int binning);
    void enableTrigger(bool enable);
    void setFPS(int fps);
    void setPacketSize(int packetSize);
    void hideCameraSettings(bool hide);

    void videoStreamLoad(void);
    void stereoImageLoad(void);

    void setSaveDirectory(QString dir = "");
    void setCalibrationFolder(QString dir = "");

    void startAutoCalibration(void);
    void runAutoCalibration(void);

    void startCalibrationFromImages(void);
    void runCalibrationFromImages(void);

    void doneCalibration(StereoCalibrate::CalibrationStatus);

    void enableVideoCapture(bool enable);
    void startVideoCapture(void){ enableVideoCapture(true); }
    void stopVideoCapture(void){ enableVideoCapture(true); }

    void updateCloud(void);
    void enable3DViz(int);
    void resetPointCloudView(void);
    void autoUpdatePointCloudBounds(void);

    void pointCloudSaveStatus(QString);

    void updateDetection(void);
    void enableSharedMemory(bool);
    void updateSharedMemory(void);
    void enableDetection(bool);
    void configureDetection(void);
    void drawBoundingBoxes(cv::Mat image, std::vector<BoundingBox> bboxes, double scale_x, double scale_y);
    void setClassColour(QString class_name, QColor class_colour);
    void setClassVisible(QString class_name, bool visible);
    void setClassFilled(QString class_name, bool fill);
    void updateClassColour(void);
    void updateClassFilled(bool checked);
    void updateClassVisible(bool checked);
    void onClassListClicked(void);
    void updateBoundingBoxAlpha(int fill_alpha);
    void updateDetectionThreshold(int value);
    void updateNMSThreshold(int value);

    void openHelp();

    void openAbout();

protected:
    void closeEvent(QCloseEvent *event);
private slots:
    void on_btnShowCameraSettings_clicked();
    void on_btnHideCameraSettings_clicked();

    void enableMatching(bool enable);

signals:
    void cameraListUpdated(void);
};

#endif  // SVTK_WINDOW_H
