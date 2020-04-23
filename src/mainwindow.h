/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#define _USE_MATH_DEFINES

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

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

#include "matcherwidgetopencvblock.h"
#include "matcherwidgetopencvsgbm.h"
#ifdef WITH_I3DRSGM
    #include "matcherwidgeti3drsgm.h"
#endif
#include "stereocameradeimos.h"
#include "stereocamerafromvideo.h"
#include "stereocameraopencv.h"
#include "stereocameratis.h"
#include "stereocamerabasler.h"
#include "stereocamerabasler2.h"
#include "qdevicebutton.h"
#include "qdevicedialog.h"

#include <disparityviewer.h>
#include "calibratefromimagesdialog.h"
#include "calibrationdialog.h"
#include "stereocalibrate.h"
#include "cameradisplaywidget.h"

#include "paramfile.h"
#ifdef WITH_FERVOR
#include "fvupdater.h"
#endif

//!  Main Window
/*!
  QT Main Window of application
*/

using namespace std;
using namespace cv;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow* ui;

    bool updatingDisplay = false;
    bool showingSettings = false;

    int CAMERA_CONNECTION_SUCCESS_EXIT_CODE = 0;
    int CAMERA_CONNECTION_FAILED_EXIT_CODE = -1;
    int CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE = -2;
    int CAMERA_CONNECTION_CANCEL_EXIT_CODE = -3;

    AbstractStereoCamera::stereoCameraSettings default_basler_init_settings;
    AbstractStereoCamera::stereoCameraSettings default_tis_init_settings;
    AbstractStereoCamera::stereoCameraSettings default_deimos_init_settings;
    AbstractStereoCamera::stereoCameraSettings default_video_init_settings;
    AbstractStereoCamera::stereoCameraSettings current_camera_settings;

    QPixmap pmap_left;
    QPixmap pmap_right;
    QPixmap pmap_disparity;
    QTimer* status_bar_timer;

    QTimer* frame_timer;

    ParamFile* parameters;

    QLabel* frame_counter;
    QLabel* fps_counter;
    QLabel* temp_label;
    QSpacerItem* status_bar_spacer;
    QWidget* status_widget;
    QVariantMap icon_options;
    QtAwesome* awesome;
    DisparityViewer* disparity_view;
    CameraDisplayWidget *left_view;
    CameraDisplayWidget *right_view;

    bool cameras_connected = false;
    int measured_fps = 0;
    int current_fps = 0;
    int current_binning = 0;
    bool using_gige = false;

    bool calibration_dialog_used = false;
    bool calibration_from_images_dialog_used = false;

    AbstractStereoCamera* stereo_cam;
    StereoCalibrate* calibrator;

    QList<MatcherWidget*> matcher_list;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    CalibrationDialog* calibration_dialog;
    CalibrateFromImagesDialog* calibration_images_dialog;

    QString calibration_directory = "";
    QString save_directory = "";

    QVTKWidget* vtk_widget;

    void setupMatchers();
    void statusBarInit();
    void controlsInit();
    void pointCloudInit();

    void stereoCameraInit(void);
    void stereoCameraRelease(void);
    void stereoCameraInitConnections(void);
    void stereoCameraInitWindow(void);

    bool gigeWarning(int binning,int new_fps=-1);

public slots:
    void updateDisplay(void);
    void updateFPS(qint64);
    void updateTemperature(double temperature);
    void updateFrameCount(qint64);
    void toggleAcquire(void);
    void singleShotClicked(void);
    void saveSingle(void);
    void displaySaved(QString fname);
    void statusMessageTimeout(void);
    void setMatcher(int matcher);

    void disableWindow();
    void enableWindow();

    int stereoCameraLoad(void);
    void autoloadCameraTriggered();
    void toggleAutoExpose(bool);
    void toggleAutoGain(bool);
    void toggleEnableBinning(bool enable);
    void changeBinning(int binning);
    void toggleFPS(bool enable);
    void changeFPS(int fps);
    void changePacketSize();

    void videoStreamLoad(void);

    void setSaveDirectory(QString dir = "");
    void setCalibrationFolder(QString dir = "");

    void startAutoCalibration(void);
    void runAutoCalibration(void);

    void startCalibrationFromImages(void);
    void runCalibrationFromImages(void);

    void doneCalibration(bool);

    void startVideoCapture(void);
    void stopVideoCapture(void);

    void updateCloud(void);
    void enable3DViz(int);
    void resetPointCloudView(void);

    void pointCloudSaveStatus(QString);

    void openHelp();

protected:
    void closeEvent(QCloseEvent *event);
private slots:
    void on_btnCameraSettings_clicked();
};

#endif  // MAINWINDOW_H
