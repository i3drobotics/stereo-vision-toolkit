/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define _USE_MATH_DEFINES

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
#ifdef BUILD_PRO
#include "matcherwidgetjrsgm.h"
#endif
#include "stereocameradeimos.h"
#include "stereocamerafromvideo.h"
#include "stereocameraopencv.h"
#include "stereocameratis.h"
#include "stereocamerabasler.h"

#include <disparityviewer.h>
#include "calibratefromimagesdialog.h"
#include "calibrationdialog.h"
#include "stereocalibrate.h"
#include "cameradisplaywidget.h"

#include "paramfile.h"

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

    QPixmap pmap_left;
    QPixmap pmap_right;
    QPixmap pmap_disparity;
    QTimer* status_bar_timer;

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
    void videoStreamLoad(void);

    void setSaveDirectory(QString dir = "");
    void setCalibrationFolder(QString dir = "");

    void startCalibration(void);
    void doneCalibration(bool);

    void startCalibrationFromImages(void);
    void runCalibrationFromImages(void);

    void startVideoCapture(void);
    void stopVideoCapture(void);

    void updateCloud(void);
    void enable3DViz(int);
    void resetPointCloudView(void);

    void pointCloudSaveStatus(QString);

    void openHelp();

protected:
    void closeEvent(QCloseEvent *event);
};

#endif  // MAINWINDOW_H
