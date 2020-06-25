/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    QCoreApplication::setApplicationName("Stereo Vision Toolkit");
    setWindowTitle(QCoreApplication::applicationName());

    ui->imageViewTab->raise();
    ui->tabWidget->lower();

    ui->tabWidget->setCurrentIndex(0);

    cameras_connected = false;
    calibration_dialog_used = false;
    calibration_from_images_dialog_used = false;

    cam_thread = new QThread;
    device_list_timer = new QTimer(this);

    // define default settings for cameras
    // if the camera does not use the setting then should be set to -1
    // TODO replace this by reading current value from camera
    default_basler_init_settings.exposure = 5;
    default_basler_init_settings.gain = 0;
    default_basler_init_settings.fps = 30;
    default_basler_init_settings.binning = -1;
    default_basler_init_settings.trigger = true;
    default_basler_init_settings.hdr = -1;
    default_basler_init_settings.autoExpose = false;
    default_basler_init_settings.autoGain = false;
    default_basler_init_settings.isGige = -1;
    default_basler_init_settings.packetDelay = -1;
    default_basler_init_settings.packetSize = -1; //TODO fix packet size problems

    default_vimba_init_settings.exposure = -1;
    default_vimba_init_settings.gain = -1;
    default_vimba_init_settings.fps = -1;
    default_vimba_init_settings.binning = -1;
    default_vimba_init_settings.trigger = -1;
    default_vimba_init_settings.hdr = -1;
    default_vimba_init_settings.autoExpose = -1;
    default_vimba_init_settings.autoGain = -1;
    default_vimba_init_settings.isGige = -1;

    default_tis_init_settings.exposure = 5;
    default_tis_init_settings.gain = 0;
    default_tis_init_settings.fps = 10;
    default_tis_init_settings.binning = -1;
    default_tis_init_settings.trigger = false;
    default_tis_init_settings.hdr = -1;
    default_tis_init_settings.autoExpose = false;
    default_tis_init_settings.autoGain = false;
    default_tis_init_settings.isGige = -1;
    default_tis_init_settings.packetDelay = -1;
    default_tis_init_settings.packetSize = -1;

    default_deimos_init_settings.exposure = 5;
    default_deimos_init_settings.gain = -1;
    default_deimos_init_settings.fps = 60;
    default_deimos_init_settings.binning = -1;
    default_deimos_init_settings.trigger = -1;
    default_deimos_init_settings.hdr = false;
    default_deimos_init_settings.autoExpose = false;
    default_deimos_init_settings.autoGain = -1;
    default_deimos_init_settings.isGige = -1;
    default_deimos_init_settings.packetDelay = -1;
    default_deimos_init_settings.packetSize = -1;

    default_usb_init_settings.exposure = -1;
    default_usb_init_settings.gain = -1;
    default_usb_init_settings.fps = 10;
    default_usb_init_settings.binning = -1;
    default_usb_init_settings.trigger = -1;
    default_usb_init_settings.hdr = -1;
    default_usb_init_settings.autoExpose = -1;
    default_usb_init_settings.autoGain = -1;
    default_usb_init_settings.isGige = -1;
    default_usb_init_settings.packetDelay = -1;
    default_usb_init_settings.packetSize = -1;

    default_video_init_settings.exposure = -1;
    default_video_init_settings.gain = -1;
    default_video_init_settings.fps = 60;
    default_video_init_settings.binning = -1;
    default_video_init_settings.trigger = -1;
    default_video_init_settings.hdr = -1;
    default_video_init_settings.autoExpose = -1;
    default_video_init_settings.autoGain = -1;
    default_video_init_settings.isGige = -1;
    default_video_init_settings.packetDelay = -1;
    default_video_init_settings.packetSize = -1;

    //frame_timer = new QTimer(this);

    /* Calibration */
    connect(ui->actionCalibration_wizard, SIGNAL(triggered(bool)), this,
            SLOT(startAutoCalibration()));
    connect(ui->actionCalibrate_from_images, SIGNAL(triggered(bool)), this,
            SLOT(startCalibrationFromImages()));
    connect(ui->actionDocumentation, SIGNAL(triggered(bool)), this,
            SLOT(openHelp()));
    connect(ui->actionExit, SIGNAL(triggered(bool)), QApplication::instance(),
            SLOT(quit()));
    connect(this, SIGNAL(cameraListUpdated(void)), this, SLOT(refreshCameraListGUI(void)));

    awesome = new QtAwesome(qApp);
    awesome->initFontAwesome();

    parameters = new ParamFile();

    save_directory = parameters->get_string("saveDir");
    calibration_directory = parameters->get_string("calDir");

    left_view = new CameraDisplayWidget(this);
    left_matcher_view = new CameraDisplayWidget(this);
    right_view = new CameraDisplayWidget(this);

    ui->stereoViewLayout->addWidget(left_view);
    ui->stereoViewLayout->addWidget(right_view);

    ui->stereoMatcherLayout->insertWidget(0,left_matcher_view);

    statusBarInit();

    //disable tabs untill camera is connected to prevent crashes
    disableWindow();
    controlsInit();
    pointCloudInit();
    setupMatchers();

    hideCameraSettings(false);

#ifdef WITH_FERVOR
    // Set the Fervor appcast url
    FvUpdater::sharedUpdater()->SetFeedURL("https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/master/Appcast.xml");

    // Print current version to debug
    qDebug() << FV_APP_NAME << FV_APP_VERSION;

    // Check for updates silently -- this will not block the initialization of
    // your application, just start a HTTP request and return immediately.
    FvUpdater::sharedUpdater()->CheckForUpdatesSilent();

#endif
    startDeviceListTimer();
}

void MainWindow::disableWindow(){
    ui->toggleSwapLeftRight->setDisabled(true);
    ui->toggleCalibrationDownsample->setDisabled(true);
    ui->spinBoxDownsample->setDisabled(true);
    ui->tabWidget->setDisabled(true);
    ui->tabCameraSettings->setDisabled(true);
    ui->tabApplicationSettings->setDisabled(true);
    ui->matcherSelectBox->setDisabled(true);
    ui->enableStereo->setDisabled(true);

    ui->captureButton->setDisabled(true);
    ui->singleShotButton->setDisabled(true);
    ui->saveButton->setDisabled(true);
    ui->toggleVideoButton->setDisabled(true);
    ui->toggleRectifyCheckBox->setDisabled(true);
    ui->actionCalibration_wizard->setDisabled(true);
}

void MainWindow::enableWindow(){

    ui->toggleSwapLeftRight->setEnabled(true);
    ui->toggleCalibrationDownsample->setEnabled(true);
    ui->spinBoxDownsample->setEnabled(true);
    ui->tabWidget->setEnabled(true);
    ui->tabCameraSettings->setEnabled(true);
    ui->tabApplicationSettings->setEnabled(true);
    ui->matcherSelectBox->setEnabled(true);

    ui->captureButton->setEnabled(true);
    ui->singleShotButton->setEnabled(true);
    //ui->saveButton->setEnabled(false);
    ui->toggleVideoButton->setEnabled(true);
    ui->actionCalibration_wizard->setEnabled(true);
}

void MainWindow::pointCloudSaveStatus(QString msg){
    qDebug() << msg;
    QMessageBox::warning(this,"Stereo Vision Toolkit",msg);
}

void MainWindow::resetStatusBar(void) {
    fps_counter->setText("FPS: 0");
    frame_counter->setText("Frame count: 0");
}

void MainWindow::statusBarInit(void) {
    status_widget = new QWidget;
    ui->statusBar->addPermanentWidget(status_widget);

    fps_counter = new QLabel(this);
    temp_label = new QLabel(this);
    frame_counter = new QLabel(this);
    status_bar_spacer = new QSpacerItem(20, 1);

    QHBoxLayout* _hlayout = new QHBoxLayout();
    _hlayout->addWidget(frame_counter);
    _hlayout->addSpacerItem(status_bar_spacer);
    _hlayout->addWidget(fps_counter);
    _hlayout->addWidget(temp_label);

    status_widget->setLayout(_hlayout);

    status_bar_timer = new QTimer;

    connect(status_bar_timer, SIGNAL(timeout()), this,
            SLOT(statusMessageTimeout()));

    ui->saveDirLabel->setText(save_directory);
}

void MainWindow::controlsInit(void) {

    ui->widgetSideSettings->setVisible(true);

    connect(ui->captureButton, SIGNAL(clicked(bool)), this, SLOT(enableCapture(bool)));
    connect(ui->singleShotButton, SIGNAL(clicked()), this, SLOT(singleShotClicked()));
    connect(ui->setSaveDirButton, SIGNAL(clicked()), this, SLOT(setSaveDirectory()));
    connect(ui->toggleVideoButton, SIGNAL(clicked(bool)), this, SLOT(enableVideoCapture(bool)));
    connect(ui->actionLoad_calibration, SIGNAL(triggered(bool)),this, SLOT(setCalibrationFolder()));
    connect(ui->autoExposeCheck, SIGNAL(clicked(bool)), ui->exposureSpinBox, SLOT(setDisabled(bool)));
    connect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), ui->gainSpinBox, SLOT(setDisabled(bool)));
    connect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), ui->fpsSpinBox, SLOT(setDisabled(bool)));

    connect(ui->btnRefreshCameras, SIGNAL(clicked(bool)), this, SLOT(refreshCameraListThreaded()));

    icon_options.insert("color", QColor(255, 255, 255));
    ui->captureButton->setIcon(awesome->icon(fa::play, icon_options));
    ui->saveButton->setIcon(awesome->icon(fa::save, icon_options));
    ui->singleShotButton->setIcon(awesome->icon(fa::camera, icon_options));
    ui->enableStereo->setIcon(awesome->icon(fa::cubes, icon_options));
    ui->toggleVideoButton->setIcon(awesome->icon(fa::videocamera, icon_options));

    connect(ui->btnLoadVideo, SIGNAL(clicked(bool)), this,
            SLOT(videoStreamLoad()));

    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this,
            SLOT(enable3DViz(int)));

    disparity_view = new DisparityViewer();
    disparity_view->setViewer(ui->disparityViewLabel);
    connect(disparity_view, SIGNAL(newDisparity(QPixmap)), ui->disparityViewLabel,
            SLOT(setPixmap(QPixmap)));
    ui->disparityViewSettingsLayout->addWidget(disparity_view);

    connect(ui->reset3DViewButton, SIGNAL(clicked(bool)), this, SLOT(resetPointCloudView()));
}

void MainWindow::enable3DViz(int tab) {
    if (!stereo_cam) return;

    if (tab == 2) {
        stereo_cam->enableReproject(true);
    } else {
        stereo_cam->enableReproject(false);
    }
}

void MainWindow::pointCloudInit() {
    QProgressDialog progressPCI("Initialising display...", "", 0, 100, this);
    progressPCI.setWindowTitle("SVT");
    progressPCI.setWindowModality(Qt::WindowModal);
    progressPCI.setCancelButton(nullptr);
    progressPCI.setMinimumDuration(0);
    progressPCI.setValue(10);
    QCoreApplication::processEvents();

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtk_widget = new QVTKWidget();

    ui->visualiserTab->layout()->addWidget(vtk_widget);
    vtk_widget->setSizePolicy(QSizePolicy::MinimumExpanding,
                              QSizePolicy::MinimumExpanding);

    progressPCI.setValue(20);
    QCoreApplication::processEvents();

    vtk_widget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtk_widget->GetInteractor(),
                            vtk_widget->GetRenderWindow());

    progressPCI.setValue(40);
    QCoreApplication::processEvents();

    vtk_widget->update();

    progressPCI.setValue(60);
    QCoreApplication::processEvents();

    resetPointCloudView();

    progressPCI.setValue(80);
    QCoreApplication::processEvents();

    vtk_widget->update();

    progressPCI.setLabelText("Display setup complete");
    progressPCI.setValue(100);
    progressPCI.close();
    QCoreApplication::processEvents();
}

void MainWindow::resetPointCloudView(){
    double min_depth = disparity_view->getMinDepth();
    double max_depth = disparity_view->getMaxDepth();

    if (min_depth == -1){
        min_depth = 0.0;
    }
    if (max_depth == -1){
        max_depth = 5.0;
    }

    viewer->resetCamera();

    ui->minZSpinBox->setValue(min_depth);
    ui->maxZSpinBox->setValue(max_depth);

    vtk_widget->update();
}

void MainWindow::stereoCameraInitWindow(void){
    double default_exposure = current_camera_settings.exposure;
    int default_gain = current_camera_settings.gain;
    int default_fps = current_camera_settings.fps;
    int default_binning = current_camera_settings.binning;
    int default_iTrigger = current_camera_settings.trigger;
    int default_iHdr = current_camera_settings.hdr;
    int default_iAutoExpose = current_camera_settings.autoExpose;
    int default_iAutoGain = current_camera_settings.autoGain;
    int default_packetDelay = current_camera_settings.packetDelay;
    int default_packetSize = current_camera_settings.packetSize;
    int default_iIsGige = current_camera_settings.isGige;
    bool default_trigger, default_hdr, default_autoExpose, default_autoGain;
    if (default_iIsGige == 1){
        using_gige = true;
    } else {
        using_gige = false;
    }

    if (default_exposure == -1 && default_iAutoExpose == -1){
        ui->lblExposure->setVisible(false);
    } else {
        ui->lblExposure->setVisible(true);
    }
    if (default_gain == -1 && default_iAutoGain == -1){
        ui->lblGain->setVisible(false);
    } else {
        ui->lblGain->setVisible(true);
    }
    if (default_fps == -1){
        ui->lblFPS->setVisible(false);
    } else {
        ui->lblFPS->setVisible(true);
    }
    if (default_iHdr == -1){
        ui->lblHDR->setVisible(false);
    } else {
        ui->lblHDR->setVisible(true);
    }
    if (default_binning == -1){
        ui->lblBinning->setVisible(false);
    } else {
        ui->lblBinning->setVisible(true);
    }
    if (default_packetDelay == -1){
        ui->lblPacketDelay->setVisible(false);
    } else {
        ui->lblPacketDelay->setVisible(true);
    }
    if (default_packetSize == -1){
        ui->lblPacketSize->setVisible(false);
    } else {
        ui->lblPacketSize->setVisible(true);
    }

    if (default_exposure != -1){
        ui->exposureSpinBox->setEnabled(true);
        ui->exposureSpinBox->setVisible(true);
    } else {
        default_exposure = 5;
        ui->exposureSpinBox->setVisible(false);
    }
    if (default_gain != -1){
        ui->gainSpinBox->setEnabled(true);
        ui->gainSpinBox->setVisible(true);
    } else {
        default_gain = 0;
        ui->gainSpinBox->setVisible(false);
    }
    if (default_fps != -1){
        ui->fpsSpinBox->setEnabled(true);
        ui->fpsSpinBox->setVisible(true);
    } else {
        default_fps = 5;
        ui->fpsSpinBox->setVisible(false);
    }
    if (default_binning != -1){
        ui->binningSpinBox->setEnabled(true);
        ui->binningSpinBox->setVisible(true);
    } else {
        //default_binning = 1;
        ui->binningSpinBox->setVisible(false);
    }
    if (default_packetDelay != -1){
        ui->packetDelaySpinBox->setEnabled(true);
        ui->packetDelaySpinBox->setVisible(true);
    } else {
        //default_packetDelay = 0;
        ui->packetDelaySpinBox->setVisible(false);
    }
    if (default_packetSize != -1){
        ui->packetSizeSpinBox->setEnabled(true);
        ui->packetSizeSpinBox->setVisible(true);
    } else {
        //default_packetSize = 3000;
        ui->packetSizeSpinBox->setVisible(false);
    }
    if (default_iTrigger != -1){
        if (default_iTrigger == 1){
            default_trigger = true;
        } else {
            default_trigger = false;
        }
        ui->enabledTriggeredCheckbox->setEnabled(true);
        ui->enabledTriggeredCheckbox->setChecked(default_trigger);
        ui->enabledTriggeredCheckbox->setVisible(true);
        AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
        if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
            ui->fpsSpinBox->setEnabled(true);
        } else {
            if (default_trigger){
                ui->fpsSpinBox->setEnabled(false);
            } else {
                ui->fpsSpinBox->setEnabled(true);
            }
        }
    } else {
        default_trigger = false;
        ui->enabledTriggeredCheckbox->setVisible(false);
    }
    if (default_iHdr != -1){
        if (default_iHdr == 1){
            default_hdr = true;
        } else {
            default_hdr = false;
        }
        ui->enableHDRCheckbox->setEnabled(true);
        ui->enableHDRCheckbox->setChecked(default_hdr);
        ui->enableHDRCheckbox->setVisible(true);
    } else {
        default_hdr = false;
        ui->enableHDRCheckbox->setVisible(false);
    }
    if (default_iAutoExpose != -1){
        if (default_iAutoExpose == 1){
            default_autoExpose = true;
        } else {
            default_autoExpose = false;
        }
        ui->autoExposeCheck->setEnabled(true);
        ui->autoExposeCheck->setChecked(default_autoExpose);
        ui->autoExposeCheck->setVisible(true);
    } else {
        default_autoExpose = false;
        ui->autoExposeCheck->setVisible(false);
    }
    if (default_iAutoGain != -1){
        if (default_iAutoGain == 1){
            default_autoGain = true;
        } else {
            default_autoGain = false;
        }
        ui->autoGainCheckBox->setEnabled(true);
        ui->autoGainCheckBox->setChecked(default_autoGain);
        ui->autoGainCheckBox->setVisible(true);
    } else {
        default_autoGain = false;
        ui->autoGainCheckBox->setVisible(false);
    }

    // set window to default values
    // set fps spin box limits
    int max_fps = 60;
    int min_fps = 0;
    int step_fps = 1;

    int max_gain = 360;
    int min_gain = 0;
    int step_gain = 1;

    int max_binning = 4;
    int min_binning = 1;
    int step_binning = 1;

    int max_packetDelay = 10000;
    int min_packetDelay = -1;
    int step_packetDelay = 1;

    int max_packetSize = 16404;
    int min_packetSize = -1;
    int step_packetSize = 1;

    AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
    if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_DEIMOS){
        min_fps = 30;
        step_fps = 30;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE){
        max_binning = 4;
        min_binning = 1;
        step_binning = 1;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
        max_binning = 4;
        min_binning = 1;
        step_binning = 1;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS){
        max_gain = 48;
        min_gain = 0;
        step_gain = 1;
    } else  if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
        //VIMBA only settings
    }

    current_fps = default_fps;
    current_binning = default_binning;

    // set fps
    ui->fpsSpinBox->blockSignals(true);
    ui->fpsSpinBox->setRange(min_fps,max_fps);
    ui->fpsSpinBox->setSingleStep(step_fps);
    ui->fpsSpinBox->setValue(default_fps);
    ui->fpsSpinBox->blockSignals(false);

    // set gain
    ui->gainSpinBox->blockSignals(true);
    ui->gainSpinBox->setRange(min_gain,max_gain);
    ui->gainSpinBox->setSingleStep(step_gain);
    ui->gainSpinBox->setValue(default_gain);
    ui->gainSpinBox->blockSignals(false);

    // set binning
    ui->binningSpinBox->blockSignals(true);
    ui->binningSpinBox->setRange(min_binning,max_binning);
    ui->binningSpinBox->setSingleStep(step_binning);
    ui->binningSpinBox->setValue(default_binning);
    ui->binningSpinBox->blockSignals(false);

    // set exposure
    ui->exposureSpinBox->blockSignals(true);
    //ui->exposureSpinBox->setRange(min_exposure,max_exposure); //set by ui
    //ui->exposureSpinBox->setSingleStep(step_exposure); //set by ui
    ui->exposureSpinBox->setValue(default_exposure);
    ui->exposureSpinBox->blockSignals(false);

    // set packet delay
    ui->packetDelaySpinBox->blockSignals(true);
    ui->packetDelaySpinBox->setRange(min_packetDelay,max_packetDelay);
    ui->packetDelaySpinBox->setSingleStep(step_packetDelay);
    ui->packetDelaySpinBox->setValue(default_packetDelay);
    ui->packetDelaySpinBox->blockSignals(false);

    // set packet delay
    ui->packetSizeSpinBox->blockSignals(true);
    ui->packetSizeSpinBox->setRange(min_packetSize,max_packetSize);
    ui->packetSizeSpinBox->setSingleStep(step_packetSize);
    ui->packetSizeSpinBox->setValue(default_packetSize);
    ui->packetSizeSpinBox->blockSignals(false);
}

void MainWindow::stereoCameraInitConnections(void) {

    stereoCameraInitWindow();

    connect(ui->exposureSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
            SLOT(setExposure(double)));
    connect(ui->gainSpinBox, SIGNAL(valueChanged(int)), stereo_cam,
            SLOT(setGain(int)));
    connect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), this,
            SLOT(setFPS(int)));
    connect(ui->binningSpinBox, SIGNAL(valueChanged(int)), this,
            SLOT(setBinning(int)));
    connect(ui->packetSizeSpinBox, SIGNAL(valueChanged(int)), this,
            SLOT(setPacketSize(int)));
    connect(ui->packetDelaySpinBox, SIGNAL(valueChanged(int)), stereo_cam,
            SLOT(setPacketDelay(int)));

    connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDisplay()));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), left_view, SLOT(setSize(int, int, int)));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), left_matcher_view, SLOT(setSize(int, int, int)));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), right_view, SLOT(setSize(int, int, int)));

    connect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), this,
            SLOT(enableFPS(bool)));

    connect(stereo_cam, SIGNAL(frametime(qint64)), this, SLOT(updateFrameTime(qint64)));
    connect(stereo_cam, SIGNAL(framecount(qint64)), this, SLOT(updateFrameCount(qint64)));
    connect(ui->saveButton, SIGNAL(clicked()), stereo_cam, SLOT(saveImageTimestamped()));
    connect(stereo_cam, SIGNAL(first_image_ready(bool)), ui->saveButton, SLOT(setEnabled(bool)));
    connect(disparity_view, SIGNAL(disparitySaveCheckChanged(bool)), stereo_cam, SLOT(enableSaveDisparity(bool)));
    connect(stereo_cam, SIGNAL(savedImage(QString)), this,
            SLOT(displaySaved(QString)));
    connect(ui->enableStereo, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableMatching(bool)));
    connect(ui->toggleRectifyCheckBox, SIGNAL(clicked(bool)), this,
            SLOT(enableRectify(bool)));
    connect(ui->toggleSwapLeftRight, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableSwapLeftRight(bool)));
    connect(ui->toggleCalibrationDownsample, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableDownsampleCalibration(bool)));
    connect(ui->spinBoxDownsample, SIGNAL(valueChanged(int)), stereo_cam,
            SLOT(setDownsampleFactor(int)));
    connect(stereo_cam, SIGNAL(matched()), disparity_view,
            SLOT(updateDisparityAsync(void)));
    connect(ui->autoExposeCheck, SIGNAL(clicked(bool)), this, SLOT(enableAutoExpose(bool)));
    connect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), this, SLOT(enableAutoGain(bool)));
    connect(ui->enableHDRCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableHDR(bool)));

    connect(stereo_cam, SIGNAL(disconnected()), this, SLOT(stereoCameraRelease()));

    /* Point cloud */
    connect(stereo_cam, SIGNAL(reprojected()), this, SLOT(updateCloud()));
    connect(ui->minZSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
            SLOT(setVisualZmin(double)));
    connect(ui->maxZSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
            SLOT(setVisualZmax(double)));
    connect(ui->savePointCloudButton, SIGNAL(clicked()), stereo_cam, SLOT(savePointCloud()));
    connect(ui->dateInFilenameCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableDateInFilename(bool)));
    connect(stereo_cam, SIGNAL(pointCloudSaveStatus(QString)),this,SLOT(pointCloudSaveStatus(QString)));

    enableWindow();
    toggleCameraActiveSettings(true);
    toggleCameraPassiveSettings(true);
}

void MainWindow::stereoCameraRelease(void) {
    disableWindow();

    ui->exposureSpinBox->setDisabled(true);
    ui->fpsSpinBox->setDisabled(true);
    ui->gainSpinBox->setDisabled(true);
    ui->binningSpinBox->setDisabled(true);
    ui->packetDelaySpinBox->setDisabled(true);
    ui->packetSizeSpinBox->setDisabled(true);
    ui->autoExposeCheck->setChecked(false);
    ui->autoExposeCheck->setDisabled(true);
    ui->autoGainCheckBox->setChecked(false);
    ui->autoGainCheckBox->setDisabled(true);
    ui->enableHDRCheckbox->setChecked(false);
    ui->enableHDRCheckbox->setDisabled(true);
    ui->enabledTriggeredCheckbox->setChecked(false);
    ui->enabledTriggeredCheckbox->setDisabled(true);

    ui->enableStereo->setChecked(false);
    ui->captureButton->setChecked(false);
    ui->singleShotButton->setChecked(false);
    ui->toggleVideoButton->setChecked(false);

    ui->toggleRectifyCheckBox->setChecked(false);
    ui->toggleSwapLeftRight->setChecked(false);

    //TODO enable this when implimented
    ui->toggleCalibrationDownsample->setVisible(false);
    ui->toggleCalibrationDownsample->setChecked(false);

    ui->tabWidget->setCurrentIndex(0);
    ui->tabLayoutSettings->setCurrentIndex(0);

    if (cameras_connected) {
        cameras_connected = false;

        QProgressDialog progressClose("Ending camera capture...", "", 0, 100, this);
        progressClose.setWindowTitle("SVT");
        progressClose.setWindowModality(Qt::WindowModal);
        progressClose.setCancelButton(nullptr);
        progressClose.setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
        progressClose.setMinimumDuration(0);
        progressClose.setValue(10);

        enableCapture(false);
        enableRectify(false);
        stereo_cam->enableMatching(false);

        progressClose.setLabelText("Closing camera connections...");
        progressClose.setValue(30);

        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDisplay()));

        disconnect(ui->exposureSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
                   SLOT(setExposure(double)));
        disconnect(ui->gainSpinBox, SIGNAL(valueChanged(int)), stereo_cam,
                   SLOT(setGain(int)));
        disconnect(ui->packetDelaySpinBox, SIGNAL(valueChanged(int)), stereo_cam,
                SLOT(setPacketDelay(int)));

        disconnect(ui->binningSpinBox, SIGNAL(valueChanged(int)), this,
                   SLOT(setBinning(int)));
        disconnect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), this,
                   SLOT(setFPS(int)));
        disconnect(ui->packetSizeSpinBox, SIGNAL(valueChanged(int)), this,
                SLOT(setPacketSize()));

        disconnect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), this,
                   SLOT(enableFPS(bool)));

        // disconnect fps connection used in video
        disconnect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), stereo_cam, SLOT(setFPS(int)));

        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDisplay()));
        disconnect(stereo_cam, SIGNAL(matched()), disparity_view,
                   SLOT(updateDisparityAsync(void)));
        disconnect(stereo_cam, SIGNAL(frametime(qint64)), this, SLOT(updateFrameTime(qint64)));
        disconnect(stereo_cam, SIGNAL(framecount(qint64)), this, SLOT(updateFrameCount(qint64)));
        disconnect(ui->saveButton, SIGNAL(clicked()), stereo_cam,
                   SLOT(saveImageTimestamped()));
        disconnect(disparity_view, SIGNAL(disparitySaveCheckChanged(bool)), stereo_cam, SLOT(enableSaveDisparity(bool)));
        disconnect(stereo_cam, SIGNAL(savedImage(QString)), this,
                   SLOT(displaySaved(QString)));
        disconnect(ui->enableStereo, SIGNAL(clicked(bool)), stereo_cam,
                   SLOT(enableMatching(bool)));
        disconnect(ui->toggleRectifyCheckBox, SIGNAL(clicked(bool)), this,
                   SLOT(enableRectify(bool)));
        disconnect(ui->toggleSwapLeftRight, SIGNAL(clicked(bool)), stereo_cam,
                   SLOT(enableSwapLeftRight(bool)));
        disconnect(ui->toggleCalibrationDownsample, SIGNAL(clicked(bool)), stereo_cam,
                SLOT(enableDownsampleCalibration(bool)));
        disconnect(ui->spinBoxDownsample, SIGNAL(valueChanged(int)), stereo_cam,
                SLOT(setDownsampleFactor(int)));
        disconnect(ui->autoExposeCheck, SIGNAL(clicked(bool)), this, SLOT(enableAutoExpose(bool)));
        disconnect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), this, SLOT(enableAutoGain(bool)));
        disconnect(ui->enableHDRCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableHDR(bool)));

        disconnect(stereo_cam, SIGNAL(disconnected()), this, SLOT(stereoCameraRelease()));

        /* Point cloud */
        disconnect(stereo_cam, SIGNAL(reprojected()), this, SLOT(updateCloud()));
        disconnect(ui->minZSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
                   SLOT(setVisualZmin(double)));
        disconnect(ui->maxZSpinBox, SIGNAL(valueChanged(double)), stereo_cam,
                   SLOT(setVisualZmax(double)));
        disconnect(ui->savePointCloudButton, SIGNAL(clicked()), stereo_cam, SLOT(savePointCloud()));
        disconnect(ui->dateInFilenameCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableDateInFilename(bool)));
        disconnect(stereo_cam, SIGNAL(pointCloudSaveStatus(QString)),this,SLOT(pointCloudSaveStatus(QString)));

        //wait 1 second to make sure connections are closed
        QTime dieTime= QTime::currentTime().addSecs(1);
        while (QTime::currentTime() < dieTime){
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }

        progressClose.setLabelText("Waiting for acquisition to finish...");
        progressClose.setValue(50);

        qDebug() << "Waiting for acquisition to finish";
        while (stereo_cam->isCapturing());
        qDebug() << "Acquisition finished";

        progressClose.setLabelText("Disconnecting camera...");
        progressClose.setValue(80);

        stereo_cam->closeCamera();

        QCoreApplication::processEvents();

        //stereo_cam->finishThread();

        delete stereo_cam;

        qDebug() << "Camera disconnected";

        progressClose.setLabelText("Camera diconnected");
        progressClose.setValue(100);
        progressClose.close();
        QCoreApplication::processEvents();

        startDeviceListTimer();
    }
    resetStatusBar();
}

int MainWindow::openCamera(AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info){
    stereoCameraRelease();
    QProgressDialog progressConnect("Connecting to camera...", "", 0, 100, this);
    progressConnect.setWindowTitle("SVT");
    progressConnect.setWindowModality(Qt::WindowModal);
    progressConnect.setCancelButton(nullptr);
    progressConnect.setMinimumDuration(0);
    progressConnect.setValue(10);
    QCoreApplication::processEvents();
    int exit_code = CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE;

    cam_thread = new QThread;

    if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_DEIMOS){
        current_camera_settings = default_deimos_init_settings;
        StereoCameraDeimos* stereo_cam_deimos = new StereoCameraDeimos(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_deimos->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_deimos);
        qDebug() << "Connecting to Deimos system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
        current_camera_settings = default_basler_init_settings;
        if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE){
            current_camera_settings.isGige = 1;
        }
        StereoCameraBasler* stereo_cam_basler = new StereoCameraBasler(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_basler->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_basler);
        qDebug() << "Connecting to Phobos system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_USB){
        current_camera_settings = default_usb_init_settings;
        StereoCameraOpenCV* stereo_cam_cv = new StereoCameraOpenCV(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_cv->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_cv);
        qDebug() << "Connecting to USB system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS){
        current_camera_settings = default_tis_init_settings;
        StereoCameraTIS* stereo_cam_tis = new StereoCameraTIS(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_tis->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_tis);
        qDebug() << "Connecting to Phobos system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
#ifdef WITH_VIMBA
       current_camera_settings = default_vimba_init_settings;
       StereoCameraVimba * stereo_cam_vimba = new StereoCameraVimba(camera_serial_info,current_camera_settings);
       cameras_connected = stereo_cam_vimba->openCamera();
       stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_vimba);
       qDebug() << "Connecting to Titania system";
#endif
   }

    QCoreApplication::processEvents();

    if (cameras_connected){
        stereo_cam->assignThread(cam_thread);
        progressConnect.setLabelText("Setting up camera interface...");
        progressConnect.setValue(50);
        stereoCameraInit();
        progressConnect.setLabelText("Camera system connected");
        progressConnect.setValue(100);
        progressConnect.close();
        QCoreApplication::processEvents();
        exit_code = CAMERA_CONNECTION_SUCCESS_EXIT_CODE;
        enableCapture(true);
    } else {
        exit_code = CAMERA_CONNECTION_FAILED_EXIT_CODE; // failed to connect to camera
    }

    if (exit_code == CAMERA_CONNECTION_SUCCESS_EXIT_CODE){
        //re-enable tabs as camera confirmed as connected
        enableWindow();
        ui->toggleVideoButton->setEnabled(true);
    } else if (exit_code == CAMERA_CONNECTION_FAILED_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        std::string msgBoxMsg;
        AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
        if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE){
            // notice to user if using gige as can become locked if not closed correctly
            // suggest power cycling to unlock the device
            msgBoxMsg = "Failed to connect to cameras. Some features will not work as expected. GigE camera may be locked if closed incorrectly, try power cycling the camera system.";
        } else {
            msgBoxMsg = "Failed to connect to cameras. Some features will not work as expected.";
        }
        msgBox.setText(msgBoxMsg.c_str());
        msgBox.exec();
    } else if (exit_code == CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        /*
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        msgBox.setText("No recognised cameras connected. Some features may not work as expected.");
        msgBox.exec();
        */
        ui->statusBar->showMessage("No cameras found.");
    } else if (exit_code == CAMERA_CONNECTION_CANCEL_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        /*
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        msgBox.setText("No camera system selected. Some features may not work as expected.");
        msgBox.exec();
        */
        ui->statusBar->showMessage("No cameras found.");
    } else {
        // should never happen
        qDebug() << "Undefined exit code in 'stereoCameraLoad()'";
    }
    return exit_code;
}

void MainWindow::refreshCameraListThreaded(){
    qfuture_refreshcameralist = QtConcurrent::run(this, &MainWindow::refreshCameraList);
}

void MainWindow::refreshCameraList(){
    current_camera_serial_info_list = StereoCameraSupport::getStereoDeviceList();
    emit cameraListUpdated();
}

void MainWindow::refreshCameraListGUI(){
    //triggered by 'cameraListUpdated()' signal
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> camera_serial_info_list = current_camera_serial_info_list;
    camera_button_signal_mapper_list = new vector<QSignalMapper*>();


    QPixmap pixmapDeimos(":/mainwindow/images/deimos_square_50.png");
    QPixmap pixmapPhobos(":/mainwindow/images/phobos_square_50.png");
    QPixmap pixmapCamera(":/mainwindow/images/camera_square_50.png");

    //Clear layout list
    deviceListButtons.clear();
    QLayoutItem *item;
    while ((item = ui->gridLayoutCameraList->takeAt(0)) != 0) {
        if (item->layout()){
            QLayoutItem *item2;
            while ((item2 = item->layout()->takeAt(0)) != 0) {
                if (item2->widget()){
                    delete item2->widget();
                }
            }
            delete item->layout();
        } else if (item->widget()){
            delete item->widget();
        }
    }

    ui->gridLayoutCameraList->setColumnStretch(0,0);
    ui->gridLayoutCameraList->setColumnStretch(1,1);
    ui->gridLayoutCameraList->setColumnStretch(2,0);

    if (camera_serial_info_list.size() <= 0){
        ui->lblNoCameras->show();
    } else {
        ui->lblNoCameras->hide();
        int i = 0;
        for (std::vector<AbstractStereoCamera::StereoCameraSerialInfo>::iterator it = camera_serial_info_list.begin() ; it != camera_serial_info_list.end(); ++it){
            AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = *it;
            std::string camera_type, camera_serial;
            QPixmap camera_icon;
            if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_DEIMOS){
                camera_type = "Deimos";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapDeimos;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS){
                camera_type = "Phobos";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapPhobos;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
                camera_type = "Titania";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapPhobos;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_USB){
                camera_type = "Generic";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapCamera;
            }

            QHBoxLayout *hlayoutCameraType = new QHBoxLayout();

            QLabel* cameraIconLabel = new QLabel();
            cameraIconLabel->setPixmap(camera_icon);
            hlayoutCameraType->addWidget(cameraIconLabel);

            std::string cameraTypeBtnText = camera_type;
            QLabel* cameraTypeLabel = new QLabel(cameraTypeBtnText.c_str());
            hlayoutCameraType->addWidget(cameraTypeLabel);

            std::string cameraSerialBtnText = camera_serial;
            QLabel* cameraSerialLabel = new QLabel(cameraSerialBtnText.c_str());
            cameraSerialLabel->setAlignment(Qt::AlignCenter);

            QPushButton *btnSelectCamera = new QPushButton("Connect");
            btnSelectCamera->setStyleSheet("background-color: green");

            ui->gridLayoutCameraList->addLayout(hlayoutCameraType,i+1,0);
            ui->gridLayoutCameraList->addWidget(cameraSerialLabel,i+1,1);
            ui->gridLayoutCameraList->addWidget(btnSelectCamera,i+1,2);

            QSignalMapper * mapper = new QSignalMapper(this);
            QObject::connect(mapper,SIGNAL(mapped(int)),this,SLOT(cameraDeviceSelected(int)));

            QObject::connect(btnSelectCamera, SIGNAL(clicked()),mapper,SLOT(map()));
            mapper->setMapping(btnSelectCamera, i);
            camera_button_signal_mapper_list->push_back(mapper);

            i++;
        }
    }
}

void MainWindow::cameraDeviceSelected(int index){
    stopDeviceListTimer();
    unsigned long long button_index = index;
    if (button_index < current_camera_serial_info_list.size()){
        // disconnect of camera if already connected
        AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = current_camera_serial_info_list.at(button_index);
        unsigned long long i = 0;
        if (openCamera(camera_serial_info) == CAMERA_CONNECTION_SUCCESS_EXIT_CODE){
            // disable all other buttons if camera open is successful
            for (std::vector<AbstractStereoCamera::StereoCameraSerialInfo>::iterator it = current_camera_serial_info_list.begin() ; it != current_camera_serial_info_list.end(); ++it){
                AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = *it;
                // Get button widget from grid layout
                QWidget *wSelectCamera = ui->gridLayoutCameraList->itemAtPosition(i+1,2)->widget();
                QPushButton *btnSelectCamera = qobject_cast<QPushButton*>(wSelectCamera);
                // Disconnect signal mapper
                QSignalMapper * mapper = camera_button_signal_mapper_list->at(i);
                QObject::disconnect(mapper,SIGNAL(mapped(int)),this,SLOT(cameraDeviceSelected(int)));
                QObject::disconnect(btnSelectCamera, SIGNAL(clicked()),mapper,SLOT(map()));
                if (i != button_index){
                    // Remove from list camera that wasn't used
                    if (ui->gridLayoutCameraList->itemAtPosition(i+1,0)->widget()){
                        QWidget *w1 = ui->gridLayoutCameraList->itemAtPosition(i+1,0)->widget();
                        delete(w1);
                    } else if (ui->gridLayoutCameraList->itemAtPosition(i+1,0)->layout()){
                        QLayout *l1 = ui->gridLayoutCameraList->itemAtPosition(i+1,0)->layout();
                        QLayoutItem *item2;
                        while ((item2 = l1->takeAt(0)) != 0) {
                            if (item2->widget()){
                                delete item2->widget();
                            }
                        }
                        delete l1;
                    }
                    QWidget *w2 = ui->gridLayoutCameraList->itemAtPosition(i+1,1)->widget();
                    delete(w2);
                    delete(btnSelectCamera);
                } else {
                    btnSelectCamera->setStyleSheet("background-color: #e83131"); //red
                    btnSelectCamera->setText("Disconnect");

                    connect(btnSelectCamera, SIGNAL(clicked()), this, SLOT(stereoCameraRelease()));
                }
                i++;
            }
        } else {
            startDeviceListTimer();
        }
    } else {
        startDeviceListTimer();
    }
}

int MainWindow::stereoCameraLoad(void) {
    cameras_connected = false;
    int exit_code = -4;

    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> all_camera_serial_info = StereoCameraSupport::getStereoDeviceList();

    AbstractStereoCamera::StereoCameraSerialInfo chosen_camera_serial_info;
    bool camera_chosen = false;

    int total_systems_found = all_camera_serial_info.size();

    if (total_systems_found > 0){
        // more than 1 stereo system found
        // user picks which camera system to use
        QMessageBox msgBoxDevType;
        msgBoxDevType.setText(tr("Multiple devices found. What type of device are you using:"));

        QPixmap pixmapDeimos(":/mainwindow/images/deimos_square_100.png");
        QPixmap pixmapPhobos(":/mainwindow/images/phobos_square_100.png");
        QPixmap pixmapCamera(":/mainwindow/images/camera_square_100.png");

        QMessageBox msgBoxDevSelect;
        msgBoxDevSelect.setText(tr("Select which stereo system to use: "));
        std::vector<QAbstractButton*> pButtons;
        for (std::vector<AbstractStereoCamera::StereoCameraSerialInfo>::iterator it = all_camera_serial_info.begin() ; it != all_camera_serial_info.end(); ++it){
            AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = *it;
            QDeviceButton* pButtonDev = new QDeviceButton();
            if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_DEIMOS){
                std::string camera_str = "Deimos \n" + camera_serial_info.i3dr_serial;
                pButtonDev->setText(camera_str.c_str());
                pButtonDev->setPixmap(pixmapDeimos);
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS){
                std::string camera_str = "Phobos \n" + camera_serial_info.i3dr_serial;
                pButtonDev->setText(camera_str.c_str());
                pButtonDev->setPixmap(pixmapPhobos);
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
                std::string camera_str = "Titania \n" + camera_serial_info.i3dr_serial;
                pButtonDev->setText(camera_str.c_str());
                pButtonDev->setPixmap(pixmapCamera);
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_USB){
                std::string camera_str = "Generic \n" + camera_serial_info.i3dr_serial;
                pButtonDev->setText(camera_str.c_str());
                pButtonDev->setPixmap(pixmapCamera);
            }
            msgBoxDevSelect.addButton(pButtonDev,QMessageBox::YesRole);
            pButtons.push_back(pButtonDev);
        }
        QPushButton* pButtonCancel = msgBoxDevSelect.addButton(tr("Cancel"), QMessageBox::RejectRole);
        msgBoxDevSelect.exec();

        if (msgBoxDevSelect.clickedButton() == pButtonCancel){
            return CAMERA_CONNECTION_CANCEL_EXIT_CODE;
        } else {
            // get serial info camera chosen with button press
            for (std::vector<QAbstractButton*>::iterator it = pButtons.begin() ; it != pButtons.end(); ++it){
                int index = it - pButtons.begin();
                if (msgBoxDevSelect.clickedButton()==*it) {
                    chosen_camera_serial_info = all_camera_serial_info.at(index);
                    camera_chosen = true;
                    break;
                }
            }
        }
    } else {
        camera_chosen = false;
    }

    if (camera_chosen) {
        // connect to chosen camera
        exit_code = openCamera(chosen_camera_serial_info);
    } else {
        exit_code = CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE; // no camera chosen
    }
    return (exit_code);
}

void MainWindow::stereoCameraInit() {
    if (cameras_connected) {
        save_directory = parameters->get_string("saveDir");
        calibration_directory = parameters->get_string("calDir");

        stereoCameraInitConnections();
        setMatcher(ui->matcherSelectBox->currentIndex());

        if (save_directory != "") {
            stereo_cam->setSavelocation(save_directory);
            disparity_view->setSavelocation(save_directory);
        }

        if (calibration_directory != ""){
            setCalibrationFolder(calibration_directory);
        }

        double focal =  stereo_cam->fx;
        double baseline = stereo_cam->baseline;
        disparity_view->setCalibration(stereo_cam->Q,baseline,focal);

        left_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);
        left_matcher_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);
        right_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);

        ui->statusBar->showMessage("Freerunning.");
    }
}

void MainWindow::startDeviceListTimer() {
    // refresh device list every 5 seconds
    //TODO replace this with event driven system
    device_list_timer->stop();
    device_list_timer = new QTimer(this);
    refreshCameraListThreaded();
    device_list_timer->start(5000);
    QObject::connect(device_list_timer, SIGNAL(timeout()), this, SLOT(refreshCameraListThreaded()));
    ui->btnRefreshCameras->setEnabled(true);
}

void MainWindow::stopDeviceListTimer() {
    QObject::disconnect(device_list_timer, SIGNAL(timeout()), this, SLOT(refreshCameraListThreaded()));
    device_list_timer->stop();
    ui->btnRefreshCameras->setEnabled(false);
}

void MainWindow::autoloadCameraTriggered() {
    stereoCameraRelease();
    int stereo_camera_exit_code = stereoCameraLoad();
    if (stereo_camera_exit_code == CAMERA_CONNECTION_SUCCESS_EXIT_CODE){
        //re-enable tabs as camera confirmed as connected
        enableWindow();
        ui->toggleVideoButton->setEnabled(true);
    } else if (stereo_camera_exit_code == CAMERA_CONNECTION_FAILED_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        std::string msgBoxMsg;
        AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
        if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE){
            // notice to user if using gige as can become locked if not closed correctly
            // suggest power cycling to unlock the device
            msgBoxMsg = "Failed to connect to cameras. Some features will not work as expected. GigE camera may be locked if closed incorrectly, try power cycling the camera system.";
        } else {
            msgBoxMsg = "Failed to connect to cameras. Some features will not work as expected.";
        }
        msgBox.setText(msgBoxMsg.c_str());
        msgBox.exec();
    } else if (stereo_camera_exit_code == CAMERA_CONNECTION_NO_CAMERA_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        /*
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        msgBox.setText("No recognised cameras connected. Some features may not work as expected.");
        msgBox.exec();
        */
        ui->statusBar->showMessage("No cameras found.");
    } else if (stereo_camera_exit_code == CAMERA_CONNECTION_CANCEL_EXIT_CODE){
        //Display warning messagebox as program does not function correctly if no camera is connected
        /*
        QMessageBox msgBox;
        msgBox.setWindowTitle("Stereo Vision Toolkit");
        msgBox.setText("No camera system selected. Some features may not work as expected.");
        msgBox.exec();
        */
        ui->statusBar->showMessage("No cameras found.");
    } else {
        // should never happen
        qDebug() << "Undefined exit code in 'stereoCameraLoad()'";
    }
}

void MainWindow::videoStreamLoad(void) {
    QMessageBox msg;

    QString fname = QFileDialog::getOpenFileName(
                this, tr("Open Stereo Video"), "/home", tr("Videos (*.avi *.mp4)"));
    if (fname != "") {
        stereoCameraRelease();
        //stopDeviceListTimer();

        AbstractStereoCamera::StereoCameraSerialInfo scis;
        scis.filename = fname.toStdString();

        current_camera_settings = default_video_init_settings;

        cam_thread = new QThread;
        StereoCameraFromVideo* stereo_cam_video = new StereoCameraFromVideo(scis,current_camera_settings);
        cameras_connected = stereo_cam_video->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_video);
        if (cameras_connected){
            stereo_cam->assignThread(cam_thread);
            ui->frameCountSlider->setEnabled(true);
            connect(stereo_cam, SIGNAL(videoPosition(int)),ui->frameCountSlider, SLOT(setValue(int)));
            connect(ui->frameCountSlider, SIGNAL(sliderMoved(int)),stereo_cam, SLOT(setPosition(int)));
            connect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), stereo_cam, SLOT(setFPS(int)));
            stereoCameraInit();
            stereoCameraInitWindow();
            //ui->toggleVideoButton->setDisabled(true);

            // Ask user if to load rectified frames
            QMessageBox msgBox;
            msgBox.setText(tr("Was video recorded with rectified images? (This is asked to avoid double rectification)"));
            QAbstractButton* pButtonRect = msgBox.addButton(tr("Yes"), QMessageBox::YesRole);
            QAbstractButton* pButtonNonRect = msgBox.addButton(tr("No"), QMessageBox::NoRole);

            msgBox.exec();

            if (msgBox.clickedButton()==pButtonRect) {
                enableRectify(false);
            } else if (msgBox.clickedButton()==pButtonNonRect) {
                enableRectify(true);
            }

            // Start frame capture
            enableCapture(true);
        } else {
            msg.setText("Failed to open video stream.");
            msg.exec();
            ui->statusBar->showMessage("Disconnected.");
            //ui->toggleVideoButton->setDisabled(true);
        }
    }
}

void MainWindow::updateCloud() {

    cloud = stereo_cam->getPointCloud();

    if(!cloud.get()){
        first_cloud = true;
    }

    if (!cloud->empty()) {
        // Initial point cloud load

        if (!viewer->updatePointCloud(cloud, "cloud")) {
            viewer->addPointCloud(cloud, "cloud");
        }

        vtk_widget->update();
    } else {
        qDebug() << "Empty point cloud";
        first_cloud = true;
    }

    if (first_cloud){
        first_cloud = false;
        resetPointCloudView();
    }

}

void MainWindow::enableVideoCapture(bool enable){
    if (enable){
        int vid_fps = current_fps;
        if (current_fps == 0){
            vid_fps = measured_fps;
        }
        stereo_cam->setVideoStreamParams("",vid_fps);

        // Ask user if to record rectified frames
        QMessageBox msgBox;
        msgBox.setText(tr("Record recitifed frames if avaiable?"));
        QAbstractButton* pButtonRect = msgBox.addButton(tr("Yes"), QMessageBox::YesRole);
        QAbstractButton* pButtonNonRect = msgBox.addButton(tr("No"), QMessageBox::NoRole);

        msgBox.exec();

        if (msgBox.clickedButton()==pButtonRect) {
            stereo_cam->enableCaptureRectifedVideo(true);
        } else if (msgBox.clickedButton()==pButtonNonRect) {
            stereo_cam->enableCaptureRectifedVideo(false);
        }

        ui->toggleVideoButton->setIcon(awesome->icon(fa::stop, icon_options));
    } else {
        ui->toggleVideoButton->setIcon(awesome->icon(fa::videocamera, icon_options));
    }

    stereo_cam->enableVideoStream(enable);
}

void MainWindow::startCalibrationFromImages(void) {
    calibration_images_dialog = new CalibrateFromImagesDialog(this);
    connect(calibration_images_dialog, SIGNAL(run_calibration()), this, SLOT(runCalibrationFromImages()));
    calibration_images_dialog->move(100, 100);
    calibration_images_dialog->show();

    calibration_from_images_dialog_used = true;
}

void MainWindow::runCalibrationFromImages(void){

    qDebug() << "Beginning calibration from images";

    if(calibration_images_dialog == nullptr) return;
    qDebug() << "Getting parameters";

    int cols = calibration_images_dialog->getPatternCols();
    int rows = calibration_images_dialog->getPatternRows();
    double square_size_mm = calibration_images_dialog->getSquareSizeMm();
    auto left_images = calibration_images_dialog->getLeftImages();
    auto right_images = calibration_images_dialog->getRightImages();
    bool save_ros = calibration_images_dialog->getSaveROS();

    calibration_images_dialog->close();

    calibrator = new StereoCalibrate(this, nullptr);
    cv::Size pattern(cols, rows);

    calibrator->setOutputPath(calibration_images_dialog->getOutputPath());
    calibrator->setPattern(pattern, square_size_mm);
    calibrator->setImages(left_images, right_images);
    calibrator->setSaveROS(save_ros);
    calibrator->jointCalibration();
}

void MainWindow::startAutoCalibration(void) {
    enableRectify(false);
    ui->enableStereo->setChecked(false);
    stereo_cam->enableMatching(false);

    calibration_dialog = new CalibrationDialog(stereo_cam);
    connect(calibration_dialog, SIGNAL(startCalibration()), this, SLOT(runAutoCalibration()));
    calibration_dialog->move(100, 100);
    calibration_dialog->show();

    calibration_dialog_used = true;
}

void MainWindow::runAutoCalibration(void){
    qDebug() << "Beginning calibration";

    if(calibration_dialog == nullptr) return;
    qDebug() << "Getting parameters";

    int cols = calibration_dialog->getPatternCols();
    int rows = calibration_dialog->getPatternRows();
    double square_size_m = calibration_dialog->getSquareSizeMm() / 1000;
    auto left_images = calibration_dialog->getLeftImages();
    auto right_images = calibration_dialog->getRightImages();
    bool save_ros = calibration_dialog->getSaveROS();

    qDebug() << "Square size: " << square_size_m;
    qDebug() << "Cols: " << cols;
    qDebug() << "Rows: " << rows;

    calibration_dialog->close();

    calibrator = new StereoCalibrate(this, nullptr);
    cv::Size pattern(cols, rows);

    calibrator->setOutputPath(calibration_dialog->getOutputPath());
    calibrator->setPattern(pattern, square_size_m);
    calibrator->setImages(left_images, right_images);
    calibrator->setSaveROS(save_ros);
    calibrator->jointCalibration();
}

void MainWindow::doneCalibration(bool) {
    //connect(stereo_cam, SIGNAL(acquired()), this, SLOT(updateDisplay())); TODO check this

    disconnect(calibration_dialog, SIGNAL(stopCalibration()), calibrator,
               SLOT(abortCalibration()));
    disconnect(calibrator, SIGNAL(doneCalibration(bool)), this,
               SLOT(doneCalibration(bool)));
    //stereo_cam->enableCapture(true);
}

void MainWindow::setMatcher(int index) {
    if (index < 0 || index > matcher_list.size()) return;

    auto matcher_widget = matcher_list.at(index);
    qDebug() << "Changing matcher to" << ui->matcherSelectBox->itemText(index);
    if (cameras_connected){
        stereo_cam->setMatcher(matcher_widget->getMatcher());
    }

    for (int i = 0; i < ui->matcherSettingsLayout->count(); ++i) {
        QWidget* widget = ui->matcherSettingsLayout->itemAt(i)->widget();
        if (widget != matcher_widget) {
            widget->setVisible(false);
        } else {
            widget->setVisible(true);
            disparity_view->setMatcher(matcher_widget->getMatcher());
        }
    }
}

void MainWindow::setupMatchers(void) {
    disconnect(ui->matcherSelectBox, SIGNAL(currentIndexChanged(int)), this,
               SLOT(setMatcher(int)));

    while (ui->matcherSelectBox->count() != 0) {
        ui->matcherSelectBox->removeItem(0);
    }

    QLayoutItem* child;
    while ((child = ui->matcherSettingsLayout->takeAt(0)) != nullptr) {
        child->widget()->hide();
        delete child;
    }

    matcher_list.clear();

    MatcherWidgetOpenCVBlock* block_matcher =
            new MatcherWidgetOpenCVBlock(this);
    matcher_list.append(block_matcher);
    ui->matcherSelectBox->insertItem(0, "OpenCV Block");
    ui->matcherSettingsLayout->addWidget(block_matcher);

    MatcherWidgetOpenCVSGBM* opencv_sgbm =
            new MatcherWidgetOpenCVSGBM(this);
    matcher_list.append(opencv_sgbm);
    ui->matcherSelectBox->insertItem(1, "OpenCV SGBM");
    ui->matcherSettingsLayout->addWidget(opencv_sgbm);

#ifdef WITH_I3DRSGM
    qDebug() << "Including I3DRSGM widget";
    MatcherWidgetI3DRSGM* i3dr_sgm =
            new MatcherWidgetI3DRSGM(this);
    matcher_list.append(i3dr_sgm);
    ui->matcherSelectBox->insertItem(2, "I3DR SGBM");
    ui->matcherSettingsLayout->addWidget(i3dr_sgm);
    if (!i3dr_sgm->getMatcher()->isLicenseValid()){
        QStandardItemModel *model =
              qobject_cast<QStandardItemModel *>(ui->matcherSelectBox->model());
          Q_ASSERT(model != nullptr);
          bool disabled = true;
          QStandardItem *item = model->item(2);
          item->setFlags(disabled ? item->flags() & ~Qt::ItemIsEnabled
                                  : item->flags() | Qt::ItemIsEnabled);
          ui->matcherSelectBox->setCurrentIndex(0);
          QMessageBox msg;
          msg.setText("No license found for I3DRSGM. <br>"
                      "You will only be able to use OpenSource matchers from OpenCV. <br>"
                      "Contact info@i3drobotics.com for a license.");
          msg.exec();
          this->setMatcher(0);
    } else {
        ui->matcherSettingsLayout->addWidget(i3dr_sgm);
        ui->matcherSelectBox->setCurrentIndex(2);
        this->setMatcher(2);
    }
#else
    ui->matcherSelectBox->setCurrentIndex(0);
    this->setMatcher(0);
#endif

    connect(ui->matcherSelectBox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(setMatcher(int)));
}

void MainWindow::setCalibrationFolder(QString dir) {
    if(dir == ""){
        dir = QFileDialog::getExistingDirectory(
                    this, tr("Open Calibration Folder"), "/home",
                    QFileDialog::DontResolveSymlinks);
    }

    if(dir == "") return;

    bool res = stereo_cam->loadCalibration(dir);

    if(!res) {
        ui->enableStereo->setEnabled(false);
        QMessageBox msg;
        msg.setText("Unable to load calibration files");
        msg.exec();
    }else{
        ui->enableStereo->setEnabled(true);
        calibration_directory = dir;
        parameters->update_string("calDir", calibration_directory);
        double focal =  stereo_cam->fx;
        double baseline = stereo_cam->baseline;
        disparity_view->setCalibration(stereo_cam->Q,baseline,focal);
    }

    ui->toggleRectifyCheckBox->setEnabled(res);
    ui->toggleRectifyCheckBox->setChecked(res);

}

void MainWindow::statusMessageTimeout(void) {
    if (stereo_cam->isCapturing()) {
        ui->statusBar->showMessage("Capturing.");
    } else {
        ui->statusBar->showMessage("Paused.");
    }
}

void MainWindow::singleShotClicked(void) {
    enableCapture(false);
    stereo_cam->captureSingle();
}

void MainWindow::saveSingle(void) {
    stereo_cam->saveImageTimestamped();
}

void MainWindow::enableRectify(bool enable) {
    ui->toggleRectifyCheckBox->setChecked(enable);
    stereo_cam->enableRectify(enable);
}

void MainWindow::enableCapture(bool enable) {
    if (enable){
        // start capture
        stereo_cam->startCapture();
        ui->statusBar->showMessage("Freerunning.");
        ui->captureButton->setIcon(awesome->icon(fa::pause, icon_options));
        ui->captureButton->setChecked(true);
        toggleCameraPassiveSettings(false);
    } else {
        // stop capture
        stereo_cam->stopCapture();
        ui->statusBar->showMessage("Paused.");
        ui->captureButton->setIcon(awesome->icon(fa::play, icon_options));
        ui->captureButton->setChecked(false);
        toggleCameraPassiveSettings(true);
   }
}

void MainWindow::displaySaved(QString fname) {
    ui->statusBar->showMessage(QString("Saved to: %1").arg(fname));
    status_bar_timer->setSingleShot(true);
    status_bar_timer->start(1500);
}

void MainWindow::updateDisplay(void) {
    cv::Mat left, right;

    if (cameras_connected){
        if (stereo_cam->isConnected()){
            stereo_cam->getLeftImage(left);
            stereo_cam->getRightImage(right);

            if (left.empty() || right.empty()){
                qDebug() << "Empty image sent to display";
                return;
            }

            float downsample_rate = left_view->getDownsampleRate();
            cv::Mat new_left, new_right;
            cv::resize(left,new_left,cv::Size(),downsample_rate,downsample_rate);
            cv::resize(right,new_right,cv::Size(),downsample_rate,downsample_rate);

            left_view->updateView(new_left);
            left_matcher_view->updateView(new_left);
            right_view->updateView(new_right);

        } else {
            qDebug() << "Cannot update display";
        }
    } else {
        qDebug() << "Cannot update display. Stereo camera missing.";
    }
}

void MainWindow::setSaveDirectory(QString dir) {

    if(dir == ""){
        dir = QFileDialog::getExistingDirectory(
                    this, tr("Open Directory"), "/home",
                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    }

    if (dir != "") {
        ui->saveDirLabel->setText(dir);

        parameters->updatePreviousDirectory(dir);
        save_directory = parameters->get_string("saveDir");
        if(stereo_cam)
            stereo_cam->setSavelocation(save_directory);
        disparity_view->setSavelocation(save_directory);
    }
}

void MainWindow::enableAutoGain(bool enable){
    stereo_cam->enableAutoGain(enable);
    if (!enable){
        int gain_val = ui->gainSpinBox->value();
        stereo_cam->setGain(gain_val);
    }
}

void MainWindow::enableAutoExpose(bool enable){
    stereo_cam->enableAutoExposure(enable);
    if (!enable){
        double exposure_val = ui->exposureSpinBox->value();
        stereo_cam->setExposure(exposure_val);
    }
}

void MainWindow::enableFPS(bool enable){
    //if (gigeWarning(current_binning)){
        //stereo_cam->toggleTrigger(enable);
    //} else {
    if (enable){
        ui->enabledTriggeredCheckbox->setChecked(false);
        ui->fpsSpinBox->setEnabled(true);
    } else {
        ui->enabledTriggeredCheckbox->setChecked(true);
        ui->fpsSpinBox->setEnabled(false);
        setFPS(ui->fpsSpinBox->value());
    }
    // TODO: re-enable this
    //}
}

void MainWindow::setFPS(int fps){
    int binning = 5;
    binning = ui->binningSpinBox->value();
    if (gigeWarning(binning,fps)){
        stereo_cam->setFPS(fps);
        current_fps = fps;
    } else {
        disconnect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), this,
                   SLOT(setFPS(int)));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        ui->fpsSpinBox->setValue(current_fps);
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        connect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), this,
                SLOT(setFPS(int)));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

void MainWindow::setPacketSize(){
    QProgressDialog progressPacketSize("Updating packet size...", "", 0, 100, this);
    progressPacketSize.setWindowTitle("SVT");
    progressPacketSize.setWindowModality(Qt::WindowModal);
    progressPacketSize.setCancelButton(nullptr);
    progressPacketSize.setMinimumDuration(0);
    progressPacketSize.setValue(30);
    QCoreApplication::processEvents();

    int packetSize = ui->packetSizeSpinBox->value();

    stereo_cam->setPacketSize(packetSize);

    progressPacketSize.setValue(100);
    progressPacketSize.close();
}

void MainWindow::setBinning(int binning){
    if (gigeWarning(binning)){
        QProgressDialog progressBinning("Updating binning...", "", 0, 100, this);
        progressBinning.setWindowTitle("SVT");
        progressBinning.setWindowModality(Qt::WindowModal);
        progressBinning.setCancelButton(nullptr);
        progressBinning.setMinimumDuration(0);
        progressBinning.setValue(30);
        QCoreApplication::processEvents();

        stereo_cam->setBinning(binning);
        current_binning = binning;

        //TODO fix crash after setting binning

        progressBinning.setValue(100);
        progressBinning.close();
    } else {
        disconnect(ui->binningSpinBox, SIGNAL(valueChanged(int)), this,
                   SLOT(setBinning(int)));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        ui->binningSpinBox->setValue(current_binning);
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        connect(ui->binningSpinBox, SIGNAL(valueChanged(int)), this,
                SLOT(setBinning(int)));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

void MainWindow::enableBinning(bool enable){
    int binning_val = 1;
    if (enable){
        binning_val = current_binning;
    } else {
        current_binning = 1;
    }

    if (gigeWarning(binning_val)){
        QProgressDialog progressBinning("Updating binning...", "", 0, 100, this);
        progressBinning.setWindowTitle("SVT");
        progressBinning.setWindowModality(Qt::WindowModal);
        progressBinning.setCancelButton(nullptr);
        progressBinning.setMinimumDuration(0);
        progressBinning.setValue(30);
        QCoreApplication::processEvents();

        stereo_cam->setBinning(binning_val);

        progressBinning.setValue(100);
        progressBinning.close();
    } else {
        ui->binningSpinBox->setEnabled(true);
    }
}

void MainWindow::toggleCameraActiveSettings(bool enable){
    ui->exposureSpinBox->setEnabled(enable);
    ui->autoExposeCheck->setEnabled(enable);
    ui->gainSpinBox->setEnabled(enable);
    ui->autoGainCheckBox->setEnabled(enable);
    ui->enableHDRCheckbox->setEnabled(enable);
}

void MainWindow::toggleCameraPassiveSettings(bool enable){
    ui->fpsSpinBox->setEnabled(enable);
    ui->enabledTriggeredCheckbox->setEnabled(enable);
    ui->binningSpinBox->setEnabled(enable);
    ui->packetDelaySpinBox->setEnabled(enable);
    ui->packetSizeSpinBox->setEnabled(enable);
}

bool MainWindow::gigeWarning(int binning,int new_fps){
    if (using_gige){
        int fps;
        if (new_fps >= 0){
            fps = new_fps;
        } else {
            if (!ui->enabledTriggeredCheckbox->isChecked()){
                fps = current_fps;
            } else { // use current measured fps hardware trigger is being used
                fps = measured_fps;
            }
        }
        if (fps == 0){
            fps = 5; // use fps estimate of 5 when fps cap is disabled (fps = 0)
        }
        int bit_per_second = ((stereo_cam->getHeight() * stereo_cam->getWidth() * 2)/binning) * fps;
        qDebug() << "BPS: " << bit_per_second;
        int max_for_warning = 8000000; // 1MB
        if (bit_per_second > max_for_warning){
            /*
            QMessageBox msgBoxWarning;
            msgBoxWarning.setText(tr("Warning! Data may exceed 1MBps. This may exceed your internet speed when running a GigE camera on a standard network."));
            msgBoxWarning.setInformativeText(tr("Are you sure you want to continue?"));
            msgBoxWarning.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            msgBoxWarning.setDefaultButton(QMessageBox::Yes);
            int ret = msgBoxWarning.exec();
            if (ret == QMessageBox::No){
                return false;
            } else {
                return true;
            }
            */
            return true;
        } else {
            return true;
        }
    } else {
        return true;
    }
}

void MainWindow::updateFrameCount(qint64 count) {
    frame_counter->setText(QString("Frame count: %1").arg(count));
}

void MainWindow::updateFrameTime(qint64 time) {
    int fps = 1000.0 / time;
    measured_fps = fps;

    fps_measure_total+= measured_fps;
    fps_measure_count++;

    int average_fps = 0;
    average_fps = ceil((float)fps_measure_total / (float)fps_measure_count);
    fps_counter->setText(QString("FPS: %1").arg(average_fps));

    if (fps_measure_count > 10){
        fps_measure_total = average_fps;
        fps_measure_count = 1;
    }
}

void MainWindow::openHelp(){
    QString link = QCoreApplication::applicationDirPath() + "/docs/help/index.html";
    QDesktopServices::openUrl(QUrl(link));
}

void MainWindow::hideCameraSettings(bool hide){
    if (hide){
        ui->widgetSideSettings->setVisible(false);
        ui->btnShowCameraSettings->setVisible(true);
        ui->disparityViewSettings->setVisible(true);
        ui->btnHideCameraSettings->setVisible(false);
    } else {
        ui->widgetSideSettings->setVisible(true);
        ui->btnShowCameraSettings->setVisible(false);
        ui->disparityViewSettings->setVisible(false);
        ui->btnHideCameraSettings->setVisible(true);
    }
}

void MainWindow::on_btnShowCameraSettings_clicked()
{
    hideCameraSettings(ui->widgetSideSettings->isVisible());
}

void MainWindow::on_btnHideCameraSettings_clicked()
{
    on_btnShowCameraSettings_clicked();
}

void MainWindow::closeEvent(QCloseEvent *) {
    qDebug() << "Closing application";
    stereoCameraRelease();
    qDebug() << "Waiting for device list timer to finish...";
    stopDeviceListTimer();
    while(device_list_timer->isActive()){QCoreApplication::processEvents(QEventLoop::AllEvents);}
    // Close external windows
    if (calibration_dialog_used){
        calibration_dialog->close();
    }
    if (calibration_from_images_dialog_used){
        calibration_images_dialog->close();
    }
    qDebug() << "Closing processes...";
}

MainWindow::~MainWindow() {
    //stereoCameraRelease();
    delete ui;
}
