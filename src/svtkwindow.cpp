/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "svtkwindow.h"
#include "ui_svtkwindow.h"

SVTKWindow::SVTKWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::SVTKWindow) {
    ui->setupUi(this);
    QCoreApplication::setApplicationName("Stereo Vision Toolkit");
    setWindowTitle(QCoreApplication::applicationName());

    settings = new QSettings("I3Dr", "Stereo Vision Toolkit");

    // Required outside of the camera class definitions as 'list_systems' is slow otherwise
    // due to it having to re-initalise api's
    Pylon::PylonInitialize();
    DShowLib::InitLibrary();
    tisgrabber = new DShowLib::Grabber();
    pylonTlFactory = &Pylon::CTlFactory::GetInstance();

    ui->imageViewTab->raise();
    ui->tabWidget->lower();

    ui->tabWidget->setCurrentIndex(0);
    ui->tabLayoutSettings->setCurrentIndex(0);

    cameras_connected = false;
    calibration_dialog_used = false;
    calibration_from_images_dialog_used = false;

    cam_thread = new QThread;
    device_list_timer = new QTimer(this);

    // define default settings for cameras
    // if the camera does not use the setting then should be set to -1
    // TODO replace this by reading current value from camera
    default_basler_usb_init_settings.exposure = 5;
    default_basler_usb_init_settings.gain = 0;
    default_basler_usb_init_settings.fps = 10;
    default_basler_usb_init_settings.binning = 1;
    default_basler_usb_init_settings.trigger = true;
    default_basler_usb_init_settings.hdr = -1;
    default_basler_usb_init_settings.autoExpose = false;
    default_basler_usb_init_settings.autoGain = false;
    default_basler_usb_init_settings.isGige = false;
    default_basler_usb_init_settings.packetDelay = -1;
    default_basler_usb_init_settings.packetSize = -1;

    default_basler_gige_init_settings.exposure = 5;
    default_basler_gige_init_settings.gain = 0;
    default_basler_gige_init_settings.fps = 5;
    default_basler_gige_init_settings.binning = 1;
    default_basler_gige_init_settings.trigger = true;
    default_basler_gige_init_settings.hdr = -1;
    default_basler_gige_init_settings.autoExpose = false;
    default_basler_gige_init_settings.autoGain = false;
    default_basler_gige_init_settings.isGige = 1;
    default_basler_gige_init_settings.packetDelay = 0;
    default_basler_gige_init_settings.packetSize = 1500;

    default_vimba_init_settings.exposure = 5;
    default_vimba_init_settings.gain = 0;
    default_vimba_init_settings.fps = 30;
    default_vimba_init_settings.binning = 1;
    default_vimba_init_settings.trigger = true;
    default_vimba_init_settings.hdr = -1;
    default_vimba_init_settings.autoExpose = false;
    default_vimba_init_settings.autoGain = false;
    default_vimba_init_settings.isGige = false;
    default_vimba_init_settings.packetDelay = -1;
    default_vimba_init_settings.packetSize = -1;

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

    default_tara_init_settings.exposure = 5;
    default_tara_init_settings.gain = -1;
    default_tara_init_settings.fps = 60;
    default_tara_init_settings.binning = -1;
    default_tara_init_settings.trigger = -1;
    default_tara_init_settings.hdr = false;
    default_tara_init_settings.autoExpose = false;
    default_tara_init_settings.autoGain = -1;
    default_tara_init_settings.isGige = -1;
    default_tara_init_settings.packetDelay = -1;
    default_tara_init_settings.packetSize = -1;

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

    default_phobos_basler_usb_init_settings.exposure = 2.5;
    default_phobos_basler_usb_init_settings.gain = 0;
    default_phobos_basler_usb_init_settings.fps = 10;
    default_phobos_basler_usb_init_settings.binning = 1;
    default_phobos_basler_usb_init_settings.trigger = true;
    default_phobos_basler_usb_init_settings.hdr = -1;
    default_phobos_basler_usb_init_settings.autoExpose = false;
    default_phobos_basler_usb_init_settings.autoGain = false;
    default_phobos_basler_usb_init_settings.isGige = false;
    default_phobos_basler_usb_init_settings.packetDelay = -1;
    default_phobos_basler_usb_init_settings.packetSize = -1;

    default_phobos_basler_gige_init_settings.exposure = 5;
    default_phobos_basler_gige_init_settings.gain = 0;
    default_phobos_basler_gige_init_settings.fps = 5;
    default_phobos_basler_gige_init_settings.binning = 1;
    default_phobos_basler_gige_init_settings.trigger = true;
    default_phobos_basler_gige_init_settings.hdr = -1;
    default_phobos_basler_gige_init_settings.autoExpose = false;
    default_phobos_basler_gige_init_settings.autoGain = false;
    default_phobos_basler_gige_init_settings.isGige = 1;
    default_phobos_basler_gige_init_settings.packetDelay = 0;
    default_phobos_basler_gige_init_settings.packetSize = 1500;

    default_phobos_tis_usb_init_settings.exposure = 5;
    default_phobos_tis_usb_init_settings.gain = 0;
    default_phobos_tis_usb_init_settings.fps = 10;
    default_phobos_tis_usb_init_settings.binning = -1;
    default_phobos_tis_usb_init_settings.trigger = false;
    default_phobos_tis_usb_init_settings.hdr = -1;
    default_phobos_tis_usb_init_settings.autoExpose = false;
    default_phobos_tis_usb_init_settings.autoGain = false;
    default_phobos_tis_usb_init_settings.isGige = -1;
    default_phobos_tis_usb_init_settings.packetDelay = -1;
    default_phobos_tis_usb_init_settings.packetSize = -1;

    default_titania_basler_usb_init_settings.exposure = 5;
    default_titania_basler_usb_init_settings.gain = 0;
    default_titania_basler_usb_init_settings.fps = 10;
    default_titania_basler_usb_init_settings.binning = 1;
    default_titania_basler_usb_init_settings.trigger = true;
    default_titania_basler_usb_init_settings.hdr = -1;
    default_titania_basler_usb_init_settings.autoExpose = false;
    default_titania_basler_usb_init_settings.autoGain = false;
    default_titania_basler_usb_init_settings.isGige = false;
    default_titania_basler_usb_init_settings.packetDelay = -1;
    default_titania_basler_usb_init_settings.packetSize = -1;

    default_titania_basler_gige_init_settings.exposure = 5;
    default_titania_basler_gige_init_settings.gain = 0;
    default_titania_basler_gige_init_settings.fps = 5;
    default_titania_basler_gige_init_settings.binning = 1;
    default_titania_basler_gige_init_settings.trigger = true;
    default_titania_basler_gige_init_settings.hdr = -1;
    default_titania_basler_gige_init_settings.autoExpose = false;
    default_titania_basler_gige_init_settings.autoGain = false;
    default_titania_basler_gige_init_settings.isGige = 1;
    default_titania_basler_gige_init_settings.packetDelay = 0;
    default_titania_basler_gige_init_settings.packetSize = 1500;

    default_titania_vimba_usb_init_settings.exposure = 5;
    default_titania_vimba_usb_init_settings.gain = 0;
    default_titania_vimba_usb_init_settings.fps = 30;
    default_titania_vimba_usb_init_settings.binning = 1;
    default_titania_vimba_usb_init_settings.trigger = true;
    default_titania_vimba_usb_init_settings.hdr = -1;
    default_titania_vimba_usb_init_settings.autoExpose = false;
    default_titania_vimba_usb_init_settings.autoGain = false;
    default_titania_vimba_usb_init_settings.isGige = false;
    default_titania_vimba_usb_init_settings.packetDelay = -1;
    default_titania_vimba_usb_init_settings.packetSize = -1;

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

    about_dialog = new AboutDialog(this);
    QString version_name = QString("Stereo Vision Toolkit v")+QString(FV_APP_VERSION);
    about_dialog->setVersion(version_name);

    /* Calibration */
    connect(ui->actionCalibration_wizard, SIGNAL(triggered(bool)), this,
            SLOT(startAutoCalibration()));
    connect(ui->actionCalibrate_from_images, SIGNAL(triggered(bool)), this,
            SLOT(startCalibrationFromImages()));
    connect(ui->actionDocumentation, SIGNAL(triggered(bool)), this,
            SLOT(openHelp()));
#ifdef WITH_FERVOR
    connect(ui->actionCheckUpdates, SIGNAL(triggered(bool)), this,
            SLOT(checkUpdates()));
#endif
    connect(ui->actionAbout, SIGNAL(triggered(bool)), this,
            SLOT(openAbout()));
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

    ui->checkBoxAutoZ->setChecked(true);
    enableAutoZ(true);

    connect(ui->checkBoxAutoZ, SIGNAL(toggled(bool)), this, SLOT(enableAutoZ(bool)));

    statusBarInit();

#ifdef WITH_VIMBA
    // Start the Vimba API in one place.
    VimbaSystem &system = VimbaSystem::GetInstance();
    auto err = system.Startup();

    if(err != VmbErrorSuccess){
        qDebug() << "Failed to start VIMBA";
    }
#endif

    //disable tabs until camera is connected to prevent crashes
    disableWindow();
    controlsInit();
    pointCloudInit();
    setupMatchers();
    detectionInit();

    // Initalise shared memory class
    sharedMemoryInst = new cvSharedMemory();

    hideCameraSettings(false);

#ifdef WITH_FERVOR
    checkUpdates();
#endif

    //refreshCameraListThreaded();
    startDeviceListTimer();
}

#ifdef WITH_FERVOR
void SVTKWindow::downloadUpdateComplete(){
    FvUpdater::sharedUpdater()->RunUpdator();
    QApplication::quit();
}

void SVTKWindow::checkUpdates(){

    // Set the Fervor appcast url
#ifdef DEV_BRANCH
    FvUpdater::sharedUpdater()->SetFeedURL("https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/dev/AppcastDev.xml");
#else
    FvUpdater::sharedUpdater()->SetFeedURL("https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/master/Appcast.xml");
#endif

    // Print current version to debug
    qDebug() << FV_APP_NAME << FV_APP_VERSION;

    // Check for updates silently -- this will not block the initialization of
    // your application, just start a HTTP request and return immediately.
    qDebug() << "Silently checking for updates...";
    FvUpdater::sharedUpdater()->CheckForUpdatesSilent();
    qDebug() << "Update check running in non blocking thread.";
    QObject::connect(FvUpdater::sharedUpdater(), SIGNAL(downloadFinished(void)), this, SLOT(downloadUpdateComplete(void)));
}
#endif

void SVTKWindow::disableWindow(){
    ui->toggleSwapLeftRight->setDisabled(true);
    ui->toggleCalibrationDownsample->setDisabled(true);
    ui->spinBoxImageDownsample->setDisabled(true);
    ui->spinBoxSharedMemDownsample->setDisabled(true);
    ui->tabWidget->setDisabled(true);
    ui->tabCameraSettings->setDisabled(true);
    ui->tabApplicationSettings->setDisabled(true);
    ui->matcherSelectBox->setDisabled(true);
    ui->enableStereo->setDisabled(true);
    ui->enableDetection->setDisabled(true);
    ui->enableSharedMem->setDisabled(true);

    ui->captureButton->setDisabled(true);
    ui->singleShotButton->setDisabled(true);
    ui->saveButton->setDisabled(true);
    ui->toggleVideoButton->setDisabled(true);
    ui->toggleRectifyCheckBox->setDisabled(true);
    ui->actionCalibration_wizard->setDisabled(true);

    toggleCameraActiveSettings(false);
    toggleCameraPassiveSettings(false);
}

void SVTKWindow::enableWindow(){

    ui->toggleSwapLeftRight->setEnabled(true);
    ui->toggleCalibrationDownsample->setEnabled(true);
    ui->spinBoxImageDownsample->setEnabled(true);
    ui->spinBoxSharedMemDownsample->setEnabled(true);
    ui->tabWidget->setEnabled(true);
    ui->tabCameraSettings->setEnabled(true);
    ui->tabApplicationSettings->setEnabled(true);
    ui->matcherSelectBox->setEnabled(true);

    //ui->enableDetection->setEnabled(true);
    ui->enableSharedMem->setEnabled(true);
    ui->txtSharedMemMap->setEnabled(true);
    ui->txtSharedMemMutex->setEnabled(true);
    ui->captureButton->setEnabled(true);
    ui->singleShotButton->setEnabled(true);
    //ui->saveButton->setEnabled(false);
    ui->toggleVideoButton->setEnabled(true);
    ui->actionCalibration_wizard->setEnabled(true);
}

void SVTKWindow::enableAutoZ(bool enable){
    autoZ = enable;
    ui->minZSpinBox->setEnabled(!enable);
    ui->maxZSpinBox->setEnabled(!enable);
}

void SVTKWindow::pointCloudSaveStatus(QString msg){
    qDebug() << msg;
    QMessageBox::warning(this,"Stereo Vision Toolkit",msg);
}

void SVTKWindow::resetStatusBar(void) {
    QString fps_number = QString::number(0, 'G', 3);
    match_fps_counter->setText(QString("Match FPS: ") + fps_number);
    fps_counter->setText(QString("Camera FPS: ") + fps_number);
    frame_counter->setText("Frame count: 0");
}

void SVTKWindow::statusBarInit(void) {
    status_widget = new QWidget;
    ui->statusBar->addPermanentWidget(status_widget);

    fps_counter = new QLabel(this);
    match_fps_counter = new QLabel(this);
    temp_label = new QLabel(this);
    frame_counter = new QLabel(this);
    status_bar_spacer = new QSpacerItem(20, 1);

    QHBoxLayout* _hlayout = new QHBoxLayout();
    _hlayout->addWidget(frame_counter);
    _hlayout->addSpacerItem(status_bar_spacer);
    _hlayout->addWidget(match_fps_counter);
    _hlayout->addWidget(fps_counter);
    _hlayout->addWidget(temp_label);

    status_widget->setLayout(_hlayout);

    status_bar_timer = new QTimer;

    connect(status_bar_timer, SIGNAL(timeout()), this,
            SLOT(statusMessageTimeout()));

    ui->saveDirLabel->setText(save_directory);
}

void SVTKWindow::controlsInit(void) {

    ui->widgetSideSettings->setVisible(true);

    connect(ui->captureButton, SIGNAL(clicked(bool)), this, SLOT(enableCapture(bool)));
    connect(ui->singleShotButton, SIGNAL(clicked()), this, SLOT(singleShotClicked()));
    connect(ui->setSaveDirButton, SIGNAL(clicked()), this, SLOT(setSaveDirectory()));
    connect(ui->toggleVideoButton, SIGNAL(clicked(bool)), this, SLOT(enableVideoCapture(bool)));
    connect(ui->btnLoadCalibration, SIGNAL(clicked(bool)),this, SLOT(setCalibrationFolder()));
    connect(ui->actionLoad_Calibration, SIGNAL(triggered(bool)),this, SLOT(setCalibrationFolder()));
    connect(ui->autoExposeCheck, SIGNAL(clicked(bool)), ui->exposureSpinBox, SLOT(setDisabled(bool)));
    connect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), ui->gainSpinBox, SLOT(setDisabled(bool)));

    //connect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), ui->fpsSpinBox, SLOT(setDisabled(bool)));

    connect(ui->btnRefreshCameras, SIGNAL(clicked(bool)), this, SLOT(refreshCameraListThreaded()));

    icon_options.insert("color", QColor(255, 255, 255));
    ui->captureButton->setIcon(awesome->icon(fa::play, icon_options));
    ui->saveButton->setIcon(awesome->icon(fa::save, icon_options));
    ui->singleShotButton->setIcon(awesome->icon(fa::camera, icon_options));
    ui->enableStereo->setIcon(awesome->icon(fa::cubes, icon_options));
    ui->enableDetection->setIcon(awesome->icon(fa::eye, icon_options));
    ui->enableSharedMem->setIcon(awesome->icon(fa::exchange, icon_options)); //TODO choose good shared memory icon
    ui->toggleVideoButton->setIcon(awesome->icon(fa::videocamera, icon_options));

    connect(ui->actionLoad_Stereo_Video, SIGNAL(triggered(bool)), this,
            SLOT(videoStreamLoad()));
    connect(ui->actionLoad_Stereo_Image_Pair, SIGNAL(triggered(bool)), this,
            SLOT(stereoImageLoad()));

    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this,
            SLOT(enable3DViz(int)));

    disparity_view = new DisparityViewer();
    disparity_view->setViewer(ui->disparityViewLabel);
    connect(disparity_view, SIGNAL(newDisparity(QPixmap)), ui->disparityViewLabel,
            SLOT(setPixmap(QPixmap)));
    ui->disparityViewSettingsLayout->addWidget(disparity_view);

    connect(ui->reset3DViewButton, SIGNAL(clicked(bool)), this, SLOT(resetPointCloudView()));
}

void SVTKWindow::resetPointCloudView(){
    cloud_viewer->resetCamera();
}

void SVTKWindow::enable3DViz(int tab = 0) {
    if (!stereo_cam) return;

    if (tab == 2) {
        stereo_cam->enableReproject(true);
    } else {
        stereo_cam->enableReproject(false);
    }
}

void SVTKWindow::enableDetection(bool enable){
    if (!stereo_cam){
        detection_enabled = false;
        return;
    }
    detection_enabled = enable;
}

void SVTKWindow::enableSharedMemory(bool enable){
    if (!stereo_cam){
        enable = false;
    }
    if (enable){
        std::string mapname = ui->txtSharedMemMap->text().toStdString();
        std::string mutexname = ui->txtSharedMemMutex->text().toStdString();
        int shared_mem_downsample_factor = ui->spinBoxSharedMemDownsample->value();
        int camera_downsample_factor = ui->spinBoxImageDownsample->value();
        double downsample_rate = (1.0/(double)shared_mem_downsample_factor) * (1.0/(double)camera_downsample_factor);
        int img_height = round(stereo_cam->getHeight() * downsample_rate);
        int img_width = round(stereo_cam->getWidth() * downsample_rate);
        if (ui->comboBoxSharedMemSource->currentText() == "RGBD"){
            connect(stereo_cam, SIGNAL(matched()), this, SLOT(updateSharedMemory()));
            // clear shared memory if open
            if (sharedMemoryInst->isOpen()){
                sharedMemoryInst->close();
            }
            if (ui->checkBoxSharedMem16bit->isChecked()){
                // create shared memory file to store 4 channel rgbd image with element size 4*unsigned short size
                sharedMemoryInst->open(img_height, img_width, 4, 4*sizeof(unsigned short), CV_16UC4, mapname, mutexname);
            } else {
                // create shared memory file to store 4 channel rgbd image with element size 4*float size
                sharedMemoryInst->open(img_height, img_width, 4, 4*sizeof(float), CV_32FC4, mapname, mutexname);
            }
        } else {
            connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateSharedMemory()));
            // clear shared memory if open
            if (sharedMemoryInst->isOpen()){
                sharedMemoryInst->close();
            }
            // create shared memory file to store 3 channel rgb image with element size 3*uchar size
            sharedMemoryInst->open(img_height, img_width, 3, 3*sizeof(unsigned char), CV_8UC3, mapname, mutexname);
        }
    } else {
        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateSharedMemory()));
        disconnect(stereo_cam, SIGNAL(matched()), this, SLOT(updateSharedMemory()));
        //stop shared memory server
        sharedMemoryInst->close();
    }
    shared_memory_enabled = enable;
    ui->txtSharedMemMap->setEnabled(!enable);
    ui->txtSharedMemMutex->setEnabled(!enable);
    ui->comboBoxSharedMemSource->setEnabled(!enable);
    ui->checkBoxSharedMemUseRectified->setEnabled(!enable);
    ui->checkBoxSharedMem16bit->setEnabled(!enable);
    ui->spinBoxSharedMemDownsample->setEnabled(!enable);
}

void SVTKWindow::detectionInit(){
    qDebug() << "Initialising detector...";
    QProgressDialog progressPCI("Initialising detection...", "", 0, 100, this);
    progressPCI.setWindowTitle("SVT");
    progressPCI.setWindowModality(Qt::WindowModal);
    progressPCI.setCancelButton(nullptr);
    progressPCI.setMinimumDuration(0);
    progressPCI.setValue(10);
    QCoreApplication::processEvents();

    object_detector = new DetectorOpenCV();
    QThread* detector_thread = new QThread;
    object_detector->assignThread(detector_thread);

    object_detection_display = new CameraDisplayWidget(this);
    ui->gridLayoutDetection->addWidget(object_detection_display);

    object_detector->setConfidenceThresholdPercent(ui->detectionThresholdSpinbox->value());
    object_detector->setNMSThresholdPercent(ui->nmsThresholdSpinbox->value());

    connect(ui->detectionSetupButton, SIGNAL(clicked(bool)), this, SLOT(configureDetection()));
    //connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(enableDetection(int)));

    connect(ui->detectionThresholdSlider, SIGNAL(valueChanged(int)), ui->detectionThresholdSpinbox, SLOT(setValue(int)));
    connect(ui->detectionThresholdSpinbox, SIGNAL(valueChanged(int)), ui->detectionThresholdSlider, SLOT(setValue(int)));
    connect(ui->detectionThresholdSpinbox, SIGNAL(valueChanged(int)), this, SLOT(updateDetectionThreshold(int)));
    ui->detectionThresholdSpinbox->setValue(settings->value("detection_threshold", 75).toInt());

    connect(ui->nmsThresholdSlider, SIGNAL(valueChanged(int)), ui->nmsThresholdSpinbox, SLOT(setValue(int)));
    connect(ui->nmsThresholdSpinbox, SIGNAL(valueChanged(int)), ui->nmsThresholdSlider, SLOT(setValue(int)));
    connect(ui->nmsThresholdSpinbox, SIGNAL(valueChanged(int)), this, SLOT(updateNMSThreshold(int)));
    ui->nmsThresholdSpinbox->setValue(settings->value("nms_threshold", 75).toInt());

    connect(ui->bboxAlphaSlider, SIGNAL(valueChanged(int)), ui->bboxAlphaSpinbox, SLOT(setValue(int)));
    connect(ui->bboxAlphaSpinbox, SIGNAL(valueChanged(int)), ui->bboxAlphaSlider, SLOT(setValue(int)));
    connect(ui->bboxAlphaSpinbox, SIGNAL(valueChanged(int)), this, SLOT(updateBoundingBoxAlpha(int)));
    ui->bboxAlphaSpinbox->setValue(settings->value("bbox_alpha", 75).toInt());

    qDebug() << "Detector initalisation complete.";
}

void SVTKWindow::configureDetection(){
    if(object_detector == nullptr) return;

    DetectorSetupDialog detection_dialog;
    detection_dialog.exec();

    if(detection_dialog.result() != QDialog::Accepted ) return;

    auto names_file = detection_dialog.getNames().toStdString();
    auto cfg_file = detection_dialog.getCfg().toStdString();
    auto weight_file = detection_dialog.getWeights().toStdString();

    object_detector->setChannels(detection_dialog.getChannels());
    object_detector->setTarget(detection_dialog.getTarget());
    object_detector->setFramework(detection_dialog.getFramework());
    object_detector->setConvertGrayscale(detection_dialog.getConvertGrayscale());
    object_detector->setConvertDepth(detection_dialog.getConvertDepth());
    object_detector->loadNetwork(names_file, cfg_file, weight_file);
    object_detector->setImageSize(detection_dialog.getWidth(), detection_dialog.getHeight());

    ui->enableDetection->setEnabled(true);
    ui->enableDetection->setChecked(true);
    this->enableDetection(true);

    // GUI feedback
    ui->numberClassesLabel->setText(QString("%1").arg(object_detector->getNumClasses()));

    if(object_detector->getNumClasses() > 0){

        ui->classListWidget->clear();

        for(auto &class_name : object_detector->getClassNames()){
            ui->classListWidget->addItem(class_name.c_str());

            auto tag_name = QString("class/%1/colour").arg(class_name.c_str());

            int r=255, g=0, b=0, a=255;

            if(settings->contains(tag_name)){
                auto colour = settings->value(tag_name).toString().split(",");

                if(colour.size() >= 3){
                    r = colour[0].toInt();
                    g = colour[1].toInt();
                    b = colour[2].toInt();
                }

                if(colour.size() >= 4){
                    a = colour[3].toInt();
                }
            }

            setClassColour(class_name.c_str(), QColor(r, g, b, a));


            tag_name = QString("class/%1/visible").arg(class_name.c_str());
            bool visible = true;
            if(settings->contains(tag_name)){
                visible = settings->value(tag_name).toBool();
            }
            setClassVisible(class_name.c_str(), visible);


            tag_name = QString("class/%1/fill").arg(class_name.c_str());
            bool fill = false;
            if(settings->contains(tag_name)){
                fill = settings->value(tag_name).toBool();
            }
            setClassFilled(class_name.c_str(), fill);
        }

        connect(ui->setClassColourButton, SIGNAL(clicked(bool)), this, SLOT(updateClassColour(void)));
        connect(ui->setClassFilledButton, SIGNAL(clicked(bool)), this, SLOT(updateClassFilled(bool)));
        connect(ui->setClassVisibleButton, SIGNAL(clicked(bool)), this, SLOT(updateClassVisible(bool)));

        connect(ui->classListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(onClassListClicked()));
    }

    ui->classListWidget->sortItems();

    QFile weight_name(detection_dialog.getWeights());
    QFileInfo weights_fileinfo(weight_name.fileName());
    ui->activeWeightsLoadedLabel->setText(weights_fileinfo.fileName());

    QFile config_name(detection_dialog.getCfg());
    QFileInfo config_fileinfo(config_name.fileName());
    ui->activeModelLabel->setText(config_fileinfo.fileName());
}

void SVTKWindow::onClassListClicked(void){

    int number_selected = ui->classListWidget->selectedItems().size();
    if(number_selected == 0){
        ui->setClassFilledButton->setDisabled(true);
        ui->setClassVisibleButton->setDisabled(true);
        ui->setClassColourButton->setDisabled(true);
        ui->bboxAlphaSlider->setDisabled(true);
        ui->bboxAlphaSpinbox->setDisabled(true);
        ui->classColourLabel->setVisible(false);
    }else{
        ui->setClassFilledButton->setEnabled(true);
        ui->setClassVisibleButton->setEnabled(true);
        ui->setClassColourButton->setEnabled(true);
        ui->bboxAlphaSlider->setEnabled(true);
        ui->bboxAlphaSpinbox->setEnabled(true);
        ui->classColourLabel->setVisible(true);

        QString class_name = ui->classListWidget->currentItem()->text();
        bool filled = class_filled_map[class_name];
        ui->setClassFilledButton->setChecked(filled);
        bool visible = class_visible_map[class_name];
        ui->setClassVisibleButton->setChecked(visible);

        int r, g, b;
        class_colour_map[class_name].getRgb(&r, &g, &b);

        ui->classColourLabel->setStyleSheet(QString("background-color: %1, %2, %3;")
                                            .arg(QString::number(r))
                                            .arg(QString::number(g))
                                            .arg(QString::number(b)));
    }
}

void SVTKWindow::updateClassColour(void){

    QColorDialog dialog;
    QString class_name = ui->classListWidget->currentItem()->text();
    auto result = dialog.getColor(class_colour_map[class_name]);

    if(result.isValid()){
        setClassColour(ui->classListWidget->currentItem()->text(), result);
    }
}

void SVTKWindow::updateClassFilled(bool checked){
    setClassFilled(ui->classListWidget->currentItem()->text(), checked);
}

void SVTKWindow::updateClassVisible(bool checked){
    setClassVisible(ui->classListWidget->currentItem()->text(), checked);
}

void SVTKWindow::updateBoundingBoxAlpha(int value){
    bounding_box_alpha = value;
    settings->setValue("bbox_alpha", value);
    qDebug() << "Updated bounding box fill transparency to: " << value;
}

void SVTKWindow::updateNMSThreshold(int value){
    object_detector->setNMSThresholdPercent(value);
    settings->setValue("nms_threshold", value);
    qDebug() << "Updated NMS threshold % to: " << value;
}

void SVTKWindow::updateDetectionThreshold(int value){
    object_detector->setConfidenceThresholdPercent(value);
    settings->setValue("Updated detection threshold % to: ", value);
}


void SVTKWindow::setClassColour(QString class_name, QColor class_colour){

    // Set class colour
    auto tag_name = QString("class/%1/colour").arg(class_name);

    int r=255, g=0, b=0, a=255; // default to red
    class_colour.getRgb(&r, &g, &b, &a);

    settings->setValue(tag_name, QString("%1,%2,%3,%4")
                                        .arg(QString::number(r))
                                        .arg(QString::number(g))
                                        .arg(QString::number(b))
                                        .arg(QString::number(a)));

    class_colour_map.insert(class_name, class_colour);

    qDebug() << "Updated colour for " << class_name << "(" << r << "," << g << "," << b << ")";
}

void SVTKWindow::setClassVisible(QString class_name, bool visible){

    auto tag_name = QString("class/%1/visible").arg(class_name);

    settings->setValue(tag_name, visible);

    class_visible_map.insert(class_name, visible);

    if(!visible)
        qDebug() << "Hiding " << class_name;
}

void SVTKWindow::setClassFilled(QString class_name, bool fill){
    auto tag_name = QString("class/%1/fill").arg(class_name);

    settings->setValue(tag_name, fill);

    class_filled_map.insert(class_name, fill);

    if(fill)
        qDebug() << "Setting " << class_name << " class to be filled";
}

void SVTKWindow::updateDetection(){

    if(!object_detector) return;

    // Protect callback against high frame rates (faster than detector)
    if(object_detector->isRunning() || detecting){
        return;
    }

    this->detecting = true;

    // Select image source
    if(ui->imageSourceComboBox->currentText() == "Left"){
        stereo_cam->getLeftImage(image_detection);
    }else{
        stereo_cam->getRightImage(image_detection);
    }

    double scale_factor_x = 1.0;
    double scale_factor_y = 1.0;

    if(detection_enabled){

        if(image_detection.empty()){
            qDebug() << "Empty image passed to detector";
            return;
        }

        // Check if we have a network loaded
        if(!object_detector->isReady())
            return;

        // We'll later resize the bounding boxes to the input image size
        scale_factor_x = static_cast<double>(object_detector->getInputWidth())/image_detection.cols;
        scale_factor_y = static_cast<double>(object_detector->getInputHeight())/image_detection.rows;

        // (Down)scale input for DNN first layer
        cv::resize(image_detection,
                   image_detection_rescale,
                   cv::Size(),
                   scale_factor_x,
                   scale_factor_y);


        // Inference
        auto results = object_detector->infer(image_detection_rescale);

        // Update UI
        ui->latencyLabel->setText(QString("%1 ms").arg(object_detector->getProcessingTime()));
        ui->numberObjectsLabel->setText(QString("%1").arg(results.size()));

        // Draw bounding boxes, but we need a 4-channel image with alpha
        if(image_detection.channels() == 1){
            cv::cvtColor(image_detection, image_detection, cv::COLOR_GRAY2RGBA);
        }else if(image_detection.channels() == 3){
            cv::cvtColor(image_detection, image_detection, cv::COLOR_BGR2RGBA); //Assumes image from camera is BGR
        }
        drawBoundingBoxes(image_detection, results, 1./scale_factor_x, 1./scale_factor_y);

        cv::cvtColor(image_detection, image_detection, cv::COLOR_RGBA2BGRA);

        object_detection_display->updateView(image_detection);
        QCoreApplication::processEvents();
    }

    this->detecting = false;
}

void SVTKWindow::drawBoundingBoxes(cv::Mat image, std::vector<BoundingBox> bboxes, double scale_x=1.0, double scale_y=1.0){

    for(auto &bbox : bboxes){

        if(!class_visible_map[bbox.classname])
            continue;

        cv::Point top_left(static_cast<int>(scale_x*bbox.rect.topLeft().x()),
                           static_cast<int>(scale_y*bbox.rect.topLeft().y()));
        cv::Point bottom_right(static_cast<int>(scale_x*bbox.rect.bottomRight().x()),
                               static_cast<int>(scale_y*bbox.rect.bottomRight().y()));
        int thickness = 2;

        QColor box_colour = class_colour_map[bbox.classname];
        int r=255, g=0, b=0, a=255; // default to red
        box_colour.getRgb(&r, &g, &b, &a);

        cv::Scalar font_colour(r, g, b, a);

        //Get the label for the class name and its confidence
        std::string label = cv::format("%s %.2f", bbox.classname.toStdString().c_str(), bbox.confidence);

        float fontScale = (float)image.size().height / 1200;
        float fontThickness = fontScale*2;

        //Display the label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_COMPLEX, fontScale, 1, &baseLine);

        cv::Point text_pos = top_left;
        text_pos.y = text_pos.y - (labelSize.height / 2);
        cv::putText(image, label, text_pos, cv::FONT_HERSHEY_COMPLEX, fontScale, font_colour, fontThickness);

        if(class_filled_map[bbox.classname]){
            cv::Scalar font_colour(r, g, b);
            thickness = -1;

            cv::Rect bbox_roi(top_left, bottom_right);
            cv::Mat subimage = image(bbox_roi);
            cv::Mat subimage_overlay = subimage.clone();
            subimage_overlay = cv::Scalar(r, g, b);

            // Overlay box
            double gamma = 0.0;
            double alpha = static_cast<double>(bounding_box_alpha) / 255.0;
            cv::addWeighted(subimage_overlay, alpha, subimage, 1-alpha, gamma, subimage);

            //image(roi) = subimage;
        }else{
            cv::rectangle(image, top_left, bottom_right, font_colour, thickness);
        }
    }
}


void SVTKWindow::pointCloudInit() {
    QProgressDialog progressPCI("Initialising display...", "", 0, 100, this);
    progressPCI.setWindowTitle("SVT");
    progressPCI.setWindowModality(Qt::WindowModal);
    progressPCI.setCancelButton(nullptr);
    progressPCI.setMinimumDuration(0);
    progressPCI.setValue(10);
    QCoreApplication::processEvents();

    cloud_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtk_widget = new QVTKWidget();

    ui->visualiserTab->layout()->addWidget(vtk_widget);
    vtk_widget->setSizePolicy(QSizePolicy::MinimumExpanding,
                              QSizePolicy::MinimumExpanding);

    progressPCI.setValue(20);
    QCoreApplication::processEvents();

    vtk_widget->SetRenderWindow(cloud_viewer->getRenderWindow());
    cloud_viewer->setupInteractor(vtk_widget->GetInteractor(),
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

void SVTKWindow::autoUpdatePointCloudBounds(){
    double min_depth = 0;
    double max_depth = 5;
    if (autoZ){
        min_depth = disparity_view->getMinDepth();
        max_depth = disparity_view->getMaxDepth();

        if (min_depth < 0){
            min_depth = 0;
        }
        if (max_depth < 0){
            max_depth = 0;
        }
        if (max_depth > 20){
            max_depth = 20;
        }

        ui->minZSpinBox->setValue(min_depth);
        ui->maxZSpinBox->setValue(max_depth);
    }
}

void SVTKWindow::updateCloud() {

    cloud = stereo_cam->getPointCloud();

    if(!cloud.get()){
        first_cloud = true;
    }

    if (!cloud->empty()) {
        // Initial point cloud load

        if (!cloud_viewer->updatePointCloud(cloud, "cloud")) {
            cloud_viewer->addPointCloud(cloud, "cloud");
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
    autoUpdatePointCloudBounds();
    vtk_widget->update();
}

void SVTKWindow::stereoCameraInitWindow(void){
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

    if (static_cast<int>(default_exposure) == -1 && static_cast<int>(default_iAutoExpose) == -1){
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

    if (static_cast<int>(default_exposure) != -1){
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
        if (stereo_cam->hasTriggerFPSControl()){
            ui->fpsSpinBox->setEnabled(true);
        } else {
            if (default_trigger){
                ui->fpsSpinBox->setEnabled(false);
            } else {
                ui->fpsSpinBox->setEnabled(true);
            }
        }
        /*
        AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
        if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
            ui->fpsSpinBox->setEnabled(true);
        } else {
            if (default_trigger){
                ui->fpsSpinBox->setEnabled(false);
            } else {
                ui->fpsSpinBox->setEnabled(true);
            }
        }
        */
    } else {
        default_trigger = false;
        ui->enabledTriggeredCheckbox->setVisible(false);
    }
    if (default_fps != -1){
        ui->fpsSpinBox->setEnabled(true);
        ui->fpsSpinBox->setVisible(true);
    } else {
        default_fps = 5;
        ui->fpsSpinBox->setVisible(false);
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
    int min_fps = 1;
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

    int max_packetSize = 9000;
    int min_packetSize = -1;
    int step_packetSize = 4;

    AbstractStereoCamera::StereoCameraSerialInfo cam_info = stereo_cam->getCameraSerialInfo();
    if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_DEIMOS || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TARA){
        min_fps = 30;
        step_fps = 30;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE){
        max_binning = 4;
        min_binning = 1;
        step_binning = 1;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB){
        max_binning = 4;
        min_binning = 1;
        step_binning = 1;
    } else if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_TIS_USB){
        max_gain = 48;
        min_gain = 0;
        step_gain = 1;
    } else  if (cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA || cam_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_VIMBA_USB){
        min_fps = 1;
        max_fps = 100;
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

    if (!stereo_cam->hasTriggerFPSControl()){
        connect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), ui->fpsSpinBox, SLOT(setDisabled(bool)));
    }
}

void SVTKWindow::stereoCameraInitConnections(void) {

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

    connect(ui->checkBoxVideoUseRectified, SIGNAL(toggled(bool)), stereo_cam, SLOT(enableCaptureRectifedVideo(bool)));
    connect(ui->comboBoxVideoSource, SIGNAL(currentIndexChanged(int)), stereo_cam, SLOT(setVideoSource(int)));

    connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDisplay()));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), left_view, SLOT(setSize(int, int, int)));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), left_matcher_view, SLOT(setSize(int, int, int)));
    connect(stereo_cam, SIGNAL(update_size(int, int, int)), right_view, SLOT(setSize(int, int, int)));
    connect(ui->checkBoxShowEpipolar, SIGNAL(clicked(bool)), left_view, SLOT(enableEpipolarLines(bool)));
    connect(ui->checkBoxShowEpipolar, SIGNAL(clicked(bool)), right_view, SLOT(enableEpipolarLines(bool)));

    connect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), this,
            SLOT(enableTrigger(bool)));

    connect(stereo_cam, SIGNAL(error(int)), this, SLOT(error(int)));

    connect(stereo_cam, SIGNAL(frametime(qint64)), this, SLOT(updateFrameTime(qint64)));
    connect(stereo_cam, SIGNAL(matchtime(qint64)), this, SLOT(updateMatchTime(qint64)));
    connect(stereo_cam, SIGNAL(framecount(qint64)), this, SLOT(updateFrameCount(qint64)));
    connect(ui->saveButton, SIGNAL(clicked()), stereo_cam, SLOT(saveImageTimestamped()));
    connect(stereo_cam, SIGNAL(first_image_ready(bool)), ui->saveButton, SLOT(setEnabled(bool)));
    connect(disparity_view, SIGNAL(disparitySaveCheckChanged(bool)), stereo_cam, SLOT(enableSaveDisparity(bool)));
    connect(stereo_cam, SIGNAL(savedImage(bool)), this,
            SLOT(displaySaved(bool)));
    connect(ui->enableStereo, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableMatching(bool)));
    connect(ui->enableDetection, SIGNAL(clicked(bool)), this,
            SLOT(enableDetection(bool)));
    connect(ui->enableSharedMem, SIGNAL(clicked(bool)), this,
            SLOT(enableSharedMemory(bool)));
    connect(ui->toggleRectifyCheckBox, SIGNAL(clicked(bool)), this,
            SLOT(enableRectify(bool)));
    connect(ui->toggleSwapLeftRight, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableSwapLeftRight(bool)));
    connect(ui->toggleCalibrationDownsample, SIGNAL(clicked(bool)), stereo_cam,
            SLOT(enableDownsampleCalibration(bool)));
    connect(ui->spinBoxImageDownsample, SIGNAL(valueChanged(int)), stereo_cam,
            SLOT(setDownsampleFactor(int)));
    connect(ui->spinBoxImageDownsample, SIGNAL(valueChanged(int)), disparity_view,
            SLOT(setDownsampleFactor(int)));
    connect(stereo_cam, SIGNAL(matched()), disparity_view,
            SLOT(updateDisparityAsync(void)));
    connect(ui->autoExposeCheck, SIGNAL(clicked(bool)), this, SLOT(enableAutoExpose(bool)));
    connect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), this, SLOT(enableAutoGain(bool)));
    connect(ui->enableHDRCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableHDR(bool))); //Deimos only

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
    connect(ui->comboBoxPointTexture, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePointTexture(int)));

    /* Detection */
    connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDetection()));

    updatePointTexture(ui->comboBoxPointTexture->currentIndex());

    enableWindow();
    toggleCameraActiveSettings(true);
    toggleCameraPassiveSettings(true);
    stopDeviceListTimer();
}

void SVTKWindow::updateSharedMemory(){
    if (shared_memory_enabled){
        //qDebug() << "Getting source image...";
        bool useRectified = ui->checkBoxSharedMemUseRectified->isChecked();
        cv::Mat left_t, right_t;
        cv::Mat wImage, Q;
        double downsample_rate = 1.0f/(double)ui->spinBoxSharedMemDownsample->value();
        if (ui->comboBoxSharedMemSource->currentText() == "Stereo"){
            if (useRectified){
                stereo_cam->getLeftImage(left_t);
                stereo_cam->getRightImage(right_t);
            } else {
                stereo_cam->getLeftRawImage(left_t);
                stereo_cam->getRightRawImage(right_t);
            }
            // convert image to rgb
            if (left_t.type() == CV_8UC1){
                cvtColor(left_t,left_t,COLOR_GRAY2RGB);
            } else if (left_t.type() == CV_8UC3){
                cvtColor(left_t,left_t,COLOR_BGR2RGB);
            } else {
                std::cerr << "Invalid image type for shared memory" << std::endl;
            }
            if (right_t.type() == CV_8UC1){
                cvtColor(right_t,right_t,COLOR_GRAY2RGB);
            } else if (right_t.type() == CV_8UC3){
                cvtColor(right_t,right_t,COLOR_BGR2RGB);
            } else {
                std::cerr << "Invalid image type for shared memory" << std::endl;
            }
            cv::hconcat(left_t, right_t, image_stream);
            cv::resize(image_stream, image_stream, cv::Size(), downsample_rate, downsample_rate);
        } else if (ui->comboBoxSharedMemSource->currentText() == "RGBD"){
            if (stereo_cam->isMatching()){
                cv::Mat color, Q, depth, depth_z, depth_split[3];
                stereo_cam->getLeftMatchImage(color);
                stereo_cam->getDepth(depth);
                stereo_cam->getQ(Q);
                // extract Z only channel from depth (xyz) image
                cv::split(depth, depth_split);
                depth_z = depth_split[2];
                // get horizontal fov from Q matrix
                float hfov = CVSupport::getHFOVFromQ(Q);
                // embed horizontal fov in top left pixel of z only depth image to simplify reconstruction
                depth_z.at<float>(0,0) = (float)hfov;
                if (!depth_z.empty() && !color.empty()){
                    // convert image to rgb
                    if (color.type() == CV_8UC1){
                        cvtColor(color,color,CV_GRAY2RGB);
                    } else if (color.type() == CV_8UC3){
                        cvtColor(color,color,CV_BGR2RGB);
                    } else {
                        std::cerr << "Invalid image type for shared memory" << std::endl;
                    }
                    // Downsample images based on parameters
                    cv::resize(depth_z, depth_z, cv::Size(), downsample_rate, downsample_rate, cv::INTER_NEAREST_EXACT);
                    cv::resize(color, color, cv::Size(), downsample_rate, downsample_rate);
                    // Create RGBD image from color image and z only depth image
                    if (ui->checkBoxSharedMem16bit->isChecked()){
                        image_stream = CVSupport::createRGBD16(color,depth_z,6553.0,false);
                    } else {
                        image_stream = CVSupport::createRGBD32(color,depth_z);
                    }
                } else {
                    image_stream = cv::Mat();
                    std::cerr << "Depth image or camera image is empty." << std::endl;
                }
            } else {
                std::cerr << "Missing disparity for sending rgbd in shared memory." << std::endl;
                image_stream = cv::Mat();
            }
        } else {
            std::cerr << "Invalid shared memory source: " << ui->comboBoxSharedMemSource->currentText().toStdString();
        }

        if(image_stream.empty()){
            qDebug() << "Empty image passed to shared memory";
            return;
        }

        sharedMemoryInst->write_threaded(image_stream);
    }
}

void SVTKWindow::stereoCameraRelease(void) {
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
    ui->enableDetection->setChecked(false);
    ui->enableSharedMem->setChecked(false);
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
    //ui->comboBoxPointTexture->setCurrentIndex(0);

    if (cameras_connected) {
        cameras_connected = false;

        ui->gridLayoutCameraList->setEnabled(false);
        QCoreApplication::processEvents();

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
        this->enableDetection(false);
        this->enableSharedMemory(false);
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
                SLOT(setPacketSize(int)));

        disconnect(ui->checkBoxVideoUseRectified, SIGNAL(toggled(bool)), stereo_cam, SLOT(enableCaptureRectifedVideo(bool)));
        disconnect(ui->comboBoxVideoSource, SIGNAL(indexChanged(int)), stereo_cam, SLOT(setVideoSource(int)));

        disconnect(ui->enabledTriggeredCheckbox, SIGNAL(clicked(bool)), this,
                   SLOT(enableTrigger(bool)));

        // disconnect fps connection used in video
        disconnect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), stereo_cam, SLOT(setFPS(int)));

        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDisplay()));
        disconnect(stereo_cam, SIGNAL(matched()), disparity_view,
                   SLOT(updateDisparityAsync(void)));
        disconnect(stereo_cam, SIGNAL(frametime(qint64)), this, SLOT(updateFrameTime(qint64)));
        disconnect(stereo_cam, SIGNAL(matchtime(qint64)), this, SLOT(updateMatchTime(qint64)));
        disconnect(stereo_cam, SIGNAL(framecount(qint64)), this, SLOT(updateFrameCount(qint64)));
        disconnect(ui->saveButton, SIGNAL(clicked()), stereo_cam,
                   SLOT(saveImageTimestamped()));
        disconnect(disparity_view, SIGNAL(disparitySaveCheckChanged(bool)), stereo_cam, SLOT(enableSaveDisparity(bool)));
        disconnect(stereo_cam, SIGNAL(savedImage(bool)), this,
                   SLOT(displaySaved(bool)));
        disconnect(ui->enableStereo, SIGNAL(clicked(bool)), stereo_cam,
                   SLOT(enableMatching(bool)));
        disconnect(ui->enableDetection, SIGNAL(clicked(bool)), this,
                   SLOT(enableDetection(bool)));
        disconnect(ui->enableSharedMem, SIGNAL(clicked(bool)), this,
                   SLOT(enableSharedMemory(bool)));
        disconnect(ui->toggleRectifyCheckBox, SIGNAL(clicked(bool)), this,
                   SLOT(enableRectify(bool)));
        disconnect(ui->toggleSwapLeftRight, SIGNAL(clicked(bool)), stereo_cam,
                   SLOT(enableSwapLeftRight(bool)));
        disconnect(ui->toggleCalibrationDownsample, SIGNAL(clicked(bool)), stereo_cam,
                SLOT(enableDownsampleCalibration(bool)));
        disconnect(ui->spinBoxImageDownsample, SIGNAL(valueChanged(int)), stereo_cam,
                SLOT(setDownsampleFactor(int)));
        disconnect(ui->spinBoxImageDownsample, SIGNAL(valueChanged(int)), disparity_view,
                SLOT(setDownsampleFactor(int)));
        disconnect(ui->autoExposeCheck, SIGNAL(clicked(bool)), this, SLOT(enableAutoExpose(bool)));
        disconnect(ui->autoGainCheckBox, SIGNAL(clicked(bool)), this, SLOT(enableAutoGain(bool)));
        disconnect(ui->enableHDRCheckbox, SIGNAL(clicked(bool)), stereo_cam, SLOT(enableHDR(bool))); //Deimos only

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
        disconnect(ui->comboBoxPointTexture, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePointTexture(int)));

        /* Detection */
        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(updateDetection()));

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

        //refreshCameraListThreaded();
        startDeviceListTimer();
    }
    resetStatusBar();
}

int SVTKWindow::openCamera(AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info){
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
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TARA){
        current_camera_settings = default_tara_init_settings;
        StereoCameraTara* stereo_cam_tara = new StereoCameraTara(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_tara->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_tara);
        qDebug() << "Connecting to Tara system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
        if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE){
            current_camera_settings = default_basler_gige_init_settings;
        } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
            current_camera_settings = default_basler_usb_init_settings;
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
        qDebug() << "Connecting to TIS system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
#ifdef WITH_VIMBA
       current_camera_settings = default_vimba_init_settings;
       StereoCameraVimba * stereo_cam_vimba = new StereoCameraVimba(camera_serial_info,current_camera_settings);
       cameras_connected = stereo_cam_vimba->openCamera();
       stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_vimba);
       qDebug() << "Connecting to Titania system";
#endif
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB){
        if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE){
            current_camera_settings = default_phobos_basler_gige_init_settings;
        } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB){
            current_camera_settings = default_phobos_basler_usb_init_settings;
        }
        StereoCameraPhobosBasler* stereo_cam_phobos_basler = new StereoCameraPhobosBasler(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_phobos_basler->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_phobos_basler);
        qDebug() << "Connecting to Phobos system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_TIS_USB){
        current_camera_settings = default_phobos_tis_usb_init_settings;
        StereoCameraPhobosTIS* stereo_cam_phobos_tis = new StereoCameraPhobosTIS(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_phobos_tis->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_phobos_tis);
        qDebug() << "Connecting to Phobos system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB){
        if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE){
            current_camera_settings = default_titania_basler_gige_init_settings;
        } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB){
            current_camera_settings = default_titania_basler_usb_init_settings;
        }
        StereoCameraTitaniaBasler* stereo_cam_titania_basler = new StereoCameraTitaniaBasler(camera_serial_info,current_camera_settings);
        cameras_connected = stereo_cam_titania_basler->openCamera();
        stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_titania_basler);
        qDebug() << "Connecting to Titania system";
    } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_VIMBA_USB){
#ifdef WITH_VIMBA
       current_camera_settings = default_titania_vimba_usb_init_settings;
       StereoCameraTitaniaVimba * stereo_cam_titania_vimba = new StereoCameraTitaniaVimba(camera_serial_info,current_camera_settings);
       cameras_connected = stereo_cam_titania_vimba->openCamera();
       stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_titania_vimba);
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

void SVTKWindow::refreshCameraListThreaded(){
    qfuture_refreshcameralist = QtConcurrent::run(this, &SVTKWindow::refreshCameraList);
}

void SVTKWindow::refreshCameraList(){
    if (!cameras_connected){
        ui->gridLayoutCameraList->setEnabled(false);
        QCoreApplication::processEvents();
        QElapsedTimer task_timer;
        task_timer.start();
        current_camera_serial_info_list = StereoCameraSupport::getStereoDeviceList(tisgrabber,pylonTlFactory);
        qDebug() << "Time to get device list: " << task_timer.elapsed();
        if (!cameras_connected){
            emit cameraListUpdated();
            //refreshCameraListGUI();
        }
    }
}

void SVTKWindow::refreshCameraListGUI(){
    //triggered by 'cameraListUpdated()' signal
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> camera_serial_info_list = current_camera_serial_info_list;
    camera_button_signal_mapper_list = new vector<QSignalMapper*>();


    QPixmap pixmapDeimos(":/mainwindow/images/deimos_square_50.png");
    QPixmap pixmapPhobos(":/mainwindow/images/phobos_square_50.png");
    QPixmap pixmapTitania(":/mainwindow/images/titania_square_50.png");
    // TODO: Set this to tara image when the problem of deimos' showing as tara when the hid is malfunctioning is fixed
    // This is currently set to a generic 3D camera so that the inner tara for deimos' is not shown to customers.
    QPixmap pixmapTara(":/mainwindow/images/camera_square_50.png");
    QPixmap pixmapCamera(":/mainwindow/images/camera_square_50.png");

    //Clear layout list
    deviceListButtons.clear();
    QLayoutItem *item;
    while ((item = ui->gridLayoutCameraList->takeAt(0)) != nullptr) {
        if (item->layout()){
            QLayoutItem *item2;
            while ((item2 = item->layout()->takeAt(0)) != nullptr) {
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
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TARA){
                camera_type = "Tara";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapTara;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_BASLER_USB){
                camera_type = "Basler";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapCamera;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_BASLER_USB || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_PHOBOS_TIS_USB){
                camera_type = "Phobos";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapPhobos;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_VIMBA){
                camera_type = "Vimba";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapCamera;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_GIGE || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_BASLER_USB || camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TITANIA_VIMBA_USB){
                camera_type = "Titania";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapTitania;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_TIS){
                camera_type = "TIS";
                camera_serial = camera_serial_info.i3dr_serial;
                camera_icon = pixmapCamera;
            } else if (camera_serial_info.camera_type == AbstractStereoCamera::CAMERA_TYPE_USB){
                camera_type = "GenericUSB";
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

    ui->gridLayoutCameraList->setEnabled(true);
    QCoreApplication::processEvents();
}

void SVTKWindow::cameraDeviceSelected(int index){
    stopDeviceListTimer();
    int button_index = index;
    if (static_cast<size_t>(button_index) < current_camera_serial_info_list.size()){
        // disconnect of camera if already connected
        AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = current_camera_serial_info_list.at(static_cast<unsigned long long>(button_index));
        int i = 0;
        if (openCamera(camera_serial_info) == CAMERA_CONNECTION_SUCCESS_EXIT_CODE){
            // disable all other buttons if camera open is successful
            for (std::vector<AbstractStereoCamera::StereoCameraSerialInfo>::iterator it = current_camera_serial_info_list.begin() ; it != current_camera_serial_info_list.end(); ++it){
                AbstractStereoCamera::StereoCameraSerialInfo camera_serial_info = *it;
                // Get button widget from grid layout
                QWidget *wSelectCamera = ui->gridLayoutCameraList->itemAtPosition(i+1,2)->widget();
                QPushButton *btnSelectCamera = qobject_cast<QPushButton*>(wSelectCamera);
                // Disconnect signal mapper
                QSignalMapper * mapper = camera_button_signal_mapper_list->at(static_cast<unsigned long long>(i));
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
                        while ((item2 = l1->takeAt(0)) != nullptr) {
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
            //refreshCameraListThreaded();
            startDeviceListTimer();
        }
    } else {
        //refreshCameraListThreaded();
        startDeviceListTimer();
    }
}

void SVTKWindow::stereoCameraInit() {
    if (cameras_connected) {
        save_directory = parameters->get_string("saveDir");
        calibration_directory = parameters->get_string("calDir");

        stereoCameraInitConnections();
        setMatcher(ui->matcherSelectBox->currentIndex());

        if (save_directory != "") {
            stereo_cam->setSavelocation(save_directory);
        }

        //if (calibration_directory != ""){
        //    setCalibrationFolder(calibration_directory);
        //}

        double focal =  stereo_cam->fx;
        double baseline = stereo_cam->baseline;
        disparity_view->setCalibration(stereo_cam->Q,baseline,focal);

        left_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);
        left_matcher_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);
        right_view->setSize(stereo_cam->getWidth(), stereo_cam->getHeight(), 1);

        ui->statusBar->showMessage("Freerunning.");
    }
}

void SVTKWindow::startDeviceListTimer() {
    // refresh device list every 5 seconds
    //TODO replace this with event driven system
    qDebug() << "Starting device list timer";
    if (!device_list_timer->isActive()){
        device_list_timer->stop();
        device_list_timer = new QTimer(this);
        refreshCameraListThreaded();
        device_list_timer->start(5000);
        QObject::connect(device_list_timer, SIGNAL(timeout()), this, SLOT(refreshCameraListThreaded()));
        qDebug() << "Device list timer started";
    } else {
        qDebug() << "Device list timer already active";
    }
    ui->btnRefreshCameras->setEnabled(true);
}

void SVTKWindow::stopDeviceListTimer() {
    QObject::disconnect(device_list_timer, SIGNAL(timeout()), this, SLOT(refreshCameraListThreaded()));
    device_list_timer->stop();
    ui->btnRefreshCameras->setEnabled(false);
}

void SVTKWindow::stereoImageLoad(void) {
    QMessageBox msg;

    // Select left and right image pair from custom dialog
    load_stereo_image_pair_dialog = new LoadStereoImagePairDialog(this);
    load_stereo_image_pair_dialog->setModal(true);
    int return_code = load_stereo_image_pair_dialog->exec();

    if (return_code == QDialog::Accepted) {
        if (load_stereo_image_pair_dialog->isFilepathsValid()) {
            stereoCameraRelease();
            //stopDeviceListTimer();

            AbstractStereoCamera::StereoCameraSerialInfo scis;
            // Store left and right image filename in serial strings
            scis.left_camera_serial = load_stereo_image_pair_dialog->getLeftImageFilepath();
            scis.right_camera_serial = load_stereo_image_pair_dialog->getRightImageFilepath();

            current_camera_settings = default_video_init_settings;

            cam_thread = new QThread;
            StereoCameraFromImage* stereo_cam_image = new StereoCameraFromImage(scis,current_camera_settings);
            cameras_connected = stereo_cam_image->openCamera();
            stereo_cam = static_cast<AbstractStereoCamera*>(stereo_cam_image);
            if (cameras_connected){
                stereo_cam->assignThread(cam_thread);
                ui->frameCountSlider->setEnabled(true);
                connect(ui->fpsSpinBox, SIGNAL(valueChanged(int)), stereo_cam, SLOT(setFPS(int)));
                stereoCameraInit();
                stereoCameraInitWindow();
                //ui->toggleVideoButton->setDisabled(true);

                // Ask user if to load rectified frames
                QMessageBox msgBox;
                msgBox.setText(tr("Was image captured with rectified images? (This is asked to avoid double rectification)"));
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
                msg.setText("Failed to open images.");
                msg.exec();
                ui->statusBar->showMessage("Disconnected.");
            }
        } else {
            msg.setText("Failed to load image files.");
            msg.exec();
            ui->statusBar->showMessage("Disconnected.");
        }
    }
}

void SVTKWindow::videoStreamLoad(void) {
    QMessageBox msg;

    QString fname = QFileDialog::getOpenFileName(
                this, tr("Open Stereo Video"), "/home", tr("Videos (*.avi *.mp4)"));
    if (fname != "") {
        stereoCameraRelease();
        //stopDeviceListTimer();

        AbstractStereoCamera::StereoCameraSerialInfo scis;
        scis.left_camera_serial = fname.toStdString(); //store video filename in left serial string

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

void SVTKWindow::enableVideoCapture(bool enable){
    if (enable){
        int vid_fps = current_fps;
        if (current_fps == 0){
            vid_fps = measured_fps;
        }
        AbstractStereoCamera::VideoSource vid_src = (AbstractStereoCamera::VideoSource)ui->comboBoxVideoSource->currentIndex();
        int vid_codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        if (vid_src == AbstractStereoCamera::VIDEO_SRC_STEREO){ // requires lossless for re-loading of stereo videos without loss
            //TODO replace this with lossless compression codec as currently creates very large files (raw uncompressed)
            vid_codec = cv::VideoWriter::fourcc('R', 'G', 'B', 'A');
        }
        stereo_cam->setVideoStreamParams("",vid_fps,vid_codec,true,vid_src);

        if (vid_src == AbstractStereoCamera::VIDEO_SRC_STEREO ||
                vid_src == AbstractStereoCamera::VIDEO_SRC_LEFT ||
                vid_src == AbstractStereoCamera::VIDEO_SRC_RIGHT){

            // Ask user if to record rectified frames
            QMessageBox msgBox;
            msgBox.setText(tr("Record recitifed frames if avaiable?"));
            QAbstractButton* pButtonRect = msgBox.addButton(tr("Yes"), QMessageBox::YesRole);
            QAbstractButton* pButtonNonRect = msgBox.addButton(tr("No"), QMessageBox::NoRole);

            msgBox.exec();

            if (msgBox.clickedButton()==pButtonRect) {
                ui->checkBoxVideoUseRectified->setChecked(true);
                stereo_cam->enableCaptureRectifedVideo(true);
            } else if (msgBox.clickedButton()==pButtonNonRect) {
                ui->checkBoxVideoUseRectified->setChecked(false);
                stereo_cam->enableCaptureRectifedVideo(false);
            }
        }

        ui->toggleVideoButton->setIcon(awesome->icon(fa::stop, icon_options));
        ui->checkBoxVideoUseRectified->setEnabled(false);
        ui->comboBoxVideoSource->setEnabled(false);
    } else {
        ui->toggleVideoButton->setIcon(awesome->icon(fa::videocamera, icon_options));
        ui->checkBoxVideoUseRectified->setEnabled(true);
        ui->comboBoxVideoSource->setEnabled(true);
    }

    stereo_cam->enableVideoStream(enable);
}

void SVTKWindow::startCalibrationFromImages(void) {
    calibration_images_dialog = new CalibrateFromImagesDialog(this);
    connect(calibration_images_dialog, SIGNAL(run_calibration()), this, SLOT(runCalibrationFromImages()));
    calibration_images_dialog->move(100, 100);
    calibration_images_dialog->show();

    calibration_from_images_dialog_used = true;
}

void SVTKWindow::runCalibrationFromImages(void){

    qDebug() << "Beginning calibration from images";

    if(calibration_images_dialog == nullptr) return;
    qDebug() << "Getting parameters";

    //calibration_images_dialog->setLeftImages();
    //calibration_images_dialog->setRightImages();
    int cols = calibration_images_dialog->getPatternCols();
    int rows = calibration_images_dialog->getPatternRows();
    double square_size_mm = calibration_images_dialog->getSquareSizeMm();
//    auto left_images = calibration_images_dialog->getLeftImages();
//    auto right_images = calibration_images_dialog->getRightImages();
    QList<std::string> left_image_paths = calibration_images_dialog->getLeftImagePaths();
    QList<std::string> right_image_paths = calibration_images_dialog->getRightImagePaths();
    bool save_ros = calibration_images_dialog->getSaveROS();

    calibration_images_dialog->close();

    // Load images from paths
    QProgressDialog progressImages("Loading images...", "", 0, 100, this);
    progressImages.setWindowTitle("SVT");
    progressImages.setWindowModality(Qt::WindowModal);
    progressImages.setCancelButton(nullptr);
    progressImages.setMinimumDuration(0);
    progressImages.setValue(0);
    progressImages.setMaximum(0);
    progressImages.setMinimum(0);
    progressImages.setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    QList<cv::Mat> left_images;
    foreach (std::string image_path, left_image_paths){
        cv::Mat im = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        left_images.append(im);
        QCoreApplication::processEvents();
    }
    QList<cv::Mat> right_images;
    foreach (std::string image_path, right_image_paths){
        cv::Mat im = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        right_images.append(im);
        QCoreApplication::processEvents();
    }

    progressImages.close();

    calibrator = new StereoCalibrate(this, nullptr);
    cv::Size pattern(cols, rows);

    stopDeviceListTimer();

    connect(calibrator, &StereoCalibrate::doneCalibration, this, &SVTKWindow::doneCalibration);

    calibrator->setOutputPath(calibration_images_dialog->getOutputPath());
    calibrator->setPattern(pattern, square_size_mm);
    calibrator->setImages(left_images, right_images);
    calibrator->setSaveROS(save_ros);
    calibrator->jointCalibration();
    //StereoCalibrate::CalibrationStatus stat = calibrator->jointCalibration();
    //doneCalibration(stat);
}

void SVTKWindow::startAutoCalibration(void) {
    enableRectify(false);
    ui->enableStereo->setChecked(false);
    stereo_cam->enableMatching(false);

    calibration_dialog = new CalibrationDialog(stereo_cam);
    connect(calibration_dialog, SIGNAL(startCalibration()), this, SLOT(runAutoCalibration()));
    calibration_dialog->move(100, 100);
    calibration_dialog->show();

    calibration_dialog_used = true;
}

void SVTKWindow::runAutoCalibration(void){
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
    calibration_dialog = nullptr;

    calibrator = new StereoCalibrate(this, nullptr);
    cv::Size pattern(cols, rows);

    calibrator->setOutputPath(calibration_dialog->getOutputPath());
    calibrator->setPattern(pattern, square_size_m);
    calibrator->setImages(left_images, right_images);
    calibrator->setSaveROS(save_ros);
    calibrator->jointCalibration();
    //StereoCalibrate::CalibrationStatus stat = calibrator->jointCalibration();
    //doneCalibration(stat);
}

void SVTKWindow::doneCalibration(StereoCalibrate::CalibrationStatus success) {
    //connect(stereo_cam, SIGNAL(acquired()), this, SLOT(updateDisplay())); TODO check this

       //TODO fix this for auto calibration
//    if(calibration_dialog != nullptr){
//        disconnect(calibration_dialog, SIGNAL(stopCalibration()), calibrator,
//                   SLOT(abortCalibration()));
//    }
    disconnect(calibrator, SIGNAL(doneCalibration(StereoCalibrate::CalibrationStatus)), this,
               SLOT(doneCalibration(StereoCalibrate::CalibrationStatus)));
    startDeviceListTimer();

    QMessageBox alert;
    if (success == StereoCalibrate::CalibrationStatus::SUCCESS) {
        alert.setText(QString("Written calibration files to: %1").arg(calibrator->output_folder.absolutePath()));
    } else if (success == StereoCalibrate::CalibrationStatus::FAILED) {
        alert.setText("Stereo camera calibration failed.");
    } else if (success == StereoCalibrate::CalibrationStatus::ABORTED) {
        alert.setText("Stereo camera calibration aborted.");
    } else if (success == StereoCalibrate::CalibrationStatus::INVALID_NUM_OF_IMAGES) {
        alert.setText("Invalid number of images provided for calibration. Minimum 5.");
    } else if (success == StereoCalibrate::CalibrationStatus::INVALID_NUM_VALID_IMAGES) {
        alert.setText("Stereo camera calibration failed due to invalid number of image detected with valid checkerboard.");
    } else {
        alert.setText("Stereo camera calibration failed due to unknown error.");
    }
    alert.exec();
    //stereo_cam->enableCapture(true);
}

void SVTKWindow::setMatcher(int index) {
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

void SVTKWindow::setupMatchers(void) {
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
    ui->matcherSelectBox->insertItem(2, "I3DRSGM");
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

          std::string hostname, hostid;
          MatcherI3DRSGM::getHostInfo(hostname,hostid);

          QString machineinfo = QString(std::string("Hostname: " + hostname + "<br>HostID: " + hostid).c_str());

          QMessageBox msg;
          msg.setText("No license found for I3DRSGM. <br>"
                      "You will only be able to use OpenSource matchers from OpenCV. <br>"
                      "Contact info@i3drobotics.com with the following details for a license. <br><br>"
                      + machineinfo);
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

void SVTKWindow::setCalibrationFolder(QString dir) {
    if(dir == ""){
        dir = QFileDialog::getExistingDirectory(
                    this, tr("Open Calibration Folder"), "/home",
                    QFileDialog::DontResolveSymlinks);
    }

    if(dir == "") return;

    bool reset_capture = false;
    if (stereo_cam){
        if (stereo_cam->isCapturing()){
            reset_capture = true;
            stereo_cam->enableCapture(false);
        }
    }

    // ask user for calibration type (xml or yaml)
    QMessageBox msgBox;
    msgBox.setText(tr("Load XML or YAML calibration?"));
    QAbstractButton* pButtonXML = msgBox.addButton(tr("XML"), QMessageBox::YesRole);
    QAbstractButton* pButtonYAML = msgBox.addButton(tr("YAML"), QMessageBox::NoRole);

    msgBox.exec();

    QProgressDialog progressPCI("Loading calibration...", "", 0, 100, this);
    progressPCI.setWindowTitle("SVT");
    progressPCI.setWindowModality(Qt::WindowModal);
    progressPCI.setCancelButton(nullptr);
    progressPCI.setMinimumDuration(0);
    progressPCI.setValue(0);
    progressPCI.setMaximum(0);
    progressPCI.setMinimum(0);
    progressPCI.setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
    QCoreApplication::processEvents();

    bool res;
    if (msgBox.clickedButton()==pButtonXML) {
        res = stereo_cam->loadCalibration(dir,AbstractStereoCamera::CALIBRATION_TYPE_XML);
    } else if (msgBox.clickedButton()==pButtonYAML) {
        res = stereo_cam->loadCalibration(dir,AbstractStereoCamera::CALIBRATION_TYPE_YAML);
    }

    progressPCI.close();

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

    if (reset_capture){
        stereo_cam->enableCapture(true);
    }
}

void SVTKWindow::statusMessageTimeout(void) {
    if (stereo_cam->isCapturing()) {
        ui->statusBar->showMessage("Capturing.");
    } else {
        ui->statusBar->showMessage("Paused.");
    }
}

void SVTKWindow::singleShotClicked(void) {
    enableCapture(false);
    stereo_cam->captureSingle();
}

void SVTKWindow::saveSingle(void) {
    stereo_cam->saveImageTimestamped();
}

void SVTKWindow::enableRectify(bool enable) {   
    ui->toggleRectifyCheckBox->setChecked(enable);
    stereo_cam->enableRectify(enable);
}

void SVTKWindow::enableCapture(bool enable) {
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

void SVTKWindow::displaySaved(bool success) {
    QString msg;
    if (success){
        msg = QString("Saved to: %1").arg(stereo_cam->getFileSaveDirectory());
    } else {
        msg = QString("Failed to save: %1").arg(stereo_cam->getFileSaveDirectory());
    }
    ui->statusBar->showMessage(msg);
    status_bar_timer->setSingleShot(true);
    status_bar_timer->start(1500);
}

void SVTKWindow::updateDisplay(void) {
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

void SVTKWindow::setSaveDirectory(QString dir) {

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
    }
}

void SVTKWindow::enableAutoGain(bool enable){
    stereo_cam->enableAutoGain(enable);
    if (!enable){
        int gain_val = ui->gainSpinBox->value();
        stereo_cam->setGain(gain_val);
    }
}

void SVTKWindow::enableAutoExpose(bool enable){
    stereo_cam->enableAutoExposure(enable);
    if (!enable){
        double exposure_val = ui->exposureSpinBox->value();
        stereo_cam->setExposure(exposure_val);
    }
}

void SVTKWindow::enableTrigger(bool enable){
    ui->enabledTriggeredCheckbox->setChecked(enable);
    stereo_cam->enableTrigger(enable);
    setFPS(ui->fpsSpinBox->value());
}

void SVTKWindow::setFPS(int fps){
    stereo_cam->setFPS(fps);
    current_fps = fps;
}

void SVTKWindow::setPacketSize(int packetSize){
    QProgressDialog progressPacketSize("Updating packet size...", "", 0, 100, this);
    progressPacketSize.setWindowTitle("SVT");
    progressPacketSize.setWindowModality(Qt::WindowModal);
    progressPacketSize.setCancelButton(nullptr);
    progressPacketSize.setMinimumDuration(0);
    progressPacketSize.setValue(30);
    QCoreApplication::processEvents();

    stereo_cam->setPacketSize(packetSize);

    progressPacketSize.setValue(100);
    progressPacketSize.close();
}

void SVTKWindow::setBinning(int binning){
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

void SVTKWindow::enableBinning(bool enable){
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

void SVTKWindow::toggleCameraActiveSettings(bool enable){
    ui->exposureSpinBox->setEnabled(enable);
    ui->autoExposeCheck->setEnabled(enable);
    ui->gainSpinBox->setEnabled(enable);
    ui->autoGainCheckBox->setEnabled(enable);
    ui->enableHDRCheckbox->setEnabled(enable);
    ui->btnLoadCalibration->setEnabled(enable);
    ui->actionLoad_Calibration->setEnabled(enable);
}

void SVTKWindow::toggleCameraPassiveSettings(bool enable){
    ui->fpsSpinBox->setEnabled(enable);
    ui->enabledTriggeredCheckbox->setEnabled(enable);
    ui->binningSpinBox->setEnabled(enable);
    ui->packetDelaySpinBox->setEnabled(enable);
    ui->packetSizeSpinBox->setEnabled(enable);
}

bool SVTKWindow::gigeWarning(int binning,int new_fps){
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

void SVTKWindow::updateFrameCount(qint64 count) {
    frame_counter->setText(QString("Frame count: %1").arg(count));
}

void SVTKWindow::updateMatchTime(qint64 time) {
    float fps = 1000.0 / time;
    measured_match_fps = fps;

    match_fps_measure_total+= measured_match_fps;
    match_fps_measure_count++;

    float average_fps = (float)match_fps_measure_total / (float)match_fps_measure_count;

    if (match_fps_measure_count > 3){
        match_fps_measure_total = average_fps;
        match_fps_measure_count = 1;
    }

    QString fps_number = QString::number(average_fps, 'G', 3);
    match_fps_counter->setText(QString("Match FPS: ") + fps_number);
}

void SVTKWindow::updateFrameTime(qint64 time) {
    float fps = 1000.0 / time;
    measured_fps = fps;

    fps_measure_total+= measured_fps;
    fps_measure_count++;

    float average_fps = (float)fps_measure_total / (float)fps_measure_count;

    if (fps_measure_count > 3){
        fps_measure_total = average_fps;
        fps_measure_count = 1;
    }

    QString fps_number = QString::number(average_fps, 'G', 3);
    fps_counter->setText(QString("Camera FPS: ") + fps_number);
}

void SVTKWindow::openHelp(){
    QString link = "https://i3drobotics.github.io/stereo-vision-toolkit/app/UserGuide.pdf";
    QDesktopServices::openUrl(QUrl(link));
}

void SVTKWindow::openAbout(){
    about_dialog->show();
    about_dialog_used = true;
}

void SVTKWindow::hideCameraSettings(bool hide){
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

void SVTKWindow::on_btnShowCameraSettings_clicked()
{
    hideCameraSettings(ui->widgetSideSettings->isVisible());
}

void SVTKWindow::on_btnHideCameraSettings_clicked()
{
    on_btnShowCameraSettings_clicked();
}

void SVTKWindow::enableMatching(bool enable){
    if (!enable){
        QString fps_number = QString::number(0, 'G', 3);
        match_fps_counter->setText(QString("Match FPS: ") + fps_number);
    }
}

void SVTKWindow::error(int error){
    std::string msgBoxMsg;
    bool msg_box_required = false;
    if (error == AbstractStereoCamera::RECTIFY_ERROR){
        enableRectify(false);
        ui->statusBar->showMessage("Failed to apply calibration.");
        msgBoxMsg = "Failed to apply calibration to images received. Calibration requires images of the same size as calibrated. Fix this an re-load calibration.";
        msg_box_required = true;
    } else if (error == AbstractStereoCamera::CAPTURE_ERROR){
        qDebug() << "Failed to capture image from cameras.";
        ui->statusBar->showMessage("Failed to capture image from cameras.");
    } else if (error == AbstractStereoCamera::MATCH_ERROR){
        qDebug() << "Failed to capture image from cameras.";
        ui->statusBar->showMessage("Failed to run stereo match on images.");
    } else if  (error == AbstractStereoCamera::CONNECT_ERROR){
        ui->statusBar->showMessage("Failed to connect to camera.");
        msgBoxMsg = "Failed to connect to camera.";
        msg_box_required = true;
    } else if (error == AbstractStereoCamera::LOST_FRAMES_ERROR){
        stereoCameraRelease();
        ui->statusBar->showMessage("Lost too many camera frames.");
        msgBoxMsg = "Lost too many camera frames, camera was closed.";
        msg_box_required = true;
    }
    if (msg_box_required){
        if (error_msgBox == NULL){
            error_msgBox = new QMessageBox();
            error_msgBox->setAttribute(Qt::WA_DeleteOnClose);
            error_msgBox->setText(msgBoxMsg.c_str());
            error_msgBox->exec();
            error_msgBox->setWindowTitle("Stereo Vision Toolkit");
        }
    }
}

void SVTKWindow::updatePointTexture(int index){
    if (cameras_connected){
        if(index == 0){
            stereo_cam->setPointCloudTexture(AbstractStereoCamera::POINT_CLOUD_TEXTURE_IMAGE);
        } else if (index == 1){
            stereo_cam->setPointCloudTexture(AbstractStereoCamera::POINT_CLOUD_TEXTURE_DEPTH);
        } else {
            qDebug() << "Invalid point texture index. MUST be 0: image or 1: depth";
        }
    }
}

void SVTKWindow::closeEvent(QCloseEvent *) {
    qDebug() << "Releasing cameras...";
    stereoCameraRelease();

    qDebug() << "Closing application...";
    stopDeviceListTimer();
    while(device_list_timer->isActive()){QCoreApplication::processEvents(QEventLoop::AllEvents);}

#ifdef WITH_VIMBA
    // Close the Vimba API here.
    VimbaSystem &system = VimbaSystem::GetInstance();
    qDebug() << "Shutting down VIMBA...";
    system.Shutdown();
#endif

    // Close external windows
    qDebug() << "Closing external windows...";
    about_dialog->close();
    if (calibration_dialog_used){
        calibration_dialog->close();
    }
    if (calibration_from_images_dialog_used){
        calibration_images_dialog->close();
    }

    qDebug() << "Shutting down Pylon...";
    Pylon::PylonTerminate();

    qDebug() << "Removing TIS grabber...";
    delete(tisgrabber);

    qDebug() << "Close event complete.";
}

SVTKWindow::~SVTKWindow() {
    qDebug() << "Cleaning up threads...";
    if (object_detector)
        delete(object_detector);
    qDebug() << "Closing ui...";
    delete ui;
    QApplication::quit();
}
