#include "qstereocamera.h"

QStereoCamera::QStereoCamera(QObject *parent) : QObject(parent) {
  this->connected = false;
}

void QStereoCamera::assignThread(QThread *thread) {
  this->moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

bool QStereoCamera::init(void) {

  auto scam = new StereoCameraOpenCV;
  qDebug() << scam->initCamera(0,1);
  scam->capture();
  scam->disconnectCamera();

  //qDebug() << GetListofDeviceseCon();

  int devid = 0;

  qDebug() << "Device ID" << devid;

  if (devid >= 0) {
    videocap = cv::VideoCapture(devid);
    if (videocap.isOpened()) {
      connected = true;
      bool res = true;
      res &= setFrameSize(752, 480);
      res &= setFrame16();

      if(!res){
          videocap.release();
          connected = false;
          return connected;
      }
    }
  }

  ptCloud =
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  //ptCloudFiltered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  setMatcher("Impact");

  return connected;
}



void QStereoCamera::updateMatchCudaBMParams(void) {
  cuda_bm_matcher =
      cv::cuda::createStereoBM(bm_maxDisparity - bm_minDisparity, bm_blockSize);
}

void QStereoCamera::updateMatchCudaBPParams(void) {
  cuda_bp_matcher = cv::cuda::createStereoBeliefPropagation(bm_maxDisparity -
                                                            bm_minDisparity);
}

void QStereoCamera::updateMatchBMParams(void) {
  bm_matcher = cv::StereoBM::create(bm_maxDisparity, bm_blockSize);

  bm_matcher->setBlockSize(bm_blockSize);
  bm_matcher->setMinDisparity(bm_minDisparity);
  bm_matcher->setNumDisparities(bm_maxDisparity - bm_minDisparity);
  bm_matcher->setTextureThreshold(bm_textureThreshold);
  bm_matcher->setUniquenessRatio(bm_uniquenessRatio);
  bm_matcher->setSpeckleWindowSize(bm_speckleWindowSize);
  bm_matcher->setSpeckleRange(bm_speckleRange);
  bm_matcher->setDisp12MaxDiff(bm_disp12MaxDiff);
  bm_matcher->setPreFilterType(CV_STEREO_BM_BASIC);
  bm_matcher->setPreFilterCap(bm_preFilterCap);
  bm_matcher->setPreFilterSize(bm_preFilterSize);

  right_matcher = cv::ximgproc::createRightMatcher(bm_matcher);
}

void QStereoCamera::updateMatchSGBMParams(void) {
  sgbm_matcher = cv::StereoSGBM::create(
      bm_minDisparity, bm_maxDisparity - bm_minDisparity, bm_blockSize);

  sgbm_matcher->setBlockSize(bm_blockSize);
  sgbm_matcher->setMinDisparity(bm_minDisparity);
  sgbm_matcher->setNumDisparities(bm_maxDisparity - bm_minDisparity);
  // left_sgbm_matcher->setTextureThreshold(bm_textureThreshold);
  sgbm_matcher->setUniquenessRatio(bm_uniquenessRatio);
  sgbm_matcher->setSpeckleWindowSize(bm_speckleWindowSize);
  sgbm_matcher->setSpeckleRange(bm_speckleRange);
  sgbm_matcher->setDisp12MaxDiff(bm_disp12MaxDiff);
  // left_sgbm_matcher->setPreFilterType(CV_STEREO_BM_BASIC);
  sgbm_matcher->setPreFilterCap(bm_preFilterCap);
  // left_sgbm_matcher->setPreFilterSize(bm_preFilterSize);

  right_matcher = cv::ximgproc::createRightMatcher(sgbm_matcher);
}

void QStereoCamera::updateFilterParams(void) {
  wls_filter->setLambda(filter_lambda);
  wls_filter->setLRCthresh(filter_consistency);
  wls_filter->setDepthDiscontinuityRadius(filter_discontinuity);
  wls_filter->setSigmaColor(filter_sigma);
}

bool QStereoCamera::loadRectificationMaps(QString src_l, QString src_r) {
  /* Load x and y rectification maps for the image */

  int flags = cv::FileStorage::READ;
  bool res = true;

  cv::FileStorage fs_l(src_l.toStdString(), flags);
  cv::FileStorage fs_r(src_r.toStdString(), flags);

  if (fs_l.isOpened()) {
    fs_l["x"] >> this->rectmapx_l;
    fs_l["y"] >> this->rectmapy_l;
  } else {
    res = false;
  }

  if (fs_r.isOpened()) {
    fs_r["x"] >> this->rectmapx_r;
    fs_r["y"] >> this->rectmapy_r;
  } else {
    res = false;
  }

  fs_l.release();
  fs_r.release();

  return res;
}

bool QStereoCamera::loadCalibration(QString directory){

    if(directory == "") return false;

    directory = QDir::cleanPath(directory);

    if (!loadRectificationMaps(directory + "/left_rectification.xml",
                                       directory + "/right_rectification.xml")) {
      validRectification = false;
      return false;
    }

    if (!loadCalibration(directory + "/left_calibration.xml",
                                     directory + "/right_calibration.xml",
                                     directory + "/stereo_calibration.xml")) {
      validCalibration = false;
      return false;
    }

    validCalibration = true;
    validRectification = true;

    enableRectify(true);
    enableStereo(true);

    return true;
}

bool QStereoCamera::loadCalibration(QString left_cal, QString right_cal,
                                    QString stereo_cal) {
  int flags = cv::FileStorage::READ;

  cv::FileStorage fs_l(left_cal.toStdString(), flags);

  if (fs_l.isOpened()) {
    fs_l["cameraMatrix"] >> l_camera_matrix;
    fs_l["distCoeffs"] >> l_dist_coeffs;
  } else {
    return false;
  }

  fs_l.release();

  cv::FileStorage fs_r(right_cal.toStdString(), flags);

  if (fs_r.isOpened()) {
    fs_r["cameraMatrix"] >> r_camera_matrix;
    fs_r["distCoeffs"] >> r_dist_coeffs;
  } else {
    return false;
  }

  fs_r.release();

  cv::FileStorage fs_s(stereo_cal.toStdString(), flags);

  if (fs_s.isOpened()) {
    fs_s["Q"] >> Q;
    Q.convertTo(Q, CV_32F);
  } else {
    return false;
  }

  fs_s.release();

  return true;
}

void QStereoCamera::reproject3D() {
  if (Q.empty() || stereo_processed.empty() || !validCalibration) {
    return;
  }

  if (cv::cuda::getCudaEnabledDeviceCount()) {
    d_disp.upload(stereo_processed);

    cv::cuda::GpuMat d_stereo_reprojected;

    cv::cuda::reprojectImageTo3D(d_disp, d_stereo_reprojected, Q);
    d_stereo_reprojected.download(stereo_reprojected);

  } else {
    cv::reprojectImageTo3D(stereo_processed, stereo_reprojected, Q, true);
  }
  cv::reprojectImageTo3D(stereo_processed, stereo_reprojected, Q, true);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB point;
  uint32_t rgb;
  uchar col;

  for (int i = 0; i < stereo_reprojected.rows; i++) {

      float* reconst_ptr = stereo_reprojected.ptr<float>(i);
      uchar* disp_ptr = stereo_processed.ptr<uchar>(i);
      uchar* rgb_ptr = left_remapped.ptr<uchar>(i);

      for (int j = 0; j < stereo_reprojected.cols; j++) {

        if (disp_ptr[j] == 0) continue;
        if (rgb_ptr[j] == 0) continue;

        point.x = reconst_ptr[3*j];
        point.y = -reconst_ptr[3*j+1];
        point.z = -reconst_ptr[3*j+2];

        col = rgb_ptr[j];

        rgb = ((int) col) << 16 | ((int) col) << 8 | ((int) col);
        point.rgb = *reinterpret_cast<float*>(&rgb);

        if(abs(point.x) > 1) continue;
        if(abs(point.y) > 1) continue;
        if(abs(point.z) > 1) continue;

        ptCloudTemp->points.push_back(point);
   }
  }

  point.x = 0;
  point.y = 0;
  point.z = 0;

  rgb = ((int) 255) << 16 | ((int) 255) << 8 | ((int) 255);
  point.rgb = *reinterpret_cast<float*>(&rgb);
  ptCloudTemp->points.push_back(point);

  //pcl::PassThrough<pcl::PointXYZRGB> pass;
  //pass.setInputCloud (ptCloudTemp);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 3.0);
  //pass.setFilterLimitsNegative (true);
  //pass.filter (*ptCloudTemp);
  /*
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  outrem.setInputCloud(ptCloudTemp);
  outrem.setRadiusSearch(0.05);
  outrem.setMinNeighborsInRadius (5);
  outrem.filter (*ptCloudTemp);


  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (ptCloudTemp);
  sor.setMeanK (5);
  sor.setStddevMulThresh (4.0);
  sor.filter(*ptCloudTemp);
  */

  ptCloud = ptCloudTemp;

  emit gotPointCloud();
}


bool QStereoCamera::setFrameSize(int width, int height) {
  bool res = false;

  if (this->connected) {
    res = this->videocap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    res &= this->videocap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    this->imageWidth = width;
    this->imageHeight = height;
    this->imageSize = cv::Size(width, height);
  }

  return res;
}

bool QStereoCamera::setFrame16() {
  bool res = false;

  if (this->connected) {
    res = this->videocap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));
  }

  return res;
}

bool QStereoCamera::setBrightness(int brightness) {
  bool res = false;

  if (this->connected) {
    res = this->videocap.set(CV_CAP_PROP_BRIGHTNESS, brightness);
  }

  return res;
}

void QStereoCamera::enableRectify(bool enable) { rectify = enable && validRectification; }

void QStereoCamera::capture(void) {
  if (this->connected) {
    if (!this->videocap.read(image_buffer)) {
      this->acquiring = false;
      this->connected = false;
      qDebug() << "Failed, need to re-init";
      this->videocap.release();
    } else {
      frames++;
      flip(image_buffer, image_buffer, 0);
      split(image_buffer, channels);

      left_raw = channels[1].clone();
      right_raw = channels[2].clone();

      if (this->rectify) {
        QFuture<void> res_l = QtConcurrent::run(
            this, &QStereoCamera::remap_parallel, left_raw,
            std::ref(left_remapped), this->rectmapx_l, this->rectmapy_l);
        QFuture<void> res_r = QtConcurrent::run(
            this, &QStereoCamera::remap_parallel, right_raw,
            std::ref(right_remapped), this->rectmapx_r, this->rectmapy_r);

        res_l.waitForFinished();
        res_r.waitForFinished();

      } else {
        left_remapped = left_raw;
        right_remapped = right_raw;
      }

      emit acquired();
      emit framecount(frames);
    }
  }
}

void QStereoCamera::remap_parallel(cv::Mat src, cv::Mat &dst, cv::Mat rmapx,
                                   cv::Mat rmapy) {
  cv::remap(src, dst, rmapx, rmapy, cv::INTER_CUBIC);
}

bool QStereoCamera::isAcquiring(void) { return this->acquiring; }

void QStereoCamera::pause(void) { this->acquiring = false; }

void QStereoCamera::singleShot(void) {
  this->acquiring = false;
  while (this->capturing) {
    QCoreApplication::processEvents();
  }

  this->capture();
  if (this->matchImages && this->rectify) {
    this->match();
  }
}

void QStereoCamera::setSavelocation(QString dir) { this->saveDir = dir; }

void QStereoCamera::requestSingle(void) {
  qDebug() << "Saving single";

  QString fname;
  QDateTime dateTime = dateTime.currentDateTime();
  QString dateTimeString = dateTime.toString("yyyyMMdd_hhmmss_zzz");

  fname = QString("%1/%2").arg(this->saveDir).arg(dateTimeString);

  qDebug() << fname;

  saveImage(fname);

  this->saveSingle = false;
}

void QStereoCamera::saveImage(QString fname) {
  QFuture<void> res_l = QtConcurrent::run(
      write_parallel, fname.toStdString() + "_l.png", this->channels[1]);
  QFuture<void> res_r = QtConcurrent::run(
      write_parallel, fname.toStdString() + "_r.png", this->channels[2]);

  if (this->rectify) {
    QFuture<void> rect_l =
        QtConcurrent::run(write_parallel, fname.toStdString() + "_l_rect.png",
                          this->left_remapped);
    QFuture<void> rect_r =
        QtConcurrent::run(write_parallel, fname.toStdString() + "_r_rect.png",
                          this->right_remapped);

    rect_l.waitForFinished();
    rect_r.waitForFinished();
  }

  res_l.waitForFinished();
  res_r.waitForFinished();

  emit savedImage(fname);
}

void QStereoCamera::freerun(void) {
  this->acquiring = true;

  do {
    this->capturing = true;
    this->capture();
    emit fps(frame_timer.elapsed());
    frame_timer.start();

    if (this->matchImages) {
      this->match();
    }
    this->capturing = false;

    QCoreApplication::processEvents();

  } while (this->acquiring);

  return;
}

void QStereoCamera::matchImpact(void){

    saveImage("./rectified_images/impact_input_000000");

    std::string input_image_dir = "rectified_images";
    std::string prefix_left = "impact_input_";
    std::string prefix_right = "impact_input_";
    std::string postfix_left = "_l_rect.png";
    std::string postfix_right = "_r_rect.png";

    std::string strPostfix = ".tif";

    int iDigits2 = 6;

    // matching parameters
    std::string strPrefixDisparities = "disparity_";
    std::string strDisparityDir = "disparities";
    std::string strInputParameterFileMatching = "match.param";
    int iNumberofPyramids = 3;
    float fTopShift = 0;

    SParametersInput oParams = {
        input_image_dir,
        input_image_dir,
        0, 1000000000,
        prefix_left,
        prefix_right,
        postfix_left,
        postfix_right,
        iDigits2,
        iDigits2,
        true,
        -99999,
        strDisparityDir,
        strPrefixDisparities,
        iDigits2,
        strPostfix,
        false,
        strDisparityDir,
        "backdisp_",
        iDigits2,
        strPostfix,
        true,
        strDisparityDir,
        "backmatchdist_",
        iDigits2,
        strPostfix,
        false,
        strDisparityDir,
        "backmatchdist_backdisp_",
        iDigits2,
        strPostfix,
        iNumberofPyramids,
        fTopShift
    };

    StereoMatcher(oParams, strInputParameterFileMatching);
}

void QStereoCamera::match(void) {

  if (this->matcher == JR_IMPACT) {

      this->matchImpact();

      stereo_raw_lr = cv::imread("./disparities/disparity_000000.tif", cv::IMREAD_UNCHANGED);

      cv::threshold( stereo_raw_lr, stereo_raw_lr, -64, 0 , cv::THRESH_TOZERO);

      stereo_raw_lr.convertTo(stereo_processed, CV_16S);

      qDebug() << "Stereo done, processing";

      stereo_processed *= -1;

      try{
        cv::ximgproc::getDisparityVis(stereo_processed*16.0, stereo_scale,
                                    255.0 / 64);
      }catch( ... ){
          qDebug() << "Disparity problem";
      }

  }else{
      if (this->matcher == OCVBM) {
        this->matchBM();
      } else if (this->matcher == OCVSGBM) {
        this->matchSGBM();
      } else if (this->matcher == OCVCBM) {
        this->matchCudaBM();
      } else if (this->matcher == OCVCBP) {
        this->matchCudaBP();
      }

      stereo_raw_lr.convertTo(stereo_processed, CV_16S);

      if (this->filterDisparity) {
        updateFilterParams();

        if (!usingCUDA) {
          right_matcher->compute(right_remapped, left_remapped, stereo_raw_rl);

          wls_filter->filter(stereo_raw_lr, left_remapped, stereo_processed,
                             stereo_raw_rl);
        }
      }

      cv::ximgproc::getDisparityVis(stereo_processed-16*bm_minDisparity, stereo_scale,
                                    255.0 / bm_maxDisparity);

  }

  applyColorMap(stereo_scale, stereo_out, cv::COLORMAP_JET);

  cvtColor(stereo_out, stereo_out, CV_BGR2RGB);

  reproject3D();

  emit matched();
}

void QStereoCamera::setMatcher(QString matcher) {
  if (matcher == "OpenCV Block") {
    qDebug() << "Using Block Matcher";
    this->matcher = OCVBM;
    usingCUDA = false;
    updateMatchBMParams();
    wls_filter = cv::ximgproc::createDisparityWLSFilter(bm_matcher);
  } else if (matcher == "OpenCV Semi-Global Block") {
    qDebug() << "Using SG Block Matcher";
    this->matcher = OCVSGBM;
    usingCUDA = false;
    updateMatchSGBMParams();
    wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm_matcher);
  } else if (matcher == "OpenCV Cuda Block") {
    qDebug() << "Using Cuda Block Matcher";
    this->matcher = OCVCBM;
    usingCUDA = true;
    updateMatchCudaBMParams();
  } else if (matcher == "OpenCV Cuda Belief Propagation") {
    qDebug() << "Using Cuda BP Matcher";
    this->matcher = OCVCBP;
    usingCUDA = true;
    updateMatchCudaBPParams();
  } else if (matcher == "Impact") {
      qDebug() << "Using JR Impact matcher";
      this->matcher = JR_IMPACT;
      usingCUDA = true;
    }
}

bool QStereoCamera::initVideoStream(cv::VideoWriter *writer, QString filename,
                                    cv::Size imsize, double fps, int codec) {
  writer->open(filename.toStdString(), codec, fps, imsize, false);
  // check if we succeeded
  if (!writer->isOpened()) {
    QMessageBox msgBox;
    msgBox.setText("Failed to open output video file.");
    msgBox.exec();
    return false;
  }

  return true;
}

void QStereoCamera::stopCaptureVideo(void) { this->acquiringVideo = false; }

void QStereoCamera::captureVideo(QString fname) {
  std::unique_ptr<cv::VideoWriter> writerLeft(new cv::VideoWriter);
  std::unique_ptr<cv::VideoWriter> writerRight(new cv::VideoWriter);

  bool ready = false;

  ready = initVideoStream(writerLeft.get(), "./outLeft.avi",
                          this->left_remapped.size());
  ready &= initVideoStream(writerRight.get(), "./outRight.avi",
                           this->left_remapped.size());

  if (!ready) {
    QMessageBox msgBox;
    msgBox.setText("Unable to set up video streams.");
    msgBox.exec();
  } else {
    this->acquiringVideo = true;
    this->acquiring = false;

    do {
      this->capture();
      emit fps(frame_timer.elapsed());
      frame_timer.start();

      writerLeft->write(this->left_remapped);
      writerRight->write(this->right_remapped);

      QCoreApplication::processEvents();

    } while (this->acquiringVideo);

    writerLeft->release();
    writerRight->release();
  }

  return;
}

void QStereoCamera::setMatcherPreFilterCap(int pfc) {
  this->bm_preFilterCap = pfc;
}

void QStereoCamera::setMatcherMinDisparity(int mindisp) {
  this->bm_minDisparity = mindisp;
}

void QStereoCamera::setMatcherMaxDisparity(int maxdisp) {
  this->bm_maxDisparity = maxdisp;
}

void QStereoCamera::setMatcherTextureThreshold(int threshold) {
  this->bm_textureThreshold = threshold;
}

void QStereoCamera::setMatcherBlockSize(int blocksize) {
  this->bm_blockSize = blocksize;
}

void QStereoCamera::setMatcherSpeckleWindow(int window) {
  this->bm_speckleWindowSize = window;
}

void QStereoCamera::setMatcherSpeckleRange(int range) {
  this->bm_speckleRange = range;
}

void QStereoCamera::setMatcherConsistency(int diff) {
  this->bm_disp12MaxDiff = diff;
}

void QStereoCamera::setMatcherUniqueness(int uniqueness) {
  this->bm_uniquenessRatio = uniqueness;
}

void QStereoCamera::setFilterLambda(int lambda) {
  this->filter_lambda = lambda;
}

void QStereoCamera::setFilterConsistency(int consistency) {
  this->filter_consistency = consistency;
}

void QStereoCamera::setFilterSigmaColour(double sigma) {
  this->filter_sigma = sigma;
}

void QStereoCamera::setFilterDiscontinuity(int discontinuity) {
  this->filter_discontinuity = discontinuity;
}

void QStereoCamera::toggleFilter(bool state) { this->filterDisparity = state; }

void QStereoCamera::matchBM(void) {
  updateMatchBMParams();

  bm_matcher->compute(left_remapped, right_remapped, stereo_raw_lr);
}

void QStereoCamera::matchCudaBM(void) {
  updateMatchCudaBMParams();

  d_left.upload(left_remapped);
  d_right.upload(right_remapped);

  cuda_bm_matcher->compute(d_left, d_right, d_disp);

  d_disp.download(stereo_raw_lr);
  stereo_raw_lr.convertTo(stereo_raw_lr, CV_32F);
  stereo_raw_lr *= 16.0;
}

void QStereoCamera::matchCudaBP(void) {
  updateMatchCudaBPParams();

  d_left.upload(left_remapped);
  d_right.upload(right_remapped);
  d_disp = cv::cuda::GpuMat(left_remapped.size(), CV_16SC1);

  cuda_bp_matcher->compute(d_left, d_right, d_disp);

  d_disp.download(stereo_raw_lr);
  stereo_raw_lr *= 16.0;
}

void QStereoCamera::matchSGBM(void) {
  updateMatchSGBMParams();

  sgbm_matcher->compute(left_remapped, right_remapped, stereo_raw_lr);
}

void QStereoCamera::enableStereo(bool toggle) { this->matchImages = toggle; }

QStereoCamera::~QStereoCamera(void) {
  disconnect(this, SIGNAL(acquired()), this, SLOT(capture()));
  videocap.release();
  //DeinitExtensionUnit();
  emit finished();
}
