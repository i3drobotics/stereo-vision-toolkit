/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "abstractstereocamera.h"

AbstractStereoCamera::AbstractStereoCamera(QObject *parent) : QObject(parent) {

#ifdef CUDA
  if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
    has_cuda = true;
    emit haveCuda();
  }
#endif

  ptCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  connect(this, SIGNAL(stereopair_captured()), this, SLOT(process_stereo()));
  connect(this, SIGNAL(left_captured()), this, SLOT(register_left_capture()));
  connect(this, SIGNAL(right_captured()), this, SLOT(register_right_capture()));
}

void AbstractStereoCamera::try_capture(){
    capture();
}

void AbstractStereoCamera::assignThread(QThread *thread) {
  this->moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

void AbstractStereoCamera::saveImageTimestamped(void) {
  QString fname;
  QDateTime dateTime = dateTime.currentDateTime();
  QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");

  fname = QString("%1/%2").arg(save_directory).arg(date_string);

  saveImage(fname);
}

void AbstractStereoCamera::saveImage(QString fname) {


  if (rectifying) {
    QFuture<void> rect_l = QtConcurrent::run(
        write_parallel, fname.toStdString() + "_l_rect.png", left_remapped);
    QFuture<void> rect_r = QtConcurrent::run(
        write_parallel, fname.toStdString() + "_r_rect.png", right_remapped);

    rect_l.waitForFinished();
    rect_r.waitForFinished();
  }else{
    QFuture<void> res_l = QtConcurrent::run(
        write_parallel, fname.toStdString() + "_l.png", left_raw);
    QFuture<void> res_r = QtConcurrent::run(
        write_parallel, fname.toStdString() + "_r.png", right_raw);

    res_l.waitForFinished();
    res_r.waitForFinished();
  }

  if(matching){
      matcher->saveDisparity(fname + "_ldisp.png");
  }

  emit savedImage(fname);
}

void AbstractStereoCamera::setVisualZmin(double zmin){
  assert(zmin > 0);
  visualisation_min_z = zmin;
  qDebug() << "Set vmin: " << zmin;
}


void AbstractStereoCamera::setVisualZmax(double zmax){
    assert(zmax > 0);
    visualisation_max_z = zmax;
    qDebug() << "Set vmax: " << zmax;
}

void AbstractStereoCamera::reproject3D() {

  cv::Mat disparity_downscale;

  matcher->getDisparity(disparity_downscale);

  cv::abs(disparity_downscale);

  // By default the disparity maps are scaled by a factor of 16.0
  disparity_downscale /= 16.0;

  int row = (disparity_downscale.rows-1)/2;
  int column = (disparity_downscale.cols-1)/2;
  float disp_val_f =  disparity_downscale.at<float>(row, column);
  qDebug() << "IMAGE CENTER: " << column+1 << "," << row+1;
  qDebug() << "DISPARITY AT CENTER: " << disp_val_f;

  if (Q.empty() || disparity_downscale.empty()) {
    return;
  }

  cv::reprojectImageTo3D(disparity_downscale, stereo_reprojected, Q, true);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB point;
  uint32_t rgb = 0;
  uchar col = 0;

  point.x = 0;
  point.y = 0;
  point.z = 0;  

  rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
  point.rgb = *reinterpret_cast<float *>(&rgb);
  ptCloudTemp->points.push_back(point);

  for (int i = 0; i < stereo_reprojected.rows; i++) {
    float *reconst_ptr = stereo_reprojected.ptr<float>(i);
    float *disp_ptr = matcher->disparity_lr.ptr<float>(i);
    uchar *rgb_ptr = left_remapped.ptr<uchar>(i);

    // TODO: Figure out why this can trigger, assume the disparity map isn't generated
    // when this function gets called.
    if(!disp_ptr || !rgb_ptr || !reconst_ptr) return;

    for (int j = 0; j < stereo_reprojected.cols; j++) {
      if (disp_ptr[j] == 0) continue;
      if (rgb_ptr[j] == 0) continue;

      if (j == column){
          if (i == row){
              float reproj_val_2 =  reconst_ptr[3 * j + 2];
              qDebug() << "REPROJECT3D AT CENTER 2: " << reproj_val_2;
          }
      }

      point.x = -reconst_ptr[3 * j];
      point.y = -reconst_ptr[3 * j + 1];
      point.z = reconst_ptr[3 * j + 2];

      if(abs(point.x) > 10) continue;
      if(abs(point.y) > 10) continue;
      if(point.z == 10000) continue;
      if(point.z == -10000) continue;
      col = rgb_ptr[j];

      rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
      point.rgb = *reinterpret_cast<float *>(&rgb);

      ptCloudTemp->points.push_back(point);
    }
  }

  if(ptCloudTemp->points.empty())
    return;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (ptCloudTemp);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(visualisation_min_z, visualisation_max_z);
  pass.filter(*ptCloudTemp);

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

  emit reprojected();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AbstractStereoCamera::getPointCloud(){
    return ptCloud;
}

void AbstractStereoCamera::toggleDateInFilename(int state){
    if (state == Qt::Checked){
        includeDateInFilename = true;
    } else if (state == Qt::Unchecked){
        includeDateInFilename = false;
    }
}

void AbstractStereoCamera::savePointCloud(){

    QString fname;

    if (includeDateInFilename){
        //Point cloud file name includes date to avoid overwritting and differentiate
        QDateTime dateTime = dateTime.currentDateTime();
        QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");
        fname = QString("%1/%2_point_cloud.ply").arg(save_directory).arg(date_string);
    } else {
        //Point cloud file name fixed to allow overwritting
        fname = QString("%1/point_cloud.ply").arg(save_directory);
    }

    qDebug() << "Point cloud saving to... " << fname;
    ptCloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    ptCloud->sensor_origin_ = Eigen::Vector4f::Zero();

    //TODO Figure out how to disable outputting camera information in the PLY file
    int exit_code = pcl::io::savePLYFile(fname.toStdString(), *ptCloud);

    QString msgBoxMessage;
    if (exit_code == 0){ //Point cloud saved successfully
        msgBoxMessage = QString("Point cloud saved to...\n%1").arg(fname);
    } else if (exit_code == -1){ //Empty point cloud
        msgBoxMessage = QString("Unable to save as point cloud is empty. Check calibration has been completed and 3D visual shows data before saving.");
    } else { //Unknown error code
        msgBoxMessage = QString("Failed to save point cloud. Unknown error occured.");
    }
    pointCloudSaveStatus(msgBoxMessage);
}

void AbstractStereoCamera::enableReproject(bool reproject) {
  reprojecting = reproject;
}

void AbstractStereoCamera::enableCapture(bool capture) { capturing = capture; }

void AbstractStereoCamera::enableAcquire(bool acquire) { acquiring = acquire; }

void AbstractStereoCamera::enableMatching(bool match) { matching = match; }

void AbstractStereoCamera::enableRectify(bool rectify) {
  rectifying = rectify && rectification_valid;
}

void AbstractStereoCamera::enableSwapLeftRight(bool swap){
    swappingLeftRight = swap;
}

bool AbstractStereoCamera::isCapturing() { return capturing; }

bool AbstractStereoCamera::isAcquiring() { return acquiring; }

bool AbstractStereoCamera::isMatching() { return matching; }

bool AbstractStereoCamera::isRectifying() { return rectifying; }

bool AbstractStereoCamera::isSwappingLeftRight() { return swappingLeftRight; }

bool AbstractStereoCamera::isConnected() { return connected; }

void AbstractStereoCamera::getLeftImage(cv::Mat &dst) {
    if (connected){
        left_output.copyTo(dst);
    }
}

void AbstractStereoCamera::getRightImage(cv::Mat &dst) {
    if (connected){
        right_output.copyTo(dst);
    }
}

cv::Mat AbstractStereoCamera::getLeftImage(void) {
    if (connected){
        return left_raw.clone();
    } else {
        return cv::Mat();
    }
}

cv::Mat AbstractStereoCamera::getRightImage(void) {
    if (connected){
        return right_raw.clone();
    } else {
        return cv::Mat();
    }
}

bool AbstractStereoCamera::loadRectificationMaps(QString src_l, QString src_r) {
  int flags = cv::FileStorage::READ;
  bool res = true;

  cv::FileStorage fs_l(src_l.toStdString(), flags);
  cv::FileStorage fs_r(src_r.toStdString(), flags);

  if (fs_l.isOpened()) {
    fs_l["x"] >> rectmapx_l;
    fs_l["y"] >> rectmapy_l;
  } else {
    res = false;
  }

  if (fs_r.isOpened()) {
    fs_r["x"] >> rectmapx_r;
    fs_r["y"] >> rectmapy_r;
  } else {
    res = false;
  }

  fs_l.release();
  fs_r.release();

  return res;
}

bool AbstractStereoCamera::loadCalibration(QString directory) {
  if (directory == "") return false;

  directory = QDir::cleanPath(directory);

  if (!loadRectificationMaps(directory + "/left_rectification.xml",
                             directory + "/right_rectification.xml")) {
    rectification_valid = false;
    qDebug() << "Couldn't find rectification maps";
    return false;
  }

  if (rectmapx_l.cols != image_width || rectmapx_r.cols != image_width ||
      rectmapy_l.rows != image_height || rectmapy_r.rows != image_height) {
    rectification_valid = false;
    qDebug() << "Image size doesn't match rectification maps";

    return false;
  }

  if (!loadCalibration(directory + "/left_calibration.xml",
                       directory + "/right_calibration.xml",
                       directory + "/stereo_calibration.xml")) {
    qDebug() << "Couldn't load camera calibration";
    calibration_valid = false;
    return false;
  }

  calibration_valid = true;
  rectification_valid = true;

  enableRectify(true);

  return true;
}

bool AbstractStereoCamera::loadCalibration(QString left_cal, QString right_cal,
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

void AbstractStereoCamera::rectifyImages(void) {
  #ifdef CUDA
  if (has_cuda) {
    cv::cuda::GpuMat d_left, d_right, d_left_remap, d_right_remap;

    d_left.upload(left_raw);
    d_right.upload(right_raw);

    cv::cuda::GpuMat d_rectmapx_l, d_rectmapy_l, d_rectmapx_r, d_rectmapy_r;

    d_rectmapx_l.upload(rectmapx_l);
    d_rectmapx_r.upload(rectmapx_r);
    d_rectmapy_l.upload(rectmapy_l);
    d_rectmapy_r.upload(rectmapy_r);

    cv::cuda::remap(d_left, d_left_remap, d_rectmapx_l, d_rectmapy_l,
                    cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    cv::cuda::remap(d_right, d_right_remap, d_rectmapx_r, d_rectmapy_r,
                    cv::INTER_CUBIC, cv::BORDER_CONSTANT);

    d_left_remap.download(left_remapped);
    d_right_remap.download(right_remapped);

  } else {
  #endif
    QFuture<void> res_l =
        QtConcurrent::run(this, &AbstractStereoCamera::remap_parallel, left_raw,
                          std::ref(left_remapped), rectmapx_l, rectmapy_l);
    QFuture<void> res_r = QtConcurrent::run(
        this, &AbstractStereoCamera::remap_parallel, right_raw,
        std::ref(right_remapped), rectmapx_r, rectmapy_r);

    res_l.waitForFinished();
    res_r.waitForFinished();
  #ifdef CUDA
  }
  #endif
}

void AbstractStereoCamera::remap_parallel(cv::Mat src, cv::Mat &dst,
                                          cv::Mat rmapx, cv::Mat rmapy) {
  cv::remap(src, dst, rmapx, rmapy, cv::INTER_CUBIC);
}


void AbstractStereoCamera::setMatcher(AbstractStereoMatcher *matcher) {

  enableAcquire(false);
  this->matcher = matcher;
  this->matcher->setImages(&left_remapped, &right_remapped);
  enableAcquire(true);

  qDebug() << "Changed matcher";
}

void AbstractStereoCamera::process_stereo(void) {

  frames++;
    emit framecount(frames);

    if (isSwappingLeftRight()){
        cv::Mat right_tmp;
        right_raw.copyTo(right_tmp);

        left_raw.copyTo(right_raw);
        right_tmp.copyTo(left_raw);
    }

    if (rectifying) {
      rectifyImages();
    } else {
      left_raw.copyTo(left_remapped);
      right_raw.copyTo(right_remapped);
    }

    left_remapped.copyTo(left_output);
    right_remapped.copyTo(right_output);

    if (matching) {
      matcher->match();
      if (reprojecting) {
        reproject3D();
      }
      emit matched();
    }

    if (connected){
        emit stereopair_processed();
    }
    emit fps(frametimer.elapsed());
    frametimer.restart();

}

void AbstractStereoCamera::register_left_capture(void){
    captured_left = true;
    register_stereo_capture();
}

void AbstractStereoCamera::register_right_capture(void){
    captured_right = true;
    register_stereo_capture();
}

void AbstractStereoCamera::register_stereo_capture(){
    if(captured_left && captured_right){
        emit stereopair_captured();

        captured_stereo = true;

        captured_left = false;
        captured_right = false;
    }

}

void AbstractStereoCamera::pause(void) {
    acquiring = false;
    finishCapture();

}

void AbstractStereoCamera::singleShot(void) {
    pause();
    capture_and_process();
}

void AbstractStereoCamera::finishCapture(void) {
  while (capturing) {
    QCoreApplication::processEvents();
  }
}

void AbstractStereoCamera::freerun(void) {
  acquiring = true;

  while(acquiring && connected){
      QCoreApplication::processEvents();
      capture_and_process();
  }
}

void AbstractStereoCamera::capture_and_process(void){
    captured_stereo = false;

    // Capture, process_stereo is called via signal/slot
    capture();

    while(!captured_stereo){
      /* Check for events, e.g. pause/quit */
      QCoreApplication::processEvents();
    }
}

void AbstractStereoCamera::videoStreamStart(QString fname) {
  std::unique_ptr<cv::VideoWriter> stereo_video(new cv::VideoWriter);

  bool ready = false;

  if (fname == "") {
    QDateTime dateTime = dateTime.currentDateTime();
    QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");
    fname = QString("%1/stereo_video_%2.mp4").arg(save_directory, date_string);
  }

  qDebug() << "Saving to: " << fname;

  cv::Mat output_frame(image_height, 2 * image_width, CV_8UC1);

  if (frame_rate == 0) frame_rate = 60;

  ready = videoStreamInit(stereo_video.get(), fname, output_frame.size(),
                          frame_rate);

  if (!ready) {
    //QMessageBox msgBox;
    //msgBox.setText("Unable to set up video streams.");
    //msgBox.exec();
    qDebug() << "Failed to set up stream.";
  } else {
    acquiring_video = true;
    acquiring = false;

    do {
      process_stereo();
      QCoreApplication::processEvents();
      cv::hconcat(left_output, right_output, output_frame);
      stereo_video->write(output_frame);
    } while (this->acquiring_video);

    videoStreamStop();
  }

  return;
}

bool AbstractStereoCamera::videoStreamInit(cv::VideoWriter *writer,
                                           QString filename, cv::Size imsize,
                                           double fps, int codec) {
  bool is_color = false;
  writer->open(filename.toStdString(), codec, fps, imsize, is_color);
  // check if we succeeded
  if (!writer->isOpened()) {
    //QMessageBox msgBox;
    //msgBox.setText("Failed to open output video file.");
    //msgBox.exec();

    qDebug() << "Failed to open output video file.";
    videoStreamStop();
    return false;
  }

  return true;
}

void AbstractStereoCamera::videoStreamStop(void) {
  this->acquiring_video = false;
  stereo_video.release();
}

bool write_parallel(std::string fname, cv::Mat src) {
  std::vector<int> params;
  int compression_level = 9;
  params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  params.push_back(compression_level);

  return cv::imwrite(fname, src, params);
}
