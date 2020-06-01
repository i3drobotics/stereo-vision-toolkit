/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "abstractstereocamera.h"

const std::string AbstractStereoCamera::CAMERA_TYPE_DEIMOS = "deimos";
const std::string AbstractStereoCamera::CAMERA_TYPE_USB = "usb";
const std::string AbstractStereoCamera::CAMERA_TYPE_BASLER_GIGE = "baslergige";
const std::string AbstractStereoCamera::CAMERA_TYPE_BASLER_USB = "baslerusb";
const std::string AbstractStereoCamera::CAMERA_TYPE_TIS = "tis";
const std::string AbstractStereoCamera::CAMERA_TYPE_VIMBA = "vimba";

AbstractStereoCamera::AbstractStereoCamera(QObject *parent) : QObject(parent) {

#ifdef CUDA
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        has_cuda = true;
        emit haveCuda();
    }
#endif

    ptCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    cv_video_writer = new cv::VideoWriter();

    connect(this, SIGNAL(stereopair_captured()), this, SLOT(process_stereo()));
    connect(this, SIGNAL(left_captured()), this, SLOT(register_left_capture()));
    connect(this, SIGNAL(right_captured()), this, SLOT(register_right_capture()));
}

void AbstractStereoCamera::try_capture(){
    capture();
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> AbstractStereoCamera::loadSerials(std::string camera_type, std::string filename){
    std::vector<stereoCameraSerialInfo> serials;
    QString qFilname = QString::fromStdString(filename);
    QFile inputFile(qFilname);
    if (inputFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputFile);
        while (!in.atEnd()) {
            QStringList line = in.readLine().split(' ');

            if(line.size() == 4 || line.size() == 5){ //TODO change this back to only 4 when excel that generates the serials is fixed
                QString cam_type = line.at(0);
                std::string file_cam_type = cam_type.toStdString();
                if (file_cam_type.compare(camera_type) == 0){
                    QString left_camera_serial = line.at(1);
                    QString right_camera_serial = line.at(2);
                    QString i3dr_serial = line.at(3);
                    AbstractStereoCamera::stereoCameraSerialInfo csi;
                    csi.camera_type = camera_type;
                    csi.left_camera_serial = left_camera_serial.toStdString();
                    csi.right_camera_serial = right_camera_serial.toStdString();
                    csi.i3dr_serial = i3dr_serial.toStdString();
                    serials.push_back(csi);
                }
            }
        }
        inputFile.close();
    } else {
        qDebug() << "Couldn't open i3dr camera serials file " << qFilname;
    }
    return serials;
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
        matcher->saveDisparity(fname + "_disp_raw.png");
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
    cv::Mat disparity;

    matcher->getDisparity(disparity);
    disparity.copyTo(disparity_downscale);

    qDebug() << "disparity image size: " << disparity_downscale.size().width << "," << disparity_downscale.size().height;

    //cv::abs(disparity_downscale);

    // By default the disparity maps are scaled by a factor of 16.0
    disparity_downscale /= 16.0;

    int row = (disparity_downscale.rows-1)/2;
    int column = (disparity_downscale.cols-1)/2;
    float disp_val_f =  disparity_downscale.at<float>(row, column);
    qDebug() << "IMAGE CENTER: " << column+1 << "," << row+1;
    qDebug() << "DISPARITY AT CENTER: " << disp_val_f;

    if (Q.empty() || disparity_downscale.empty()) {
        qDebug() << "Q or disparity map is empty";
        return;
    }

    cv::reprojectImageTo3D(disparity_downscale, stereo_reprojected, Q, true);

    qDebug() << "reprojected image size: " << stereo_reprojected.size().width << "," << stereo_reprojected.size().height;

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

    cv::Matx44d _Q;
    Q.convertTo(_Q, CV_64F);

    for (int i = 0; i < disparity_downscale.rows; i++)
    {
        for (int j = 0; j < disparity_downscale.cols; j++)
        {
            float d = disparity_downscale.at<float>(i, j);

            if (d < 10000 && d > 0){
                float x_index = j;
                float y_index = i;

                //qDebug() << d;

                cv::Vec4d homg_pt = _Q * cv::Vec4d((double)x_index, (double)y_index, (double)d, 1.0);

                float x = (float)homg_pt[0] / (float)homg_pt[3];
                float y = (float)homg_pt[1] / (float)homg_pt[3];
                float z = (float)homg_pt[2] / (float)homg_pt[3];

                uchar intensity = left_remapped.at<uchar>(i,j);

                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = intensity;
                point.g = intensity;
                point.b = intensity;
                ptCloudTemp->points.push_back(point);
            }
        }
    }

    if(ptCloudTemp->points.empty()){
        qDebug() << "Failed to create Point cloud. Point cloud is empty";
        return;
    }

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

    qDebug() << "tmp point cloud size: " << ptCloudTemp->points.size();
    qDebug() << "point cloud size: " << ptCloud->points.size();

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

bool AbstractStereoCamera::isConnected() {
    bool connected_tmp = connected;
    return connected_tmp;
}

void AbstractStereoCamera::getLeftImage(cv::Mat &dst) {
    left_output.copyTo(dst);
}

void AbstractStereoCamera::getRightImage(cv::Mat &dst) {
    right_output.copyTo(dst);
}

cv::Mat AbstractStereoCamera::getLeftImage(void) {
    return left_raw.clone();
}

cv::Mat AbstractStereoCamera::getRightImage(void) {
    return right_raw.clone();
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
        fx = l_camera_matrix.at<double>(0,2);
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
        cv::Mat T;
        fs_s["T"] >> T;
        baseline = T.at<double>(0,0);
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

    emit stereopair_processed();
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

void AbstractStereoCamera::videoStreamProcess(){
    acquiring = true;
    while(acquiring && connected){
        QCoreApplication::processEvents();
        capture_and_process();
        cv::Mat left,right;
        getLeftImage(left);
        getRightImage(right);
        cv::hconcat(left, right, video_frame);
        cv_video_writer->write(video_frame);
    }
}

bool AbstractStereoCamera::videoStreamInit(QString filename, int fps) {

    cv_video_writer = new cv::VideoWriter();

    if (filename == "") {
        QDateTime dateTime = dateTime.currentDateTime();
        QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");
        filename = QString("%1/stereo_video_%2.mp4").arg(save_directory, date_string);
    }

    qDebug() << "Saving to: " << filename;

    video_frame = cv::Mat (image_height, 2 * image_width, CV_8UC1);

    if (fps == 0) fps = 60;

    int codec = CV_FOURCC('H', '2', '6', '4');

    bool is_color = false;
    cv_video_writer->open(filename.toStdString(), codec, fps, video_frame.size(), is_color);
    // check if we succeeded
    if (!cv_video_writer->isOpened()) {
        qDebug() << "Failed to open output video file.";
        videoStreamStop();
        return false;
    }

    return true;
}

void AbstractStereoCamera::videoStreamStop(){
    //TODO fix read access violation when closing the program
    if (cv_video_writer->isOpened()){
        cv_video_writer->release();
        cv_video_writer = new cv::VideoWriter();
    }
}

bool write_parallel(std::string fname, cv::Mat src) {
    std::vector<int> params;
    int compression_level = 9;
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(compression_level);

    return cv::imwrite(fname, src, params);
}
