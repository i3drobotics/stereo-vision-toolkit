/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "abstractstereocamera.h"

AbstractStereoCamera::AbstractStereoCamera(StereoCameraSerialInfo serial_info, StereoCameraSettings camera_settings, QObject *parent) :
    QObject(parent),
    stereoCameraSerialInfo_(serial_info),
    stereoCameraSettings_(camera_settings),
    ptCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>)),
    cv_video_writer(new cv::VideoWriter())
{

#ifdef WITH_CUDA
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        cuda_device_found = true;
    }
#endif
    //startThread();
    connect(this, SIGNAL(captured_success()), this, SLOT(processStereo()));
    connect(this, SIGNAL(captured_success()), this, SLOT(resetFailFrameCount()));
}

void AbstractStereoCamera::assignThread(QThread *thread){
    thread_ = thread;
    this->moveToThread(thread_);
    connect(this, SIGNAL(finished()), thread_, SLOT(quit()));
    //connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));
    thread_->start();
    //thread_->setPriority(QThread::LowestPriority);
}

void AbstractStereoCamera::stopThread(){
    emit finished();
}

std::string AbstractStereoCamera::StereoCameraType2String(StereoCameraType camera_type){
    switch(camera_type)
    {
        case CAMERA_TYPE_DEIMOS  : return "deimos"; break;
        case CAMERA_TYPE_BASLER_GIGE  : return "baslergige"; break;
        case CAMERA_TYPE_BASLER_USB  : return "baslerusb"; break;
        case CAMERA_TYPE_TIS  : return "tis"; break;
        case CAMERA_TYPE_VIMBA  : return "vimba"; break;
        default  : return ""; break;
    }
}

AbstractStereoCamera::StereoCameraType AbstractStereoCamera::String2StereoCameraType(std::string camera_type){
    if (camera_type.compare("deimos") == 0){
        return CAMERA_TYPE_DEIMOS;
    } else if (camera_type.compare("usb") == 0){
        return CAMERA_TYPE_USB;
    } else if (camera_type.compare("baslergige") == 0){
        return CAMERA_TYPE_BASLER_GIGE;
    } else if (camera_type.compare("baslerusb") == 0){
        return CAMERA_TYPE_BASLER_USB;
    } else if (camera_type.compare("tis") == 0){
        return CAMERA_TYPE_TIS;
    } else if (camera_type.compare("vimba") == 0){
        return CAMERA_TYPE_VIMBA;
    }
    return CAMERA_TYPE_INVALID;
}

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> AbstractStereoCamera::loadSerials(AbstractStereoCamera::StereoCameraType camera_type, std::string filename){
    std::vector<StereoCameraSerialInfo> serials;
    QString qFilname = QString::fromStdString(filename);
    QFile inputFile(qFilname);
    if (inputFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputFile);
        while (!in.atEnd()) {
            QStringList line = in.readLine().split(' ');

            if(line.size() == 4 || line.size() == 5){ //TODO change this back to only 4 when excel that generates the serials is fixed
                QString cam_type = line.at(0);
                std::string file_cam_type_str = cam_type.toStdString();
                StereoCameraType file_cam_type = String2StereoCameraType(file_cam_type_str);
                if (file_cam_type == camera_type){
                    QString left_camera_serial = line.at(1);
                    QString right_camera_serial = line.at(2);
                    QString i3dr_serial = line.at(3);
                    AbstractStereoCamera::StereoCameraSerialInfo csi;
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

void AbstractStereoCamera::saveImageTimestamped(void) {
    QString fname;
    QDateTime dateTime = dateTime.currentDateTime();
    QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");

    fname = QString("%1/%2").arg(save_directory).arg(date_string);

    saveImage(fname);
}

void AbstractStereoCamera::imageSaved(bool success){
    emit savedImage(file_save_directory);
}

void AbstractStereoCamera::saveImage(QString fname) {
    file_save_directory = fname;

    cv::Mat left, right;
    std::string filename_r, filename_l;
    if (rectifying) {
        filename_l = fname.toStdString() + "_l_rect.png";
        filename_r = fname.toStdString() + "_r_rect.png";
        left_remapped.copyTo(left);
        right_remapped.copyTo(right);
    } else {
        filename_l = fname.toStdString() + "_l.png";
        filename_r = fname.toStdString() + "_r.png";
        left_raw.copyTo(left);
        right_raw.copyTo(right);
    }

    QFuture<bool> rect_l = QtConcurrent::run(
                CVSupport::write_parallel, filename_l, left);
    QFuture<bool> rect_r = QtConcurrent::run(
                CVSupport::write_parallel, filename_r, right);

    QFutureWatcher<bool> futureWatcher_l, futureWatcher_r;
    QObject::connect(&futureWatcher_l, SIGNAL(finished(bool)), this, SLOT(imageSaved(bool)));
    QObject::connect(&futureWatcher_r, SIGNAL(finished(bool)), this, SLOT(imageSaved(bool)));;

    futureWatcher_l.setFuture(rect_l);
    futureWatcher_r.setFuture(rect_r);

    if(matching && savingDisparity){
        matcher->saveDisparity(fname + "_disp_raw.png");
        matcher->saveDisparityColormap(fname + "_disp_colormap.png");
    }
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

    cv::Mat depth;
    matcher->calcDepth(disparity,depth);

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

    //cv::reprojectImageTo3D(disparity_downscale, stereo_reprojected, Q, true);

    //qDebug() << "reprojected image size: " << stereo_reprojected.size().width << "," << stereo_reprojected.size().height;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
                new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB point;
    uint32_t rgb = 0;

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

void AbstractStereoCamera::enableDateInFilename(bool enable){
    includeDateInFilename = enable;
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

void AbstractStereoCamera::enableMatching(bool match) { matching = match; }

void AbstractStereoCamera::enableRectify(bool rectify) {
    rectifying = rectify && rectification_valid;
}

void AbstractStereoCamera::enableCaptureRectifedVideo(bool rectify) {
    capturing_rectified_video = rectify;
}

void AbstractStereoCamera::enableSwapLeftRight(bool swap){
    swappingLeftRight = swap;
}

void AbstractStereoCamera::enableSaveDisparity(bool enable){
    savingDisparity = enable;
}

void AbstractStereoCamera::enableDownsampleCalibration(bool enable){
    downsamplingCalibration = enable;
    //TODO Re-load calibration with downsample applied
}

void AbstractStereoCamera::setDownsampleFactor(int factor){
    downsample_factor = 1.0f/(float)factor;
    emit update_size((float)getWidth()*downsample_factor, (float)getHeight()*downsample_factor, getBitDepth());
}

bool AbstractStereoCamera::isCapturing() { return capturing; }

bool AbstractStereoCamera::isMatching() { return matching; }

bool AbstractStereoCamera::isRectifying() { return rectifying; }

bool AbstractStereoCamera::isSwappingLeftRight() { return swappingLeftRight; }

bool AbstractStereoCamera::isDownsamplingCalibration() { return downsamplingCalibration; }

bool AbstractStereoCamera::isSavingDisparity() { return savingDisparity; }

bool AbstractStereoCamera::isConnected() { return connected; }

void AbstractStereoCamera::getLeftImage(cv::Mat &dst) { left_output.copyTo(dst); }

void AbstractStereoCamera::getRightImage(cv::Mat &dst) { right_output.copyTo(dst); }

cv::Mat AbstractStereoCamera::getLeftImage(void) { return left_raw.clone(); }

cv::Mat AbstractStereoCamera::getRightImage(void) { return right_raw.clone(); }

void AbstractStereoCamera::generateRectificationMaps(cv::Size image_size){
    cv::initUndistortRectifyMap(l_camera_matrix,l_dist_coeffs,l_rect_mat,l_proj_mat,image_size,CV_32FC1,rectmapx_l,rectmapy_l);
    cv::initUndistortRectifyMap(r_camera_matrix,r_dist_coeffs,r_rect_mat,r_proj_mat,image_size,CV_32FC1,rectmapx_r,rectmapy_r);
    rectification_valid = true;
}

bool AbstractStereoCamera::loadXMLRectificationMaps(QString src_l, QString src_r) {
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

bool AbstractStereoCamera::loadCalibrationYaml(QString directory){
    directory = QDir::cleanPath(directory);

    if (!loadCalibrationYamlFiles(directory + "/left.yaml", directory + "/right.yaml")) {
        qDebug() << "Couldn't load camera calibration";
        return false;
    }
    return true;
}

bool AbstractStereoCamera::loadCalibrationXML(QString directory) {
    directory = QDir::cleanPath(directory);

    bool load_rectification = true;
    if (load_rectification){
        if (!loadXMLRectificationMaps(directory + "/left_rectification.xml", directory + "/right_rectification.xml")){
            rectification_valid = false;
            qDebug() << "Couldn't find rectification maps";
            return false;
        }
        cal_image_width = rectmapx_l.cols;
        cal_image_height = rectmapx_l.rows;
        if (cal_image_width != image_width || rectmapx_r.cols != image_width ||
                cal_image_height != image_height || rectmapy_r.rows != image_height) {
            rectification_valid = false;
            qDebug() << "Image size doesn't match rectification maps";

            return false;
        }
    }
    if (!loadCalibrationXMLFiles(directory + "/left_calibration.xml",
                         directory + "/right_calibration.xml",
                         directory + "/stereo_calibration.xml")) {
        qDebug() << "Couldn't load camera calibration";
        calibration_valid = false;
        return false;
    }

    rectification_valid = true;

    return true;
}

bool AbstractStereoCamera::loadCalibrationXMLFiles(QString left_cal, QString right_cal,
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

bool AbstractStereoCamera::loadCalibrationYamlFiles(QString left_cal, QString right_cal) {
    // implimented from (https://github.com/i3drobotics/pyStereo3D/blob/master/Stereo3D/Stereo3D/StereoCalibration.py)
    int flags = cv::FileStorage::READ;

    cv::FileStorage fs_l(left_cal.toStdString(), flags);

    if (fs_l.isOpened()) {
        fs_l["image_width"] >> cal_image_width;
        fs_l["image_height"] >> cal_image_height;
        fs_l["camera_matrix"] >> l_camera_matrix;
        fs_l["distortion_coefficients"] >> l_dist_coeffs;
        fs_l["rectification_matrix"] >> l_rect_mat;
        fs_l["projection_matrix"] >> l_proj_mat;
    } else {
        return false;
    }

    fs_l.release();

    if (cal_image_width != image_width || cal_image_height != image_height){
        qDebug() << "Image size does not match calibrated image size";
        return false;
    }

    cv::FileStorage fs_r(right_cal.toStdString(), flags);

    int cal_r_image_width, cal_r_image_height;
    if (fs_r.isOpened()) {
        fs_r["image_width"] >> cal_r_image_width;
        fs_r["image_height"] >> cal_r_image_height;
        fs_r["camera_matrix"] >> r_camera_matrix;
        fs_r["distortion_coefficients"] >> r_dist_coeffs;
        fs_r["rectification_matrix"] >> r_rect_mat;
        fs_r["projection_matrix"] >> r_proj_mat;
    } else {
        return false;
    }

    fs_r.release();

    if (cal_r_image_width != image_width || cal_r_image_height != image_height){
        qDebug() << "Image size does not match calibrated image size";
        return false;
    }

    //calculate q
    Q = cv::Mat::zeros(4, 4, CV_64F);
    double cx = l_proj_mat.at<double>(0,2);
    double cxr = r_proj_mat.at<double>(0,2);
    double cy = l_proj_mat.at<double>(1,2);
    double fx = l_camera_matrix.at<double>(0,0);
    double fy = l_camera_matrix.at<double>(1,1);

    double p14 = r_proj_mat.at<double>(0,3);
    baseline = -p14 / fx;

    double q33 = -(cx - cxr) / baseline;

    Q.at<double>(0,0) = 1.0;
    Q.at<double>(0,3) = -cx;
    Q.at<double>(1,1) = 1.0;
    Q.at<double>(1,3) = -cy;
    Q.at<double>(2,3) = fx;
    Q.at<double>(3,2) = 1.0 / baseline;
    Q.at<double>(3,3) = q33;

    Q.convertTo(Q, CV_32F);

    generateRectificationMaps(cv::Size(image_width,image_height));

    return true;
}


bool AbstractStereoCamera::rectifyImages(void) {
    if (cal_image_width != image_width || cal_image_height != image_height){
        qDebug() << "Image size does not match calibrated image size";
        return false;
    }
#ifdef WITH_CUDA
    if (cuda_device_found){
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
#ifdef WITH_CUDA
    }
#endif
    return true;
}

void AbstractStereoCamera::remap_parallel(cv::Mat src, cv::Mat &dst,
                                          cv::Mat rmapx, cv::Mat rmapy) {
    cv::remap(src, dst, rmapx, rmapy, cv::INTER_CUBIC);
}


void AbstractStereoCamera::setMatcher(AbstractStereoMatcher *matcher) {

    bool reenable_required = false;
    if (this->isMatching()){
        reenable_required = true;
        enableMatching(false);
    }
    this->matcher = matcher;
    this->matcher->setImages(&left_remapped, &right_remapped);
    if (reenable_required){
        enableMatching(true);
    }
    qDebug() << "Changed matcher";
}

void AbstractStereoCamera::processStereo(void) {

    frames++;
    emit framecount(frames);

    if (isSwappingLeftRight()){
        cv::Mat right_tmp;
        right_raw.copyTo(right_tmp);

        left_raw.copyTo(right_raw);
        right_tmp.copyTo(left_raw);
    }

    if (downsample_factor != 1){
        cv::resize(left_raw,left_raw,cv::Size(),downsample_factor,downsample_factor);
        cv::resize(right_raw,right_raw,cv::Size(),downsample_factor,downsample_factor);
    }

    if (rectifying) {
        bool res = rectifyImages();
        if (!res){
            send_error(RECTIFY_ERROR);
            left_raw.copyTo(left_remapped);
            right_raw.copyTo(right_remapped);
        }
    } else {
        left_raw.copyTo(left_remapped);
        right_raw.copyTo(right_remapped);
    }

    left_remapped.copyTo(left_output);
    right_remapped.copyTo(right_output);

    if (capturing_video){
        if (capturing_rectified_video){
            addVideoStreamFrame(left_remapped,right_remapped);
        } else {
            addVideoStreamFrame(left_raw,right_raw);
        }
    }

    if (matching) {
        matcher->match();
        if (reprojecting) {
            reproject3D();
        }
        emit matched();
    }

    emit stereopair_processed();
    if (!first_image_received){
        first_image_received = true;
        emit first_image_ready(true);
    }

    emit frametime(frametimer.elapsed());
    frametimer.restart();
}

bool AbstractStereoCamera::addVideoStreamFrame(cv::Mat left, cv::Mat right){
    cv::hconcat(left, right, video_frame);
    cv_video_writer->write(video_frame);
    return true;
}

bool AbstractStereoCamera::enableVideoStream(bool enable){
    bool res;
    if (enable){
        //start video capture
        cv_video_writer = new cv::VideoWriter();
        video_frame = cv::Mat (image_height, 2 * image_width, CV_8UC1);
        res = cv_video_writer->open(video_filename, video_codec, video_fps, video_frame.size(), video_is_color);
        if (res){
            capturing_video = true;
        }
    } else {
        //stop video capture
        capturing_video = false;
        if (cv_video_writer->isOpened()){
            cv_video_writer->release();
            cv_video_writer = new cv::VideoWriter();
        }
        res = true;
    }
    return res;
}

bool AbstractStereoCamera::setVideoStreamParams(QString filename, int fps, int codec, bool is_color){
    if (filename == "") {
        QDateTime dateTime = dateTime.currentDateTime();
        QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");
        video_filename = QString("%1/stereo_video_%2.mp4").arg(save_directory, date_string).toStdString();
    }

    if (fps == 0) fps = 30;
    video_fps = fps;

    video_codec = codec;
    video_is_color = is_color;
    return true;
}

AbstractStereoCamera::~AbstractStereoCamera(void){
    stopThread();
    enableVideoStream(false);
}
