/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "abstractstereocamera.h"

std::string AbstractStereoCamera::CAMERA_NAME_TARA = "tara";
std::string AbstractStereoCamera::CAMERA_NAME_TIS = "tis";
std::string AbstractStereoCamera::CAMERA_NAME_VIMBA = "vimba";
std::string AbstractStereoCamera::CAMERA_NAME_USB = "usb";
std::string AbstractStereoCamera::CAMERA_NAME_BASLER_GIGE = "baslergige";
std::string AbstractStereoCamera::CAMERA_NAME_BASLER_USB = "baslerusb";
std::string AbstractStereoCamera::CAMERA_NAME_DEIMOS = "deimos";
std::string AbstractStereoCamera::CAMERA_NAME_PHOBOS_BASLER_GIGE = "phobosbaslergige";
std::string AbstractStereoCamera::CAMERA_NAME_PHOBOS_BASLER_USB = "phobosbaslerusb";
std::string AbstractStereoCamera::CAMERA_NAME_PHOBOS_TIS_USB = "phobostisusb";
std::string AbstractStereoCamera::CAMERA_NAME_TITANIA_BASLER_GIGE = "titaniabaslergige";
std::string AbstractStereoCamera::CAMERA_NAME_TITANIA_BASLER_USB = "titaniabaslerusb";
std::string AbstractStereoCamera::CAMERA_NAME_TITANIA_VIMBA_USB = "titaniavimbausb";
std::string AbstractStereoCamera::CAMERA_NAME_INVALID = "invalid";

AbstractStereoCamera::AbstractStereoCamera(StereoCameraSerialInfo serial_info, StereoCameraSettings camera_settings, QObject *parent) :
    QObject(parent),
    ptCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>)),
    cv_video_writer(new cv::VideoWriter()),
    stereoCameraSettings_(camera_settings),
    stereoCameraSerialInfo_(serial_info)
{

#ifdef WITH_CUDA
    
    try
    {
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            cuda_device_found = true;
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        std::cout << "Disabling GPU CUDA functions" << std::endl;
        cuda_device_found = false;
    }
#endif
    //startThread();
    //connect(this, SIGNAL(captured_success()), this, SLOT(processStereo()));
    connect(this, SIGNAL(captured_success()), this, SLOT(processNewCapture()));
    connect(this, SIGNAL(stereopair_processed()), this, SLOT(processNewStereo()));
    //connect(this, SIGNAL(matched()), this, SLOT(processNewMatch()));
    connect(this, SIGNAL(captured_success()), this, SLOT(resetFailFrameCount()));
}

void AbstractStereoCamera::assignThread(QThread *thread){
    thread_ = thread;
    this->moveToThread(thread_);
    connect(this, SIGNAL(finished()), thread_, SLOT(quit()));
    connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
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
        case CAMERA_TYPE_TARA : return CAMERA_NAME_TARA; break;
        case CAMERA_TYPE_TIS : return CAMERA_NAME_TIS; break;
        case CAMERA_TYPE_VIMBA : return CAMERA_NAME_VIMBA; break;
        case CAMERA_TYPE_USB : return CAMERA_NAME_USB; break;
        case CAMERA_TYPE_BASLER_GIGE : return CAMERA_NAME_BASLER_GIGE; break;
        case CAMERA_TYPE_BASLER_USB : return CAMERA_NAME_BASLER_USB; break;
        case CAMERA_TYPE_DEIMOS : return CAMERA_NAME_DEIMOS; break;
        case CAMERA_TYPE_PHOBOS_BASLER_GIGE : return CAMERA_NAME_PHOBOS_BASLER_GIGE; break;
        case CAMERA_TYPE_PHOBOS_BASLER_USB : return CAMERA_NAME_PHOBOS_BASLER_USB; break;
        case CAMERA_TYPE_PHOBOS_TIS_USB : return CAMERA_NAME_PHOBOS_TIS_USB; break;
        case CAMERA_TYPE_TITANIA_BASLER_GIGE : return CAMERA_NAME_TITANIA_BASLER_GIGE; break;
        case CAMERA_TYPE_TITANIA_BASLER_USB : return CAMERA_NAME_TITANIA_BASLER_USB; break;
        case CAMERA_TYPE_TITANIA_VIMBA_USB : return CAMERA_NAME_TITANIA_VIMBA_USB; break;
        default  : return CAMERA_NAME_INVALID; break;
    }
}

AbstractStereoCamera::StereoCameraType AbstractStereoCamera::String2StereoCameraType(std::string camera_type){
    if (camera_type.compare(CAMERA_NAME_TARA) == 0){
        return CAMERA_TYPE_TARA;
    } else if (camera_type.compare(CAMERA_NAME_TIS) == 0){
        return CAMERA_TYPE_TIS;
    } else if (camera_type.compare(CAMERA_NAME_VIMBA) == 0){
        return CAMERA_TYPE_VIMBA;
    } else if (camera_type.compare(CAMERA_NAME_USB) == 0){
        return CAMERA_TYPE_USB;
    } else if (camera_type.compare(CAMERA_NAME_BASLER_GIGE) == 0){
        return CAMERA_TYPE_BASLER_GIGE;
    } else if (camera_type.compare(CAMERA_NAME_BASLER_USB) == 0){
        return CAMERA_TYPE_BASLER_USB;
    } else if (camera_type.compare(CAMERA_NAME_DEIMOS) == 0){
        return CAMERA_TYPE_DEIMOS;
    } else if (camera_type.compare(CAMERA_NAME_PHOBOS_BASLER_GIGE) == 0){
        return CAMERA_TYPE_PHOBOS_BASLER_GIGE;
    } else if (camera_type.compare(CAMERA_NAME_PHOBOS_BASLER_USB) == 0){
        return CAMERA_TYPE_PHOBOS_BASLER_USB;
    } else if (camera_type.compare(CAMERA_NAME_PHOBOS_TIS_USB) == 0){
        return CAMERA_TYPE_PHOBOS_TIS_USB;
    } else if (camera_type.compare(CAMERA_NAME_TITANIA_BASLER_GIGE) == 0){
        return CAMERA_TYPE_TITANIA_BASLER_GIGE;
    } else if (camera_type.compare(CAMERA_NAME_TITANIA_BASLER_USB) == 0){
        return CAMERA_TYPE_TITANIA_BASLER_USB;
    } else if (camera_type.compare(CAMERA_NAME_TITANIA_VIMBA_USB) == 0){
        return CAMERA_TYPE_TITANIA_VIMBA_USB;
    } else {
        return CAMERA_TYPE_INVALID;
    }
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
    emit savedImage(success);
}

void AbstractStereoCamera::saveDisparity(QString filename) {
    cv::Mat disparity_output;

    getDisparityFiltered(disparity_output);

    cv::imwrite(filename.toStdString(), disparity_output);

    return;
}

void AbstractStereoCamera::saveDisparityColormap(QString filename) {
    cv::Mat disparity_main, disparity_output;

    getDisparity(disparity_main);
    //generate normalised colormap for saving
    CVSupport::disparity2colormap(disparity_main,Q,disparity_output);

    cv::imwrite(filename.toStdString(), disparity_output);

    return;
}

void AbstractStereoCamera::saveRGBD(QString filename, bool enable_16bit, float scale_16bit){
    cv::Mat rgbd, color, depth, Q, depth_z, depth_split[3];;
    getLeftMatchImage(color);
    getDepth(depth);
    getQ(Q);

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
            cvtColor(color,color,cv::COLOR_GRAY2RGB);
        } else if (color.type() == CV_8UC3){
            cvtColor(color,color,cv::COLOR_BGR2RGB);
        } else {
            std::cerr << "Invalid image type for saving RGBD" << std::endl;
        }
        // Create RGBD image from color image and z only depth image
        if (enable_16bit){
            rgbd = CVSupport::createRGBD16(color,depth_z,scale_16bit,false);
        } else {
            rgbd = CVSupport::createRGBD32(color,depth_z);
        }
        qDebug() << "Saving rgbd image...";
        cv::imwrite(filename.toStdString(), rgbd);
    } else {
        std::cerr << "Depth image or camera image is empty." << std::endl;
        return;
    }
}  

void AbstractStereoCamera::saveImage(QString fname) {
    setFileSaveDirectory(fname);

    cv::Mat left, right;
    std::string filename_r, filename_l;
    if (rectifying) {
        filename_l = fname.toStdString() + "_l_rect.png";
        filename_r = fname.toStdString() + "_r_rect.png";
    } else {
        filename_l = fname.toStdString() + "_l.png";
        filename_r = fname.toStdString() + "_r.png";
    }

    left = left_output.clone();
    right = right_output.clone();

    std::vector<int> params(cv::IMWRITE_PNG_COMPRESSION,0);
    qDebug() << "Saving left image...";
    cv::imwrite(filename_l,left,params);
    qDebug() << "Saving right image...";
    cv::imwrite(filename_r,right,params);
    qDebug() << "Image saving complete.";
    imageSaved(true);

    /*
    QFuture<bool> rect_l = QtConcurrent::run(
                CVSupport::write_parallel, filename_l, left);
    QFuture<bool> rect_r = QtConcurrent::run(
                CVSupport::write_parallel, filename_r, right);

    QFutureWatcher<bool> futureWatcher_l, futureWatcher_r;
    QObject::connect(&futureWatcher_l, SIGNAL(finished(bool)), this, SLOT(imageSaved(bool)));
    QObject::connect(&futureWatcher_r, SIGNAL(finished(bool)), this, SLOT(imageSaved(bool)));;

    futureWatcher_l.setFuture(rect_l);
    futureWatcher_r.setFuture(rect_r);
    */

    if(matching && savingDisparity){
        saveDisparity(fname + "_disp_raw.tif");
        saveDisparityColormap(fname + "_disp_colormap.png");
        saveRGBD(fname + "_rgbd.png");
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
    int exit_code = pcl::io::savePLYFileBinary(fname.toStdString(), *ptCloud);

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

void AbstractStereoCamera::getQ(cv::Mat &Q_){
    Q.copyTo(Q_);
}

bool AbstractStereoCamera::setVideoSource(int source_index){
    // 0: stereo-mono, 1: stereo, 2: left, 3:right, 4:disparity
    switch(source_index) {
    case 0: // stereo-mono
        video_src = VIDEO_SRC_STEREO_RG;
        break;
    case 1: // stereo
        video_src = VIDEO_SRC_STEREO_CONCAT;
        break;
    case 2: // left
        video_src = VIDEO_SRC_LEFT;
        break;
    case 3: // right
        video_src = VIDEO_SRC_RIGHT;
        break;
    case 4: // disparity
        video_src = VIDEO_SRC_DISPARITY;
        break;
    default:
        qDebug() << "Invalid video source index. MUST be 0: stereo RG, 1: stereo concat, 2: left, 3:right, 4:disparity";
        return false;
        break;
    }
    return true;
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

void AbstractStereoCamera::getLeftImage(cv::Mat &dst) {
    lr_image_mutex.lock();
    left_output.copyTo(dst);
    lr_image_mutex.unlock();
}

void AbstractStereoCamera::getRightImage(cv::Mat &dst) {
    lr_image_mutex.lock();
    right_output.copyTo(dst);
    lr_image_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getLeftImage(void) {
    lr_image_mutex.lock();
    return left_output.clone();
    lr_image_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getRightImage(void) {
    lr_image_mutex.lock();
    return right_output.clone();
    lr_image_mutex.unlock();
}

void AbstractStereoCamera::getLeftRawImage(cv::Mat &dst) {
    lr_raw_image_mutex.lock();
    left_unrectified.copyTo(dst);
    lr_raw_image_mutex.unlock();
}

void AbstractStereoCamera::getRightRawImage(cv::Mat &dst) {
    lr_raw_image_mutex.lock();
    right_unrectified.copyTo(dst);
    lr_raw_image_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getLeftRawImage(void) {
    lr_raw_image_mutex.lock();
    return left_unrectified.clone();
    lr_raw_image_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getRightRawImage(void) {
    lr_raw_image_mutex.lock();
    return right_unrectified.clone();
    lr_raw_image_mutex.unlock();
}

void AbstractStereoCamera::getRightMatchImage(cv::Mat &dst){
    disparity_mutex.lock();
    right_match.copyTo(dst);
    disparity_mutex.unlock();
}

void AbstractStereoCamera::getLeftMatchImage(cv::Mat &dst){
    disparity_mutex.lock();
    left_match.copyTo(dst);
    disparity_mutex.unlock();
}

void AbstractStereoCamera::getDisparity(cv::Mat &dst) {
    disparity_mutex.lock();
    disparity.copyTo(dst);
    disparity_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getDisparity(){
    disparity_mutex.lock();
    return disparity.clone();
    disparity_mutex.unlock();
}

void AbstractStereoCamera::getDisparityFiltered(cv::Mat &dst) {
    disparity_mutex.lock();
    disparity_filtered.copyTo(dst);
    disparity_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getDisparityFiltered(){
    disparity_mutex.lock();
    return disparity_filtered.clone();
    disparity_mutex.unlock();
}

void AbstractStereoCamera::getDepth(cv::Mat &dst) {
    disparity_mutex.lock();
    depth.copyTo(dst);
    disparity_mutex.unlock();
}

cv::Mat AbstractStereoCamera::getDepth(){
    disparity_mutex.lock();
    return depth.clone();
    disparity_mutex.unlock();
}

void AbstractStereoCamera::generateRectificationMaps(cv::Size image_size){
    /*
    cv::Mat dl_camera_matrix = l_camera_matrix.clone();
    cv::Mat dr_camera_matrix = r_camera_matrix.clone();
    cv::Mat dl_proj_mat = l_proj_mat.clone();
    cv::Mat dr_proj_mat = r_proj_mat.clone();
    dl_camera_matrix.at<double>(0,2) *= downsample_factor;
    dl_camera_matrix.at<double>(1,2) *= downsample_factor;
    dl_camera_matrix.at<double>(0,2) *= downsample_factor;
    dl_camera_matrix.at<double>(1,2) *= downsample_factor;
    dr_camera_matrix.at<double>(0,2) *= downsample_factor;
    dr_camera_matrix.at<double>(1,2) *= downsample_factor;
    dl_proj_mat.at<double>(0,2) *= downsample_factor;
    dl_proj_mat.at<double>(1,2) *= downsample_factor;
    dr_proj_mat.at<double>(0,2) *= downsample_factor;
    dr_proj_mat.at<double>(1,2) *= downsample_factor;
    */

    cv::initUndistortRectifyMap(l_camera_matrix,l_dist_coeffs,l_rect_mat,l_proj_mat,image_size,CV_32FC1,rectmapx_l,rectmapy_l);
    cv::initUndistortRectifyMap(r_camera_matrix,r_dist_coeffs,r_rect_mat,r_proj_mat,image_size,CV_32FC1,rectmapx_r,rectmapy_r);
    rectification_valid = true;
    QCoreApplication::processEvents();
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
    QCoreApplication::processEvents();

    if (fs_r.isOpened()) {
        fs_r["x"] >> rectmapx_r;
        fs_r["y"] >> rectmapy_r;
    } else {
        res = false;
    }

    fs_l.release();
    fs_r.release();
    QCoreApplication::processEvents();

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

    if (!loadCalibrationXMLFiles(directory + "/left_calibration.xml",
                         directory + "/right_calibration.xml",
                         directory + "/stereo_calibration.xml")) {
        qDebug() << "Couldn't load camera calibration";
        calibration_valid = false;
        return false;
    }
    QCoreApplication::processEvents();

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
            qDebug() << "Cal image size: " << cal_image_width << "," << cal_image_height;
            qDebug() << "Rectmap size: " << rectmapx_r.cols << "," << rectmapy_r.rows;
            qDebug() << "Camera image size: " << image_width << "," << image_height;

            return false;
        }
    }
    QCoreApplication::processEvents();

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
    QCoreApplication::processEvents();

    cv::FileStorage fs_r(right_cal.toStdString(), flags);

    if (fs_r.isOpened()) {
        fs_r["cameraMatrix"] >> r_camera_matrix;
        fs_r["distCoeffs"] >> r_dist_coeffs;
    } else {
        return false;
    }

    fs_r.release();
    QCoreApplication::processEvents();

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
    QCoreApplication::processEvents();

    return true;
}

bool AbstractStereoCamera::loadCalibrationYamlFiles(QString left_cal, QString right_cal) {
    // implimented from (https://github.com/i3drobotics/pyStereo3D/blob/master/Stereo3D/Stereo3D/StereoCalibration.py)
    int cal_r_image_width, cal_r_image_height;

    // Detect yaml format type (cv or ros)
    // Read first line of file
    QFile yamlFile(left_cal);
    QString first_line = "";
    if (yamlFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&yamlFile);
       first_line = in.readLine();
       yamlFile.close();
    }
    if (first_line.contains("%YAML:1.0")){
        // CV Filestorage yaml
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
        QCoreApplication::processEvents();

        if (cal_image_width != image_width || cal_image_height != image_height){
            qDebug() << "Image size does not match calibrated image size";
            return false;
        }

        cv::FileStorage fs_r(right_cal.toStdString(), flags);

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
    } else if (first_line.contains("image_width")){
        // ROS Perception yaml

        //TODO support badly formatted yaml

        // Read left yaml
        QFile leftYamlFile(left_cal);
        QList<QString> left_lines;
        if (leftYamlFile.open(QIODevice::ReadOnly))
        {
           QTextStream in(&leftYamlFile);
           while (!in.atEnd())
           {
              QString line = in.readLine();
              left_lines.append(line);
           }
           leftYamlFile.close();
        }
        if (left_lines.length() == 20){

            if (left_lines[0].contains("image_width:")){
                QStringList split_line = left_lines[0].split(':');
                cal_image_width = split_line[1].toInt();
            } else {
                return false;
            }
            if (left_lines[1].contains("image_height:")){
                QStringList split_line = left_lines[1].split(':');
                cal_image_height = split_line[1].toInt();
            } else {
                return false;
            }
            if (left_lines[6].contains("data:")){
                QStringList camera_matrix_line = left_lines[6].split(':');
                QStringList camera_matrix_data = camera_matrix_line[1].split(',');
                l_camera_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0));

                l_camera_matrix.at<double>(0,0) = camera_matrix_data[0].split('[')[1].toDouble();
                l_camera_matrix.at<double>(0,1) = camera_matrix_data[1].toDouble();
                l_camera_matrix.at<double>(0,2) = camera_matrix_data[2].toDouble();
                l_camera_matrix.at<double>(1,0) = camera_matrix_data[3].toDouble();
                l_camera_matrix.at<double>(1,1) = camera_matrix_data[4].toDouble();
                l_camera_matrix.at<double>(1,2) = camera_matrix_data[5].toDouble();
                l_camera_matrix.at<double>(2,0) = camera_matrix_data[6].toDouble();
                l_camera_matrix.at<double>(2,1) = camera_matrix_data[7].toDouble();
                l_camera_matrix.at<double>(2,2) = camera_matrix_data[8].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (left_lines[11].contains("data:")){
                QStringList dist_matrix_line = left_lines[11].split(':');
                QStringList dist_matrix_data = dist_matrix_line[1].split(',');
                l_dist_coeffs = cv::Mat(1, 5, CV_64F, cv::Scalar(0));

                l_dist_coeffs.at<double>(0,0) = dist_matrix_data[0].split('[')[1].toDouble();
                l_dist_coeffs.at<double>(0,1) = dist_matrix_data[1].toDouble();
                l_dist_coeffs.at<double>(0,2) = dist_matrix_data[2].toDouble();
                l_dist_coeffs.at<double>(0,3) = dist_matrix_data[3].toDouble();
                l_dist_coeffs.at<double>(0,4) = dist_matrix_data[4].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (left_lines[15].contains("data:")){
                QStringList rect_matrix_line = left_lines[15].split(':');
                QStringList rect_matrix_data = rect_matrix_line[1].split(',');
                l_rect_mat = cv::Mat(3, 3, CV_64F, cv::Scalar(0));

                l_rect_mat.at<double>(0,0) = rect_matrix_data[0].split('[')[1].toDouble();
                l_rect_mat.at<double>(0,1) = rect_matrix_data[1].toDouble();
                l_rect_mat.at<double>(0,2) = rect_matrix_data[2].toDouble();
                l_rect_mat.at<double>(1,0) = rect_matrix_data[3].toDouble();
                l_rect_mat.at<double>(1,1) = rect_matrix_data[4].toDouble();
                l_rect_mat.at<double>(1,2) = rect_matrix_data[5].toDouble();
                l_rect_mat.at<double>(2,0) = rect_matrix_data[6].toDouble();
                l_rect_mat.at<double>(2,1) = rect_matrix_data[7].toDouble();
                l_rect_mat.at<double>(2,2) = rect_matrix_data[8].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (left_lines[19].contains("data:")){
                QStringList proj_matrix_line = left_lines[19].split(':');
                QStringList proj_matrix_data = proj_matrix_line[1].split(',');
                l_proj_mat = cv::Mat(3, 4, CV_64F, cv::Scalar(0));

                l_proj_mat.at<double>(0,0) = proj_matrix_data[0].split('[')[1].toDouble();
                l_proj_mat.at<double>(0,1) = proj_matrix_data[1].toDouble();
                l_proj_mat.at<double>(0,2) = proj_matrix_data[2].toDouble();
                l_proj_mat.at<double>(0,3) = proj_matrix_data[3].toDouble();
                l_proj_mat.at<double>(1,0) = proj_matrix_data[4].toDouble();
                l_proj_mat.at<double>(1,1) = proj_matrix_data[5].toDouble();
                l_proj_mat.at<double>(1,2) = proj_matrix_data[6].toDouble();
                l_proj_mat.at<double>(1,3) = proj_matrix_data[7].toDouble();
                l_proj_mat.at<double>(2,0) = proj_matrix_data[8].toDouble();
                l_proj_mat.at<double>(2,1) = proj_matrix_data[9].toDouble();
                l_proj_mat.at<double>(2,2) = proj_matrix_data[10].toDouble();
                l_proj_mat.at<double>(2,3) = proj_matrix_data[11].split(']')[0].toDouble();
            } else {
                return false;
            }

        } else {
            return false;
        }

        // Read right yaml
        QFile rightYamlFile(right_cal);
        QList<QString> right_lines;
        if (rightYamlFile.open(QIODevice::ReadOnly))
        {
           QTextStream in(&rightYamlFile);
           while (!in.atEnd())
           {
              QString line = in.readLine();
              right_lines.append(line);
           }
           rightYamlFile.close();
        }

        if (right_lines.length() == 20){

            if (right_lines[0].contains("image_width:")){
                QStringList split_line = right_lines[0].split(':');
                cal_r_image_width = split_line[1].toInt();
            }
            if (right_lines[1].contains("image_height:")){
                QStringList split_line = right_lines[1].split(':');
                cal_r_image_height = split_line[1].toInt();
            }

            if (right_lines[6].contains("data:")){
                QStringList camera_matrix_line = right_lines[6].split(':');
                QStringList camera_matrix_data = camera_matrix_line[1].split(',');
                r_camera_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0));

                r_camera_matrix.at<double>(0,0) = camera_matrix_data[0].split('[')[1].toDouble();
                r_camera_matrix.at<double>(0,1) = camera_matrix_data[1].toDouble();
                r_camera_matrix.at<double>(0,2) = camera_matrix_data[2].toDouble();
                r_camera_matrix.at<double>(1,0) = camera_matrix_data[3].toDouble();
                r_camera_matrix.at<double>(1,1) = camera_matrix_data[4].toDouble();
                r_camera_matrix.at<double>(1,2) = camera_matrix_data[5].toDouble();
                r_camera_matrix.at<double>(2,0) = camera_matrix_data[6].toDouble();
                r_camera_matrix.at<double>(2,1) = camera_matrix_data[7].toDouble();
                r_camera_matrix.at<double>(2,2) = camera_matrix_data[8].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (right_lines[11].contains("data:")){
                QStringList dist_matrix_line = right_lines[11].split(':');
                QStringList dist_matrix_data = dist_matrix_line[1].split(',');
                r_dist_coeffs = cv::Mat(1, 5, CV_64F, cv::Scalar(0));

                r_dist_coeffs.at<double>(0,0) = dist_matrix_data[0].split('[')[1].toDouble();
                r_dist_coeffs.at<double>(0,1) = dist_matrix_data[1].toDouble();
                r_dist_coeffs.at<double>(0,2) = dist_matrix_data[2].toDouble();
                r_dist_coeffs.at<double>(0,3) = dist_matrix_data[3].toDouble();
                r_dist_coeffs.at<double>(0,4) = dist_matrix_data[4].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (right_lines[15].contains("data:")){
                QStringList rect_matrix_line = right_lines[15].split(':');
                QStringList rect_matrix_data = rect_matrix_line[1].split(',');
                r_rect_mat = cv::Mat(3, 3, CV_64F, cv::Scalar(0));

                r_rect_mat.at<double>(0,0) = rect_matrix_data[0].split('[')[1].toDouble();
                r_rect_mat.at<double>(0,1) = rect_matrix_data[1].toDouble();
                r_rect_mat.at<double>(0,2) = rect_matrix_data[2].toDouble();
                r_rect_mat.at<double>(1,0) = rect_matrix_data[3].toDouble();
                r_rect_mat.at<double>(1,1) = rect_matrix_data[4].toDouble();
                r_rect_mat.at<double>(1,2) = rect_matrix_data[5].toDouble();
                r_rect_mat.at<double>(2,0) = rect_matrix_data[6].toDouble();
                r_rect_mat.at<double>(2,1) = rect_matrix_data[7].toDouble();
                r_rect_mat.at<double>(2,2) = rect_matrix_data[8].split(']')[0].toDouble();
            } else {
                return false;
            }
            if (right_lines[19].contains("data:")){
                QStringList proj_matrix_line = right_lines[19].split(':');
                QStringList proj_matrix_data = proj_matrix_line[1].split(',');
                r_proj_mat = cv::Mat(3, 4, CV_64F, cv::Scalar(0));

                r_proj_mat.at<double>(0,0) = proj_matrix_data[0].split('[')[1].toDouble();
                r_proj_mat.at<double>(0,1) = proj_matrix_data[1].toDouble();
                r_proj_mat.at<double>(0,2) = proj_matrix_data[2].toDouble();
                r_proj_mat.at<double>(0,3) = proj_matrix_data[3].toDouble();
                r_proj_mat.at<double>(1,0) = proj_matrix_data[4].toDouble();
                r_proj_mat.at<double>(1,1) = proj_matrix_data[5].toDouble();
                r_proj_mat.at<double>(1,2) = proj_matrix_data[6].toDouble();
                r_proj_mat.at<double>(1,3) = proj_matrix_data[7].toDouble();
                r_proj_mat.at<double>(2,0) = proj_matrix_data[8].toDouble();
                r_proj_mat.at<double>(2,1) = proj_matrix_data[9].toDouble();
                r_proj_mat.at<double>(2,2) = proj_matrix_data[10].toDouble();
                r_proj_mat.at<double>(2,3) = proj_matrix_data[11].split(']')[0].toDouble();
            } else {
                return false;
            }

       } else {
            return false;
        }

    } else {
        qDebug() << "Cannot detect yaml type in calibration file. Invalid first line.";
        return false;
    }

    QCoreApplication::processEvents();

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
    //double fy = l_camera_matrix.at<double>(1,1);

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

    QCoreApplication::processEvents();

    generateRectificationMaps(cv::Size(image_width,image_height));

    return true;
}


bool AbstractStereoCamera::rectifyImages(cv::Mat left, cv::Mat right, cv::Mat& left_rect, cv::Mat& right_rect) {
    if (cal_image_width != image_width || cal_image_height != image_height){
        qDebug() << "Image size does not match calibrated image size";
        left.copyTo(left_rect);
        right.copyTo(right_rect);
        return false;
    }
    if (cuda_device_found){
#ifdef WITH_CUDA

        cv::cuda::GpuMat d_left, d_right, d_left_remap, d_right_remap;

        d_left.upload(left);
        d_right.upload(right);

        cv::cuda::GpuMat d_rectmapx_l, d_rectmapy_l, d_rectmapx_r, d_rectmapy_r;

        d_rectmapx_l.upload(rectmapx_l);
        d_rectmapx_r.upload(rectmapx_r);
        d_rectmapy_l.upload(rectmapy_l);
        d_rectmapy_r.upload(rectmapy_r);

        cv::cuda::remap(d_left, d_left_remap, d_rectmapx_l, d_rectmapy_l,
                        cv::INTER_CUBIC, cv::BORDER_CONSTANT);
        cv::cuda::remap(d_right, d_right_remap, d_rectmapx_r, d_rectmapy_r,
                        cv::INTER_CUBIC, cv::BORDER_CONSTANT);

        d_left_remap.download(left_rect);
        d_right_remap.download(right_rect);
        return true;
#else
        return false;
#endif
    } else {
        remap_parallel(left,left_rect, rectmapx_l, rectmapy_l);
        remap_parallel(right,right_rect, rectmapx_r, rectmapy_r);
        return true;
    }
}

void AbstractStereoCamera::remap_parallel(cv::Mat src, cv::Mat &dst,
                                          cv::Mat rmapx, cv::Mat rmapy) {
    cv::remap(src, dst, rmapx, rmapy, cv::INTER_CUBIC);
}


void AbstractStereoCamera::setMatcher(AbstractStereoMatcher *matcher) {

    new_matcher = matcher;
    changed_matcher = true;
    /*
    bool reenable_required = false;
    if (this->isMatching()){
        reenable_required = true;
        enableMatching(false);
    }
    this->matcher = matcher;
    if (reenable_required){
        enableMatching(true);
    }
    qDebug() << "Changed matcher";
    */
}

void AbstractStereoCamera::processNewCapture(void){
    if (!processing_busy){
        processing_busy = true;
        //processStereo();
        stereo_future = QtConcurrent::run(this, &AbstractStereoCamera::processStereo);
    }
}

void AbstractStereoCamera::processNewStereo(void){
    if (!match_busy && matching){
        match_busy = true;
        //processMatch();
        match_future = QtConcurrent::run(this, &AbstractStereoCamera::processMatch);
    }
}

/*
void AbstractStereoCamera::processNewMatch(void){
    if (!reproject_busy && reprojecting){
        reproject_busy = true;
        reproject_future = QtConcurrent::run(this, &AbstractStereoCamera::reproject3D);
    }
}
*/

void AbstractStereoCamera::processStereo(void) {

    frames++;
    emit framecount(frames);

    if (!right_raw.empty() && !left_raw.empty()){

        cv::Mat left_tmp, right_tmp;

        if (isSwappingLeftRight()){
            right_raw.copyTo(left_tmp);
            left_raw.copyTo(right_tmp);
        } else {
            left_raw.copyTo(left_tmp);
            right_raw.copyTo(right_tmp);
        }

        if (downsample_factor != 1){
            cv::resize(left_tmp,left_tmp,cv::Size(),downsample_factor,downsample_factor);
            cv::resize(right_tmp,right_tmp,cv::Size(),downsample_factor,downsample_factor);
            /*
            if (calibration_valid){
                if (rectmapx_l.size() != left_tmp.size()){
                    //adjust rectification maps to downsampled size
                    generateRectificationMaps(left_tmp.size());
                    emit update_size((float)right_tmp.size().width, (float)right_tmp.size().height, getBitDepth());
                }
            }
            */
        }

        lr_raw_image_mutex.lock();
        left_tmp.copyTo(left_unrectified);
        right_tmp.copyTo(right_unrectified);
        lr_raw_image_mutex.unlock();

        if (rectifying) {
            lr_raw_image_mutex.lock();
            //bool res = rectifyImages(left_unrectified,right_unrectified,left_remapped,right_remapped);
            bool res = rectifyImages(left_tmp,right_tmp,left_remapped,right_remapped);
            if (downsample_factor != 1){
                cv::resize(left_remapped,left_remapped,cv::Size(),downsample_factor,downsample_factor);
                cv::resize(right_remapped,right_remapped,cv::Size(),downsample_factor,downsample_factor);
            }
            lr_raw_image_mutex.unlock();
            if (!res){
                send_error(RECTIFY_ERROR);
                lr_raw_image_mutex.lock();
                left_unrectified.copyTo(left_remapped);
                right_unrectified.copyTo(right_remapped);
                lr_raw_image_mutex.unlock();
            }
        } else {
            lr_raw_image_mutex.lock();
            left_unrectified.copyTo(left_remapped);
            right_unrectified.copyTo(right_remapped);
            lr_raw_image_mutex.unlock();
        }

        lr_image_mutex.lock();
        left_remapped.copyTo(left_output);
        right_remapped.copyTo(right_output);
        lr_image_mutex.unlock();

        if (capturing_video){
            if (video_src == VIDEO_SRC_STEREO_CONCAT || video_src == VIDEO_SRC_LEFT || video_src == VIDEO_SRC_RIGHT || video_src == VIDEO_SRC_STEREO_RG){
                if (video_src == VIDEO_SRC_STEREO_CONCAT){
                    if (capturing_rectified_video){
                        cv::hconcat(left_output, right_output, video_frame);
                    } else {
                        cv::hconcat(left_unrectified, right_unrectified, video_frame);
                    }
                } else if (video_src == VIDEO_SRC_STEREO_RG){
                    cv::Mat left_tmp_st_mono, right_tmp_st_mono;
                    if (capturing_rectified_video){
                        left_output.copyTo(left_tmp_st_mono);
                        right_output.copyTo(right_tmp_st_mono);
                    } else {
                        left_unrectified.copyTo(left_tmp_st_mono);
                        right_unrectified.copyTo(right_tmp_st_mono);
                    }
                    // convert color frames into mono
                    cv::Mat left_st_mono, right_st_mono;
                    if (left_tmp_st_mono.type() == CV_8UC3){
                        cvtColor(left_tmp_st_mono, left_st_mono, cv::COLOR_BGR2GRAY);
                        cvtColor(right_tmp_st_mono, right_st_mono, cv::COLOR_BGR2GRAY);
                    } else if (left_tmp_st_mono.type() == CV_8UC1){
                        left_st_mono = left_tmp_st_mono;
                        right_st_mono = right_tmp_st_mono;
                    } else {
                        std::cerr << "Invalid image type for saving stereo mono" << std::endl;
                    }
                    cv::Mat b = cv::Mat::zeros(cv::Size(left_st_mono.cols, left_st_mono.rows), CV_8UC1);
                    cv::Mat g = left_st_mono;
                    cv::Mat r = right_st_mono;
                    std::vector<cv::Mat> channels;
                    channels.push_back(b);
                    channels.push_back(g);
                    channels.push_back(r);
                    cv::merge(channels, video_frame);
                } else if (video_src == VIDEO_SRC_LEFT){
                    if (capturing_rectified_video){
                        left_output.copyTo(video_frame);
                    } else {
                        left_unrectified.copyTo(video_frame);
                    }
                } else if (video_src == VIDEO_SRC_RIGHT){
                    if (capturing_rectified_video){
                        right_output.copyTo(video_frame);
                    } else {
                        right_unrectified.copyTo(video_frame);
                    }
                }
                addVideoStreamFrame(video_frame);
            }
        }

        emit stereopair_processed();
        if (!first_image_received){
            first_image_received = true;
            emit first_image_ready(true);
        }
        emit frametime(frametimer.elapsed());
        frametimer.restart();
    }
    processing_busy = false;
}

void AbstractStereoCamera::processMatch(){
    //qDebug() << "Processing match...";
    lr_image_mutex.lock();
    if (left_output.empty() || right_output.empty()){
        return;
    }
    cv::Mat left_img, right_img, disp, left_bgr, disp_filtered, depth_tmp;
    left_img = left_output.clone();
    right_img = right_output.clone();
    lr_image_mutex.unlock();

    if (changed_matcher){
        changed_matcher = false;
        this->matcher = new_matcher;
    }

    //qDebug() << "Stereo matching...";
    bool match_success = matcher->match(left_img,right_img);
    if (match_success){
        //qDebug() << "Getting disparity from stereo match...";
        matcher->getDisparity(disp);
    } else {
        disp = cv::Mat::zeros(cv::Size(left_img.rows, left_img.cols), CV_64FC1);
    }
    disparity_mutex.lock();
    left_match = left_img.clone();
    right_match = right_img.clone();
    disparity = disp.clone();
    CVSupport::removeInvalidDisparity(disparity/16,Q,disp_filtered);
    disparity_filtered = disp_filtered.clone();
    CVSupport::disparity2Depth(disp_filtered, Q, depth_tmp, 10000, downsample_factor);
    depth = depth_tmp.clone();
    disparity_mutex.unlock();
    //qDebug() << "Getting left image from stereo match...";
    left_bgr = matcher->getLeftBGRImage();
    emit matched();

    cv::Mat disp_colormap;
    if (video_src == VIDEO_SRC_DISPARITY || (reprojecting && getPointCloudTexture() == POINT_CLOUD_TEXTURE_DEPTH)){
        CVSupport::disparity2colormap(disp,Q,disp_colormap);

        if (video_src == VIDEO_SRC_DISPARITY){
            if (capturing_video)
                addVideoStreamFrame(disp_colormap);
        }
        //TODO add RGBD support for with VIDEO_SRC_RGBD
    }

    if (reprojecting){
        qDebug() << "Reprojecting 3D...";
        cv::Mat texture_image;
        if (getPointCloudTexture() == POINT_CLOUD_TEXTURE_IMAGE){
            left_bgr.copyTo(texture_image);
        } else if (getPointCloudTexture() == POINT_CLOUD_TEXTURE_DEPTH){
            disp_colormap.copyTo(texture_image);
        }

        if (depth_tmp.empty() || texture_image.empty()){
            return;
        }

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp = PCLSupport::disparity2PointCloud(disp,texture_image,Q, downsample_factor);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp = PCLSupport::depth2PointCloud(depth_tmp,texture_image);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (ptCloudTemp);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(visualisation_min_z, visualisation_max_z);
        pass.filter(*ptCloudTemp);

        ptCloud = ptCloudTemp;

        emit reprojected();
    }

    match_busy = false;

    emit matchtime(matchtimer.elapsed());
    matchtimer.restart();
}

bool AbstractStereoCamera::addVideoStreamFrame(cv::Mat frame){
    video_mutex.lock();
    if (capturing_video){
        if (cv_video_writer != nullptr)
            if (cv_video_writer->isOpened())
                cv_video_writer->write(frame);
    }
    video_mutex.unlock();
    return true;
}

bool AbstractStereoCamera::enableVideoStream(bool enable){
    video_mutex.lock();
    bool res;
    if (enable){
        //start video capture
        cv_video_writer = new cv::VideoWriter();
        int video_width = image_width;
        if (video_src == VIDEO_SRC_STEREO_CONCAT){
            video_width = 2 * image_width;
        }
        if (video_src == VIDEO_SRC_STEREO_RG){
            video_is_color = true;
        }
        if (video_is_color){
            video_frame = cv::Mat (image_height, video_width, CV_8UC3);
        } else {
            video_frame = cv::Mat (image_height, video_width, CV_8UC1);
        }
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
    video_mutex.unlock();
    return res;
}

bool AbstractStereoCamera::setVideoStreamParams(int fps, bool is_color, VideoSource vid_src){
    int codec = cv::VideoWriter::fourcc('M','J','P','G');
    QDateTime dateTime = dateTime.currentDateTime();
    QString date_string = dateTime.toString("yyyyMMdd_hhmmss_zzz");
    QString filename_prefix = "";
    QString extension = "mp4";
    QString filename_suffix = "";
    switch(vid_src) {
    case VIDEO_SRC_STEREO_RG:
        codec = cv::VideoWriter::fourcc('M','P','4','V');
        filename_prefix = "stereo_rg_video_";
        filename_suffix = "_srgvid";
        if (is_color){
            qDebug() << "is_color is true but video source is stereo mono so color will be ignored.";
            is_color = false;
        }
        break;
    case VIDEO_SRC_STEREO_CONCAT:
        filename_prefix = "stereo_concat_video_";
        filename_suffix = "_sctvid";
        extension = "avi";
        //TODO replace this codec as currently creates very large files (raw uncompressed)
        // RGBA used due to very wide video not compatible with standard codecs
        codec = cv::VideoWriter::fourcc('R', 'G', 'B', 'A');
        break;
    case VIDEO_SRC_LEFT:
        filename_prefix = "left_video_";
        filename_suffix = "_lvid";
        break;
    case VIDEO_SRC_RIGHT:
        filename_prefix = "right_video_";
        filename_suffix = "_rvid";
        break;
    case VIDEO_SRC_DISPARITY:
        filename_prefix = "disparity_video_";
        filename_suffix = "_dvid";
        break;
    case VIDEO_SRC_RGBD:
        filename_prefix = "rgbd_video_";
        filename_suffix = "_rgbdvid";
        extension = "avi";
        //TODO replace this codec as currently creates very large files (raw uncompressed)
        // RGBA used due to 4 channel video not compatible with standard codecs
        codec = cv::VideoWriter::fourcc('R','G','B','A');
        break;
    }
    video_filename = QString("%1/%2%3%4.%5").arg(save_directory, filename_prefix, date_string, filename_suffix, extension).toStdString();

    if (fps == 0) fps = 30;
    video_fps = fps;

    video_codec = codec;
    video_is_color = is_color;
    video_src = vid_src;
    return true;
}

AbstractStereoCamera::~AbstractStereoCamera(void){
    stopThread();
    enableVideoStream(false);
}
