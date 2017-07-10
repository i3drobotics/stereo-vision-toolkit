#ifndef QSTEREOCAMERA_H
#define QSTEREOCAMERA_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QElapsedTimer>
#include <QtConcurrent/QtConcurrent>
#include <QDateTime>
#include <QMessageBox>

#include <iostream>
#include <memory>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

// Board driver and OpenCV
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/cudastereo.hpp>

#include <DigVTKIntegration.hpp>

#include<stereocameraopencv.h>

using namespace std;

enum stereoMatcher { OCVBM, OCVSGBM, OCVCBM, OCVCBP, JR_IMPACT};

class QStereoCamera : public QObject
{
    Q_OBJECT
public:
    explicit QStereoCamera(QObject *parent = 0);
    void assignThread(QThread* thread);
    ~QStereoCamera(void);
    bool init(void);
    cv::Mat channels[3];
    int getExposure(void);
    bool isAcquiring(void);
    cv::Mat left_remapped;
    cv::Mat right_remapped;
    cv::Mat stereo_out;

    cv::Mat left_raw;
    cv::Mat right_raw;

    cv::Mat r_camera_matrix;
    cv::Mat l_camera_matrix;
    cv::Mat l_dist_coeffs;
    cv::Mat r_dist_coeffs;
    cv::Mat Q;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr ptCloudFiltered;

    QString saveDir = ".";
    void matchImpact(void);

    int imageWidth;
    int imageHeight;

private:

    void updateMatchBMParams(void);
    void updateMatchSGBMParams(void);
    void updateMatchCudaBMParams(void);
    void updateMatchCudaBPParams(void);
    void updateFilterParams(void);
    void matchLeft(void);
    void matchRight(void);

    cv::VideoCapture videocap;
    cv::Mat image_buffer;
    bool connected = false;
    int exposure;
    bool acquiring = false;
    bool capturing = false;
    bool grabSingle = false;
    QElapsedTimer frame_timer;
    qint64 frames = 0;
    cv::Mat rectmapx_l;
    cv::Mat rectmapy_l;
    cv::Mat rectmapx_r;
    cv::Mat rectmapy_r;
    cv::Mat stereo_raw_lr;
    cv::Mat stereo_raw_rl;
    cv::Mat stereo_scale;
    cv::Mat stereo_processed;
    cv::Mat stereo_reprojected;
    cv::Size imageSize;

    cv::Ptr<cv::StereoBM> bm_matcher;
    cv::Ptr<cv::StereoSGBM> sgbm_matcher;
    cv::Ptr<cv::StereoMatcher> right_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

    cv::Ptr<cv::cuda::StereoBM> cuda_bm_matcher;
    cv::Ptr<cv::cuda::StereoBeliefPropagation> cuda_bp_matcher;
    cv::Ptr<cv::cuda::StereoConstantSpaceBP> cuda_csbp_matcher;

    bool rectify = false;
    bool saveSingle = false;
    bool matchImages = false;
    bool filterDisparity = false;
    bool acquiringVideo = false;

    int bm_preFilterCap		=	25;
    int bm_minDisparity		=	0;
    int bm_maxDisparity     =   128;
    int bm_blockSize        =   21;
    int bm_textureThreshold	=	5;
    int bm_uniquenessRatio	=	2;
    int bm_speckleRange		=	31;
    int bm_disp12MaxDiff	=	1;
    int bm_speckleWindowSize = 350;
    int bm_preFilterSize = 5;

    double filter_lambda = 10000;
    int filter_consistency = 24; // Divide by 16 internally
    double filter_sigma = 1.5;
    int filter_discontinuity = 1;

    bool validRectification = false;
    bool validCalibration = false;

    bool usingCUDA = false;

    cv::cuda::GpuMat d_left, d_right, d_disp_l, d_disp;

    stereoMatcher matcher = OCVBM;

signals:
    void acquired();
    void fps(qint64);
    void savedImage();
    void finished();
    void framecount(qint64);
    void savedImage(QString);
    void matched();
    void gotPointCloud();

public slots:
    bool setBrightness(int brightness);
    bool setFrameSize(int width, int height);
    bool setFrame16(void);
    void capture(void);
    void freerun(void);
    void pause(void);
    void singleShot(void);
    bool loadCalibration(QString directory);
    bool loadRectificationMaps(QString src_l, QString src_r);
    void requestSingle(void);
    void saveImage(QString fname);
    void remap_parallel(cv::Mat, cv::Mat&, cv::Mat, cv::Mat);
    void reproject3D();

    void enableStereo(bool);
    bool loadCalibration(QString left_cal, QString right_cal, QString stereo_cal);
    void enableRectify(bool enable);

    void match(void);
    void matchBM(void);
    void matchCudaBP(void);
    void matchSGBM(void);
    void matchCudaBM(void);
    void setMatcher(QString matcher);
    void setMatcherPreFilterCap(int pfc);
    void setMatcherMinDisparity(int mindisp);
    void setMatcherMaxDisparity(int maxdisp);
    void setMatcherTextureThreshold(int threshold);
    void setMatcherBlockSize(int blocksize);
    void setMatcherSpeckleWindow(int window);
    void setMatcherSpeckleRange(int range);
    void setMatcherConsistency(int diff);
    void setMatcherUniqueness(int uniqueness);

    void setFilterLambda(int lambda);
    void setFilterConsistency(int consistency);
    void setFilterSigmaColour(double sigma);
    void setFilterDiscontinuity(int discontinuity);
    void toggleFilter(bool state);

    void setSavelocation(QString dir);

    bool initVideoStream(cv::VideoWriter* writer, QString filename, cv::Size imsize, double fps = 60.0, int codec = CV_FOURCC('h', '2', '6', '4') );
    void captureVideo(QString fname = "");
    void stopCaptureVideo(void);

};

#endif // QSTEREOCAMERA_H
