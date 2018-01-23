/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef ABSTRACTSTEREOCAMERA_H
#define ABSTRACTSTEREOCAMERA_H

#include <abstractstereomatcher.h>

#include <QCoreApplication>
#include <QObject>
#include <QThread>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <QDir>

#include<memory>

#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>

// Point Cloud Library
#define _MATH_DEFINES_DEFINED
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

class AbstractStereoCamera : public QObject {
  Q_OBJECT

 signals:
    void acquired();
    void fps(qint64);
    void savedImage();
    void finished();
    void framecount(qint64);
    void savedImage(QString);
    void matched();
    void gotPointCloud();
    void captured();
    void haveCuda();
    void temperature(double);

 public:
  explicit AbstractStereoCamera(QObject *parent = 0);
  //virtual void initCamera() = 0;
  void assignThread(QThread *thread);

  virtual ~AbstractStereoCamera(void) = 0;

  cv::Mat left_remapped;
  cv::Mat right_remapped;
  cv::Mat left_raw;
  cv::Mat right_raw;
  cv::Mat Q;

  cv::Mat stereo_reprojected;



 public slots:

  void setMatcher(AbstractStereoMatcher *matcher);
  bool setCalibration(QString directory);
  void singleShot(void);
  void freerun(void);
  void pause();
  virtual bool capture(void) = 0;
  void requestSingle();

  void videoStreamStart(QString fname = "");
  void videoStreamStop(void);

  void enableCapture(bool capture);
  void enableAcquire(bool acquire);
  void enableMatching(bool match);
  void enableRectify(bool rectify);
  void enableReproject(bool reproject);

  bool isCapturing();
  bool isAcquiring();
  bool isMatching();
  bool isRectifying();

  void getLeftImage(cv::Mat &dst);
  void getRightImage(cv::Mat &dst);

  void setVisualZmin(double zmin);
  void setVisualZmax(double zmax);

  void savePointCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();

  cv::Mat getLeftImage();
  cv::Mat getRightImage();

  int getWidth(void){ return image_width; }
  int getHeight(void){ return image_height; }
  cv::Size getSize(void){ return image_size; }
  void setSavelocation(QString dir){ save_directory = dir; }

 private:
  qint64 frames = 0;
  bool capturing = false;
  bool acquiring = false;
  bool matching = false;
  bool rectifying = false;
  bool reprojecting = false;
  bool has_cuda = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud;
  QString save_directory = ".";

  cv::VideoCapture stereo_video;
  bool acquiring_video = false;
  bool videoStreamInit(cv::VideoWriter* writer, QString filename, cv::Size imsize, double fps = 60.0, int codec = CV_FOURCC('D', 'I', 'B', ' ') );

  void finishCapture(void);
  void captureAndProcess(void);
  void saveImage(QString fname);
  bool loadRectificationMaps(QString src_l, QString src_r);
  bool loadCalibration(QString left_cal, QString right_cal,
                       QString stereo_cal);

  void rectifyImages(void);
  void remap_parallel(cv::Mat src, cv::Mat &dst, cv::Mat rmapx,
                      cv::Mat rmapy);

  void reproject3D();


  cv::Mat rectmapx_l;
  cv::Mat rectmapy_l;
  cv::Mat rectmapx_r;
  cv::Mat rectmapy_r;
  cv::Mat r_camera_matrix;
  cv::Mat l_camera_matrix;
  cv::Mat l_dist_coeffs;
  cv::Mat r_dist_coeffs;

  cv::Mat left_output;
  cv::Mat right_output;

  AbstractStereoMatcher *matcher = NULL;

  double visualisation_min_z = 0.2;
  double visualisation_max_z = 2;


 protected:

  int frame_rate = 30;
  int image_width = 0;
  int image_height = 0;
  cv::Size image_size;

  bool rectification_valid = false;
  bool calibration_valid = false;
  bool connected = false;

};

inline AbstractStereoCamera::~AbstractStereoCamera() { emit finished(); }

bool write_parallel(std::string fname, cv::Mat src);

#endif  // ABSTRACTSTEREOCAMERA_H
