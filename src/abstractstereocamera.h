/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef ABSTRACTSTEREOCAMERA_H
#define ABSTRACTSTEREOCAMERA_H

#define _USE_MATH_DEFINES

#include <abstractstereomatcher.h>

#include <QCoreApplication>
#include <QObject>
#include <QThread>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <QDir>
#include <QDebug>
#include <QFile>
#include <QProgressDialog>

#include<memory>

#include <opencv2/opencv.hpp>
#ifdef CUDA
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#endif

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

static std::string CAMERA_TYPE_DEIMOS = "usb";
static std::string CAMERA_TYPE_BASLER = "basler";
static std::string CAMERA_TYPE_TIS = "tis";

//!  Stereo camera base class
/*!
  An abstract class to process stereo images from arbitrary cameras. This class should not be used directly,
  instead you should subclass it (e.g. StereoCameraDeimos()). The bare minimum is to implment a capture function
  that will capture a stereo pair from your camera.
*/
class AbstractStereoCamera : public QObject {
    Q_OBJECT

signals:
    //! Emmitted when point cloud is saved
    void pointCloudSaveStatus(QString);

    //! Emitted when a frame has been captured and processed
    void acquired();

    //! Emitted after a frame has been processed to indicate the current framerate
    void fps(qint64);

    //! Emitted when an image has been saved
    void savedImage();

    //! Indicates that the camera has finished acquiring
    void finished();

    //! Indicates that the camera has disconnected
    void disconnected();

    //! Indicates the current frame count
    void framecount(qint64);

    //! Emit when an image has been saved, including the filename
    void savedImage(QString filename);

    //! Emitted when an image has been matched
    void matched();

    //! Emitted when a disparity map has been reprojected to a point cloud
    void reprojected();

    //! Emitted when a camera has captured an image, typically used in sub-classes
    void captured();

    //! Emitted when the left camera captures an image
    void left_captured();

    //! Emitted when the right camera captures an image
    void right_captured();

    //! Emitted when a stereo pair is processed
    void stereopair_processed();

    //! Emitted when the frame size of a camera changes
    void update_size(int width, int height, int bitdepth);

#ifdef CUDA
    //! Emitted if the host system is found to have a CUDA-capable graphics card installed
    void haveCuda();
#endif
    //! Indicates the internal temperature of the camera in Celcius
    void temperature_C(double);

    void stereo_grab();

    void stereopair_captured();

public:
    explicit AbstractStereoCamera(QObject *parent = nullptr);

    virtual void toggleAutoExpose(bool) = 0;
    virtual void adjustExposure(double) = 0;

    virtual void toggleAutoGain(bool) = 0;
    virtual void adjustGain(int) = 0;

    virtual void adjustBinning(int) = 0;

    struct stereoCameraSerialInfo {
        std::string left_camera_serial;
        std::string right_camera_serial;
        std::string camera_type; // type of camera: CAMERA_TYPE_DEIMOS/CAMERA_TYPE_BASLER/CAMERA_TYPE_TIS
        std::string i3dr_serial; // defined i3dr serial for camera pair
        std::string filename; // filename for video [only used for stereoCameraFromVideo]
    };

    stereoCameraSerialInfo camera_serial_info;

    //! Load known camera serials
    /*!
  * @param[in] camera_type Camera type of serials to read (usb/basler/tis)
  * @param[in] filename Camera serials parameter file (default is: camera_serials.ini)
  */
    std::vector<stereoCameraSerialInfo> loadSerials(std::string camera_type, std::string filename=qApp->applicationDirPath().toStdString() + "/camera_serials.ini");

    bool connected = false;

    virtual bool initCamera(stereoCameraSerialInfo) = 0;

    //! Assign the stereo camera object to a thread so as not to block the GUI. Typically called just after instantiation.
    /*!
    @param[in] thread Pointer to thread
  */
    void assignThread(QThread *thread);

    //! Returns whether the camera is currently capturing or processing a frame
    bool isCapturing();

    //! Returns whether the camera is currently acquiring images (in general)
    bool isAcquiring();

    //! Returns whether matching is enabled
    bool isMatching();

    //! Returns whether rectification is being performed
    bool isRectifying();

    //! Returns whether left and right images are being swapped
    bool isSwappingLeftRight();

    //! Returns wheather the camera is connected
    bool isConnected();

    //! Get the left stereo image
    /*!
  * @param[out] dst OpenCV matrix to store image into
  */
    void getLeftImage(cv::Mat &dst);

    //! Get the right stereo image
    /*!
  * @param[out] dst OpenCV matrix to store image into
  */
    void getRightImage(cv::Mat &dst);

    //! Get the left stereo image
    /*!
  * @return OpenCV matrix containing left image
  */
    cv::Mat getLeftImage();

    //! Get the right stereo image
    /*!
  * @return OpenCV matrix containing right image
  */
    cv::Mat getRightImage();

    //! Get a pointer to the current point cloud
    /*!
  * @return Pointer to the current point cloud
  */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();

    //! Get the image width
    /*!
  * @return Image width
  */
    int getWidth(void){ return image_width; }

    //! Get the image height
    /*!
  * @return Image height
  */
    int getHeight(void){ return image_height; }

    //! Get the image size
    /*!
  * @return Image size
  */
    cv::Size getSize(void){ return image_size; }

    virtual ~AbstractStereoCamera(void) = 0;
    virtual std::vector<AbstractStereoCamera::stereoCameraSerialInfo> listSystems(void) = 0;
    virtual bool autoConnect(void) = 0;

    cv::Mat left_remapped;
    cv::Mat right_remapped;
    cv::Mat left_raw;
    cv::Mat right_raw;
    cv::Mat Q;

    cv::Mat stereo_reprojected;

public slots:

    void setMatcher(AbstractStereoMatcher *matcher);

    //! Load calibration files from a directory and check them for validity
    /*!
    @param[in] directory The folder too check.
    @return True or false, depending on whether the parameters are valid.
  */
    bool loadCalibration(QString directory);

    //! Acquire a single frame from the camera and then pause
    void singleShot(void);

    //! Place the camera in freerun mode (continuous acquisition
    void freerun(void);

    //! Pause the camera (note this may not actually pause acquisition, but halts frame requests)
    void pause();

    //! Capture an image
    /*!
   * This is a virtual function which should be implmented by a particular camera driver.
   *
   * @return True if an image was captured successfully, false otherwise
  */
    virtual bool capture(void) = 0;

    //! Disconnect camera
    /*!
   * This is a virtual function which should be implmented by a particular camera driver to close
   * the connection to the camera/s
  */
    virtual void disconnectCamera(void) = 0;

    //! Save an image from the camera with a timestamped filename
    /*!
   *  The timestamp format is: yyyyMMdd_hhmmss_zzz (year, month, day, hour, minute, second, millisecond)
   *
   * @sa setSavelocation()
  */
    void saveImageTimestamped();

    //! Start writing a video stream to a file
    /*!
   * If no filename is supplied, a timestamped video will be stored in the current selected save folder.
   *
   * @param[out] fname The output filename
   * @sa setSavelocation(), videoStreamStop()
  */
    void videoStreamStart(QString fname = "");

    //! Stop writing a video stream
    /*!
   * @sa videoStreamStart()
  */
    void videoStreamStop(void);

    //! Start or stop capturing an image
    void enableCapture(bool capture);

    //! Start or stop capturing images
    void enableAcquire(bool acquire);

    //! Enable or disable stereo matching
    void enableMatching(bool match);

    //! Enable or disable image rectification
    void enableRectify(bool rectify);

    //! Emable or disable disparity map reprojection to 3D
    void enableReproject(bool reproject);

    void enableSwapLeftRight(bool swap);


    //! Set the point cloud clipping distance closest to the camera (i.e. specify the closest distance to display)
    /*!
  * @param[in] zmin Distance to the camera in metres
  */
    void setVisualZmin(double zmin);

    //! Set the point cloud clipping distance furthest from the camera (i.e. specify the farthest distance to display)
    /*!
  * @param[in] zmax Distance to the camera in metres
  */
    void setVisualZmax(double zmax);

    //! Toggle saving date in filename
    void toggleDateInFilename(int state);

    //! Save the current 3D reconstruction to a .PLY file
    void savePointCloud();

    //! Set the save directory
    /*!
  * @param dir Desired save directory, will attempt to create if it doesn't exist.
  */
    void setSavelocation(QString dir){

        if(!QDir(dir).exists()){
            auto saved = QDir(dir);
            saved.mkpath(".");
        }

        save_directory = dir;
    }

private slots:
    void register_stereo_capture(void);
    void try_capture();
    void capture_and_process();

    //! Grab and process a frame from the camera
    /*!
    * This will perform an image capture, followed by optional rectification, matching and reprojection
    * @sa enableRectify(), enableMatching(), enableReproject()
    */
    void process_stereo(void);

private:
    qint64 frames = 0;

    bool includeDateInFilename = false;
    bool acquiring = false;
    bool matching = false;
    bool rectifying = false;
    bool swappingLeftRight = false;
    bool reprojecting = false;
    bool has_cuda = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud;
    QString save_directory = ".";

    bool captured_left = false;
    bool captured_right = false;

    cv::VideoCapture stereo_video;
    bool acquiring_video = false;

    //! Initialise a video stream
    /*!
  * @param[in] writer OpenCV VideoWriter pointer to use
  * @param[in] filename Output filename
  * @param[in] imsize Image size
  * @param[in] fps Framerate
  * @param[in] codec FOURCC video codec to use
  */
    bool videoStreamInit(cv::VideoWriter* writer, QString filename, cv::Size imsize, double fps = 60.0, int codec = CV_FOURCC('H', '2', '6', '4') );

    //! Block until a capture has finished
    void finishCapture(void);

    //! Save an image
    /*!
  * @param[in] fname Output filename
  */
    void saveImage(QString fname);

    //! Load rectification maps from calibration files
    /*!
  * @param[in] src_l Left image rectification map file
  * @param[in] src_r Right image rectification map file
  * @return true/false whether the file was loaded successfully
  */
    bool loadRectificationMaps(QString src_l, QString src_r);

    //! Load camera intrinsic/extinrisc calibration files
    /*!
  * @param[in] left_cal Left camera calibration parameter file
  * @param[in] right_cal Right camera calibration parameter file
  * @param[in] stereo_cal Stereo camera calibration parameter file
  * @return true/false whether the filse were loaded successfully
  */
    bool loadCalibration(QString left_cal, QString right_cal,
                         QString stereo_cal);

    //! Rectify the current stereo image pair
    /*!
  * Note this will update the image matrices: #left_remapped and #right_remapped
  */
    void rectifyImages(void);

    //! Wrapper around OpenCV rectify function for paralell calls.
    /*!
  * @param[in] src Input image
  * @param[out] dst Output image
  * @param[in] rmapx X rectification map
  * @param[in] rmapy Y rectification map
  */
    void remap_parallel(cv::Mat src, cv::Mat &dst, cv::Mat rmapx,
                        cv::Mat rmapy);

    //! Project the disparity map to 3D
    /*!
  * Call prior to getPointCloud()
  * Internally this:
  *
  * 1. Downscales the disparity map by a factor of 16 (to convert the OpenCV representation into actual disparities)
  * 2. Use the calibration Q matrix to reproject the image to 3D
  * 3. Creates a PCL point cloud and textures it using image intensity data
  * 4. Filters the cloud based on the #zmin, #zmax parameters. Any points farther than 10m are discarded as they're likely to be
  *    wrong anyway.
  *
  * @sa setVisualZmin(), setVisualZmax(), getPointCloud()
  */
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

    AbstractStereoMatcher *matcher = nullptr;

    double visualisation_min_z = 0.2;
    double visualisation_max_z = 2;

protected:

    int frame_rate = 30;
    int image_width = 0;
    int image_height = 0;
    cv::Size image_size;

    QElapsedTimer frametimer;

    bool rectification_valid = false;
    bool calibration_valid = false;
    bool captured_stereo = false;
    bool capturing = false;

protected slots:
    void register_right_capture(void);
    void register_left_capture(void);

};

inline AbstractStereoCamera::~AbstractStereoCamera() { emit finished(); }

//! Wrapper around cv::imwrite for saving in parallel
/*!
* Saves an image, can also be called sequentially.
*
* @param[in] fname Output filename
* @param[in] src Image matrix
*
* @return True/false if the write was successful
*/
bool write_parallel(std::string fname, cv::Mat src);

#endif  // ABSTRACTSTEREOCAMERA_H
