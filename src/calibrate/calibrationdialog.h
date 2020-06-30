/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <math.h>
#include <QDialog>
#include <QFileDialog>
#include <QSettings>
#include "stereocalibrate.h"
#include "abstractstereocamera.h"
#include "calibrateconfirmdialog.h"
#include "chessboard.h"

namespace Ui {
class CalibrationDialog;
}

//!  Stereo calibration dialog
/*!
  Dialog for performing stereo calibration from live camera feed
*/
class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    //! Constructor
    /*! \param stereo_cam stereo camera to grab image feed from */
    explicit CalibrationDialog(AbstractStereoCamera* stereo_cam, QWidget *parent = 0);
    //! Get number of columns in pattern
    /*! \return number of columns */
    int getPatternCols();
    //! Get number of rows in pattern
    /*! \return number of rows */
    int getPatternRows();
    //! Get square size of pattern
    /*! \return square size (in mm) */
    double getSquareSizeMm();
    //! Get if saving ROS calibration files
    /*! \return if saving ROS */
    bool getSaveROS();
    //! Get left images
    /*! \return list of images */
    QList<cv::Mat> getLeftImages(){return left_image_list;}
    //! Get right images
    /*! \return list of images */
    QList<cv::Mat> getRightImages(){return right_image_list;}
    //! Get output path where files are being saved
    /*! \return output path for files */
    QString getOutputPath(void){return output_path;}
    ~CalibrationDialog();

private:
    //! QT UI dialog
    Ui::CalibrationDialog *ui;
    //! Stereo camera
    AbstractStereoCamera *stereo_cam;
    //! Is calibration running
    bool running = false;

    //! Output path for calibration files
    QString output_path = "";
    //! List of left images to use in calibration
    QList<cv::Mat> left_image_list;
    //! List of right images to use in calibration
    QList<cv::Mat> right_image_list;
    //! App settings used to retreive 'calDir'
    QSettings *settings;

    //! Minimum change in x and y for calibration success
    int p_min_change;
    //! Minimum change in skew for calibration success
    double skew_min_change;
    //! Minimum change in size for calibration success
    int size_min_change;

    //! Image width
    int image_width;
    //! Image height
    int image_height;

    //! Lowest x value of grid measured
    int x_lowest = -1;
    //! Highest x value of grid measured
    int x_highest = -1;
    //! Lowest y value of grid measured
    int y_lowest = -1;
    //! Highest y value of grid measured
    int y_highest = -1;
    //! Lowest y skew of grid measured
    double min_y_skew = -1;
    //! Hightest y skew of grid measured
    double max_y_skew = -1;
    //! Lowest x skew of grid measured
    double min_x_skew = -1;
    //! Highest x skew of grid measured
    double max_x_skew = -1;
    //! Lowest size of grid measured
    double min_size = -1;
    //! Highest size of grid measured
    double max_size = -1;

    //! Downsample amount (from image size measurement)
    double m_scale = 1;

    //! Check images
    /*! Check validity of left and right images
     * \param left image
     * \param right image
     * \return validity of images */
    bool checkImages(cv::Mat left, cv::Mat right);
    //! Add images
    /*! Add image pair to list of image to be processed
     * \param left image
     * \param right image */
    void addImages(cv::Mat left, cv::Mat right);
    //! Find corners
    /*! Search for grid pattern corners in image
     * \param image input
     * \param corners output list of corners found
     * \return corners found */
    bool findCorners(cv::Mat image,std::vector<cv::Point2f> &corners);
    //! Get board rectangle
    /*! Outer rectangle of grid pattern from grid points (uses cv::minAreaRect)
     * \param corners points from 'findCorners'
     * \return rectangle that fits corners */
    cv::RotatedRect getBoardRect(std::vector<cv::Point2f> corners);
    //! Get outside corner points of grid
    /*! Outer corner points of grid pattern from grid points
     * \param corners points from 'findCorners'
     * \return four corner points */
    std::vector<cv::Point2f> getBoardOutsideCorners(std::vector<cv::Point2f> corners);
    //! Get corner index in grid points
    /*!
     * \param row row
     * \param col column
     * \return index of row and column in grid points */
    int getCornerIndex(int row, int col);
    //! Check XY is different enough
    /*! To reduce the number of images used in calibration only images with a
     * significant diffence to previous images are used. This checks the X and Y are different enough.
     * \param left_rot_rect left rectangle (from getBoardRect)
     * \param right_rot_rect right rectangle (from getBoardRect)
     * \return XY is different enough to be included */
    bool checkXYDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect);
    //! Check size is different enough
    /*! To reduce the number of images used in calibration only images with a
     * significant diffence to previous images are used. This checks the grid sizes are different enough.
     * \param left_rot_rect left rectangle (from getBoardRect)
     * \param right_rot_rect right rectangle (from getBoardRect)
     * \return grid size is different enough to be included */
    bool checkSizeDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect);
    //! Check skew is different enough
    /*! To reduce the number of images used in calibration only images with a
     * significant diffence to previous images are used. This checks the skew is different enough.
     * \param left_outer_corners left rectangle (from getBoardOutsideCorners)
     * \param right_outer_corners right rectangle (from getBoardOutsideCorners)
     * \return skew is different enough to be included */
    bool checkSkewDiff(std::vector<cv::Point2f> left_outer_corners, std::vector<cv::Point2f> right_outer_corners); // uses realitive side length for skew measure
    //! Downsample image
    /*! Image is downsampled by m_scale */
    cv::Mat downsampleImage(cv::Mat image);
    //! Update progress dialog
    void updateProgress();

public slots:
    //! Start calibration (emits startCalibration)
    void calibrate();
    //! Toggle capturing of images from stereo camera
    void toggleRun();
    //! Process images
    void processImages();
    //! Select output path (opens folder dialog)
    void selectOutputPath();
    //! Update output path (checks path is valid)
    void updateOutputPath(void);

signals:
    //! Emit to start calibration
    void startCalibration();
    //! Emit to stop calibration
    void stopCalibration();
};

#endif // CALIBRATIONDIALOG_H
