/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef CALIBRATEFROMIMAGESDIALOG_H
#define CALIBRATEFROMIMAGESDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QDebug>
#include <QDir>
#include <QStandardPaths>
#include <QFileSystemModel>
#include <QSettings>

#include <stereocalibrate.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace Ui {
class CalibrateFromImagesDialog;
}

//!  Calibrate from images dialog
/*!
  Dialog for performing stereo calibration from images
*/
class CalibrateFromImagesDialog : public QDialog
{
    Q_OBJECT

signals:
    //! Emit to run calibration
    void run_calibration();

public:
    explicit CalibrateFromImagesDialog(QWidget *parent = 0);
    ~CalibrateFromImagesDialog();
//    //! Get left images
//    /*! \return list of images */
//    QList<cv::Mat> getLeftImages(){return left_image_list;}
//    //! Get right images
//    /*! \return list of images */
//    QList<cv::Mat> getRightImages(){return right_image_list;}
    //! Get left image paths
    /*! \return list of images */
    QList<std::string> getLeftImagePaths(){return left_image_path_list;}
    //! Get right image paths
    /*! \return list of images */
    QList<std::string> getRightImagePaths(){return right_image_path_list;}
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
    //! Get output path where files are being saved
    /*! \return output path for files */
    QString getOutputPath(void){return output_path;}

private:
    //! QT UI dialog
    Ui::CalibrateFromImagesDialog *ui;

    //! Output path for calibration files
    QString output_path = "";
    //! Folderpath to search for left images
    QString left_path = "";
    //! Folderpath to search for right images
    QString right_path = "";
    //! Extension filters to search for left images (e.g. .JPEG)
    QString left_mask = "";
    //! Extension filters to search for right images (e.g. .JPEG)
    QString right_mask = "";

    //! Search model for finding left images
    QFileSystemModel *left_file_model;
    //! Search model for finding right images
    QFileSystemModel *right_file_model;

//    //! List of left images found
//    QList<cv::Mat> left_image_list;
//    //! List of right images found
//    QList<cv::Mat> right_image_list;

    //! List of left images filepath found
    QList<std::string> left_image_path_list;
    //! List of right images filepath found
    QList<std::string> right_image_path_list;

    //! App settings used to retreive 'calLeftDir', 'calRightDir' and 'calDir'
    QSettings *settings;

    //! Check number of images found
    /* Check same number of left and right images are found
     * and at least one of each where found */
    void checkImageCount(void);

    bool left_paths_checked = false;
    bool right_paths_checked = false;

private slots:
    //! Select left image root (open file dialog)
    void selectLeftImageRoot(void);
    //! Select right image root (open file dialog)
    void selectRightImageRoot(void);
    //! Select output path (open file dialog)
    void selectOutputPath();

    //! Update left mask
    void updateLeftMask(void);
    //! Update left mask
    void updateRightMask(void);

    //! Find images
    /* Run left and right file search models */
    void findImages(void);

    //! Update left path from GUI and check it's valid
    void updateLeftPath(void);
    //! Update right path from GUI and check it's valid
    void updateRightPath(void);
    //! Update output path from GUI and check it's valid
    void updateOutputPath(void);

    //! Set left images
    /* Get images from left file model */
    void setLeftImages(void);
    //! Set right images
    /* Get images from right file model */
    void setRightImages(void);

    //! Ok button clicked (Run calibration)
    void okClicked(void);
    //! Cancel button clicked (Close window)
    void cancelClicked(void);

};

#endif // CALIBRATEFROMIMAGESDIALOG_H
