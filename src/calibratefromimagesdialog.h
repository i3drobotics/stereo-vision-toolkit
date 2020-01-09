/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
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

//!  Calibrate from images dialog
/*!
  Dialog for performing stereo calibration from images
*/

namespace Ui {
class CalibrateFromImagesDialog;
}

class CalibrateFromImagesDialog : public QDialog
{
    Q_OBJECT

signals:
    void run_calibration();

public:
    explicit CalibrateFromImagesDialog(QWidget *parent = 0);
    ~CalibrateFromImagesDialog();
    QList<QString> getLeftImages(){return left_image_list;}
    QList<QString> getRightImages(){return right_image_list;}
    int getPatternCols();
    int getPatternRows();
    double getSquareSizeMm();
    bool getSaveROS();
    QString getOutputPath(void){return output_path;}

private:
    Ui::CalibrateFromImagesDialog *ui;

    QString output_path = "";
    QString left_path = "";
    QString right_path = "";
    QString left_mask = "";
    QString right_mask = "";

    QFileSystemModel *left_file_model;
    QFileSystemModel *right_file_model;

    QList<QString> left_image_list;
    QList<QString> right_image_list;

    QSettings *settings;

    void checkImageCount(void);

private slots:
    void selectLeftImageRoot(void);
    void selectRightImageRoot(void);
    void selectOutputPath();

    void updateLeftMask(void);
    void updateRightMask(void);

    void findImages(void);

    void updateLeftPath(void);
    void updateRightPath(void);
    void updateOutputPath(void);

    void setLeftImages(void);
    void setRightImages(void);

    void okClicked(void);
    void cancelClicked(void);

};

#endif // CALIBRATEFROMIMAGESDIALOG_H
