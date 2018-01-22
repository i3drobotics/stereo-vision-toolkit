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

#include <stereocalibrate.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace Ui {
class CalibrateFromImagesDialog;
}

class CalibrateFromImagesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrateFromImagesDialog(QWidget *parent = 0);
    ~CalibrateFromImagesDialog();

private:
    Ui::CalibrateFromImagesDialog *ui;

    QString left_path = "";
    QString right_path = "";
    QString left_mask = "";
    QString right_mask = "";

    QFileSystemModel *left_file_model;
    QFileSystemModel *right_file_model;

    QList<QString> left_image_list;
    QList<QString> right_image_list;

private slots:
    void selectLeftImageRoot(void);
    void selectRightImageRoot(void);
    void updateLeftMask(void);
    void updateRightMask(void);

    void findImages(void);

    void updateLeftPath(void);
    void updateRightPath(void);

    void setLeftImages(void);
    void setRightImages(void);

    void runCalibration(void);

};

#endif // CALIBRATEFROMIMAGESDIALOG_H
