/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <QDialog>
#include "stereocalibrate.h"
#include "calibrateconfirmdialog.h"
#include "chessboard.h"

//!  Stereo calibration dialog
/*!
  Dialog for performing stereo calibration
*/

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrationDialog(QWidget *parent = 0, StereoCalibrate* calibrator = 0);
    ~CalibrationDialog();

private:
    Ui::CalibrationDialog *ui;
    StereoCalibrate *calibrator;

public slots:
    void setImageProgress(int current, int total);
    void updateLabels(Chessboard* board);

signals:
    void stopCalibration();
};

#endif // CALIBRATIONDIALOG_H
