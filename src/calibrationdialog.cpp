/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "calibrationdialog.h"
#include "ui_calibrationdialog.h"

CalibrationDialog::CalibrationDialog(QWidget* parent,
                                     StereoCalibrate* calibrator)
    : QDialog(parent), ui(new Ui::CalibrationDialog) {
  ui->setupUi(this);
  this->calibrator = calibrator;
  this->setAttribute(Qt::WA_DeleteOnClose);

  connect(ui->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
  connect(calibrator, SIGNAL(imageProgress(int, int)), this,
          SLOT(setImageProgress(int, int)));
  connect(calibrator, SIGNAL(chessboardFound(Chessboard*)), this,
          SLOT(updateLabels(Chessboard*)));
}

void CalibrationDialog::setImageProgress(int current, int total) {
  double progress = (double)current / (double)total;

  ui->calProgressBar->setValue((int)(progress * 100));
  ui->calProgressLabel->setText(
      QString("Image %1 of %2").arg(current + 1).arg(total));
}

void CalibrationDialog::updateLabels(Chessboard* board) {
  ui->areaActual->setText(QString("%1").arg(board->getArea()));

  ui->areaTarget->setText(
      QString("Between %1 and %2").arg(board->min_area).arg(board->max_area));

  ui->hTiltActual->setText(QString("%1").arg(board->horizontal_tilt));
  ui->hTiltTarget->setText(
      QString("Between %1 and %2").arg(board->min_horizontal_tilt).arg(board->max_horizontal_tilt));

  ui->vTiltActual->setText(QString("%1").arg(board->vertical_tilt));
  ui->vTiltTarget->setText(
      QString("Between %1 and %2").arg(board->min_vertical_tilt).arg(board->max_vertical_tilt));
}

CalibrationDialog::~CalibrationDialog() {
  emit stopCalibration();
  delete ui;
}
