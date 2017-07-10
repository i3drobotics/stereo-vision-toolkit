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
      QString("Between %1 and %2").arg(board->minArea).arg(board->maxArea));

  ui->hTiltActual->setText(QString("%1").arg(board->horizontalTilt));
  ui->hTiltTarget->setText(
      QString("Between %1 and %2").arg(board->minHt).arg(board->maxHt));

  ui->vTiltActual->setText(QString("%1").arg(board->verticalTilt));
  ui->vTiltTarget->setText(
      QString("Between %1 and %2").arg(board->minVt).arg(board->maxVt));
}

CalibrationDialog::~CalibrationDialog() {
  emit stopCalibration();
  delete ui;
}
