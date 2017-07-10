#include "calibratefromimagesdialog.h"
#include "ui_calibratefromimagesdialog.h"

CalibrateFromImagesDialog::CalibrateFromImagesDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::CalibrateFromImagesDialog) {
  ui->setupUi(this);

  connect(ui->leftPathButton, SIGNAL(clicked(bool)), this,
          SLOT(selectLeftImageRoot()));
  connect(ui->rightPathButton, SIGNAL(clicked(bool)), this,
          SLOT(selectRightImageRoot()));
  connect(ui->findButton, SIGNAL(clicked(bool)), this, SLOT(findImages()));
  connect(ui->leftPath, SIGNAL(editingFinished()), this,
          SLOT(updateLeftPath()));
  connect(ui->rightPath, SIGNAL(editingFinished()), this,
          SLOT(updateRightPath()));
  connect(ui->leftMask, SIGNAL(editingFinished()), this,
          SLOT(updateLeftMask()));
  connect(ui->rightMask, SIGNAL(editingFinished()), this,
          SLOT(updateRightMask()));
  connect(ui->okPushButton, SIGNAL(clicked(bool)), this,
          SLOT(runCalibration()));

  leftFileModel = new QFileSystemModel(this);
  rightFileModel = new QFileSystemModel(this);

  connect(leftFileModel, SIGNAL(directoryLoaded(QString)), this,
          SLOT(setLeftImages()));
  connect(rightFileModel, SIGNAL(directoryLoaded(QString)), this,
          SLOT(setRightImages()));

  leftFileModel->setFilter(QDir::NoDotAndDotDot | QDir::Files);
  rightFileModel->setFilter(QDir::NoDotAndDotDot | QDir::Files);
}

void CalibrateFromImagesDialog::runCalibration() {
  StereoCalibrate calibrate(this, NULL);
  cv::Size pattern(ui->patternCols->value(), ui->patternRows->value());
  calibrate.setPattern(pattern, 1e-3 * ui->squareSizeBox->value());
  calibrate.fromImages(leftImageList, rightImageList);

  close();

  return;
}

CalibrateFromImagesDialog::~CalibrateFromImagesDialog() { delete ui; }

void CalibrateFromImagesDialog::setLeftImages() {
  QModelIndex parentIndex = leftFileModel->index(leftPath);
  int numRows = leftFileModel->rowCount(parentIndex);

  for (int row = 0; row < numRows; ++row) {
    QModelIndex childIndex = leftFileModel->index(row, 0, parentIndex);
    QString path = leftFileModel->data(childIndex).toString();
    leftImageList.append(leftPath + "/" + path);
  }
}

void CalibrateFromImagesDialog::setRightImages() {
  QModelIndex parentIndex = rightFileModel->index(leftPath);
  int numRows = rightFileModel->rowCount(parentIndex);

  for (int row = 0; row < numRows; ++row) {
    QModelIndex childIndex = rightFileModel->index(row, 0, parentIndex);
    QString path = rightFileModel->data(childIndex).toString();
    rightImageList.append(rightPath + "/" + path);
  }
}

void CalibrateFromImagesDialog::findImages() {
  if (leftPath != "") {
    QStringList leftFilters;

    if (leftMask == "") {
      leftFilters << "*.png"
                  << "*.tif"
                  << "*.jpg"
                  << "*.jpeg";
    } else {
      leftFilters << leftMask;
    }

    leftFileModel->setNameFilters(leftFilters);
    leftFileModel->setRootPath(leftPath);
    leftFileModel->setNameFilterDisables(false);

    ui->leftImageTable->setModel(leftFileModel);
    ui->leftImageTable->setRootIndex(leftFileModel->index(leftPath));

  } else {
    ui->leftImageTable->clearSpans();
  }

  if (rightPath != "") {
    QStringList rightFilters;

    if (rightMask == "") {
      rightFilters << "*.png"
                   << "*.tif"
                   << "*.jpg"
                   << "*.jpeg";
    } else {
      rightFilters << rightMask;
    }

    rightFileModel->setNameFilters(rightFilters);
    rightFileModel->setRootPath(rightPath);
    rightFileModel->setNameFilterDisables(false);

    ui->rightImageTable->setModel(rightFileModel);
    ui->rightImageTable->setRootIndex(rightFileModel->index(rightPath));
  } else {
    ui->rightImageTable->clearSpans();
  }
}

void CalibrateFromImagesDialog::selectLeftImageRoot(void) {
  QFileDialog dialog;
  QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
  leftPath = dialog.getExistingDirectory(
      this, tr("Left images path"), startPath,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (leftPath != "") {
    ui->leftPath->setText(QDir::cleanPath(leftPath));
  }
}

void CalibrateFromImagesDialog::selectRightImageRoot(void) {
  QFileDialog dialog;
  QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
  rightPath = dialog.getExistingDirectory(
      this, tr("Right images path"), startPath,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (rightPath != "") {
    ui->rightPath->setText(QDir::cleanPath(rightPath));
  }
}

void CalibrateFromImagesDialog::updateLeftPath(void) {
  QString dir = ui->leftPath->text();
  if (QDir(dir).exists()) {
    leftPath = dir;
  }
}

void CalibrateFromImagesDialog::updateRightPath(void) {
  QString dir = ui->rightPath->text();
  if (QDir(dir).exists()) {
    rightPath = dir;
  }
}

void CalibrateFromImagesDialog::updateLeftMask(void) {
  leftMask = ui->leftMask->text();
}

void CalibrateFromImagesDialog::updateRightMask(void) {
  rightMask = ui->rightMask->text();
}
