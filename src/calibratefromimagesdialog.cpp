/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "calibratefromimagesdialog.h"
#include "ui_calibratefromimagesdialog.h"

CalibrateFromImagesDialog::CalibrateFromImagesDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::CalibrateFromImagesDialog) {
  ui->setupUi(this);

  connect(ui->calibrationPathButton, SIGNAL(clicked(bool)), this,
          SLOT(selectOutputPath()));
  connect(ui->leftPathButton, SIGNAL(clicked(bool)), this,
          SLOT(selectLeftImageRoot()));
  connect(ui->rightPathButton, SIGNAL(clicked(bool)), this,
          SLOT(selectRightImageRoot()));
  connect(ui->findButton, SIGNAL(clicked(bool)), this, SLOT(findImages()));
  connect(ui->leftPath, SIGNAL(editingFinished()), this,
          SLOT(updateLeftPath()));
  connect(ui->rightPath, SIGNAL(editingFinished()), this,
          SLOT(updateRightPath()));
  connect(ui->outputPath, SIGNAL(editingFinished()), this,
          SLOT(updateOutputPath()));
  connect(ui->leftMask, SIGNAL(editingFinished()), this,
          SLOT(updateLeftMask()));
  connect(ui->rightMask, SIGNAL(editingFinished()), this,
          SLOT(updateRightMask()));
  connect(ui->okPushButton, SIGNAL(clicked(bool)), this,
          SLOT(okClicked()));
  connect(ui->cancelPushButton, SIGNAL(clicked(bool)), this,
          SLOT(cancelClicked()));

  left_file_model = new QFileSystemModel(this);
  right_file_model = new QFileSystemModel(this);

  settings = new QSettings("I3Dr", "Stereo Vision Toolkit", this);

  ui->leftPath->setText(settings->value("calLeftDir").toString());
  ui->rightPath->setText(settings->value("calRightDir").toString());
  ui->outputPath->setText(settings->value("calDir").toString());

  updateLeftPath();
  updateRightPath();
  updateOutputPath();
  updateLeftMask();
  updateRightMask();

  connect(left_file_model, SIGNAL(directoryLoaded(QString)), this,
          SLOT(setLeftImages()));
  connect(right_file_model, SIGNAL(directoryLoaded(QString)), this,
          SLOT(setRightImages()));

  left_file_model->setFilter(QDir::NoDotAndDotDot | QDir::Files);
  right_file_model->setFilter(QDir::NoDotAndDotDot | QDir::Files);
}

void CalibrateFromImagesDialog::okClicked() {
  emit run_calibration();
  return;
}

void CalibrateFromImagesDialog::cancelClicked() {
  close();
  return;
}

int CalibrateFromImagesDialog::getPatternCols(){
    return ui->patternCols->value();
}

int CalibrateFromImagesDialog::getPatternRows(){
    return ui->patternRows->value();
}

double CalibrateFromImagesDialog::getSquareSizeMm(){
    return ui->squareSizeBox->value() * 1e-3;
}

bool CalibrateFromImagesDialog::getSaveROS(){
    return ui->rosOutputCheckBox->isChecked();
}

CalibrateFromImagesDialog::~CalibrateFromImagesDialog() { delete ui; }

void CalibrateFromImagesDialog::setLeftImages() {
  QModelIndex parentIndex = left_file_model->index(left_path);
  int numRows = left_file_model->rowCount(parentIndex);

  for (int row = 0; row < numRows; ++row) {
    QModelIndex childIndex = left_file_model->index(row, 0, parentIndex);
    QString path = left_file_model->data(childIndex).toString();
    left_image_list.append(left_path + "/" + path);
  }

   checkImageCount();
}

void CalibrateFromImagesDialog::setRightImages() {
  QModelIndex parentIndex = right_file_model->index(left_path);
  int numRows = right_file_model->rowCount(parentIndex);

  for (int row = 0; row < numRows; ++row) {
    QModelIndex childIndex = right_file_model->index(row, 0, parentIndex);
    QString path = right_file_model->data(childIndex).toString();
    right_image_list.append(right_path + "/" + path);
  }

   checkImageCount();
}

void CalibrateFromImagesDialog::findImages() {
  if (left_path != "") {
    QStringList leftFilters;

    if (left_mask == "") {
      leftFilters << "*.png"
                  << "*.tif"
                  << "*.jpg"
                  << "*.jpeg";
    } else {
      leftFilters << left_mask;
    }

    left_file_model->setNameFilters(leftFilters);
    left_file_model->setRootPath(left_path);
    left_file_model->setNameFilterDisables(false);

    ui->leftImageTable->setModel(left_file_model);
    ui->leftImageTable->setRootIndex(left_file_model->index(left_path));

  } else {
    ui->leftImageTable->clearSpans();
  }

  if (right_path != "") {
    QStringList rightFilters;

    if (right_mask == "") {
      rightFilters << "*.png"
                   << "*.tif"
                   << "*.jpg"
                   << "*.jpeg";
    } else {
      rightFilters << right_mask;
    }

    right_file_model->setNameFilters(rightFilters);
    right_file_model->setRootPath(right_path);
    right_file_model->setNameFilterDisables(false);

    ui->rightImageTable->setModel(right_file_model);
    ui->rightImageTable->setRootIndex(right_file_model->index(right_path));
  } else {
    ui->rightImageTable->clearSpans();
  }
}

void CalibrateFromImagesDialog::checkImageCount(void){
    // Only allow user to continue if we've got enough cal images
    int n_left_images = left_image_list.size();
    int n_right_images = right_image_list.size();

    if(n_left_images >= 6 && n_right_images >= 6 && n_left_images == n_right_images){
        ui->okPushButton->setEnabled(true);
    }else{
        ui->okPushButton->setEnabled(false);
    }
}

void CalibrateFromImagesDialog::selectOutputPath(void){
    QFileDialog dialog;
    QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
    output_path = dialog.getExistingDirectory(
        this, tr("Calibration output path"), startPath,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (output_path != "") {
      ui->outputPath->setText(QDir::cleanPath(output_path));
      settings->setValue("calDir", output_path);
    }
}

void CalibrateFromImagesDialog::selectLeftImageRoot(void) {
  QFileDialog dialog;
  QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
  left_path = dialog.getExistingDirectory(
      this, tr("Left images path"), startPath,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (left_path != "") {
    ui->leftPath->setText(QDir::cleanPath(left_path));
    settings->setValue("calLeftDir", left_path);
  }
}

void CalibrateFromImagesDialog::selectRightImageRoot(void) {
  QFileDialog dialog;
  QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
  right_path = dialog.getExistingDirectory(
      this, tr("Right images path"), startPath,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (right_path != "") {
    ui->rightPath->setText(QDir::cleanPath(right_path));
    settings->setValue("calRightDir", right_path);
  }
}

void CalibrateFromImagesDialog::updateLeftPath(void) {
  QString dir = ui->leftPath->text();
  if (QDir(dir).exists()) {
    left_path = dir;
  }else{
      qDebug() << "Directory doesn't exist";
  }

  qDebug() << "Left path set to: " << left_path;

}

void CalibrateFromImagesDialog::updateRightPath(void) {
  QString dir = ui->rightPath->text();
  if (QDir(dir).exists()) {
    right_path = dir;
  }else{
      qDebug() << "Directory doesn't exist";
  }

  qDebug() << "Right path set to: " << right_path;
}

void CalibrateFromImagesDialog::updateOutputPath(void) {
  QString dir = ui->outputPath->text();
  if (QDir(dir).exists()) {
    output_path = dir;
  }else{
      qDebug() << "Directory doesn't exist";
  }

  qDebug() << "Output path set to: " << output_path;
}

void CalibrateFromImagesDialog::updateLeftMask(void) {
  left_mask = ui->leftMask->text();
}

void CalibrateFromImagesDialog::updateRightMask(void) {
  right_mask = ui->rightMask->text();
}
