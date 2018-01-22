/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "MatcherWidgetOpenCVSGBM.h"
#include "ui_MatcherWidgetOpenCVSGBM.h"

MatcherWidgetOpenCVSGBM::MatcherWidgetOpenCVSGBM(QWidget* parent,
                                                 cv::Size image_size)
    : MatcherWidget(parent), ui(new Ui::MatcherWidgetOpenCVSGBM) {
  ui->setupUi(this);
  matcher = new MatcherOpenCVSGBM(this, image_size);

  ui->blockSizeSlider->setValue((matcher->getBlockSize() - 1) / 2.0);
  ui->blockSizeLabel->setText(QString::number(matcher->getBlockSize()));

  ui->disparityRangeSlider->setValue(matcher->getDisparityRange() / 16.0);
  ui->disparityRangeLabel->setText(
      QString::number(matcher->getDisparityRange()));

  ui->minDisparitySlider->setValue(matcher->getMinDisparity());
  ui->minDisparityLabel->setText(QString::number(matcher->getMinDisparity()));

  ui->uniquenessRatioSlider->setValue(matcher->getUniquenessRatio());
  ui->uniquenessRatioLabel->setText(
      QString::number(matcher->getUniquenessRatio()));

  if (matcher->getSpeckleFilterWindow() <= 0 ||
      matcher->getSpeckleFilterRange() <= 0) {
    ui->speckleFilterCheck->setChecked(false);
  } else {
    ui->speckleFilterCheck->setChecked(true);
    ui->speckleRangeSlider->setEnabled(true);
    ui->speckleWindowSlider->setEnabled(true);
    ui->speckleRangeLabel->setEnabled(true);
    ui->speckleWindowLabel->setEnabled(true);
  }

  if (matcher->getDisp12MaxDiff() <= 0 || matcher->getDisp12MaxDiff() <= 0) {
    ui->consistencyCheck->setChecked(false);
  } else {
    ui->consistencyCheck->setChecked(true);
    ui->consistencyLabel->setEnabled(true);
    ui->consistencySlider->setEnabled(true);
  }

  ui->consistencySlider->setValue(matcher->getDisp12MaxDiff());
  ui->consistencyLabel->setText(QString::number(matcher->getDisp12MaxDiff()));

  ui->speckleRangeSlider->setValue(matcher->getSpeckleFilterRange());
  ui->speckleRangeLabel->setText(
      QString::number(matcher->getSpeckleFilterRange()));

  ui->speckleWindowSlider->setValue(matcher->getSpeckleFilterWindow());
  ui->speckleWindowLabel->setText(
      QString::number(matcher->getSpeckleFilterWindow()));

  connect(ui->blockSizeSlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateBlockSize(int)));
  connect(ui->minDisparitySlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateMinDisparity(int)));
  connect(ui->disparityRangeSlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateDisparityRange(int)));

  connect(ui->consistencySlider, SIGNAL(sliderMoved(int)), ui->consistencyLabel,
          SLOT(setNum(int)));
  connect(ui->consistencySlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updateConsistency(int)));

  connect(ui->uniquenessRatioSlider, SIGNAL(sliderMoved(int)),
          ui->uniquenessRatioLabel, SLOT(setNum(int)));
  connect(ui->uniquenessRatioSlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updateUniquenessRatio(int)));

  connect(ui->speckleRangeSlider, SIGNAL(sliderMoved(int)),
          ui->speckleRangeLabel, SLOT(setNum(int)));
  connect(ui->speckleWindowSlider, SIGNAL(sliderMoved(int)),
          ui->speckleWindowLabel, SLOT(setNum(int)));

  connect(ui->speckleRangeSlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updateSpeckleRange(int)));
  connect(ui->speckleWindowSlider, SIGNAL(sliderMoved(int)), this,
          SLOT(updateSpeckleWindow(int)));

  connect(ui->speckleFilterCheck, SIGNAL(toggled(bool)), this,
          SLOT(enableSpeckleFilter(bool)));
  connect(ui->consistencyCheck, SIGNAL(toggled(bool)), this,
          SLOT(enableConsistency(bool)));

  connect(ui->saveParametersButton, SIGNAL(clicked(bool)), this,
          SLOT(onSaveClicked()));
}

AbstractStereoMatcher* MatcherWidgetOpenCVSGBM::getMatcher() { return matcher; }

void MatcherWidgetOpenCVSGBM::onSaveClicked() { matcher->saveParams(); }

void MatcherWidgetOpenCVSGBM::updateSpeckleRange(int range) {
  matcher->setSpeckleFilterRange(range);
}

void MatcherWidgetOpenCVSGBM::updateSpeckleWindow(int window) {
  matcher->setSpeckleFilterWindow(window);
}

void MatcherWidgetOpenCVSGBM::enableSpeckleFilter(bool enable) {
  if (enable) {
    matcher->setSpeckleFilterRange(ui->speckleRangeSlider->value());
    matcher->setSpeckleFilterWindow(ui->speckleWindowSlider->value());

    ui->speckleRangeSlider->setEnabled(true);
    ui->speckleWindowSlider->setEnabled(true);
    ui->speckleRangeLabel->setEnabled(true);
    ui->speckleWindowLabel->setEnabled(true);
  } else {
    matcher->setSpeckleFilterRange(0);
    matcher->setSpeckleFilterWindow(0);

    ui->speckleRangeSlider->setEnabled(false);
    ui->speckleWindowSlider->setEnabled(false);
    ui->speckleRangeLabel->setEnabled(false);
    ui->speckleWindowLabel->setEnabled(false);
  }
}

void MatcherWidgetOpenCVSGBM::enableConsistency(bool enable) {
  if (enable) {
    matcher->setDisp12MaxDiff(ui->consistencySlider->value());

    ui->consistencyLabel->setEnabled(true);
    ui->consistencySlider->setEnabled(true);
  } else {
    matcher->setDisp12MaxDiff(-1);

    ui->consistencyLabel->setEnabled(false);
    ui->consistencySlider->setEnabled(false);
  }
}

void MatcherWidgetOpenCVSGBM::updateConsistency(int con) {
  ui->consistencyLabel->setNum(con);
  matcher->setDisp12MaxDiff(con);
}

void MatcherWidgetOpenCVSGBM::updateUniquenessRatio(int ratio) {
  matcher->setUniquenessRatio(ratio);
}

void MatcherWidgetOpenCVSGBM::setImageWidth(int width) { image_width = width; }

void MatcherWidgetOpenCVSGBM::updateMinDisparity(int min_disparity) {
  //if (image_width - min_disparity > disparity_range) {
    ui->minDisparityLabel->setNum(min_disparity);
    this->min_disparity = min_disparity;
    matcher->setMinDisparity(min_disparity);
  //}
}

void MatcherWidgetOpenCVSGBM::updateDisparityRange(int range) {
  disparity_range = range;
  ui->disparityRangeLabel->setNum(16 * range);
  matcher->setDisparityRange(16 * range);
}

void MatcherWidgetOpenCVSGBM::updateBlockSize(int size) {
  block_size = size;
  ui->blockSizeLabel->setNum(2 * size + 1);
  matcher->setBlockSize(2 * size + 1);
}

MatcherWidgetOpenCVSGBM::~MatcherWidgetOpenCVSGBM() { delete ui; }
