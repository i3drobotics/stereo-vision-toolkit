/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "MatcherWidgetOpenCVSGBM.h"
#include "ui_MatcherWidgetOpenCVSGBM.h"

MatcherWidgetOpenCVSGBM::MatcherWidgetOpenCVSGBM(QWidget* parent)
    : MatcherWidget(parent), ui(new Ui::MatcherWidgetOpenCVSGBM) {
  ui->setupUi(this);
  matcher = new MatcherOpenCVSGBM(this);

  ui->blockSizeSlider->setValue((matcher->getBlockSize() - 1) / 2.0);
  ui->blockSizeLabel->setText(QString::number(matcher->getBlockSize()));

  negative_disparity = matcher->getMinDisparity() <= 0;
  ui->checkBoxNegativeDisparity->setEnabled(negative_disparity);

  ui->disparityRangeSlider->setValue(matcher->getDisparityRange() / 16.0);
  ui->disparityRangeLabel->setText(
      QString::number(matcher->getDisparityRange()));

  min_disparity = matcher->getMinDisparity();
  ui->minDisparitySlider->setValue(abs(min_disparity));
  ui->minDisparityLabel->setText(QString::number(abs(min_disparity)));

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

  ui->speckleRangeSlider->setValue(matcher->getSpeckleFilterRange());
  ui->speckleRangeLabel->setText(
      QString::number(matcher->getSpeckleFilterRange()));

  ui->speckleWindowSlider->setValue(matcher->getSpeckleFilterWindow());
  ui->speckleWindowLabel->setText(
      QString::number(matcher->getSpeckleFilterWindow()));

  enableExtendDisparity(ui->checkBoxExtendDisparity->isChecked());
  enableWLSFilter(ui->checkBoxWLSFilter->isChecked());

  connect(ui->blockSizeSlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateBlockSize(int)));
  connect(ui->minDisparitySlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateMinDisparity(int)));
  connect(ui->checkBoxNegativeDisparity, SIGNAL(toggled(bool)), this,
          SLOT(enableNegativeDisparity(bool)));
  connect(ui->disparityRangeSlider, SIGNAL(valueChanged(int)), this,
          SLOT(updateDisparityRange(int)));

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
  connect(ui->checkBoxExtendDisparity, SIGNAL(toggled(bool)), this,
          SLOT(enableExtendDisparity(bool)));

  connect(ui->checkBoxWLSFilter, SIGNAL(toggled(bool)), this,
          SLOT(enableWLSFilter(bool)));

//  connect(ui->saveParametersButton, SIGNAL(clicked(bool)), this,
//          SLOT(onSaveClicked()));

  ui->saveParametersButton->setVisible(false);

#ifdef WITH_OPENCV_CONTRIB
    ui->checkBoxWLSFilter->setVisible(true);
    ui->lblWLSFilter->setVisible(true);
#else
    ui->checkBoxWLSFilter->setVisible(false);
    ui->lblWLSFilter->setVisible(false);
#endif
}

AbstractStereoMatcher* MatcherWidgetOpenCVSGBM::getMatcher() { return matcher; }

void MatcherWidgetOpenCVSGBM::onSaveClicked() { matcher->saveParams(); }

void MatcherWidgetOpenCVSGBM::enableNegativeDisparity(bool enable){
    negative_disparity = enable;
    updateMinDisparity(this->min_disparity);
}

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

void MatcherWidgetOpenCVSGBM::enableWLSFilter(bool enable){
    matcher->setWLSFilterEnabled(enable);
    if (ui->speckleFilterCheck->isChecked()){
        enableSpeckleFilter(true);
    }
}

void MatcherWidgetOpenCVSGBM::enableExtendDisparity(bool enable) {
    if (enable){
        ui->disparityRangeSlider->setMaximum(128);
        ui->minDisparitySlider->setMaximum(2048);
    } else {
        ui->disparityRangeSlider->setMaximum(32);
        ui->minDisparitySlider->setMaximum(256);
    }
}

void MatcherWidgetOpenCVSGBM::updateUniquenessRatio(int ratio) {
  matcher->setUniquenessRatio(ratio);
}

void MatcherWidgetOpenCVSGBM::setImageWidth(int width) { image_width = width; }

void MatcherWidgetOpenCVSGBM::updateMinDisparity(int min_disparity) {
    this->min_disparity = min_disparity;
    if (negative_disparity){
        ui->minDisparityLabel->setNum(-min_disparity);
        matcher->setMinDisparity(-min_disparity);
    } else {
        ui->minDisparityLabel->setNum(min_disparity);
        matcher->setMinDisparity(min_disparity);
    }
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
