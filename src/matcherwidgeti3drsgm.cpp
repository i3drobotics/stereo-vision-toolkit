/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "matcherwidgeti3drsgm.h"
#include "ui_matcherwidgeti3drsgm.h"

MatcherWidgetI3DRSGM::MatcherWidgetI3DRSGM(QWidget* parent,
                                           cv::Size image_size)
    : MatcherWidget(parent), ui(new Ui::MatcherWidgetI3DRSGM) {
    ui->setupUi(this);
    matcher = new QMatcherI3DRSGM(this, image_size);

    connect(ui->blockSizeSlider, SIGNAL(valueChanged(int)), this,
            SLOT(updateBlockSize(int)));
    connect(ui->minDisparitySlider, SIGNAL(valueChanged(int)), this,
            SLOT(updateMinDisparity(int)));
    connect(ui->disparityRangeSlider, SIGNAL(valueChanged(int)), this,
            SLOT(updateDisparityRange(int)));

    //connect(ui->pyramidLevelSlider, SIGNAL(sliderMoved(int)), this,
    //        SLOT(updatePyramidLevel(int)));

    connect(ui->interpolateCheck, SIGNAL(toggled(bool)), this,
              SLOT(enableInterpolatation(bool)));

    connect(ui->saveParametersButton, SIGNAL(clicked(bool)), this,
            SLOT(onSaveClicked()));

    updateBlockSize(ui->blockSizeSlider->value());
    updateMinDisparity(ui->minDisparitySlider->value());
    updateDisparityRange(ui->disparityRangeSlider->value());
    enableInterpolatation(ui->interpolateCheck->isChecked());

    updatePyramidLevel(6);
    matcher->enableOcclusionDetection(false);
    matcher->enableOccInterpol(false);
}

AbstractStereoMatcher* MatcherWidgetI3DRSGM::getMatcher() {
    return matcher;
}

void MatcherWidgetI3DRSGM::onSaveClicked() {
    //TODO save params to file for quick loading
    //matcher->saveParams();
}

void MatcherWidgetI3DRSGM::updatePyramidLevel(int level) {
    //ui->prefilterSizeLabel->setNum(level);
    matcher->maxPyramid(level);
}

void MatcherWidgetI3DRSGM::enableInterpolatation(bool enable) {
    matcher->enableInterpolation(enable);
}

void MatcherWidgetI3DRSGM::setImageWidth(int width) { image_width = width; }

void MatcherWidgetI3DRSGM::updateMinDisparity(int min_disparity) {
    ui->minDisparityLabel->setNum(min_disparity);
    this->min_disparity = min_disparity;

    double shift_p = (double)min_disparity / 20;
    matcher->setDisparityShift(shift_p);
}

void MatcherWidgetI3DRSGM::updateDisparityRange(int range) {
    disparity_range = range;
    ui->disparityRangeLabel->setNum(16 * range);

    int m_range = (16 * range) / 10;
    //force odd number
    if (m_range % 2 == 0)
    {
        m_range++;
    }
    matcher->setDisparityRange(m_range);
}

void MatcherWidgetI3DRSGM::updateBlockSize(int size) {
    block_size = size;
    ui->blockSizeLabel->setNum(2 * size + 1);

    int census_size = size;
    if (census_size % 2 == 0)
    {
        census_size++;
    }

    matcher->setWindowSize(census_size);
}

MatcherWidgetI3DRSGM::~MatcherWidgetI3DRSGM() { delete ui; }
