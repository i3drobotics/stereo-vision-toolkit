/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "matcheri3drsgm.h"

//Initialise matcher
void MatcherI3DRSGM::init()
{
    QString qparam_file = QCoreApplication::applicationDirPath() + "/i3drsgm.param";
    std::string param_file = qparam_file.toStdString();

    QString qtmp_param_file = QStandardPaths::writableLocation(QStandardPaths::TempLocation) + "/tmp.param";
    std::string tmp_param_file = qtmp_param_file.toStdString();

    i3drsgm = new I3DRSGM(tmp_param_file,param_file);
}

int MatcherI3DRSGM::getStatus()
{
    return this->i3drsgm->getStatus();
}

bool MatcherI3DRSGM::isLicenseValid()
{
    return this->i3drsgm->isLicenseValid();
}

int MatcherI3DRSGM::getErrorDisparity(void)
{
    return this->i3drsgm->getErrorDisparity();
}

//compute disparity
void MatcherI3DRSGM::forwardMatch()
{
    cv::Mat oDisparity;
    oDisparity = this->i3drsgm->forwardMatch(*left,*right);
    oDisparity.convertTo(disparity_lr, CV_32F, -16);
}

//backward match disparity
void MatcherI3DRSGM::backwardMatch()
{
    cv::Mat oDisparity;
    oDisparity = this->i3drsgm->backwardMatch(*left,*right);
    oDisparity.convertTo(disparity_rl, CV_32F, -16);
}

void MatcherI3DRSGM::enableCPU(bool enable)
{
    this->i3drsgm->enableCPU(enable);
}

void MatcherI3DRSGM::setDisparityError(int val)
{
    this->i3drsgm->setDisparityError(val);
}

void MatcherI3DRSGM::setSpeckleDifference(float diff)
{
  this->i3drsgm->setSpeckleDifference(diff);
}

void MatcherI3DRSGM::setSpeckleSize(int size)
{
   this->i3drsgm->setSpeckleSize(size);
}

void MatcherI3DRSGM::setMatchCosts(float P1, float P2){
    setP1(P1);
    setP2(P2);
}

void MatcherI3DRSGM::setP1(float P1)
{
   this->i3drsgm->setP1(P1);
}

void MatcherI3DRSGM::setP2(float P2)
{
   this->i3drsgm->setP2(P2);
}

void MatcherI3DRSGM::setWindowSize(int census_size)
{
   this->i3drsgm->setWindowSize(census_size);
}

void MatcherI3DRSGM::setDisparityShift(int shift)
{
    this->i3drsgm->setDisparityShift(shift);
}

void MatcherI3DRSGM::maxPyramid(int pyramid_num)
{
   this->i3drsgm->maxPyramid(pyramid_num);
}

void MatcherI3DRSGM::enablePyramid(bool enable, int pyramid_num)
{
    this->i3drsgm->enablePyramid(enable, pyramid_num);
}

void MatcherI3DRSGM::enableSubpixel(bool enable)
{
    this->i3drsgm->enableSubpixel(enable);
}

void MatcherI3DRSGM::setDisparityRange(int n)
{
    this->i3drsgm->setDisparityRange(n);
}

void MatcherI3DRSGM::enableTextureDSI(bool enable)
{
  this->i3drsgm->enableTextureDSI(enable);
}

void MatcherI3DRSGM::enableInterpolation(bool enable)
{
    this->i3drsgm->enableInterpolation(enable);
}

void MatcherI3DRSGM::enableOcclusionDetection(bool enable)
{
    this->i3drsgm->enableOcclusionDetection(enable);
}

void MatcherI3DRSGM::enableOccInterpol(bool enable)
{
    this->i3drsgm->enableOccInterpol(enable);
}
