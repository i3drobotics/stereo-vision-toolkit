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
    initMatcherParams();
}

void MatcherI3DRSGM::initMatcherParams()
{
    enableCPU(CPU);
    setDisparityError(disparityError);
    setSpeckleDifference(speckleDifference);
    setSpeckleSize(speckleSize);
    setMatchCosts(p1,p2);
    setWindowSize(windowSize);
    setDisparityShift(disparityShift);
    maxPyramid(maxPyramidLevel);
    enableSubpixel(subpixel);
    setDisparityRange(disparityRange);
    enableInterpolation(interpolation);
    //enableTextureDSI(textureDSI);
    //enableOcclusionDetection(occlusionDetection);
    //enableOccInterpol(occlusionInterpolation);
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
bool MatcherI3DRSGM::forwardMatch(cv::Mat left_img, cv::Mat right_img)
{
    //recreate handle if size changed
    //(to fix issue: 'Ressolution of frames changed in same handle, not allowed, create a new handle for new sized frames')
    if (sizeChangedThisFrame){
        delete(i3drsgm);
        init();
    }
    if (left_img.type() == CV_8UC1 && right_img.type() == CV_8UC1){
        cv::Mat oDisparity;
        oDisparity = this->i3drsgm->forwardMatch(left_img,right_img);
        oDisparity.convertTo(disparity_lr, CV_32F, -16);
        return true;
    } else {
        qDebug() << "Invalid image type for stereo matcher. MUST be CV_8UC1.";
        return false;
    }
}

//backward match disparity
bool MatcherI3DRSGM::backwardMatch(cv::Mat left_img, cv::Mat right_img)
{
    if (left_img.type() == CV_8UC1 && right_img.type() == CV_8UC1){
        cv::Mat oDisparity;
        oDisparity = this->i3drsgm->backwardMatch(left_img,right_img);
        oDisparity.convertTo(disparity_rl, CV_32F, -16);
        return true;
    } else {
        qDebug() << "Invalid image type for stereo matcher. MUST be CV_8UC1.";
        return false;
    }
}

void MatcherI3DRSGM::enableCPU(bool enable)
{
    this->CPU = enable;
    this->i3drsgm->enableCPU(enable);
}

void MatcherI3DRSGM::setDisparityError(int val)
{
    this->disparityError = val;
    this->i3drsgm->setDisparityError(val);
}

void MatcherI3DRSGM::setSpeckleDifference(float diff)
{
    this->speckleDifference = diff;
    this->i3drsgm->setSpeckleDifference(diff);
}

void MatcherI3DRSGM::setSpeckleSize(int size)
{
    this->speckleSize = size;
    this->i3drsgm->setSpeckleSize(size);
}

void MatcherI3DRSGM::setMatchCosts(float P1, float P2){
    setP1(P1);
    setP2(P2);
}

void MatcherI3DRSGM::setP1(float P1)
{
    this->p1 = P1;
    this->i3drsgm->setP1(P1);
}

void MatcherI3DRSGM::setP2(float P2)
{
    this->p2 = P2;
    this->i3drsgm->setP2(P2);
}

void MatcherI3DRSGM::setWindowSize(int census_size)
{
    this->windowSize = census_size;
    this->i3drsgm->setWindowSize(census_size);
}

void MatcherI3DRSGM::setDisparityShift(int shift)
{
    this->disparityShift = shift;
    this->i3drsgm->setDisparityShift(shift);
}

void MatcherI3DRSGM::maxPyramid(int pyramid_num)
{
    this->maxPyramidLevel = pyramid_num;
    this->i3drsgm->maxPyramid(pyramid_num);
}

void MatcherI3DRSGM::enablePyramid(bool enable, int pyramid_num)
{
    this->i3drsgm->enablePyramid(enable, pyramid_num);
}

void MatcherI3DRSGM::enableSubpixel(bool enable)
{
    this->subpixel = enable;
    this->i3drsgm->enableSubpixel(enable);
}

void MatcherI3DRSGM::setDisparityRange(int n)
{
    this->disparityRange = n;
    this->i3drsgm->setDisparityRange(n);
}

void MatcherI3DRSGM::enableTextureDSI(bool enable)
{
    this->textureDSI = enable;
    this->i3drsgm->enableTextureDSI(enable);
}

void MatcherI3DRSGM::enableInterpolation(bool enable)
{
    this->interpolation = enable;
    this->i3drsgm->enableInterpolation(enable);
}

void MatcherI3DRSGM::enableOcclusionDetection(bool enable)
{
    this->occlusionDetection = enable;
    this->i3drsgm->enableOcclusionDetection(enable);
}

void MatcherI3DRSGM::enableOccInterpol(bool enable)
{
    this->occlusionInterpolation = enable;
    this->i3drsgm->enableOccInterpol(enable);
}
