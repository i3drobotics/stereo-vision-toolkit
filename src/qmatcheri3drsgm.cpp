#include "qmatcheri3drsgm.h"

//Initialise matcher
void QMatcherI3DRSGM::init()
{
    QString qparam_file = QCoreApplication::applicationDirPath() + "/i3drsgm.param";
    std::string param_file = qparam_file.toStdString();

    QString qtmp_param_file = QCoreApplication::applicationDirPath() + "/tmp.param";
    std::string tmp_param_file = qtmp_param_file.toStdString();

    i3drsgm = new MatcherI3DRSGM(tmp_param_file,param_file);
}

int QMatcherI3DRSGM::getStatus()
{
    return this->i3drsgm->getStatus();
}

int QMatcherI3DRSGM::getErrorDisparity(void)
{
    return this->i3drsgm->getErrorDisparity();
}

//compute disparity
void QMatcherI3DRSGM::forwardMatch()
{
    cv::Mat oDisparity;
    oDisparity = this->i3drsgm->forwardMatch(*left,*right);
    oDisparity.convertTo(disparity_lr, CV_32F, -16);
}

//backward match disparity
void QMatcherI3DRSGM::backwardMatch()
{
    cv::Mat oDisparity;
    oDisparity = this->i3drsgm->backwardMatch(*left,*right);
    oDisparity.convertTo(disparity_rl, CV_32F, -16);
}

void QMatcherI3DRSGM::enableCPU(bool enable)
{
    this->i3drsgm->enableCPU(enable);
}

void QMatcherI3DRSGM::setNoDataValue(int val)
{
    this->i3drsgm->setNoDataValue(val);
}

void QMatcherI3DRSGM::setSpeckleDifference(float diff)
{
  this->i3drsgm->setSpeckleDifference(diff);
}

void QMatcherI3DRSGM::setSpeckleSize(int size)
{
   this->i3drsgm->setSpeckleSize(size);
}

void QMatcherI3DRSGM::setMatchCosts(float P1, float P2){
    setP1(P1);
    setP2(P2);
}

void QMatcherI3DRSGM::setP1(float P1)
{
   this->i3drsgm->setP1(P1);
}

void QMatcherI3DRSGM::setP2(float P2)
{
   this->i3drsgm->setP2(P2);
}

void QMatcherI3DRSGM::setWindowSize(int census_size)
{
   this->i3drsgm->setWindowSize(census_size);
}

void QMatcherI3DRSGM::setDisparityShift(int shift)
{
    this->i3drsgm->setDisparityShift(shift);
}

void QMatcherI3DRSGM::maxPyramid(int pyramid_num)
{
   this->i3drsgm->maxPyramid(pyramid_num);
}

void QMatcherI3DRSGM::enablePyramid(bool enable, int pyramid_num)
{
    this->i3drsgm->enablePyramid(enable, pyramid_num);
}

void QMatcherI3DRSGM::enableSubpixel(bool enable)
{
    this->i3drsgm->enableSubpixel(enable);
}

void QMatcherI3DRSGM::setDisparityRange(int n)
{
    this->i3drsgm->setDisparityRange(n);
}

void QMatcherI3DRSGM::enableTextureDSI(bool enable)
{
  this->i3drsgm->enableTextureDSI(enable);
}

void QMatcherI3DRSGM::enableInterpolation(bool enable)
{
    this->i3drsgm->enableInterpolation(enable);
}

void QMatcherI3DRSGM::enableOcclusionDetection(bool enable)
{
    this->i3drsgm->enableOcclusionDetection(enable);
}

void QMatcherI3DRSGM::enableOccInterpol(bool enable)
{
    this->i3drsgm->enableOccInterpol(enable);
}
