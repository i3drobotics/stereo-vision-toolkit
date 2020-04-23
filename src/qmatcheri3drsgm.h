#ifndef QMATCHERI3DRSGM_H
#define QMATCHERI3DRSGM_H

#include <abstractstereomatcher.h>
#include <matcheri3drsgm.h>
#include <QDir>
#include <QDebug>

#include <iostream>
#include <fstream>

//!  Qt interface for Matcher I3DRSGM
/*!
  Qt wrapper around Stereo matcher using I3DR's SGM algorithm
*/

class QMatcherI3DRSGM : public AbstractStereoMatcher {
    Q_OBJECT
public:
    explicit QMatcherI3DRSGM(QObject *parent = 0,
                            cv::Size image_size = cv::Size(0, 0))
        : AbstractStereoMatcher(parent, image_size) {
        init();
    }

    ~QMatcherI3DRSGM(void){}

    void parseConfig(std::string input_file);
    int getErrorDisparity();

    void setDisparityShift(int shift);
    void setDisparityRange(int n);
    void enableSubpixel(bool enable);
    void setP1(float P1);
    void setP2(float P2);
    void setMatchCosts(float P1, float P2);
    void setWindowSize(int census_size);
    void setSpeckleDifference(float diff);
    void setSpeckleSize(int size);
    void enableInterpolation(bool enable);
    void enableOcclusionDetection(bool enable);
    void enableOccInterpol(bool enable);
    void enableTextureDSI(bool enable);
    void enablePyramid(bool enable, int pyramid_num);
    void maxPyramid(int pyramid_num);

    float getP1(void){ return this->i3drsgm->getP1(); }
    float getP2(void){ return this->i3drsgm->getP2(); }
    int getDisparityRange(void){ return this->i3drsgm->getDisparityRange(); }
    int getCensusSize(void){ return this->i3drsgm->getCensusSize(); }
    bool getInterpolate(void){return this->i3drsgm->getInterpolate(); }
    bool getOcclusionDetect(void){ return this->i3drsgm->getOcclusionDetect(); }
    bool getSubpixel(void){return this->i3drsgm->getSubpixel(); }
    int getDisparityShift(void){return this->i3drsgm->getDisparityShift(); }

    void setNoDataValue(int val);
    void enableCPU(bool enable);

    void forwardMatch();
    void backwardMatch();

    int getStatus();

private:
    int min_disparity, disparity_range;

    void init();

    MatcherI3DRSGM * i3drsgm;
};

#endif  // QMATCHERI3DRSGM_H
