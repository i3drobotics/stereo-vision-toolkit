#ifndef STEREOCAMERASUPPORT_H
#define STEREOCAMERASUPPORT_H

#include <QObject>
#include "stereocameradeimos.h"
#include "stereocamerafromvideo.h"
#include "stereocameraopencv.h"
#include "stereocameratis.h"
#include "stereocamerabasler.h"
#ifdef WITH_VIMBA
    #include "stereocameravimba.h"
#endif

class StereoCameraSupport : public QObject {
    Q_OBJECT

public:
    StereoCameraSupport();
    ~StereoCameraSupport();

public slots:
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> getDeviceList(void);

};

#endif // STEREOCAMERASUPPORT_H
