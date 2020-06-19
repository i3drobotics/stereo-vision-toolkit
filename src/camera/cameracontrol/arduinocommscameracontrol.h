#ifndef ARDUINOCOMMSCAMERACONTROL_H
#define ARDUINOCOMMSCAMERACONTROL_H

#include "abstractarduinocoms.h"

class ArduinoCommsCameraControl : public AbstractArduinoComs
{
public:
    explicit ArduinoCommsCameraControl(QObject *parent = 0) :
        AbstractArduinoComs(parent)
    {}

    void updateFPS(int fps);
};

#endif // ARDUINOCOMMSCAMERACONTROL_H
