/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#include "arduinocommscameracontrol.h"

void ArduinoCommsCameraControl::updateFPS(int fps){
    QString message = QString::number(fps) + "\n";
    if (isConnected()){
        write(message);
    } else {
        open(m_serialPortInfo,m_baudrate);
    }
}
