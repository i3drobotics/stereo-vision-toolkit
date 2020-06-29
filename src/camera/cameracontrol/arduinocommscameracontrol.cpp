#include "arduinocommscameracontrol.h"

void ArduinoCommsCameraControl::updateFPS(int fps){
    QString message = QString::number(fps) + "\n";
    if (isConnected()){
        write(message);
    } else {
        open(m_serialPortInfo,m_baudrate);
    }
}
