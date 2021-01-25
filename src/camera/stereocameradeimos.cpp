/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameradeimos.h"

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraDeimos::listSystems(){
    return StereoCameraTara::listSystems("See3CAM_Stereo");
}

StereoCameraDeimos::~StereoCameraDeimos(void) {
}
