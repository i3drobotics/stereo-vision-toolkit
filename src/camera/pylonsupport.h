/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef PYLONSUPPORT_H
#define PYLONSUPPORT_H

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

//!  Pylon support
/*!
  Support class for using pylon api. Includes functions for easy use of pylon api.
*/
class PylonSupport
{
public:
    PylonSupport();

    static bool grabImage2mat(const Pylon::CGrabResultPtr& ptrGrabResult, Pylon::CImageFormatConverter *formatConverter, cv::Mat& imageOut){
        bool res;
        if (ptrGrabResult == NULL){
            //qDebug() << "Camera grab pointer is null";
            res = false;
        } else {
            if (ptrGrabResult->GrabSucceeded())
            {
                int frameCols = ptrGrabResult->GetWidth();
                int frameRows = ptrGrabResult->GetHeight();

                if (frameCols == 0 || frameRows == 0){
                    //qDebug() << "Image buffer size is incorrect";
                    res = false;
                } else {
                    Pylon::CPylonImage pylonImage;
                    formatConverter->Convert(pylonImage, ptrGrabResult);
                    cv::Mat image_temp = cv::Mat(frameRows, frameCols, CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

                    if (image_temp.cols == 0 || image_temp.rows == 0){
                        //qDebug() << "Image result buffer size is incorrect";
                        res = false;
                    } else {
                        image_temp.copyTo(imageOut);
                        res = true;
                    }
                    pylonImage.Release();
                }
            } else {
                res = false;
            }
        }
        return res;
    }
};

#endif // PYLONSUPPORT_H
