/*!
 * @file cvsupport.h
 * @authors Ben Knight (bknight@i3drobotics.com)
 * @brief Support class for using openCV
 * @version 1.3
 * @date 2020-07-02
 * 
 * @copyright Copyright (c) I3D Robotics Ltd, 2020
 * 
 */

#ifndef CVSUPPORT_H
#define CVSUPPORT_H

#include <opencv2/opencv.hpp>

/**
 * @brief Support class for using openCV
 */
class CVSupport
{
public:
    /**
     * @brief Wrapper around cv::imwrite for saving in parallel
     * 
     * Saves an image, can also be called sequentially.
     * 
     * @param[in] fname Output filename
     * @param[in] src Image matrix
     *
     * @return True/false if the write was successful
     */
    static bool write_parallel(std::string fname, cv::Mat src)
    {
        std::vector<int> params;
        int compression_level = 0;
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(compression_level);
        //params.push_back(cv::IMWRITE_PNG_STRATEGY);
        //params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT)

        return cv::imwrite(fname, src, params);
    }
};

#endif // CVSUPPORT_H
