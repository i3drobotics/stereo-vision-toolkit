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

    static void normaliseDisparity(cv::Mat inDisparity, cv::Mat &outNormalisedDisparity){
        cv::Mat disparity_norm;

        inDisparity.copyTo(disparity_norm);

        cv::normalize(disparity_norm, disparity_norm, 0, 255, cv::NORM_MINMAX);

        disparity_norm.convertTo(disparity_norm, CV_8U);

        outNormalisedDisparity = disparity_norm;
    }

    static cv::Vec3d genXYZ(cv::Matx44d Q_, int x_index, int y_index, float d){
        cv::Vec4d homg_pt = Q_ * cv::Vec4d((double)x_index, (double)y_index, (double)d, 1.0);

        float x = (float)homg_pt[0] / (float)homg_pt[3];
        float y = (float)homg_pt[1] / (float)homg_pt[3];
        float z = (float)homg_pt[2] / (float)homg_pt[3];

        cv::Vec3d xyz(x,y,z);
        return xyz;
    }

    static double genZ(cv::Matx44d Q_, int x_index, int y_index, float d){
        //TODO simplify this to only calculate what is nessacary for getting Z
        return genXYZ(Q_,x_index,y_index,d)[2];
    }

    static void getMinMaxDepth(cv::Mat inDisparity, cv::Mat Q, double &min_depth, double &max_depth){
        float min_disp = 10000;
        float max_disp = 0;

        min_depth = -1;
        max_depth = -1;

        for (int i = 0; i < inDisparity.rows; i++)
        {
            for (int j = 0; j < inDisparity.cols; j++)
            {
                float d = inDisparity.at<float>(i, j);
                if (d < 10000){
                    float d = inDisparity.at<float>(i, j);
                    if (d < min_disp){
                        float depth = CVSupport::genZ(Q,i,j,d);
                        if (depth > 0){ //depth must be in front of camera so positive
                            min_disp = d;
                            max_depth = depth;
                        }
                    }
                    if (d > max_disp){
                        max_disp = d;
                        min_depth = CVSupport::genZ(Q,i,j,d);
                    }
                }
            }
        }
    }

    static void getMinMaxDisparity(cv::Mat inDisparity, double &min_disp, double &max_disp){
        min_disp = 10000;
        max_disp = 0;

        for (int i = 0; i < inDisparity.rows; i++)
        {
            for (int j = 0; j < inDisparity.cols; j++)
            {
                float d = inDisparity.at<float>(i, j);
                if (d < 10000){
                    float d = inDisparity.at<float>(i, j);
                    if (d < min_disp){
                        min_disp = d;
                    }
                    if (d > max_disp){
                        max_disp = d;
                    }
                }
            }
        }
    }

    static void removeInvalidDisparity(cv::Mat inDisparity, cv::Mat &outValidDisparity){
        cv::Mat disparity_thresh;

        inDisparity.copyTo(disparity_thresh);

        double min_disp, max_disp;
        getMinMaxDisparity(disparity_thresh, min_disp, max_disp);

        for (int i = 0; i < disparity_thresh.rows; i++)
        {
            for (int j = 0; j < disparity_thresh.cols; j++)
            {
                float d = disparity_thresh.at<float>(i, j);
                if (d > max_disp || d < min_disp){
                    disparity_thresh.at<float>(i, j) = 0;
                }
            }
        }

        outValidDisparity = disparity_thresh;
    }

    static void disparity2colormap(cv::Mat inDisparity, cv::Mat &outColormap){
        // threshold value disparity
        cv::Mat validDisp;
        removeInvalidDisparity(inDisparity, validDisp);

        cv::Mat floatDisp;
        validDisp.convertTo(floatDisp, CV_32F);

        // normalise disparity
        cv::Mat normDisp;
        normaliseDisparity(validDisp, normDisp);

        // apply colormap
        int colourmap_index = 2;
        cv::Mat colormap;
        cv::applyColorMap(normDisp, colormap, colourmap_index);

        for (int x = 0; x < colormap.cols; x++) {
            for (int y = 0; y < colormap.rows; y++) {
                if (validDisp.at<float>(y,x) == 0)
                {
                    colormap.at<cv::Vec3b>(y, x)[0] = 0;
                    colormap.at<cv::Vec3b>(y, x)[1] = 0;
                    colormap.at<cv::Vec3b>(y, x)[2] = 0;
                }
            }
        }

        outColormap = colormap;
    }
};

#endif // CVSUPPORT_H
