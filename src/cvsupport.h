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
    static cv::Mat createRGBD32FC4(cv::Mat color_img, cv::Mat disparity){
        std::vector<cv::Mat> rgbChannels(3);
        cv::split(color_img, rgbChannels);
        cv::Mat r = rgbChannels[0];
        cv::Mat g = rgbChannels[1];
        cv::Mat b = rgbChannels[2];

        //convert color channels to float to keep precsion in the rgbd image
        b.convertTo(b,CV_32FC1, 1.0/255.0);
        g.convertTo(g,CV_32FC1, 1.0/255.0);
        r.convertTo(r,CV_32FC1, 1.0/255.0);

        std::vector<cv::Mat> channels;
        channels.push_back(r);
        channels.push_back(g);
        channels.push_back(b);
        channels.push_back(disparity);
        cv::Mat rgbd;
        cv::merge(channels, rgbd);
        return rgbd;
    }

    static cv::Mat createRGBD16UC4(cv::Mat color_img, cv::Mat disparity){
        std::vector<cv::Mat> rgbChannels(3);
        cv::split(color_img, rgbChannels);
        cv::Mat r = rgbChannels[0];
        cv::Mat g = rgbChannels[1];
        cv::Mat b = rgbChannels[2];
        cv::Mat d = disparity.clone();

        //convert color channels to float to keep precsion in the rgbd image
        b.convertTo(b,CV_16SC1*10);
        g.convertTo(g,CV_16SC1*10);
        r.convertTo(r,CV_16SC1*10);
        d.convertTo(d,CV_16SC1*10);

        std::vector<cv::Mat> channels;
        channels.push_back(r);
        channels.push_back(g);
        channels.push_back(b);
        channels.push_back(d);
        cv::Mat rgbd;
        cv::merge(channels, rgbd);
        int merge_type = rgbd.type();
        return rgbd;
    }

    static cv::Mat genWImage(cv::Mat disp, cv::Mat Q, float downsample_factor = 1.0){
        float q32 = Q.at<float>(3, 2);
        float q33 = Q.at<float>(3, 3);
        float d, w;
        cv::Mat wImage = cv::Mat(disp.rows, disp.cols, CV_32FC3, cv::Scalar(0));

        float downsample_rate = 1 / downsample_factor;

        for (int i = 0; i < disp.rows; i++)
        {
            for (int j = 0; j < disp.cols; j++)
            {
                d = disp.at<float>(i, j);
                if (d != 0)
                {
                    w = ((d * downsample_rate) * q32) + q33;
                    wImage.at<float>(i,j) = w;
                }
            }
        }

        return wImage;
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
        min_depth = 10000;
        max_depth = 0;

        double min_disp = 10000;
        double max_disp = 0;
        int min_i = 0;
        int max_i = 0;
        int min_j = 0;
        int max_j = 0;

        cv::Matx44d _Q;
        Q.convertTo(_Q, CV_64F);

        for (int i = 0; i < inDisparity.rows; i++)
        {
            for (int j = 0; j < inDisparity.cols; j++)
            {
                float d = inDisparity.at<float>(i, j);
                if (d < 10000){
                    float d = inDisparity.at<float>(i, j);
                    if (d < min_disp){
                        //TODO find out why issue with disp < ~3 causing negative w and so negative z
                        if (CVSupport::genZ(_Q,i,j,d) > 0){
                            min_disp = d;
                            min_i = i;
                            min_j = j;
                        }
                    }
                    if (d > max_disp){
                        max_disp = d;
                        max_i = i;
                        max_j = j;
                    }
                }
            }
        }

        min_depth = (double)CVSupport::genZ(_Q,(double)max_i,(double)max_j,(double)max_disp);
        max_depth = (double)CVSupport::genZ(_Q,(double)min_i,(double)min_j,(double)min_disp);

        /*
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
        */
    }

    static void getMinMaxDisparity(cv::Mat inDisparity, cv::Mat Q, double &min_disp, double &max_disp){
        min_disp = 10000;
        max_disp = 0;
        int min_i = 0;
        int max_i = 0;
        int min_j = 0;
        int max_j = 0;

        cv::Matx44d _Q;
        Q.convertTo(_Q, CV_64F);

        for (int i = 0; i < inDisparity.rows; i++)
        {
            for (int j = 0; j < inDisparity.cols; j++)
            {
                float d = inDisparity.at<float>(i, j);
                if (d < 10000){
                    float d = inDisparity.at<float>(i, j);
                    if (d < min_disp){
                        //TODO find out why issue with disp < ~3 causing negative w and so negative z
                        if (CVSupport::genZ(_Q,i,j,d) > 0){
                            min_disp = d;
                            min_i = i;
                            min_j = j;
                        }
                    }
                    if (d > max_disp){
                        max_disp = d;
                        max_i = i;
                        max_j = j;
                    }
                }
            }
        }


        /*
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
        */
    }

    static void removeInvalidDisparity(cv::Mat inDisparity, cv::Mat Q, cv::Mat &outValidDisparity){
        cv::Mat disparity_thresh;

        inDisparity.copyTo(disparity_thresh);

        double min_disp, max_disp;
        getMinMaxDisparity(disparity_thresh, Q, min_disp, max_disp);

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

        // normalise disparity
        cv::Mat normDisp;
        normaliseDisparity(disparity_thresh, normDisp);

        for (int i = 0; i < disparity_thresh.rows; i++)
        {
            for (int j = 0; j < disparity_thresh.cols; j++)
            {
                if (normDisp.at<uchar>(i, j) == 255 || normDisp.at<uchar>(i, j) == 0){
                    disparity_thresh.at<float>(i, j) = 0;
                }
            }
        }

        outValidDisparity = disparity_thresh;
    }

    static void disparity2colormap(cv::Mat inDisparity, cv::Mat Q, cv::Mat &outColormap, bool isFiltered=false){
        // threshold value disparity
        cv::Mat validDisp;
        if (!isFiltered){
            removeInvalidDisparity(inDisparity, Q, validDisp);
        } else {
            validDisp = inDisparity.clone();
        }

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
