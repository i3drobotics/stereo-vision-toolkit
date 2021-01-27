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
    static cv::Mat createRGBD32(cv::Mat color_img, cv::Mat disparity){
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

    static cv::Mat translate(cv::Mat input, double fromMin, double fromMax, double toMin, double toMax){
        //Figure out how 'wide' each range is
        double fromSpan = fromMax - fromMin;
        double toSpan = toMax - toMin;

        //Convert the left range into a 0-1 range (float)
        cv::Mat valueScaled = input - fromMin / fromSpan;

        //Convert the 0-1 range into a value in the right range.
        cv::Mat translated = toMin + (valueScaled * toSpan);
        return translated;
    }

    static cv::Mat createRGBD16(cv::Mat color_img, cv::Mat disparity, double scaling_factor=1.0, bool allow_negatives=true){
        std::vector<cv::Mat> rgbChannels(3);
        cv::split(color_img, rgbChannels);
        cv::Mat r = rgbChannels[0];
        cv::Mat g = rgbChannels[1];
        cv::Mat b = rgbChannels[2];
        cv::Mat d = disparity.clone();

        int CV_TYPE = CV_16UC1;
        if (allow_negatives){
            //CV_TYPE = CV_16SC1; //TODO find way to send 16SC1 over PNG encoder
        }

        //convert color channels to float to keep precsion in the rgbd image
        b.convertTo(b,CV_TYPE);
        g.convertTo(g,CV_TYPE);
        r.convertTo(r,CV_TYPE);

        double min_d, max_d;
        cv::minMaxLoc(d, &min_d, &max_d);
        std::cout << "Disp range (32bit): " << min_d << max_d << std::endl;
        if (allow_negatives){
            d = d + d.size().width;
        }
        cv::minMaxLoc(d, &min_d, &max_d);
        std::cout << "Disp range (32bit no negatives): " << min_d << max_d << std::endl;
        d.convertTo(d,CV_TYPE, scaling_factor);
        cv::minMaxLoc(d, &min_d, &max_d);
        std::cout << "Disp range (16bit): " << min_d << max_d << std::endl;

        //d.convertTo(d,CV_16UC1, 1.0/255.0);

        std::vector<cv::Mat> channels;
        channels.push_back(r);
        channels.push_back(g);
        channels.push_back(b);
        channels.push_back(d);
        cv::Mat rgbd;
        cv::merge(channels, rgbd);
        std::cout << "Disp type (16bit): " << rgbd.type() << std::endl;
        return rgbd;
    }

    // replaces the top right 4x4 pixels in the disparity image with the q matrix
    // this is useful so that all the information to create 3D geometry is contained
    // in the image. The top right is used as it's the least likely place for matching.
    static cv::Mat embedQinDisp(cv::Mat disp, cv::Mat Q){
        cv::Mat dispQ = disp.clone();
        for (int i = 0; i < Q.rows; i++){
            for (int j = 0; j < Q.cols; j++){
                dispQ.at<cv::Vec3f>(i,disp.size().width-Q.cols+j) = Q.at<float>(i,j);
            }
        }
        return dispQ;
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

    static double getHFOVFromQ(cv::Mat Q)
    {
        // Get horitonzal Field Of View of camera from Q matrix
        // Useful for reconstructing XYZ depth from Z only depth
        // Can be embedded in top left corner pixel of RGBD image to simplify reconstruction

        // Q Matrix has the format
        // [ 1 0   0       -cx     ]
        // [ 0 1   0       -cy     ]
        // [ 0 0   0        fx     ]
        // [ 0 0 -1/Tx (cx-cxr)/Tx ]

        // Cx and Cy are principal point of the left camera (in pixels so half the resolution)
        // Fx is horizontal focal length of left camera (in pixels)

        // Extract focal length and principal point from Q
        double fx = Q.at<double>(2, 3);
        double cx = -Q.at<double>(0, 3);

        // Calculate FOV
        // See here for explaination of the geometric reasoning
        // https://photo.stackexchange.com/questions/41273/how-to-calculate-the-fov-in-degrees-from-focal-length-or-distance
        double fov_x = 2 * atan(cx / fx);

        // Return horizonal FOV in radians
        return fov_x;
    }

    static void disparity2CVPointCloud(cv::Mat disparity, cv::Mat image, cv::Mat Q, cv::Mat &depth, cv::Mat &depthColors, float max_z = 10000){
        cv::Mat disparity16;
        disparity.copyTo(disparity16);

        if (Q.empty() || disparity16.empty()) {
            return;
        }

        depth = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
        depthColors = cv::Mat::zeros(cv::Size(3, 1), CV_8UC1);

        float wz = Q.at<float>(2, 3);
        float q03 = Q.at<float>(0, 3);
        float q13 = Q.at<float>(1, 3);
        float q32 = Q.at<float>(3, 2);
        float q33 = Q.at<float>(3, 3);
        float w, d;
        uchar intensity;

        float xyz[3] = {0,0,0};
        uchar rgb[3] = {0,0,0};

        for (int i = 0; i < disparity16.rows; i++)
        {
            for (int j = 0; j < disparity16.cols; j++)
            {
                d = disparity16.at<float>(i, j);
                if (d != 0)
                {
                    w = (d * q32) + q33;
                    xyz[0] = (j + q03) / w;
                    xyz[1] = (i + q13) / w;
                    xyz[2] = wz / w;

                    if (w > 0 && xyz[2] > 0 && xyz[2] < max_z){ // negative W or Z which is not possible (behind camera)
                        if (image.type() == CV_8UC1){
                            // MONO8
                            intensity = image.at<uchar>(i,j);
                            rgb[0] = intensity;
                            rgb[1] = intensity;
                            rgb[2] = intensity;
                        } else if (image.type() == CV_8UC3){
                            // BGR8
                            rgb[2] = image.at<cv::Vec3b>(i,j)[0];
                            rgb[1] = image.at<cv::Vec3b>(i,j)[1];
                            rgb[0] = image.at<cv::Vec3b>(i,j)[2];
                        } else {
                            rgb[0] = 0;
                            rgb[1] = 0;
                            rgb[2] = 0;
                            //qDebug() << "Invalid image type. MUST be CV_8UC1 or CV_8UC3";
                        }

                        cv::Mat point = cv::Mat(1, 3, CV_32FC1, &xyz);
                        cv::Mat color = cv::Mat(1, 3, CV_8UC1, &rgb);
                        //std::cout << "(" << rgb[0] << "," << rgb[1] << "," << rgb[2] << ")" << std::endl;
                        depth.push_back(point);
                        depthColors.push_back(color);
                    }
                }
            }
        }
    }

    static void disparity2Depth(cv::Mat disparity, cv::Mat Q, cv::Mat &depth, float max_z = 10000, float downsample_factor = 1.0){
        cv::Mat disparity16;
        disparity.copyTo(disparity16);

        float downsample_rate = 1 / downsample_factor;

        if (Q.empty() || disparity16.empty()) {
            return;
        }

        depth = cv::Mat::zeros(disparity.size(), CV_32FC3);

        float wz = Q.at<float>(2, 3);
        float q03 = Q.at<float>(0, 3);
        float q13 = Q.at<float>(1, 3);
        float q32 = Q.at<float>(3, 2);
        float q33 = Q.at<float>(3, 3);
        float w, d;

        float xyz[3] = {0,0,0};

        for (int i = 0; i < disparity16.rows; i++)
        {
            for (int j = 0; j < disparity16.cols; j++)
            {
                d = disparity16.at<float>(i, j);
                if (d != 0)
                {
                    w = ((d * downsample_rate) * q32) + q33;
                    xyz[0] = ((j * downsample_rate) + q03) / w;
                    xyz[1] = ((i * downsample_rate) + q13) / w;
                    xyz[2] = wz / w;

                    if (w > 0 && xyz[2] > 0 && xyz[2] < max_z){ // negative W or Z which is not possible (behind camera)
                        depth.at<cv::Vec3f>(i,j)[0] = xyz[0];
                        depth.at<cv::Vec3f>(i,j)[1] = xyz[1];
                        depth.at<cv::Vec3f>(i,j)[2] = xyz[2];
                    }
                }
            }
        }
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
