/*!
 * @file cvsupport.h
 * @authors Ben Knight (bknight@i3drobotics.com)
 * @brief Support class for using PCL
 * @version 1.3
 * @date 2020-07-02
 * 
 * @copyright Copyright (c) I3D Robotics Ltd, 2020
 * 
 */

#ifndef PCLSUPPORT_H
#define PCLSUPPORT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>

/**
 * @brief Support class for using PCL
 */
class PCLSupport
{
public:

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr disparity2PointCloud(cv::Mat disparity, cv::Mat image, cv::Mat Q) {
        cv::Mat disparity_downscale;

        disparity.copyTo(disparity_downscale);

        // By default the disparity maps are scaled by a factor of 16.0
        disparity_downscale /= 16.0;

        int row = (disparity_downscale.rows-1)/2;
        int column = (disparity_downscale.cols-1)/2;
        float disp_val_f =  disparity_downscale.at<float>(row, column);

        if (Q.empty() || disparity_downscale.empty()) {
            //qDebug() << "Q or disparity map is empty";
        }

        //cv::reprojectImageTo3D(disparity_downscale, stereo_reprojected, Q, true);

        //qDebug() << "reprojected image size: " << stereo_reprojected.size().width << "," << stereo_reprojected.size().height;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointXYZRGB point;
        uint32_t rgb = 0;

        point.x = 0;
        point.y = 0;
        point.z = 0;

        rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
        point.rgb = *reinterpret_cast<float *>(&rgb);
        ptCloudTemp->points.push_back(point);

        cv::Matx44d _Q;
        Q.convertTo(_Q, CV_64F);

        for (int i = 0; i < disparity_downscale.rows; i++)
        {
            for (int j = 0; j < disparity_downscale.cols; j++)
            {
                float d = disparity_downscale.at<float>(i, j);

                if (d < 10000){
                    float x_index = j;
                    float y_index = i;

                    //qDebug() << d;

                    cv::Vec4d homg_pt = _Q * cv::Vec4d((double)x_index, (double)y_index, (double)d, 1.0);
                    if (homg_pt[3] > 0){ // negative W would give negative z which is not possible (behind camera)

                        float x = (float)homg_pt[0] / (float)homg_pt[3];
                        float y = (float)homg_pt[1] / (float)homg_pt[3];
                        float z = (float)homg_pt[2] / (float)homg_pt[3];

                        if (z > 0){ // negative z is not possible (behind camera)
                            uchar b,g,r;
                            if (image.type() == CV_8UC1){
                                uchar intensity = image.at<uchar>(i,j);
                                b = intensity;
                                g = intensity;
                                r = intensity;
                            } else if (image.type() == CV_8UC3){
                                b = image.at<cv::Vec3b>(i,j)[0];
                                g = image.at<cv::Vec3b>(i,j)[1];
                                r = image.at<cv::Vec3b>(i,j)[2];
                            } else {
                                b = 0;
                                g = 0;
                                r = 0;
                                //qDebug() << "Invalid image type. MUST be CV_8UC1 or CV_8UC3";
                            }

                            pcl::PointXYZRGB point;
                            point.x = x;
                            point.y = y;
                            point.z = z;
                            point.r = r;
                            point.g = g;
                            point.b = b;
                            ptCloudTemp->points.push_back(point);
                        }
                    }
                }
            }
        }

        if(ptCloudTemp->points.empty()){
            //qDebug() << "Failed to create Point cloud. Point cloud is empty";
            //return;
        }

        return ptCloudTemp;
    }
    
};

#endif // PCLSUPPORT_H
