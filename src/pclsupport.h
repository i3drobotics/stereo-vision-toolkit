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
#include "cvsupport.h"

/**
 * @brief Support class for using PCL
 */
class PCLSupport
{
public:

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr disparity2PointCloud(cv::Mat disparity, cv::Mat image, cv::Mat Q, float downsample_factor = 1.0) {
        cv::Mat disparity16;

        disparity.copyTo(disparity16);

        float downsample_rate = 1 / downsample_factor;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

        if (Q.empty() || disparity16.empty()) {
            //qDebug() << "Q or disparity map is empty";
            return ptCloudTemp;
        }

        // Add inital point at (0,0,0)
        pcl::PointXYZRGB point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        ptCloudTemp->points.push_back(point);

        cv::Matx44d _Q;
        Q.convertTo(_Q, CV_64F);

        float wz = Q.at<float>(2, 3);
        float q03 = Q.at<float>(0, 3);
        float q13 = Q.at<float>(1, 3);
        float q32 = Q.at<float>(3, 2);
        float q33 = Q.at<float>(3, 3);
        float w, d;
        uchar b,g,r;
        uchar intensity;

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

                    if (w > 0 && xyz[2] > 0){ // negative W or Z is not possible (behind camera)
                        if (image.type() == CV_8UC1){
                            intensity = image.at<uchar>(i,j);
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

                        point.x = xyz[0];
                        point.y = xyz[1];
                        point.z = xyz[2];
                        point.r = r;
                        point.g = g;
                        point.b = b;
                        ptCloudTemp->points.push_back(point);
                    }
                }
            }
        }

        if(ptCloudTemp->points.empty()){
            //qDebug() << "Failed to create Point cloud. Point cloud is empty";
            return ptCloudTemp;
        }

        return ptCloudTemp;
    }

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2PointCloud(cv::Mat depth, cv::Mat image) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

        if (depth.empty()) {
            //qDebug() << "Q or disparity map is empty";
            return ptCloudTemp;
        }

        // Add inital point at (0,0,0)
        pcl::PointXYZRGB point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        ptCloudTemp->points.push_back(point);

        uchar b,g,r;
        uchar intensity;

        cv::Vec3f xyz;

        for (int i = 0; i < depth.rows; i++)
        {
            for (int j = 0; j < depth.cols; j++)
            {
                xyz = depth.at<cv::Vec3f>(i, j);
                if (xyz[2] > 0){ // negative Z is not possible (behind camera)
                    if (image.type() == CV_8UC1){
                        intensity = image.at<uchar>(i,j);
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

                    point.x = xyz[0];
                    point.y = xyz[1];
                    point.z = xyz[2];
                    point.r = r;
                    point.g = g;
                    point.b = b;
                    ptCloudTemp->points.push_back(point);
                }
            }
        }

        if(ptCloudTemp->points.empty()){
            //qDebug() << "Failed to create Point cloud. Point cloud is empty";
            return ptCloudTemp;
        }

        return ptCloudTemp;
    }
    
};

#endif // PCLSUPPORT_H
