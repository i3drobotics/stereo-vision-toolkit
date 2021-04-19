/*
* Copyright Deeplabel, used with permission of the author
* Author: Josh Veitch-Michaelis (jveitch@i3drobotics.com)
*/
#ifndef DETECTOROPENCV_H
#define DETECTOROPENCV_H

#include<iostream>
#include<fstream>
#include<string>
#include<vector>

#include<opencv2/opencv.hpp>
#include<opencv2/core/ocl.hpp>
#include<opencv2/dnn.hpp>

#include <detection/boundingbox.h>

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QCoreApplication>

typedef enum {
    FRAMEWORK_TENSORFLOW,
    FRAMEWORK_DARKNET,
    FRAMEWORK_ONNX,
    FRAMEWORK_PYTORCH
} model_framework;

class DetectorOpenCV : public QObject {

Q_OBJECT

public:
    DetectorOpenCV(QObject *parent = nullptr);

    void setImageSize(int width, int height);

    void loadNetwork(std::string names_file, std::string cfg_file, std::string model_file);

    void annotateImage(cv::Mat &image,
                       std::vector<BoundingBox> boxes,
                       cv::Scalar colour = cv::Scalar(0,0,255),
                       cv::Scalar font_colour = cv::Scalar(255,255,255));

    std::vector<BoundingBox> inferDarknet(cv::Mat image);
    std::vector<BoundingBox> inferTensorflow(cv::Mat image);

    double getConfidenceThreshold(void){ return confThreshold;}
    double getNMSThreshold(void){ return nmsThreshold;}

    void assignThread(QThread* thread);
    int getChannels(void){return input_channels;}
    int getInputWidth(void){return input_width;}
    int getInputHeight(void){return input_height;}
    bool isReady(void){return ready;}
    double getProcessingTime(void){return processing_time;}
    int getNumClasses(void){return static_cast<int>(class_names.size());}
    std::vector<std::string> getClassNames(void){return class_names;}
    bool isRunning(void){return running;}
    ~DetectorOpenCV();

public slots:
    std::vector<BoundingBox> infer(cv::Mat image);
    void setFramework(model_framework framework){this->framework = framework;}

    void setConfidenceThresholdPercent(int thresh_pc){confThreshold = std::max(0.0, thresh_pc / 100.);}
    void setConfidenceThreshold(double thresh){confThreshold = std::max(0.0, thresh);}

    void setNMSThresholdPercent(int thresh){nmsThreshold = std::max(0.0, thresh / 100.0);}
    void setNMSThreshold(double thresh){nmsThreshold = std::max(0.0, thresh);}

    void setConvertGrayscale(bool convert){convert_grayscale = convert;}
    void setConvertDepth(bool convert){convert_depth = convert;}
    void setTarget(int target);
    void setChannels(int channels);

private:

    void postProcess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<BoundingBox> &filtered_outputs);
    void readNamesFile(std::string class_file = "coco.names");
    void getOutputClassNames(void);

    bool convert_grayscale = true;
    bool convert_depth = true;
    double processing_time;
    double confThreshold = 0.5; // Confidence threshold
    double nmsThreshold = 0.4;  // Non-maximum suppression threshold
    int input_width = 416;        // Width of network's input image
    int input_height = 416;       // Height of network's input image
    int input_channels = 3;
    int preferable_target = cv::dnn::DNN_TARGET_OPENCL;
    model_framework framework = FRAMEWORK_DARKNET;
    bool ready = false;
    bool running = false;
    QThread* thread_;

    std::vector<std::string> class_names;
    std::vector<std::string> output_names;
    cv::dnn::Net net;
    void postProcessTensorflow(cv::Mat &frame, const std::vector<cv::Mat> &outputs, std::vector<BoundingBox> &filtered_boxes);

signals:
    void finished();
    void objectsDetected();
};

#endif // DETECTOROPENCV_H
