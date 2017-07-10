/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef DISPARITYVIEWERCONTROLS_H
#define DISPARITYVIEWERCONTROLS_H

#include <QWidget>
#include <QLabel>
#include <QtConcurrent>
#include <opencv2/opencv.hpp>

#include <abstractstereomatcher.h>

namespace Ui {
class DisparityViewer;
}

class DisparityViewer : public QWidget
{
    Q_OBJECT

signals:
    void finished();
    void newDisparity(QPixmap);

public:
    explicit DisparityViewer(QWidget *parent = 0);
    ~DisparityViewer();
    void setViewer(QLabel *viewer);
    void assignThread(QThread *thread);

public slots:
    void updateDisparity(void);
    void updateDisparityAsync(void);
    void setMatcher(AbstractStereoMatcher *matcher);
    void setColourmap(int);
    void setCalibration(cv::Mat &Q, double baseline = 1.0, double focal = 1.0);

private:
    Ui::DisparityViewer *ui;
    cv::Mat Q;
    int min_disparity = 1;
    int disparity_range = 128;
    int colourmap = 2;
    double baseline = 60e-3;
    double focal = 4e-3;
    double pixel_size = 6e-6;
    QLabel *viewer;
    cv::Mat disparity;
    AbstractStereoMatcher *matcher;
    bool processing_disparity;

public slots:
    void updatePixmapRange();
};

#endif // DISPARITYVIEWERCONTROLS_H
