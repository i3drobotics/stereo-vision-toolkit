/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef DISPARITYVIEWERCONTROLS_H
#define DISPARITYVIEWERCONTROLS_H

#include <QWidget>
#include <QLabel>
#include <QtConcurrent>
#include <opencv2/opencv.hpp>

#include <abstractstereomatcher.h>
#include <cvsupport.h>

namespace Ui {
class DisparityViewer;
}

//!  Disparity Viewer
/*!
  QT Widget for displaying disparity images
*/

class DisparityViewer : public QWidget
{
    Q_OBJECT

signals:
    void finished();
    void newDisparity(QPixmap);
    //! Emitted when an image has been saved
    void savedImage();
    //! Emit when an image has been saved, including the filename
    void savedImage(QString filename);
    //! Emit when disparity save checkbox is toggled
    void disparitySaveCheckChanged(bool checked);

public:
    explicit DisparityViewer(QWidget *parent = 0);
    ~DisparityViewer();
    void setViewer(QLabel *viewer);
    void assignThread(QThread *thread);

    double getMinDepth(){
        return min_depth_;
    }

    double getMaxDepth(){
        return max_depth_;
    }

public slots:
    void updateDisparity(void);
    void updateDisparityAsync(void);
    void setMatcher(AbstractStereoMatcher *matcher);
    void setColourmap(int);
    void setCalibration(cv::Mat &Q, double baseline = 1.0, double focal = 1.0);
    void saveDisparityChanged(bool enable);

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
    AbstractStereoMatcher *matcher;
    bool processing_disparity;
    QString save_directory = ".";

    double min_depth_ = -1;
    double max_depth_ = -1;

public slots:
    //void updateDisparityRange_slide(int val);
    //void updateDisparityRange_spin(int val);
    //void updatePixmapRange();
};

#endif // DISPARITYVIEWERCONTROLS_H
