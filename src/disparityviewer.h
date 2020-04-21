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

//!  Disparity Viewer
/*!
  QT Widget for displaying disparity images
*/

namespace Ui {
class DisparityViewer;
}

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

public:
    explicit DisparityViewer(QWidget *parent = 0);
    ~DisparityViewer();
    void setViewer(QLabel *viewer);
    void assignThread(QThread *thread);

    //! Set the save directory
    /*!
    * @param dir Desired save directory, will attempt to create if it doesn't exist.
    */
    void setSavelocation(QString dir){

        if(!QDir(dir).exists()){
            auto saved = QDir(dir);
            saved.mkpath(".");
        }

        save_directory = dir;
    }

public slots:
    void updateDisparity(void);
    void updateDisparityAsync(void);
    void setMatcher(AbstractStereoMatcher *matcher);
    void setColourmap(int);
    void setCalibration(cv::Mat &Q, double baseline = 1.0, double focal = 1.0);
    void saveImageTimestamped(void);

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
    //cv::Mat disparity;
    cv::Mat colour_disparity;
    AbstractStereoMatcher *matcher;
    bool processing_disparity;
    QString save_directory = ".";

    void saveImage(QString fname);
    float genZ(cv::Matx44d Q_, int x_index, int y_index, float d);

public slots:
    //void updateDisparityRange_slide(int val);
    //void updateDisparityRange_spin(int val);
    //void updatePixmapRange();
};

//! Wrapper around cv::imwrite for saving in parallel
/*!
* Saves an image, can also be called sequentially.
*
* @param[in] fname Output filename
* @param[in] src Image matrix
*
* @return True/false if the write was successful
*/
bool write_parallel(std::string fname, cv::Mat src);

#endif // DISPARITYVIEWERCONTROLS_H
