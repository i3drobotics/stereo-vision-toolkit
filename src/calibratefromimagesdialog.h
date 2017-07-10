#ifndef CALIBRATEFROMIMAGESDIALOG_H
#define CALIBRATEFROMIMAGESDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QDebug>
#include <QDir>
#include <QStandardPaths>
#include <QFileSystemModel>

#include <stereocalibrate.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace Ui {
class CalibrateFromImagesDialog;
}

class CalibrateFromImagesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrateFromImagesDialog(QWidget *parent = 0);
    ~CalibrateFromImagesDialog();

private:
    Ui::CalibrateFromImagesDialog *ui;

    QString leftPath = "";
    QString rightPath = "";
    QString leftMask = "";
    QString rightMask = "";

    QFileSystemModel *leftFileModel;
    QFileSystemModel *rightFileModel;

    QList<QString> leftImageList;
    QList<QString> rightImageList;

    std::vector<cv::Mat> leftImages;
    std::vector<cv::Mat> rightImages;

    std::vector< std::vector<cv::Point2f> > leftImagePoints;
    std::vector< std::vector<cv::Point2f> > rightImagePoints;
    std::vector< bool > leftValid;
    std::vector< bool > rightValid;


private slots:
    void selectLeftImageRoot(void);
    void selectRightImageRoot(void);
    void updateLeftMask(void);
    void updateRightMask(void);

    void findImages(void);

    void updateLeftPath(void);
    void updateRightPath(void);

    void setLeftImages(void);
    void setRightImages(void);

    void runCalibration(void);

};

#endif // CALIBRATEFROMIMAGESDIALOG_H
