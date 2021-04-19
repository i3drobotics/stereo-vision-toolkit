#ifndef LOADSTEREOIMAGEPAIRDIALOG_H
#define LOADSTEREOIMAGEPAIRDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QFileInfo>

namespace Ui {
class LoadStereoImagePairDialog;
}

class LoadStereoImagePairDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LoadStereoImagePairDialog(QWidget *parent = nullptr);
    ~LoadStereoImagePairDialog();

    bool isFilepathsValid(){return leftFilepathValid && rightFilepathValid;}

    std::string getLeftImageFilepath(){return left_image_filepath;}
    std::string getRightImageFilepath(){return right_image_filepath;}

private slots:
    void requestLeftImageFilepath();
    void requestRightImageFilepath();
    void reject();

private:
    Ui::LoadStereoImagePairDialog *ui;

    std::string left_image_filepath = "";
    std::string right_image_filepath = "";
    bool leftFilepathValid = false;
    bool rightFilepathValid = false;

    bool requestImageFilepath(std::string &fname);
};

#endif // LOADSTEREOIMAGEPAIRDIALOG_H
