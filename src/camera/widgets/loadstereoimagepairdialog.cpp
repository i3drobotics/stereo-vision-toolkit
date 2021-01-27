#include "loadstereoimagepairdialog.h"
#include "ui_loadstereoimagepairdialog.h"

LoadStereoImagePairDialog::LoadStereoImagePairDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LoadStereoImagePairDialog)
{
    ui->setupUi(this);

   connect(ui->btnBrowseLeft, SIGNAL(clicked()), this, SLOT(requestLeftImageFilepath()));
   connect(ui->btnBrowseRight, SIGNAL(clicked()), this, SLOT(requestRightImageFilepath()));
   connect(ui->btnCancel, SIGNAL(clicked()), this, SLOT(reject()));
   connect(ui->btnLoad, SIGNAL(clicked()), this, SLOT(accept()));
}

void LoadStereoImagePairDialog::reject()
{
    leftFilepathValid = rightFilepathValid = false;
    QDialog::reject();
}

void LoadStereoImagePairDialog::requestLeftImageFilepath(){
    leftFilepathValid = requestImageFilepath(left_image_filepath);
    ui->txtLeftImageFilename->setText(QString::fromStdString(left_image_filepath));
}

void LoadStereoImagePairDialog::requestRightImageFilepath(){
    rightFilepathValid = requestImageFilepath(right_image_filepath);
    ui->txtRightImageFilename->setText(QString::fromStdString(right_image_filepath));
}

bool LoadStereoImagePairDialog::requestImageFilepath(std::string &fname){
    QString qfname = QFileDialog::getOpenFileName(
                this, tr("Open image"), "", tr("Images (*.png *.jpeg *.jpg)"));
    fname = qfname.toStdString();
    QFileInfo check_file(qfname);
    return (fname != "" && check_file.exists() && check_file.isFile());
}

LoadStereoImagePairDialog::~LoadStereoImagePairDialog()
{
    delete ui;
}
