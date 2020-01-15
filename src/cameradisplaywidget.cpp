#include "cameradisplaywidget.h"
#include "ui_cameradisplaywidget.h"

CameraDisplayWidget::CameraDisplayWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraDisplayWidget)
{
    ui->setupUi(this);

    /* Standard grayscale */
    for (int i = 0; i < 256; i++) colourMap.push_back(qRgb(i, i, i));

    ui->settingsButton->setHidden(true);
}

CameraDisplayWidget::~CameraDisplayWidget()
{
    delete ui;
    if(display_image) delete display_image;
}

void CameraDisplayWidget::setSize(int width, int height, int bit_depth) {

  camwidth = width;
  camheight = height;

  displayBuffer.resize(camwidth * camheight * bit_depth);
  display_image = new QImage(camwidth, camheight, QImage::Format_Indexed8);
  display_image->setColorTable(colourMap);
}

void CameraDisplayWidget::setSettingsCallback(QObject *receiver, const char* slot){
    connect(ui->settingsButton, SIGNAL(clicked(bool)), receiver, slot);
    ui->settingsButton->setHidden(false);
}
