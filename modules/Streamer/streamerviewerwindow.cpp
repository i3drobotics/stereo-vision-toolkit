#include "streamerviewerwindow.h"
#include "ui_streamerviewerwindow.h"

StreamerViewerWindow::StreamerViewerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StreamerViewerWindow)
{
    ui->setupUi(this);
}

StreamerViewerWindow::~StreamerViewerWindow()
{
    delete ui;
}
