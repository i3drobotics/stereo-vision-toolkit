/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#ifndef ABOUTDIALOG_H
#define ABOUTDIALOG_H

#include <QDialog>
#include <QDebug>

namespace Ui {
class AboutDialog;
}

//!  About dialog
/*!
  Dialog for show information about the software
*/
class AboutDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AboutDialog(QWidget *parent = 0);
    void setVersion(QString version);
    ~AboutDialog();

private:
    //! QT UI dialog
    Ui::AboutDialog *ui;

};

#endif // CALIBRATEFROMIMAGESDIALOG_H
