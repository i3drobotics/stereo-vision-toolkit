/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#ifndef CALIBRATECONFIRMDIALOG_H
#define CALIBRATECONFIRMDIALOG_H

#include <QDialog>

namespace Ui {
class calibrateconfirmdialog;
}

class calibrateconfirmdialog : public QDialog {
  Q_OBJECT

 public:
  explicit calibrateconfirmdialog(QWidget *parent = 0);
  ~calibrateconfirmdialog();

 private:
  Ui::calibrateconfirmdialog *ui;
};

#endif  // CALIBRATECONFIRMDIALOG_H
