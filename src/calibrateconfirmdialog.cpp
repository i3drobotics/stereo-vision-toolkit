#include "calibrateconfirmdialog.h"
#include "ui_calibrateconfirmdialog.h"

calibrateconfirmdialog::calibrateconfirmdialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::calibrateconfirmdialog) {
  ui->setupUi(this);
}

calibrateconfirmdialog::~calibrateconfirmdialog() { delete ui; }
