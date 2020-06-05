/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include <QApplication>
#include <QFile>
#include <QFontDatabase>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  QFile f(":qdarkstyle/style.qss");

  QFontDatabase::addApplicationFont(":/fonts/fontawesome-webfont.ttf");

  if (!f.exists()) {
    qDebug() << "Unable to set stylesheet, file not found";
  } else {
    f.open(QFile::ReadOnly | QFile::Text);
    QTextStream ts(&f);
    a.setStyleSheet(ts.readAll());
  }

#ifdef WITH_FERVOR
  QApplication::setApplicationName(FV_APP_NAME);
  QApplication::setApplicationVersion(FV_APP_VERSION);
  QApplication::setOrganizationName("Industrial 3D Robotics");
  QApplication::setOrganizationDomain("i3drobotics.com");

  qDebug() << QSslSocket::supportsSsl() << QSslSocket::sslLibraryBuildVersionString() << QSslSocket::sslLibraryVersionString();
  if (!QSslSocket::supportsSsl()){
      QMessageBox msgBox;
      msgBox.setText("Missing SSL support. <br>You will not be able to receive automatic updates. <br>Please check <a href='https://github.com/i3drobotics/stereo-vision-toolkit/releases'>SVTK Github Releases</a> for updates.");
      msgBox.exec();
  }
#endif

  MainWindow w;
  w.showMaximized();

  return a.exec();
}
