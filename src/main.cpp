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

#ifdef WITH_FERVOR
  QApplication::setApplicationName(FV_APP_NAME);
  QApplication::setApplicationVersion(FV_APP_VERSION);
  QApplication::setOrganizationName("Industrial 3D Robotics");
  QApplication::setOrganizationDomain("i3drobotics.com");

  qDebug() << QSslSocket::supportsSsl() << QSslSocket::sslLibraryBuildVersionString() << QSslSocket::sslLibraryVersionString();
#endif

  QFile f(":qdarkstyle/style.qss");

  QFontDatabase::addApplicationFont(":/fonts/fontawesome-webfont.ttf");

  if (!f.exists()) {
    printf("Unable to set stylesheet, file not found\n");
  } else {
    f.open(QFile::ReadOnly | QFile::Text);
    QTextStream ts(&f);
    a.setStyleSheet(ts.readAll());
  }

  MainWindow w;
  w.show();

  return a.exec();
}
