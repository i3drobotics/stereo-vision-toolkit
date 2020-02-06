/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include <QApplication>
#include <QFile>
#include <QFontDatabase>
#include "fvupdater.h"

#include "mainwindow.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  //QApplication::setApplicationName(APP_NAME);
  //QApplication::setApplicationVersion(APP_VERSION);
  QApplication::setOrganizationName("Industrial 3D Robotics");
  QApplication::setOrganizationDomain("i3drobotics.com");

  // Set the Fervor appcast url
  FvUpdater::sharedUpdater()->SetFeedURL("https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/dev/Appcast.xml");
  // Check for updates silently -- this will not block the initialization of
  // your application, just start a HTTP request and return immediately.
  FvUpdater::sharedUpdater()->CheckForUpdatesNotSilent();

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
