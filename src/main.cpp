#include <QApplication>
#include <QFile>
#include <QFontDatabase>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

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
