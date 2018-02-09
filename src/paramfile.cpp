#include <QDomDocument>
#include <QDebug>
#include "paramfile.h"

ParamFile::ParamFile()
{
    settings = new QSettings("I3Dr", "Stereo Vision Toolkit");

    if(!settings->contains("saveDir")){
        settings->setValue("saveDir", default_save_dir.absolutePath());
    }

    if(!settings->contains("calDir")){
        settings->setValue("calDir", default_cal_dir.absolutePath());
    }

    if(!settings->contains("exposure")){
        settings->setValue("exposure", 5);
    }
}

QString ParamFile::get_string(QString tagName) {
  return settings->value(tagName).toString();
}

double ParamFile::get_double(QString tagName) {
  return settings->value(tagName).toDouble();
}

void ParamFile::update_string(QString tagName, QString value) {
  settings->setValue(tagName, value);
  return;
}

void ParamFile::update_double(QString tagName, double value) {
  settings->setValue(tagName, value);
  return;
}

void ParamFile::updatePreviousDirectory(QString dir) {
  update_string("saveDir", dir);
}
