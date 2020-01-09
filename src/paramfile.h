#ifndef PARAMFILE_H
#define PARAMFILE_H

#include <QObject>
#include <QDir>
#include <QCoreApplication>
#include <QSettings>

//!  Parameter file class
/*!
  Updating and setting paramter files
*/

class ParamFile
{
public:
    ParamFile();
    void updatePreviousDirectory(QString dir);
    QString get_string(QString tagName);
    double get_double(QString tagName);
    void update_string(QString tagName, QString value);
    void update_double(QString tagName, double value);
private:
    QSettings *settings;
    QDir default_save_dir = QDir::homePath() + "/i3dr_stereo_toolkit/save/";
    QDir default_cal_dir = QDir::homePath() + "/i3dr_stereo_toolkit/cal/";
};

#endif // PARAMFILE_H
