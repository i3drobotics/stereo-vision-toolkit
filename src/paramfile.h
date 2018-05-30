#ifndef PARAMFILE_H
#define PARAMFILE_H

#include <QObject>
#include <QDir>
#include <QCoreApplication>
#include <QSettings>

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
    QDir default_save_dir = QDir::homePath() + "/deimos/";
    QDir default_cal_dir = QDir::homePath() + "/deimos/";
};

#endif // PARAMFILE_H
