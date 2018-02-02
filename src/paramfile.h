#ifndef PARAMFILE_H
#define PARAMFILE_H

#include <QObject>
#include <QFile>
#include <QDir>

class ParamFile
{
public:
    ParamFile();
    void updatePreviousDirectory(QString dir);
    QString get(QString tagName);
    void load();
    void create(QDir path);
    void update(QString tagName, QString value);
private:
    QFile* file_handle;
};

#endif // PARAMFILE_H
