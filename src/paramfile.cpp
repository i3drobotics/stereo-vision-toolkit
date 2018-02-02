#include <QDomDocument>
#include <QCoreApplication>
#include <QDebug>

#include "paramfile.h"

ParamFile::ParamFile()
{
    file_handle = new QFile(QCoreApplication::applicationDirPath() + "/params/params.xml");
}

void ParamFile::create(QDir path){
    QDomDocument doc("deimosParameters");

    auto dir = QDir(QCoreApplication::applicationDirPath() + "/params/");
    dir.mkpath(".");

    QDomElement root = doc.createElement("deimosParameters");
    doc.appendChild(root);

    QString save_directory = QDir::homePath() + "/deimos/";

    QDomElement tag = doc.createElement("saveDir");
    tag.appendChild(doc.createTextNode(save_directory));
    root.appendChild(tag);

    if(!QDir(save_directory).exists()){
        auto saved = QDir(save_directory);
        saved.mkpath(".");
    }

    tag = doc.createElement("exposure");
    tag.appendChild(doc.createTextNode("5"));
    root.appendChild(tag);

    tag = doc.createElement("calDir");
    tag.appendChild(doc.createTextNode(QCoreApplication::applicationDirPath() + "/params/"));
    root.appendChild(tag);

    QString xml = doc.toString();

    if (!file_handle->open(QIODevice::WriteOnly | QIODevice::Text)) {
      qDebug() << "Failed to write file";
      return;
    }

    QTextStream out(file_handle);
    out << xml;

    file_handle->close();
}

void ParamFile::load(void) {
  if (!file_handle->exists()) {
    qDebug() << "Making parameter file.";
    create(QCoreApplication::applicationDirPath() + "/params/params.xml");
  }else{
    qDebug() << "Found parameter file";
  }
}

QString ParamFile::get(QString tagName) {
  QDomDocument doc("deimosParameters");
  QString text = "";

  if (!file_handle->open(QIODevice::ReadOnly | QIODevice::Text)) {
    qDebug() << "Failed to open file";
  } else {
    if (!doc.setContent(file_handle)) {
      qDebug() << "Failed to set XML Content";
    } else {
      auto node = doc.elementsByTagName(tagName);

      if (node.count() > 0) {
        auto el = node.at(0).firstChild().toText();

        if (!el.isNull()) {
          text = el.data();
        }
      }
    }

    file_handle->close();
  }

  return text;
}

void ParamFile::update(QString tagName, QString value) {
  load();

  QDomDocument doc("deimosParameters");

  if (!file_handle->open(QIODevice::ReadWrite | QIODevice::Text)) {
    qDebug() << "Failed to open param file";
    return;
  }

  if (!doc.setContent(file_handle)) {
    qDebug() << "Failed to load XML";
  } else {
    auto node = doc.elementsByTagName(tagName);

    if (node.count() > 0) {
      auto el = node.at(0).firstChild().toText();

      if (!el.isNull()) {
        el.setData(value);
      }
    } else {
      auto tag = doc.createElement(tagName);
      tag.appendChild(doc.createTextNode(value));
      auto root = doc.firstChild();
      root.appendChild(tag);
    }
  }

  file_handle->close();

  QFile file(QCoreApplication::applicationDirPath() + "/params/~params_temp.xml");
  QDir outdir(QCoreApplication::applicationDirPath() + "/params/");

  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return;
  }
  QByteArray xml = doc.toByteArray();
  auto bytes = file.write(xml);
  file.close();

  if (bytes == xml.count()) {
    outdir.remove(QCoreApplication::applicationDirPath() + "/params/params.xml");
    outdir.rename(QCoreApplication::applicationDirPath() + "/params/~params_temp.xml",
                  QCoreApplication::applicationDirPath() + "/params/params.xml");
  } else {
    qDebug() << xml.count() << bytes;
  }

  return;
}

void ParamFile::updatePreviousDirectory(QString dir) {
  update("saveDir", dir);
}
