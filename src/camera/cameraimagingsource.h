#ifndef CAMERAIMAGINGSOURCE_H
#define CAMERAIMAGINGSOURCE_H

#include <tisudshl.h>
#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QElapsedTimer>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <fstream>
#include <iostream>
#include <string>

class Listener : public QObject, public DShowLib::GrabberListener {
  Q_OBJECT
 signals:
  void grabbed(void*);
  void deviceDisconnected();
  void grabFailed();
  void done();
  void frame_number(int);
  void frameTime(uint);

 private:
  QString saveFolder = ".";
  QString suffix = "";
  QString folderLabel = "stereo";
  bool save_files = false;
  bool need_grab = false;
  void* output_buffer = nullptr;
  QElapsedTimer timer;

 public slots:
  void enableSave(bool enable) { save_files = enable; }

  void requestGrab() { need_grab = true; }

  void setSuffix(QString text) { suffix = text; }

  void setOutputBuffer(void* buffer){ output_buffer = buffer;}

  void setSaveFolder(QString folder, QString label) { saveFolder = folder; folderLabel = label;}

 public:
  explicit Listener(QObject* parent = 0) : QObject(parent) {}
  ~Listener(void) {}

  void deviceLost(DShowLib::Grabber& /* caller */){
    emit deviceDisconnected();
  }

  void frameReady(DShowLib::Grabber& /* caller */,
                  smart_ptr<DShowLib::MemBuffer> pBuffer, DWORD FrameNumber) {
    pBuffer->lock();

    auto frame_data = pBuffer.get()->getPtr();

    memcpy(output_buffer, frame_data,  pBuffer.get()->getBufferSize());

    emit grabbed((void*) frame_data);

    if (need_grab || save_files) {
      QDateTime timestamp(QDateTime::currentDateTime());
      QString fname;
      QString subfolder;

      subfolder = QString("%1/%2/%3/%4/%5")
                      .arg(saveFolder)
                      .arg(timestamp.toString("yyyyMMdd"))
                      .arg(timestamp.toString("hh"))
                      .arg(folderLabel)
                      .arg(suffix);

      QDir dir(subfolder);
      if (!dir.exists()) {
        dir.mkpath(".");
      }

      fname = QString("%1/%2_%3.tiff")
                  .arg(subfolder)
                  .arg(timestamp.toString("yyyyMMdd.hh.mm.ss.zzz"))
                  .arg(suffix);

      DShowLib::saveToFileTIFF(*pBuffer.get(), fname.toStdString());

      need_grab = false;
    }

    pBuffer->unlock();

    emit done();
    emit frame_number((int)FrameNumber);
    emit frameTime((uint)timer.nsecsElapsed());
    timer.restart();
  }
};

class CameraImagingSource : public QObject {
  Q_OBJECT
 public:
  explicit CameraImagingSource(DShowLib::VideoCaptureDeviceItem device);
  explicit CameraImagingSource(void);
  void assignThread(QThread* thread);
  void setListener(DShowLib::GrabberListener* listener);
  bool open(std::string serial);
  void close();
  std::string getSerial(void);
  ~CameraImagingSource(void);
  int height = -1;
  int width = -1;
  char* estring;

  double frameRate = -1;
  double temperature = 0;

  DShowLib::Grabber handle;

  bool saveFull = false;
  bool saveEncoded = false;

  void enableAutoExposure(bool enable);
  void enableAutoGain(bool enable);
  void changeExposure(double exposure);
  void changeGain(int gain);

 private:
  DShowLib::GrabberListener* grabber_listener;
  int result = -1;
  int bitDepth = -1;
  int colourMode = -1;
  bool capturing = false;

  int imageBufferID = 0;
  QTimer* timer = nullptr;
  int frameCounter = 0;
  int buffersize = 0;
  qint64 serial = -1;

  DShowLib::tFrameHandlerSinkPtr frame_sink;
  DShowLib::GrabberListener grab_event_handler;

  void setVideoFormat16(int width, int height);
  void setVideoFormat(int width, int height);
  void setExposure(double exposure);

 signals:
  void finished();
  void grabbed(short*);
  void gotPeaks(uint*);
  void pixelClockChanged();
  void exposureChanged();
  void frameRateChanged(double);
  void gainChanged();
  void frameTime(uint);
  void grabError();
  void stopped();

 private slots:
  void debugMessage(QString);

 public slots:
  void grabImage(void);
  void startCapture(void);
  void stopCapture(void);
  void setup(int width, int height, int fps = -1);
  void toggleCapture(bool capture);
  void setTrigger(bool trigger);

  void finishThread(void);

  void setFrameRate(double);
  double getFrameRate(void);

  void saveFrame(QString);
  void saveBitmap(QString fname);

  double getExposure();

  int getGain(void);
  void setGain(int);

  void showProperties();
  DShowLib::Grabber::tFPSListPtr getFrameRates();
  void setFrameRateIndex(int index);

  DShowLib::GrabberListener* getListener();

};

#endif // CAMERAIMAGINGSOURCE_H
