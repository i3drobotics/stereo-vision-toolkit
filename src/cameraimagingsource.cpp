#include "cameraimagingsource.h"

CameraImagingSource::CameraImagingSource(){
    saveFull = false;
    saveEncoded = false;

    this->frameCounter = 0;
}

CameraImagingSource::CameraImagingSource(DShowLib::VideoCaptureDeviceItem device) {
  handle.closeDev();
  handle.openDev(device);

  saveFull = false;
  saveEncoded = false;

  this->frameCounter = 0;

  handle.getDev().getSerialNumber(serial);

  QMessageBox alert;

  // Initialise the camera
  if (!handle.isDevValid()) {
    debugMessage("Couldn't open camera");
    alert.setText(QString("Couldn't open camera with serial %1.").arg(serial));
    alert.exec();
    exit(1);
  }else{
      debugMessage("Opened camera");
  }
}

bool CameraImagingSource::open(qint64 serial){
    this->serial = serial;
    handle.closeDev();
    bool res = handle.openDev(serial);

    if (!handle.isDevValid()) {
      debugMessage("Couldn't open camera");
      QMessageBox alert;
      alert.setText(QString("Couldn't open camera with serial %1.").arg(serial));
      alert.exec();
    }else{
        debugMessage("Opened camera");
    }

    return res;
}

qint64 CameraImagingSource::getSerial(void){
    long long serial;
    handle.getDev().getSerialNumber(serial);
    return serial;
}

void CameraImagingSource::setup(int width, int height, int fps) {
  setVideoFormat(width, height);

  this->width = handle.getVideoFormat().getSize().cx;
  this->height = handle.getVideoFormat().getSize().cy;

  frame_sink = DShowLib::FrameHandlerSink::create(
      DShowLib::FrameTypeInfoArray(DShowLib::eY800), 5);
  frame_sink->setSnapMode(true);

  if (fps > 0) {
    if (!handle.setFPS(fps)) {
      debugMessage("Failed to set framerate");
    }
  } else {
    auto maximum_framerate = handle.getCurrentMaxAvailableFPS();
    if (!handle.setFPS(maximum_framerate)) {
      debugMessage("Failed to set framerate");
    }
  }
}

DShowLib::GrabberListener* CameraImagingSource::getListener() { return grabber_listener; }

void CameraImagingSource::setListener(DShowLib::GrabberListener* listener) {
  grabber_listener = listener;
  handle.addListener((DShowLib::GrabberListener*)listener);
}

void CameraImagingSource::showProperties() { handle.showVCDPropertyPage(nullptr, ""); }

void CameraImagingSource::setVideoFormat(int width, int height) {

  QString format_string = QString("Y800 (%1x%2)o")
                              .arg(QString::number(width))
                              .arg(QString::number(height));
  if (!handle.setVideoFormat(format_string.toStdString())) {
    debugMessage("Failed to set 8-bit video format");
  }

  this->width = handle.getVideoFormat().getSize().cx;
  this->height = handle.getVideoFormat().getSize().cy;
}

void CameraImagingSource::setFrameRateIndex(int index) {
  setFrameRate(getFrameRates()->at(index));
}

void CameraImagingSource::setVideoFormat16(int width, int height) {
  QString format_string = QString("Y16 (%1x%2)")
                              .arg(QString::number(width))
                              .arg(QString::number(height));

  if (!handle.setVideoFormat(format_string.toStdString())) {
    debugMessage("Failed to set 16-bit video format");
  }

  this->width = handle.getVideoFormat().getSize().cx;
  this->height = handle.getVideoFormat().getSize().cy;
}

DShowLib::Grabber::tFPSListPtr CameraImagingSource::getFrameRates() {
  return handle.getAvailableFPS();
}

void CameraImagingSource::assignThread(QThread* thread) {
  this->moveToThread(thread);
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

  thread->start();
}

void CameraImagingSource::startCapture() {

  if (handle.setSinkType(frame_sink)) {
    debugMessage("Capture started");
  } else {
    debugMessage("Failed to set frame sink");
  }

  handle.startLive(false);
}

void CameraImagingSource::stopCapture() {
  handle.stopLive();
  handle.setSinkType(nullptr);

  emit stopped();
}

void CameraImagingSource::toggleCapture(bool capture) {
  if (capture) {
    startCapture();
  } else {
    stopCapture();
  }
}

void CameraImagingSource::saveFrame(QString fname) {
  grabImage();
  fname =
      QString("/home/htp/%1_%2.png")
          .arg(fname)
          .arg(QDateTime::currentDateTime().toString("yyyyMMdd.hh.mm.ss.zzz"));
  saveBitmap(fname);
}

void CameraImagingSource::saveBitmap(QString fname) {
  frame_sink->getLastAcqMemBuffer()->save(fname.toStdString());
}

double CameraImagingSource::getExposure() {
  auto camera_properties = handle.getAvailableVCDProperties();

  // Retrieve the absolute value interface for exposure.
  DShowLib::tIVCDAbsoluteValuePropertyPtr pAbsVal = nullptr;
  camera_properties->findInterfacePtr(DShowLib::VCDID_Exposure,
                                      DShowLib::VCDElement_Value, pAbsVal);

  return pAbsVal.get()->getValue();
}

void CameraImagingSource::grabImage() {
  if (!handle.isLive()) debugMessage("Not live");

  DShowLib::Error result = frame_sink->snapImagesAsync(1);

  if (result.isError()) {
    debugMessage("Failed to capture image");
    qDebug() << result.toString().c_str();
    emit grabError();
  }

  return;
}

void CameraImagingSource::setFrameRate(double setRate) {
  bool was_live = false;

  if (handle.isLive()) {
    was_live = true;
  }

  stopCapture();
  while (handle.isLive())
    ;

  if (!handle.setFPS(setRate)) {
    debugMessage("Failed to change frame rate");
  } else {
    emit frameRateChanged(handle.getFPS());
  }

  qDebug() << "New frame rate" << handle.getFPS();

  if (was_live) {
    startCapture();
  }

  return;
}

double CameraImagingSource::getFrameRate(void) { return handle.getFPS(); }

void CameraImagingSource::setGain(int gain) {
  if (handle.setProperty(VideoProcAmp_Gain, (long)gain)) {
    debugMessage("Couldn't set gain");
  }
}

int CameraImagingSource::getGain(void) {
  return (int)handle.getProperty(VideoProcAmp_Gain);
}

void CameraImagingSource::debugMessage(QString message) {
  qDebug() << (qint64) serial << ": " << message;
  return;
}

void CameraImagingSource::setTrigger(bool trigger){
    bool was_live = false;

    if (handle.isLive()) {
      was_live = true;
    }

    stopCapture();
    while (handle.isLive())
      ;

    handle.setExternalTrigger(trigger);

    if (was_live) {
      startCapture();
    }
}

void CameraImagingSource::close(){
    debugMessage("Freeing camera");
    stopCapture();
    while (handle.isLive());
    handle.closeDev();
    emit finished();
}

CameraImagingSource::~CameraImagingSource() {
    close();
}
