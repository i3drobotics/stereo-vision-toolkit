#include "cameraimagingsource.h"

CameraImagingSource::CameraImagingSource(void){
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

bool CameraImagingSource::open(std::string serial){
    qint64 qSerial = QString::fromStdString(serial).toInt();
    this->serial = qSerial;
    handle.closeDev();
    bool res = handle.openDev(qSerial);

    if (!handle.isDevValid()) {
      debugMessage("Couldn't open camera");
      QMessageBox alert;
      alert.setText(QString("Couldn't open camera with serial %1.").arg(qSerial));
      alert.exec();
    }else{
        this->width = handle.getVideoFormat().getSize().cx;
        this->height = handle.getVideoFormat().getSize().cy;

        debugMessage("Opened camera");
    }

    return res;
}

std::string CameraImagingSource::getSerial(void){
    qint64 serial_tmp;
    handle.getDev().getSerialNumber(serial_tmp);
    std::string serial_str = QString::number(serial_tmp).toStdString();
    return serial_str;
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
  handle.addListener((DShowLib::GrabberListener*)listener, Listener::eFRAMEREADY);
}

void CameraImagingSource::showProperties() { handle.showVCDPropertyPage(0, ""); }

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

  return;
}

void CameraImagingSource::stopCapture() {
  handle.stopLive();
  handle.setSinkType(NULL);

  emit stopped();
  return;
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

void CameraImagingSource::changeGain(int gain){
    setGain(gain);
}

void CameraImagingSource::changeExposure(double exposure){
    setExposure(exposure);
}

void CameraImagingSource::enableAutoExposure(bool enable){
    DShowLib::tIVCDPropertyItemsPtr pItems = handle.getAvailableVCDProperties();
    if( pItems != 0 )
    {
        // Try to find the exposure item
        DShowLib::tIVCDPropertyItemPtr pExposureItem = pItems->findItem( DShowLib::VCDID_Exposure );

        DShowLib::tIVCDSwitchPropertyPtr m_pExposureAuto;

        DShowLib::tIVCDPropertyElementPtr pExposureAutoElement =  pExposureItem->findElement( DShowLib::VCDElement_Auto );

        // If an auto element exists, try to acquire a switch interface
        if( pExposureAutoElement != 0 )
        {
            pExposureAutoElement->getInterfacePtr( m_pExposureAuto );

            // If successful, disable automation
            if( m_pExposureAuto != 0 )
                m_pExposureAuto->setSwitch( enable );
        }
    }
}

void CameraImagingSource::enableAutoGain(bool enable){
    DShowLib::tIVCDPropertyItemsPtr pItems = handle.getAvailableVCDProperties();
    if( pItems != 0 )
    {
        // Try to find the exposure item
        DShowLib::tIVCDPropertyItemPtr pGainItem = pItems->findItem( DShowLib::VCDID_Gain );

        DShowLib::tIVCDSwitchPropertyPtr m_pGainAuto;

        DShowLib::tIVCDPropertyElementPtr pGainAutoElement =  pGainItem->findElement( DShowLib::VCDElement_Auto );

        // If an auto element exists, try to acquire a switch interface
        if( pGainAutoElement != 0 )
        {
            pGainAutoElement->getInterfacePtr( m_pGainAuto );

            // If successful, disable automation
            if( m_pGainAuto != 0 )
                m_pGainAuto->setSwitch( enable );
        }
    }
}


void CameraImagingSource::setExposure(double exposure) {
    DShowLib::tIVCDPropertyItemsPtr pItems = handle.getAvailableVCDProperties();
    if( pItems != 0 )
    {
        // Try to find the exposure item
        DShowLib::tIVCDPropertyItemPtr pExposureItem = pItems->findItem( DShowLib::VCDID_Exposure );

        DShowLib::tIVCDAbsoluteValuePropertyPtr m_pExposureValue;

        DShowLib::tIVCDPropertyElementPtr pExposureValueElement = pExposureItem->findElement( DShowLib::VCDElement_Value );

        // If a value element exists, try to acquire a range interface
        if( pExposureValueElement != 0 )
        {
            pExposureValueElement->getInterfacePtr( m_pExposureValue );

            double current_exposure = m_pExposureValue->getValue();
            double exposure_d = exposure/100;

            qDebug() << "current exposure: " << current_exposure;
            qDebug() << "new exposure: " << exposure_d;

            m_pExposureValue->setValue( exposure_d );
        }
    }
}

void CameraImagingSource::setGain(int gain) {
    DShowLib::tIVCDPropertyItemsPtr pItems = handle.getAvailableVCDProperties();
    if( pItems != 0 )
    {
        // Try to find the exposure item
        DShowLib::tIVCDPropertyItemPtr pGainItem = pItems->findItem( DShowLib::VCDID_Gain );

        DShowLib::tIVCDAbsoluteValuePropertyPtr m_pGainValue;

        DShowLib::tIVCDPropertyElementPtr pGainValueElement = pGainItem->findElement( DShowLib::VCDElement_Value );

        // If a value element exists, try to acquire a range interface
        if( pGainValueElement != 0 )
        {
            pGainValueElement->getInterfacePtr( m_pGainValue );

            double current_gain = m_pGainValue->getValue();
            int gain_i = gain;

            qDebug() << "current gain: " << current_gain;
            qDebug() << "new gain: " << gain_i;

            m_pGainValue->setValue( gain_i );
        }
    }
}

/*
void CameraImagingSource::setGain(int gain) {
  if (handle.setProperty(VideoProcAmp_Gain, (long)gain)) {
    debugMessage("Couldn't set gain");
  }
}
*/

double CameraImagingSource::getExposure() {
  auto camera_properties = handle.getAvailableVCDProperties();

  // Retrieve the absolute value interface for exposure.
  DShowLib::tIVCDAbsoluteValuePropertyPtr pAbsVal = 0;
  camera_properties->findInterfacePtr(DShowLib::VCDID_Exposure,
                                      DShowLib::VCDElement_Value, pAbsVal);

  return pAbsVal.get()->getValue();
}

void CameraImagingSource::grabImage() {

  if (!handle.isLive()){
      qDebug() << "Not live";
      emit grabError();
  }

  auto result = frame_sink->snapImagesAsync(1);

  if (result.isError()) {
    qDebug() << "Failed to capture image";
    qDebug() << result.c_str();
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

  if (setRate == 0){
      setRate = handle.getCurrentMaxAvailableFPS();
  }
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
    while (handle.isLive());

    handle.setExternalTrigger(trigger);

    if (was_live) {
      startCapture();
    }
}

void CameraImagingSource::close(){
    debugMessage("Freeing camera");
    stopCapture();
    debugMessage("Waiting for camera handle to close...");
    while (handle.isLive());
    handle.closeDev();
    debugMessage("Closing camera thread");
    finishThread();
}

void CameraImagingSource::finishThread(){
    emit finished();
}

CameraImagingSource::~CameraImagingSource() {
  close();
}
