#include "stereocameratis.h"

std::vector<qint64> StereoCameraTIS::listSystems(){
    std::vector<qint64> camera_serials;
    /* Get number of cameras */
    DShowLib::InitLibrary();

    DShowLib::Grabber handle;

    auto devices = handle.getAvailableVideoCaptureDevices();

    QList<qint64> serials;

    for (auto& device : *devices) {
        qint64 serial;
        device.getSerialNumber(serial);
        serials.append((qint64) serial);
        camera_serials.push_back((qint64) serial);
        qDebug() << "Found camera: " << (qint64) serial;
    }
    return camera_serials;
}

bool StereoCameraTIS::autoConnect(){
    std::vector<qint64> camera_serials = listSystems();

    int nCameras = camera_serials.size();

    auto known_serials = load_serials(qApp->applicationDirPath() + "/params/phobos_serials.ini");

    /* Camera connected */
    if(known_serials.length() < 2){
        /* No serials */

        QString s = "";
        for(auto& serial : camera_serials){
            s += QString::number(serial) + "\n";
        }

        //QMessageBox msg;
        //msg.setText("Not enough valid serials in serial file. Cameras connected: \n"+s);
        //msg.exec();
        return false;
    }else{
        /* Serials known */
        int number_stereo_boxes = nCameras/ 2;
        int number_found = 0;

        for (int i = 0; i < number_stereo_boxes; i++) {

            auto left_camera = new CameraImagingSource();
            auto right_camera = new CameraImagingSource();

            qint64 left_serial = known_serials.at(2 * i);
            qint64 right_serial = known_serials.at(2 * i + 1);

            if (left_camera->open(left_serial) && right_camera->open(right_serial)) {
              stereo_cameras.emplace_back(left_camera);
              stereo_listeners.emplace_back(new Listener);

              stereo_cameras.emplace_back(right_camera);
              stereo_listeners.emplace_back(new Listener);

              assert(left_camera->getSerial() == left_serial);
              assert(right_camera->getSerial() == right_serial);

              number_found++;
          }
        }

        if(number_found > 0){
            setCameras(stereo_cameras.at(0).get(), stereo_cameras.at(1).get());
            return true;
        }
    }

    return false;

}

QList<qint64> StereoCameraTIS::load_serials(QString filename) {
  QList<qint64> serials;

  QFile inputFile(filename);
  if (inputFile.open(QIODevice::ReadOnly)) {
    QTextStream in(&inputFile);
    while (!in.atEnd()) {
      QStringList line = in.readLine().split(' ');

      if(line.size() == 3){
          qint64 serial = line.at(0).toLongLong();
          int width = line.at(1).toInt();
          int height = line.at(2).toInt();

          if (serial) {
            serials.append(serial);
            widths.append(width);
            heights.append(height);
            qDebug() << "Looking for Phobos camera serial " << (qint64)serial;
          }
      }
    }
    inputFile.close();
  } else {
    qDebug() << "Couldn't open Phobos camera serials file " << filename;
  }

  return serials;
}

bool StereoCameraTIS::setCameras(CameraImagingSource *left_camera, CameraImagingSource *right_camera){
    this->left_camera = left_camera;
    this->right_camera = right_camera;

    setup_cameras();

    return true;
}

void StereoCameraTIS::setup_cameras(){

    QThread* left_camera_thread = new QThread();
    left_camera->assignThread(left_camera_thread);

    QThread* right_camera_thread = new QThread();
    right_camera->assignThread(right_camera_thread);

    image_height = heights.at(0);
    image_width = widths.at(1);

    image_size = cv::Size(image_width, image_height);
    emit update_size(image_width, image_height, 1);

    left_camera->setup(image_width, image_height, 50);
    right_camera->setup(image_width, image_height, 50);

    left_raw.create(image_size, CV_8UC1);
    right_raw.create(image_size, CV_8UC1);

    auto left_listener = stereo_listeners.at(0).get();
    auto right_listener = stereo_listeners.at(1).get();

    left_listener->setOutputBuffer(left_raw.data);
    right_listener->setOutputBuffer(right_raw.data);

    left_camera->setListener((DShowLib::GrabberListener*)  left_listener);
    right_camera->setListener((DShowLib::GrabberListener*) right_listener);

    assert(left_camera->getListener() != nullptr);
    assert(right_camera->getListener() != nullptr);

    connect((Listener*) left_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(register_left_capture()));
    connect((Listener*) right_camera->getListener(), SIGNAL(grabbed(void*)), this, SLOT(register_right_capture()));

    connect(this, SIGNAL(start_capture(void)), left_camera, SLOT(startCapture(void)));
    connect(this, SIGNAL(start_capture(void)), right_camera, SLOT(startCapture(void)));

    connect(this, SIGNAL(stereo_grab(void)), left_camera, SLOT(grabImage(void)));
    connect(this, SIGNAL(stereo_grab(void)), right_camera, SLOT(grabImage(void)));

    connected = true;

    emit start_capture();
}

void StereoCameraTIS::loadLeftSettings(){
    left_camera->showProperties();
}

void StereoCameraTIS::loadRightSettings(){
    right_camera->showProperties();
}

bool StereoCameraTIS::capture(){

    frametimer.restart();

    capturing = true;
    captured_stereo = false;

    /* Asynchronous grab */
    emit stereo_grab();

    return true;

}

void StereoCameraTIS::disconnectCamera(){
    if (connected){
        left_camera->close();
        right_camera->close();
    }
    connected = false;
    emit finished();
    emit disconnected();
}

StereoCameraTIS::~StereoCameraTIS(){
   disconnectCamera();
}
