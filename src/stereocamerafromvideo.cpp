/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocamerafromvideo.h"

bool StereoCameraFromVideo::initCamera(QString fname) {
  stream = cv::VideoCapture(fname.toStdString());

  if (stream.isOpened()) {
    stream_valid = true;
    stream_file = fname.toStdString();
  }

  image_height = stream.get(CV_CAP_PROP_FRAME_HEIGHT);
  image_width = stream.get(CV_CAP_PROP_FRAME_WIDTH) / 2;
  image_size = cv::Size(image_width, image_height);
  frame_rate = stream.get(CV_CAP_PROP_FPS);

  if(frame_rate <= 0) frame_rate = 60;

  number_frames = stream.get(CV_CAP_PROP_FRAME_COUNT);

  connected = true;

  frame_timer.restart();

  return stream.isOpened();
}

void StereoCameraFromVideo::setPosition(int position){

    stream.set(CV_CAP_PROP_POS_FRAMES, (0.01*number_frames) * position);
}

bool StereoCameraFromVideo::enableAutoExpose(bool enable) {
  return enable;
}

bool StereoCameraFromVideo::capture() {
  bool res = stream.read(image_buffer);

  if (res) {
    // Simulate frame rate

    double delay_needed = 1000.0/frame_rate - frame_timer.elapsed();

    if(delay_needed > 0){
       QThread::msleep(delay_needed);
    }

    if(!isMatching())
        QThread::msleep(1000 / frame_rate);

    emit videoPosition(100*((float) stream.get(CV_CAP_PROP_POS_FRAMES))/number_frames);

    if(image_buffer.channels() == 3)
        cv::cvtColor(image_buffer, image_buffer, CV_RGB2GRAY);

    emit captured();

    cv::Mat(image_buffer,
            cv::Rect(0, 0, image_buffer.cols / 2, image_buffer.rows))
        .copyTo(left_raw);
    cv::Mat(image_buffer, cv::Rect(image_buffer.cols / 2, 0,
                                   image_buffer.cols / 2, image_buffer.rows))
        .copyTo(right_raw);

    frame_timer.restart();

  } else {
    // Loop video
    if (stream_valid) {
      stream = cv::VideoCapture(stream_file);
    } else {
      qDebug() << "Capture fail";
    }
  }

  return res;
}
