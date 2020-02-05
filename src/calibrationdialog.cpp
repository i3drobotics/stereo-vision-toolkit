/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "calibrationdialog.h"
#include "ui_calibrationdialog.h"

CalibrationDialog::CalibrationDialog(AbstractStereoCamera* stereo_cam, QWidget* parent)
    : QDialog(parent), ui(new Ui::CalibrationDialog) {
  ui->setupUi(this);
  this->stereo_cam = stereo_cam;
  this->setAttribute(Qt::WA_DeleteOnClose);
  this->running = false;
  image_width = stereo_cam->getWidth();
  image_height = stereo_cam->getHeight();

  double scale = sqrt( (image_width*image_height) / (640.*480.));

  if (scale > 1.0){
      m_scale = scale;
  } else {
      m_scale = 1;
  }
  //p_min_change = 50 * m_scale;
  //size_min_change = 50 * m_scale;
  p_min_change = 50;
  size_min_change = 100;
  skew_min_change = 0.1;

  connect(ui->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
  connect(ui->calibrateButton, SIGNAL(clicked()), this, SLOT(calibrate()));
  connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(processImages()));
  connect(ui->runButton, SIGNAL(clicked()), this, SLOT(toggleRun()));
  connect(ui->toolButtonOutput, SIGNAL(clicked(bool)), this,
          SLOT(selectOutputPath()));

  settings = new QSettings("I3Dr", "Stereo Vision Toolkit", this);
  ui->lineEditOutput->setText(settings->value("calDir").toString());
  updateOutputPath();
}

void CalibrationDialog::calibrate(){
    emit startCalibration();
}

void CalibrationDialog::toggleRun(){
    if (this->running){
        ui->runButton->setText("Run");
    } else {
        ui->runButton->setText("Stop");
    }
    this->running = !this->running;
}

int CalibrationDialog::getCornerIndex(int row, int col){
    int ydim = ui->patternRowSpinBox->value();
    return ((((row-1)*ydim)+col)-1);
}

std::vector<cv::Point2f> CalibrationDialog::getBoardOutsideCorners(std::vector<cv::Point2f> corners){

    int xdim = ui->patternColSpinBox->value();
    int ydim = ui->patternRowSpinBox->value();

    std::vector<cv::Point2f> outside_corners;
    int top_left_index = getCornerIndex(1,1);
    int top_right_index = getCornerIndex(1,ydim);
    int bot_left_index = getCornerIndex(xdim,1);
    int bot_right_index = getCornerIndex(xdim,ydim);

    cv::Point2f top_left = corners.at(top_left_index);
    cv::Point2f top_right = corners.at(top_right_index);
    cv::Point2f bot_left = corners.at(bot_left_index);
    cv::Point2f bot_right = corners.at(bot_right_index);

    outside_corners.push_back(top_left);
    outside_corners.push_back(top_right);
    outside_corners.push_back(bot_left);
    outside_corners.push_back(bot_right);

    return outside_corners;
}

cv::RotatedRect CalibrationDialog::getBoardRect(std::vector<cv::Point2f> corners){
    cv::RotatedRect box = cv::minAreaRect(corners);
    return box;
}

cv::Mat CalibrationDialog::downsampleImage(cv::Mat image){
    cv::Mat image_d;

    if (m_scale > 1.0){
        if (!image.empty()){
            cv::resize(image,image_d,cv::Size(image_width/m_scale,image_height/m_scale));
        }
    } else {
        image.copyTo(image_d);
    }
    return image_d;
}

bool CalibrationDialog::checkImages(cv::Mat left, cv::Mat right){
    // check images should be added to list
    std::vector<cv::Point2f> left_corners;
    // downsample image
    cv::Mat left_d = downsampleImage(left);
    cv::Mat right_d = downsampleImage(right);
    if (findCorners(left_d,left_corners)){
        std::vector<cv::Point2f> right_corners;
        if (findCorners(right_d,right_corners)){
            //Add images to database only if it's sufficiently different from any previous sample.
            cv::RotatedRect left_rect = getBoardRect(left_corners);
            cv::RotatedRect right_rect = getBoardRect(right_corners);
            std::vector<cv::Point2f> left_outer_corners = getBoardOutsideCorners(left_corners);
            std::vector<cv::Point2f> right_outer_corners = getBoardOutsideCorners(right_corners);
            bool skew_diff = checkSkewDiff(left_outer_corners,right_outer_corners);
            bool size_diff = checkSizeDiff(left_rect,right_rect);
            bool XY_diff = checkXYDiff(left_rect,right_rect);
            updateProgress();
            if (XY_diff || skew_diff || size_diff){
                return true;
            }
        }
    }
    return false;
}

bool CalibrationDialog::checkSizeDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect){

    int l_size = left_rot_rect.size.area();
    int r_size = right_rot_rect.size.area();

    int c_min_size, c_max_size;
    if (l_size < r_size){
        c_max_size = r_size;
        c_min_size = l_size;
    } else {
        c_max_size = l_size;
        c_min_size = r_size;
    }

    bool isDifferent = false;

    if (min_size == -1){
        min_size = c_min_size;
        isDifferent = true;
    } else if (c_min_size < (min_size)-size_min_change){
        min_size = c_min_size;
        isDifferent = true;
    }

    if (max_size == -1){
        max_size = c_max_size;
        isDifferent = true;
    } else if (c_max_size > (max_size)+size_min_change){
        max_size = c_max_size;
        isDifferent = true;
    }

    qDebug() << "Size: " << c_max_size;

    return isDifferent;
}

bool CalibrationDialog::checkSkewDiff(std::vector<cv::Point2f> left_outer_corners, std::vector<cv::Point2f> right_outer_corners){

    bool isDifferent = false;

    cv::Point2f l_topLeft = left_outer_corners.at(0);
    cv::Point2f l_topRight = left_outer_corners.at(1);
    cv::Point2f l_bottomLeft = left_outer_corners.at(2);
    cv::Point2f l_bottomRight = left_outer_corners.at(3);

    cv::Point2f r_topLeft = right_outer_corners.at(0);
    cv::Point2f r_topRight = right_outer_corners.at(1);
    cv::Point2f r_bottomLeft = right_outer_corners.at(2);
    cv::Point2f r_bottomRight = right_outer_corners.at(3);

    double l_top_length = cv::norm(l_topRight-l_topLeft);
    double l_bottom_length = cv::norm(l_bottomRight-l_bottomLeft);
    double l_right_length = cv::norm(l_bottomRight-l_topRight);
    double l_left_length = cv::norm(l_bottomLeft-l_topLeft);

    double l_y_skew = l_top_length/l_bottom_length;
    double l_x_skew = l_left_length/l_right_length;

    double r_top_length = cv::norm(r_topRight-r_topLeft);
    double r_bottom_length = cv::norm(r_bottomRight-r_bottomLeft);
    double r_right_length = cv::norm(r_bottomRight-r_topRight);
    double r_left_length = cv::norm(r_bottomLeft-r_topLeft);

    double r_y_skew = r_top_length/r_bottom_length;
    double r_x_skew = r_left_length/r_right_length;

    double c_min_x_skew,c_max_x_skew, c_min_y_skew, c_max_y_skew;
    if (l_x_skew < r_x_skew){
        c_max_x_skew = r_x_skew;
        c_min_x_skew = l_x_skew;
    } else {
        c_max_x_skew = l_x_skew;
        c_min_x_skew = r_x_skew;
    }
    if (l_y_skew < r_y_skew){
        c_max_y_skew = r_y_skew;
        c_min_y_skew = l_y_skew;
    } else {
        c_max_y_skew = l_y_skew;
        c_min_y_skew = r_y_skew;
    }

    if (min_x_skew == -1){
        min_x_skew = c_min_x_skew;
        isDifferent = true;
    } else if (c_min_x_skew < (min_x_skew)-skew_min_change){
        min_x_skew = c_min_x_skew;
        isDifferent = true;
    }
    if (min_y_skew == -1){
        min_y_skew = c_min_y_skew;
        isDifferent = true;
    } else if (c_min_y_skew < (min_y_skew)-skew_min_change){
        min_y_skew = c_min_y_skew;
        isDifferent = true;
    }
    if (max_x_skew == -1){
        max_x_skew = c_max_x_skew;
        isDifferent = true;
    } else if (c_max_x_skew > (max_x_skew)+skew_min_change){
        max_x_skew = c_max_x_skew;
        isDifferent = true;
    }
    if (max_y_skew == -1){
        max_y_skew = c_max_y_skew;
        isDifferent = true;
    } else if (c_max_y_skew > (max_y_skew)+skew_min_change){
        max_y_skew = c_max_y_skew;
        isDifferent = true;
    }

    return isDifferent;
}

bool CalibrationDialog::checkXYDiff(cv::RotatedRect left_rot_rect, cv::RotatedRect right_rot_rect){

    cv::Rect left_rect = left_rot_rect.boundingRect();
    cv::Rect right_rect = right_rot_rect.boundingRect();

    cv::Point left_minVal = left_rect.tl();
    cv::Point right_minVal = right_rect.tl();

    cv::Point left_maxVal = left_rect.br() - cv::Point(1, 1);
    cv::Point right_maxVal = right_rect.br() - cv::Point(1, 1);

    int c_x_lowest,c_y_lowest,c_x_highest,c_y_highest;

    if (left_minVal.x < right_minVal.x){
        c_x_lowest = left_minVal.x;
    } else {
        c_x_lowest = right_minVal.x;
    }

    if (left_minVal.y < right_minVal.y){
        c_y_lowest = left_minVal.y;
    } else {
        c_y_lowest = right_minVal.y;
    }

    if (left_maxVal.x > right_maxVal.x){
        c_x_highest = left_maxVal.x;
    } else {
        c_x_highest = right_maxVal.x;
    }

    if (left_maxVal.y > right_maxVal.y){
        c_y_highest = left_maxVal.y;
    } else {
        c_y_highest = right_maxVal.y;
    }

    bool isDifferent = false;

    if (x_lowest == -1){
        x_lowest = c_x_lowest;
        isDifferent = true;
    } else if (c_x_lowest < (x_lowest)-p_min_change){
        x_lowest = c_x_lowest;
        isDifferent = true;
    }
    if (y_lowest == -1) {
        y_lowest = c_y_lowest;
        isDifferent = true;
    } else if ( c_y_lowest < (y_lowest)-p_min_change){
        y_lowest = c_y_lowest;
        isDifferent = true;
    }
    if (x_highest == -1){
        x_highest = c_x_highest;
        isDifferent = true;
    } else if (c_x_highest > (x_highest)+p_min_change){
        x_highest = c_x_highest;
        isDifferent = true;
    }
    if (y_highest == -1){
        y_highest = c_y_highest;
        isDifferent = true;
    } else if (c_y_highest > (y_highest)+p_min_change){
        y_highest = c_y_highest;
        isDifferent = true;
    }

    return isDifferent;
}

bool CalibrationDialog::findCorners(cv::Mat image,std::vector<cv::Point2f> &corners) {
    int flags = cv::CALIB_CB_FAST_CHECK;
    cv::Size pattern_size(getPatternCols(), getPatternRows());
    bool found = cv::findChessboardCorners(image, pattern_size, corners, flags);
    return found;
}

void CalibrationDialog::addImages(cv::Mat left, cv::Mat right){
    left_image_list.append(left);
    right_image_list.append(right);
    int image_count = left_image_list.size();
    ui->lblImageCount->setText(
        QString("Images saved: %1").arg(image_count));
}

void CalibrationDialog::processImages(){
    if (this->running){
        //qDebug() << "Checking valid";
        disconnect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(processImages()));

        cv::Mat left,right;
        stereo_cam->getLeftImage(left);
        stereo_cam->getRightImage(right);

        if (checkImages(left,right)){
            //qDebug() << "Adding to list";
            ui->calibrateButton->setEnabled(true);
            addImages(left,right);
        }
        //qDebug() << "Waiting 1 second before new image";
        //TODO remove this wait if checkerboard filters are good enough
        //not to accept too many images (>25)
        // wait 1 second
        QTime dieTime= QTime::currentTime().addSecs(1);
        while (QTime::currentTime() < dieTime){
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }
        connect(stereo_cam, SIGNAL(stereopair_processed()), this, SLOT(processImages()));

    }
}

int CalibrationDialog::getPatternCols(){
    return(ui->patternColSpinBox->value());
}
int CalibrationDialog::getPatternRows(){
    return(ui->patternRowSpinBox->value());
}
double CalibrationDialog::getSquareSizeMm(){
    return(ui->sqSizeSpinBox->value());
}
bool CalibrationDialog::getSaveROS(){
    return(ui->checkBoxROS->isChecked());
}

void CalibrationDialog::updateOutputPath(void) {
  QString dir = ui->lineEditOutput->text();
  if (QDir(dir).exists()) {
    output_path = dir;
  }else{
      qDebug() << "Directory doesn't exist";
  }

  qDebug() << "Output path set to: " << output_path;
}

void CalibrationDialog::selectOutputPath(void){
    QFileDialog dialog;
    QString startPath = QStandardPaths::displayName(QStandardPaths::HomeLocation);
    output_path = dialog.getExistingDirectory(
        this, tr("Calibration output path"), startPath,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (output_path != "") {
      ui->lineEditOutput->setText(QDir::cleanPath(output_path));
      settings->setValue("calDir", output_path);
    }
}

void CalibrationDialog::updateProgress(){
    int x_range, y_range,size_range;
    double x_skew_range,y_skew_range;
    if (x_highest == -1 || x_lowest == -1){
        x_range = 0;
    } else {
        x_range = x_highest - x_lowest;
    }
    if (y_highest == -1 || y_lowest == -1){
        y_range = 0;
    } else {
        y_range = y_highest - y_lowest;
    }
    if (max_size == -1 || min_size == -1){
        size_range = 0;
    } else {
        size_range = max_size - min_size;
    }
    if (max_x_skew == -1 || min_x_skew == -1){
        x_skew_range = 0;
    } else {
        x_skew_range = max_x_skew - min_x_skew;
    }
    if (max_y_skew == -1 || min_y_skew == -1){
        y_skew_range = 0;
    } else {
        y_skew_range = max_y_skew - min_y_skew;
    }

    qDebug() << "X range: " << x_range << " (" << x_lowest << " - " << x_highest << ")";
    qDebug() << "Y range: " << y_range << " (" << y_lowest << " - " << y_highest << ")";
    qDebug() << "Size range: " << size_range << " (" << min_size << " - " << max_size << ")";
    qDebug() << "X Skew range: " << x_skew_range << " (" << min_x_skew << " - " << max_x_skew << ")";
    qDebug() << "X Skew range: " << y_skew_range << " (" << min_y_skew << " - " << max_y_skew << ")";

    int x_goal = image_width*0.6;
    int y_goal = image_height*0.6;
    int size_goal = ((image_height*image_width)/2)*0.6;
    double x_skew_goal = 0.5;
    double y_skew_goal = 0.3;

    int x_complete = ((float)(x_range*m_scale)/(float)x_goal)*100;
    int y_complete = ((float)(y_range*m_scale)/(float)y_goal)*100;
    int size_complete = ((float)(size_range*(m_scale*m_scale))/(float)size_goal)*100;
    int x_skew_complete = ((float)x_skew_range/(float)x_skew_goal)*100;
    int y_skew_complete = ((float)y_skew_range/(float)y_skew_goal)*100;

    // calculate total complete from other progress status
    int total_complete = ((float)(x_complete+y_complete+size_complete+x_skew_complete+y_skew_complete)/(float)500)*100;

    // output progress to progress bars
    ui->progressBarX->setValue(x_complete);
    ui->progressBarY->setValue(y_complete);
    ui->progressBarSize->setValue(size_complete);
    ui->progressBarXSkew->setValue(x_skew_complete);
    ui->progressBarYSkew->setValue(y_skew_complete);

    ui->progressBarComplete->setValue(total_complete);
}

CalibrationDialog::~CalibrationDialog() {
  emit stopCalibration();
  delete ui;
}
