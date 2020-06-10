/*
* Copyright I3D Robotics Ltd, 2017
* Author: Josh Veitch-Michaelis
*/

#include "stereocameradeimos.h"

bool StereoCameraDeimos::initCamera(AbstractStereoCamera::stereoCameraSerialInfo camera_serial_info,AbstractStereoCamera::stereoCameraSettings inital_camera_settings){

    double exposure = inital_camera_settings.exposure;
    int fps = inital_camera_settings.fps;
    bool hdr = inital_camera_settings.hdr;
    bool autoExpose;
    if (inital_camera_settings.autoExpose == 1){
        autoExpose = true;
    } else {
        autoExpose = false;
    }
    this->camera_serial_info = camera_serial_info;
    int usb_index = usb_index_from_serial(camera_serial_info.left_camera_serial);
    if (usb_index >= 0) {
        openHID();

        camera = cv::VideoCapture(cv::CAP_DSHOW + usb_index);

        if (camera.isOpened()) {
            bool res = setFrameSize(752, 480);

            toggleHDR(hdr);
            enableAutoExpose(autoExpose);
            setExposure(exposure);
            changeFPS(fps);
            getFrameRate();
            getTemperature();

            temperature_timer = new QTimer(parent());
            temperature_timer->setInterval(1000);
            //connect(temperature_timer, SIGNAL(timeout()), this, SLOT(getTemperature()));
            temperature_timer->start();

            if (!res) {
                camera.release();
                connected = false;
            } else {
                connected = true;
            }
        }
    }
    return connected;
}

std::string StereoCameraDeimos::get_device_path_serial(IMoniker *pMoniker){
    std::string device_path_serial;
    IPropertyBag *pPropBag;
    HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
    if (FAILED(hr)) {
        pMoniker->Release();
        return "";
    }

    VARIANT var;
    VariantInit(&var);

    // Get description or friendly name.
    hr = pPropBag->Read(L"Description", &var, 0);
    if (FAILED(hr)) {
        hr = pPropBag->Read(L"FriendlyName", &var, 0);
    }

    if (SUCCEEDED(hr)) {
        std::wstring str(var.bstrVal);

        if (std::string(str.begin(), str.end()) == "See3CAM_Stereo") {
            qDebug() << "Found Deimos at device";

            hr = pPropBag->Read(L"DevicePath", &var, 0);

            device_path = var.bstrVal;
            std::wstring device_path_wstr(device_path);
            std::string device_path_str = std::string(device_path_wstr.begin(), device_path_wstr.end());
            device_path_serial = serial_from_device_path(device_path_str);
        }
        VariantClear(&var);
    }

    hr = pPropBag->Write(L"FriendlyName", &var);

    pPropBag->Release();
    pMoniker->Release();
    return device_path_serial;
}

std::vector<AbstractStereoCamera::stereoCameraSerialInfo> StereoCameraDeimos::listSystems(){
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_DEIMOS);
    std::vector<AbstractStereoCamera::stereoCameraSerialInfo> connected_serial_infos;

    // Create the System Device Enumerator.
    ICreateDevEnum *pDevEnum;
    IEnumMoniker *pEnum;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
                                  CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));
    if (SUCCEEDED(hr)) {
        // Create an enumerator for the category.
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum,
                                             0);
        if (hr != S_OK) {
            hr = VFW_E_NOT_FOUND;  // The category is empty - no camera connected
        } else {
            pDevEnum->Release();

            IMoniker *pMoniker = NULL;
            int i = 1;
            while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
                IPropertyBag *pPropBag;
                HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
                if (FAILED(hr)) {
                    pMoniker->Release();
                    continue;
                }

                VARIANT var;
                VariantInit(&var);

                // Get description or friendly name.
                hr = pPropBag->Read(L"Description", &var, 0);
                if (FAILED(hr)) {
                    hr = pPropBag->Read(L"FriendlyName", &var, 0);
                }

                if (SUCCEEDED(hr)) {
                    std::wstring str(var.bstrVal);
                    if (std::string(str.begin(), str.end()) == "See3CAM_Stereo") {
                        std::string device_path_serial = get_device_path_serial(pMoniker);
                        bool device_known = false;
                        AbstractStereoCamera::stereoCameraSerialInfo serial_info;
                        for (auto& known_serial_info : known_serial_infos) {
                            if (device_path_serial == known_serial_info.left_camera_serial){
                                serial_info = known_serial_info;
                                device_known = true;
                                break;
                            }
                        }
                        if (!device_known){
                            serial_info.camera_type = CAMERA_TYPE_DEIMOS;
                            serial_info.left_camera_serial = device_path_serial;
                            serial_info.right_camera_serial = device_path_serial;
                            std::string tmp_identity = QString::number(i).rightJustified(5, '0').toStdString();
                            serial_info.i3dr_serial = tmp_identity;
                            //serial_info.filename
                        }
                        connected_serial_infos.push_back(serial_info);
                        i++;
                    }
                    VariantClear(&var);
                }
                hr = pPropBag->Write(L"FriendlyName", &var);

                pPropBag->Release();
                pMoniker->Release();
            }
        }
    }
    return connected_serial_infos;
}

void StereoCameraDeimos::openHID(void) {
    hid_init();

    deimos_device = hid_open(0x2560, 0xC114, NULL);

    if (deimos_device == NULL) {
        qDebug() << "Couldn't open device";
    } else {
        hid_set_nonblocking(deimos_device, 1);
        exposure = getExposure();
        qDebug() << "Serial: " << (qint64) getSerial();
    }

    hid_exit();
}

qint64 StereoCameraDeimos::getSerial() {
    if (deimos_device == NULL) return -1;

    qint64 serial = 0;

    std::vector<unsigned char> buffer;
    buffer.resize(16);

    buffer.at(1) = GETCAMERA_UNIQUEID;

    if (!send_hid(buffer, 1)) {
        serial = 0;
    } else {
        for (int i = 1, k = 3; i < 5; i++, k--) serial |= buffer[i] << (k * 8);
    }

    return serial;
}

int StereoCameraDeimos::usb_index_from_serial(std::string serial){
    // Create the System Device Enumerator.
    ICreateDevEnum *pDevEnum;
    IEnumMoniker *pEnum;
    int usb_device_index;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
                                  CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));
    if (SUCCEEDED(hr)) {
        // Create an enumerator for the category.
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum,
                                             0);
        if (hr != S_OK) {
            hr = VFW_E_NOT_FOUND;  // The category is empty - no camera connected
        } else {
            pDevEnum->Release();

            IMoniker *pMoniker = NULL;
            int i = 0;
            while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
                IPropertyBag *pPropBag;
                HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
                if (FAILED(hr)) {
                    pMoniker->Release();
                    continue;
                }

                VARIANT var;
                VariantInit(&var);

                // Get description or friendly name.
                hr = pPropBag->Read(L"Description", &var, 0);
                if (FAILED(hr)) {
                    hr = pPropBag->Read(L"FriendlyName", &var, 0);
                }

                if (SUCCEEDED(hr)) {
                    std::wstring str(var.bstrVal);

                    if (std::string(str.begin(), str.end()) == "See3CAM_Stereo") {
                        qDebug() << "Found Deimos at device " << i;

                        hr = pPropBag->Read(L"DevicePath", &var, 0);

                        device_path = var.bstrVal;
                        std::wstring device_path_wstr(device_path);
                        std::string device_path_str = std::string(device_path_wstr.begin(), device_path_wstr.end());
                        std::string usb_device_serial = serial_from_device_path(device_path_str);
                        if (usb_device_serial == serial){
                            usb_device_index = i;
                        }
                    }
                    VariantClear(&var);
                }

                hr = pPropBag->Write(L"FriendlyName", &var);

                pPropBag->Release();
                pMoniker->Release();
                i++;
            }
        }
    }
    return usb_device_index;
}

std::string StereoCameraDeimos::serial_from_usb_index(int index){
    // Create the System Device Enumerator.
    ICreateDevEnum *pDevEnum;
    IEnumMoniker *pEnum;
    std::string usb_device_serial;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
                                  CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));
    if (SUCCEEDED(hr)) {
        // Create an enumerator for the category.
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum,
                                             0);
        if (hr != S_OK) {
            hr = VFW_E_NOT_FOUND;  // The category is empty - no camera connected
        } else {
            pDevEnum->Release();

            IMoniker *pMoniker = NULL;
            int i = 0;
            while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
                if (i == index){
                    IPropertyBag *pPropBag;
                    HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
                    if (FAILED(hr)) {
                        pMoniker->Release();
                        continue;
                    }

                    VARIANT var;
                    VariantInit(&var);

                    // Get description or friendly name.
                    hr = pPropBag->Read(L"Description", &var, 0);
                    if (FAILED(hr)) {
                        hr = pPropBag->Read(L"FriendlyName", &var, 0);
                    }

                    if (SUCCEEDED(hr)) {
                        std::wstring str(var.bstrVal);

                        if (std::string(str.begin(), str.end()) == "See3CAM_Stereo") {
                            qDebug() << "Found Deimos at device " << i;

                            hr = pPropBag->Read(L"DevicePath", &var, 0);

                            device_path = var.bstrVal;
                            std::wstring device_path_wstr(device_path);
                            std::string device_path_str = std::string(device_path_wstr.begin(), device_path_wstr.end());
                            usb_device_serial = serial_from_device_path(device_path_str);
                        }
                        VariantClear(&var);
                    }

                    hr = pPropBag->Write(L"FriendlyName", &var);

                    pPropBag->Release();
                    pMoniker->Release();
                }
                i++;
            }
        }
    }
    return usb_device_serial;
}

std::string StereoCameraDeimos::serial_from_device_path(std::string usb_device_path){
    // get unique usb serial
    // see: https://www.silabs.com/community/interface/knowledge-base.entry.html/2013/11/21/windows_usb_devicep-aGxD
    std::string camera_serial;
    std::vector<std::string> device_path_delim;
    std::string token;
    std::istringstream tokenStream(usb_device_path);
    while (std::getline(tokenStream, token, '#'))
    {
        device_path_delim.push_back(token);
    }
    if (device_path_delim.size() == 4) {
        camera_serial = device_path_delim.at(2);
        qDebug() << "Deimos serial: " << camera_serial.c_str();
    } else {
        qDebug() << "Deimos device path formatted incorrectly: " << usb_device_path.c_str();
        for (std::vector<std::string>::iterator it = device_path_delim.begin() ; it != device_path_delim.end(); ++it)
            qDebug() << (*it).c_str();
    }
    return camera_serial;
}

int StereoCameraDeimos::getExposure() {
    /*
   *  Returns the current exposure time in uSeconds, or -1 if an error occurred.
   */

    if (deimos_device == NULL) return -1;

    std::vector<unsigned char> buffer;
    buffer.resize(16);

    buffer.at(1) = CAMERA_CONTROL_STEREO;
    buffer.at(2) = GET_EXPOSURE_VALUE;

    if (!send_hid(buffer, 2)) return -1;

    int exposure;

    if (!buffer.at(10)) {
        return -1;
    } else {
        exposure = ((int)buffer.at(2) << 24) + ((int)buffer.at(3) << 16) +
                ((int)buffer.at(4) << 8) + (int)buffer.at(5);
        return exposure;
    }
}

double StereoCameraDeimos::getTemperature(void){
    std::vector<unsigned char> buffer(16);

    buffer.at(1) = CAMERA_CONTROL_STEREO;
    buffer.at(2) = GET_IMU_TEMP_DATA;

    if (!send_hid(buffer, 2)) return -1;

    if (!buffer.at(6)) {
        return -1;
    } else {
        uint8_t msb = buffer.at(2);
        uint8_t lsb = buffer.at(3);
        double temp = (msb << 8) | lsb;

        temp = ((temp - 20.0) / 16.0) + 21.0;
        emit temperature_C(temp);

        return temp;
    }

}

void StereoCameraDeimos::adjustFPS(int fps){
    changeFPS(fps);
}

void StereoCameraDeimos::toggleAutoExpose(bool enable){
    enableAutoExpose(enable);
}

void StereoCameraDeimos::adjustExposure(double exposure){
    setExposure(exposure);
}

bool StereoCameraDeimos::enableAutoExpose(bool enable) {
    if (enable)
        return setExposure(0.001);
    else
        return setExposure(exposure / 1000);
}

bool StereoCameraDeimos::setExposure(double exposure_milliseconds) {
    if (deimos_device == NULL) return false;
    if (exposure_milliseconds <= 0 || exposure_milliseconds > 1000) return false;

    std::vector<unsigned char> buffer;
    buffer.resize(16);

    buffer[1] = CAMERA_CONTROL_STEREO;
    buffer[2] = SET_EXPOSURE_VALUE;

    int exposure_val = exposure_milliseconds * 1000;

    qDebug() << "Setting exposure to" << exposure_val;

    buffer[3] = (char)((exposure_val >> 24) & 0xFF);
    buffer[4] = (char)((exposure_val >> 16) & 0xFF);
    buffer[5] = (char)((exposure_val >> 8) & 0xFF);
    buffer[6] = (char)(exposure_val & 0xFF);

    if (!send_hid(buffer, 2)){
        qDebug() << "Failed to send exposure command";
        return false;
    }

    if (getExposure() == exposure_val) {
        if (exposure_val != 1) exposure = exposure_val;
        return true;
    }else{
        qDebug() << "Exposure read back incorrect";
    }

    return false;
}

void StereoCameraDeimos::changeFPS(int fps){
    double fps_d;
    if (fps == 30){
        fps_d = 1;
    } else {
        fps_d = 0;
    }
    const std::lock_guard<std::mutex> lock(mtx);
    camera.set(CV_CAP_PROP_FPS, (double)fps_d);
}

bool StereoCameraDeimos::toggleHDR(bool enable){
    if (deimos_device == NULL) return false;

    std::vector<unsigned char> buffer;
    buffer.resize(4);

    buffer.at(1) = CAMERA_CONTROL_STEREO;
    buffer.at(2) = SET_HDR_MODE_STEREO;

    if(enable){
        buffer.at(3) = 1;
    }else{
        buffer.at(3) = 0;
    }

    if (!send_hid(buffer, 2)){
        qDebug() << "Failed to send HDR command";
        return false;
    }else{
        return true;
    }
}


bool StereoCameraDeimos::send_hid(std::vector<unsigned char> &buffer,
                                  size_t command_len) {
    if (deimos_device == NULL) return false;

    std::vector<unsigned char> commands;
    for (int i = 0; i < command_len; i++) {
        commands.push_back(buffer[i + 1]);
    }

    int result = hid_write(deimos_device, buffer.data(), buffer.size());

    if (result < 0) {
        qDebug() << "Failed to write HID data";
        return false;
    }

    result = hid_read_timeout(deimos_device, buffer.data(), buffer.size(), 500);

    if (result < 0) {
        qDebug() << "Failed to read HID data";
        return false;
    } else {
        for (int i = 0; i < command_len; i++) {
            if (commands[i] != buffer[i]) return false;
        }
    }

    return true;
}

bool StereoCameraDeimos::setFrameSize(int width, int height) {
    bool res = false;

    res = camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
    res &= camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    image_width = width;
    image_height = height;
    image_size = cv::Size(width, height);

    return res;
}

bool StereoCameraDeimos::setFrame16(void) {
    bool res = false;
    const std::lock_guard<std::mutex> lock(mtx);
    res = camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));

    return res;
}

void StereoCameraDeimos::getFrameRate() {
    const std::lock_guard<std::mutex> lock(mtx);
    frame_rate = (int)camera.get(CV_CAP_PROP_FPS);
    qDebug() << "Frame rate: " << frame_rate;
}

void StereoCameraDeimos::disconnectCamera() {
    if (connected){
        if (camera.isOpened()) {
            camera.release();
            hid_close(deimos_device);
            hid_exit();
        }
    }
    connected = false;
    emit disconnected();
    //emit finished();
}

bool StereoCameraDeimos::capture() {

    if(capturing) return false;

    frametimer.restart();

    captured_stereo = false;
    capturing = true;

    bool res = false;

    //const std::lock_guard<std::mutex> lock(mtx);
    if(connected && camera.grab()){
        if(camera.retrieve(image_buffer)){

            flip(image_buffer, image_buffer, 0);
            split(image_buffer, channels);

            left_raw = channels[1].clone();
            right_raw = channels[2].clone();

            res = true;

        }else{
            qDebug() << "Retrieve fail";
            res = false;
        }
    }else{
        qDebug() << "Grab fail";
        res = false;
    }

    emit left_captured();
    emit right_captured();

    capturing = false;

    return res;
}

std::string StereoCameraDeimos::wchar_to_string(WCHAR * buffer) {
    std::wstring ws(buffer);
    std::string buffer_s(ws.begin(), ws.end());
    return buffer_s;
}

StereoCameraDeimos::~StereoCameraDeimos(void) {
    if (connected){
        disconnectCamera();
    }
}
