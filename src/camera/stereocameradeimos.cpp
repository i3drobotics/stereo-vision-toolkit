/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameradeimos.h"

bool StereoCameraDeimos::openCamera(){
    if (isConnected()){
        closeCamera();
    }
    double exposure = stereoCameraSettings_.exposure;
    int fps = stereoCameraSettings_.fps;
    bool hdr = stereoCameraSettings_.hdr;
    bool autoExpose;
    if (stereoCameraSettings_.autoExpose == 1){
        autoExpose = true;
    } else {
        autoExpose = false;
    }
    // Get usb index of camera with selected serial
    // use left serial as deimos only has a single connection so right and left are the same in serialinfo
    int usb_index = usb_index_from_serial(stereoCameraSerialInfo_.left_camera_serial);
    if (usb_index >= 0) {
        openHID();

        camera = cv::VideoCapture(cv::CAP_DSHOW + usb_index);

        if (camera.isOpened()) {
            bool res = setFrameSize(752, 480);

            enableHDR(hdr);
            enableAutoExposure(autoExpose);
            setExposure(exposure);
            setFPS(fps);
            getFrameRate();

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

bool StereoCameraDeimos::closeCamera(){
    if (connected){
        if (camera.isOpened()) {
            camera.release();
            hid_close(deimos_device);
            hid_exit();
        }
    }
    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraDeimos::captureSingle(){
    bool res = false;
    if(connected && camera.grab()){
        if (camera.grab()){
            if(camera.retrieve(image_buffer)){

                flip(image_buffer, image_buffer, 0);
                split(image_buffer, channels);

                cv::Mat left_raw_tmp = channels[1].clone();
                cv::Mat right_raw_tmp = channels[2].clone();
                //TODO remove this (only here for testing colour)
                cv::cvtColor(left_raw_tmp, left_raw, cv::COLOR_GRAY2BGR);
                cv::cvtColor(right_raw_tmp, right_raw, cv::COLOR_GRAY2BGR);
                //left_raw = left_raw_tmp.clone();
                //right_raw = right_raw_tmp.clone();

                res = true;

            }else{
                qDebug() << "Retrieve fail";
                res = false;
            }
        } else {
            qDebug() << "Grab fail";
            res = false;
        }
    }else{
        qDebug() << "Cannot grab image. Camera is not connected";
        res = false;
        closeCamera();
    }
    if (!res){
        send_error(CAPTURE_ERROR);
        emit captured_fail();
    } else {
        emit captured_success();
    }
    emit captured();
    return res;
}

void StereoCameraDeimos::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraDeimos::captureSingle);
}

bool StereoCameraDeimos::enableCapture(bool enable){
    if (enable){
        //Start capture thread
        connect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = true;
        captureThreaded();
    } else {
        //Stop capture thread
        disconnect(this, SIGNAL(captured()), this, SLOT(captureThreaded()));
        capturing = false;
    }
    return true;
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

            BSTR device_path_bstr = var.bstrVal;
            std::wstring device_path_wstr(device_path_bstr);
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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraDeimos::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_DEIMOS);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    // Create the System Device Enumerator.
    CoInitialize(NULL);
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
                        AbstractStereoCamera::StereoCameraSerialInfo serial_info;
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
    CoInitialize(NULL);
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

                        BSTR device_path_bstr = var.bstrVal;
                        std::wstring device_path_wstr(device_path_bstr);
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
    CoInitialize(NULL);
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

                            BSTR device_path_bstr = var.bstrVal;
                            std::wstring device_path_wstr(device_path_bstr);
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

bool StereoCameraDeimos::enableAutoExposure(bool enable) {
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

bool StereoCameraDeimos::setFPS(int fps){
    if (!isCapturing()){
        double fps_d;
        if (fps == 30){
            frame_rate = 30;
            fps_d = 1;
        } else if (fps == 60){
            frame_rate = 60;
            fps_d = 0;
        } else {
            frame_rate = 60;
            fps_d = 0;
        }
        camera.set(cv::CAP_PROP_FPS, fps_d);
        frame_rate = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

bool StereoCameraDeimos::enableHDR(bool enable){
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

    res = camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    res &= camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    image_width = width;
    image_height = height;
    image_bitdepth = 1; //TODO get bit depth
    emit update_size(image_width, image_height, image_bitdepth);

    return res;
}

bool StereoCameraDeimos::setFrame16(void) {
    bool res = false;
    res = camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));

    return res;
}

void StereoCameraDeimos::getFrameRate() {
    int frame_rate_index = (int)camera.get(cv::CAP_PROP_FPS);
    if (frame_rate_index == 0){
        frame_rate = 60;
    } else if (frame_rate_index == 1){
        frame_rate = 30;
    } else {
        frame_rate = 60;
    }
    qDebug() << "Frame rate: " << frame_rate;
}

std::string StereoCameraDeimos::wchar_to_string(WCHAR * buffer) {
    std::wstring ws(buffer);
    std::string buffer_s(ws.begin(), ws.end());
    return buffer_s;
}

StereoCameraDeimos::~StereoCameraDeimos(void) {
    if (connected){
        closeCamera();
    }
}
