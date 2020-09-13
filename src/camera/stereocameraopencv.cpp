/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameraopencv.h"

bool StereoCameraOpenCV::openCamera(){
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
    int usb_index_l = usb_index_from_serial(stereoCameraSerialInfo_.left_camera_serial);
    int usb_index_r = usb_index_from_serial(stereoCameraSerialInfo_.right_camera_serial);
    if (usb_index_l >= 0 && usb_index_r >= 0) {
        openHID();

        camera_l = cv::VideoCapture(cv::CAP_DSHOW + usb_index_l);
        camera_r = cv::VideoCapture(cv::CAP_DSHOW + usb_index_r);

        if (camera_l.isOpened() && camera_r.isOpened()) {
            bool res = setFrameSize(752, 480);

            enableHDR(hdr);
            enableAutoExposure(autoExpose);
            setExposure(exposure);
            setFPS(fps);
            getFrameRate();

            if (!res) {
                camera_l.release();
                camera_r.release();
                connected = false;
            } else {
                connected = true;
            }
        }
    }
    return connected;
}

bool StereoCameraOpenCV::closeCamera(){
    if (connected){
        if (camera_l.isOpened()) {
            camera_l.release();
        }
        if (camera_r.isOpened()) {
            camera_r.release();
        }
        hid_close(cam_device_l);
        hid_close(cam_device_r);
        hid_exit();
    }
    connected = false;
    emit disconnected();
    return true;
}

bool StereoCameraOpenCV::captureSingle(){
    bool res = false;
    if(connected){
        if (camera_l.grab() && camera_r.grab()){
            if(camera_l.retrieve(image_buffer_l) && camera_r.retrieve(image_buffer_r)){

                //flip(image_buffer_l, image_buffer_l, 0);
                //flip(image_buffer_r, image_buffer_r, 0);

                left_raw = image_buffer_l.clone();
                right_raw = image_buffer_r.clone();

                if (left_raw.size().width != image_width || left_raw.size().height != image_height){
                    image_height = left_raw.size().height;
                    image_width = left_raw.size().width;
                    image_bitdepth = 1; //TODO get bit depth
                    emit update_size(image_width, image_height, image_bitdepth);
                    qDebug() << "Image size changed in incomming image";
                    qDebug() << "(" << image_width << "," << image_height << ")";
                }

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
        qDebug() << "Cannot grab image. Camera is not connected.";
        res = false;
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

void StereoCameraOpenCV::captureThreaded(){
    future = QtConcurrent::run(this, &StereoCameraOpenCV::captureSingle);
}

bool StereoCameraOpenCV::enableCapture(bool enable){
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

std::string StereoCameraOpenCV::get_device_path_serial(IMoniker *pMoniker){
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

        if (std::string(str.begin(), str.end()) != "See3CAM_Stereo") {
            qDebug() << "Found USB at device";

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

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraOpenCV::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_USB);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    // Create the System Device Enumerator.
    ICreateDevEnum *pDevEnum;
    IEnumMoniker *pEnum;

    std::vector<std::string> connected_camera_names;
    std::vector<std::string> connected_serials;

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
                    std::wstring wstr(var.bstrVal);
                    std::string str = std::string(wstr.begin(), wstr.end());
                    if (str != "See3CAM_Stereo") {
                        std::string device_path_serial = get_device_path_serial(pMoniker);
                        connected_serials.push_back(device_path_serial);
                        connected_camera_names.push_back(str);
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

    for (auto& known_serial_info : known_serial_infos) {
        bool left_found = false;
        bool right_found = false;
        std::string left_serial;
        std::string right_serial;
        //find left
        for (auto& connected_serial : connected_serials)
        {
            left_serial = connected_serial;
            if (left_serial == known_serial_info.left_camera_serial){
                left_found = true;
                break;
            }
        }
        if (left_found){
            //find right
            for (auto& connected_serial : connected_serials)
            {
                right_serial = connected_serial;
                if (right_serial == known_serial_info.right_camera_serial){
                    right_found = true;
                    break;
                }
            }
        }
        if (left_found && right_found){ //only add if both cameras found
            connected_serial_infos.push_back(known_serial_info);
        }
    }

    return connected_serial_infos;
}

void StereoCameraOpenCV::openHID(void) {
    hid_init();

    cam_device_l = hid_open(0x2560, 0xC114, NULL);
    cam_device_r = hid_open(0x2560, 0xC114, NULL);

    if (cam_device_l == NULL) {
        qDebug() << "Couldn't open device";
    } else {
        hid_set_nonblocking(cam_device_l, 1);
    }
    if (cam_device_r == NULL) {
        qDebug() << "Couldn't open device";
    } else {
        hid_set_nonblocking(cam_device_r, 1);
    }

    hid_exit();
}

qint64 StereoCameraOpenCV::getSerial() {
    qint64 serial = 1;

    return serial;
}

int StereoCameraOpenCV::usb_index_from_serial(std::string serial){
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

                    if (std::string(str.begin(), str.end()) != "See3CAM_Stereo") {
                        qDebug() << "Found USB at device " << i;

                        hr = pPropBag->Read(L"DevicePath", &var, 0);

                        BSTR device_path = var.bstrVal;
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

std::string StereoCameraOpenCV::serial_from_usb_index(int index){
    /// Create the System Device Enumerator.
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

                        if (std::string(str.begin(), str.end()) != "See3CAM_Stereo") {
                            qDebug() << "Found USB at device " << i;

                            hr = pPropBag->Read(L"DevicePath", &var, 0);

                            BSTR device_path = var.bstrVal;
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

std::string StereoCameraOpenCV::serial_from_device_path(std::string usb_device_path){
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
        qDebug() << "USB serial: " << camera_serial.c_str();
    } else {
        qDebug() << "USB device path formatted incorrectly: " << usb_device_path.c_str();
        for (std::vector<std::string>::iterator it = device_path_delim.begin() ; it != device_path_delim.end(); ++it)
            qDebug() << (*it).c_str();
        qDebug() << "Will use full device path as camera serial";
        camera_serial = usb_device_path;
    }
    return camera_serial;
}

int StereoCameraOpenCV::getExposure() {
   return 0;
}

bool StereoCameraOpenCV::enableAutoExposure(bool enable) {
    if (enable)
        return setExposure(0.001);
    else
        return setExposure(exposure / 1000);
}

bool StereoCameraOpenCV::setExposure(double exposure_milliseconds) {
    //TODO set exposure
    return false;
}

bool StereoCameraOpenCV::setFPS(int fps){
    if (!isCapturing()){
        double fps_d = fps;
        camera_l.set(cv::CAP_PROP_FPS, (double)fps_d);
        camera_r.set(cv::CAP_PROP_FPS, (double)fps_d);
        frame_rate = fps;
        return true;
    } else {
        qDebug() << "Cannot set FPS while capturing. Stop capturing and try again.";
        return false;
    }
}

bool StereoCameraOpenCV::enableHDR(bool enable){
    //TODO set hdr
    return false;
}

bool StereoCameraOpenCV::send_hid(hid_device* cam_device, std::vector<unsigned char> &buffer,
                                  size_t command_len) {
    if (cam_device == NULL) return false;

    std::vector<unsigned char> commands;
    for (int i = 0; i < command_len; i++) {
        commands.push_back(buffer[i + 1]);
    }

    int result = hid_write(cam_device, buffer.data(), buffer.size());

    if (result < 0) {
        qDebug() << "Failed to write HID data";
        return false;
    }

    result = hid_read_timeout(cam_device, buffer.data(), buffer.size(), 500);

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

bool StereoCameraOpenCV::setFrameSize(int width, int height) {
    bool res = false;

    res = camera_l.set(cv::CAP_PROP_FRAME_WIDTH, width);
    res &= camera_l.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    res &= camera_r.set(cv::CAP_PROP_FRAME_WIDTH, width);
    res &= camera_r.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    image_width = width;
    image_height = height;
    image_bitdepth = 1; //TODO get bit depth
    emit update_size(image_width, image_height, image_bitdepth);

    return res;
}

bool StereoCameraOpenCV::setFrame16(void) {
    bool res = false;
    res = camera_l.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));
    res &= camera_r.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));

    return res;
}

void StereoCameraOpenCV::getFrameRate() {
    frame_rate = (int)camera_l.get(cv::CAP_PROP_FPS);
    //TODO check frame rate is the same in both cameras
    qDebug() << "Frame rate: " << frame_rate;
}

std::string StereoCameraOpenCV::wchar_to_string(WCHAR * buffer) {
    std::wstring ws(buffer);
    std::string buffer_s(ws.begin(), ws.end());
    return buffer_s;
}

StereoCameraOpenCV::~StereoCameraOpenCV(void) {
    if (connected){
        closeCamera();
    }
}
