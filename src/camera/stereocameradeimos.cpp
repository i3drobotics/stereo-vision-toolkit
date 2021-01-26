/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#include "stereocameradeimos.h"

std::vector<AbstractStereoCamera::StereoCameraSerialInfo> StereoCameraDeimos::listSystems(){
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> known_serial_infos = loadSerials(CAMERA_TYPE_DEIMOS);
    std::vector<AbstractStereoCamera::StereoCameraSerialInfo> connected_serial_infos;

    std::string friendly_name = "See3CAM_Stereo";

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
                    if (std::string(str.begin(), str.end()) == friendly_name) {
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
                        if (device_known){
                            connected_serial_infos.push_back(serial_info);
                        }
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

StereoCameraDeimos::~StereoCameraDeimos(void) {
}
