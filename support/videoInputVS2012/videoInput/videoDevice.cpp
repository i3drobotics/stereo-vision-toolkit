#include <mfobjects.h>
#include <mfidl.h>
#include <mfapi.h>
#include <string.h>

#include <Strmif.h>


#include "videoDevice.h"
#include "FormatReader.h"
#include "Common.h"
#include "DebugPrintOut.h"
#include "ImageGrabberThread.h"
#include "ImageGrabber.h"
#include "RawImage.h"


#pragma comment(lib, "Strmiids")
videoDevice::videoDevice(void): vd_IsSetuped(false), vd_LockOut(OpenLock), vd_pFriendlyName(NULL),
	vd_Width(0), vd_Height(0), vd_pSource(NULL), vd_func(NULL), vd_userData(NULL)
{	

}

void videoDevice::setParametrs(CamParametrs parametrs)
{
	if(vd_IsSetuped)
	{
		if(vd_pSource)
		{
			unsigned int shift = sizeof(Parametr);

			Parametr *pParametr = (Parametr *)(&parametrs);

			Parametr *pPrevParametr = (Parametr *)(&vd_PrevParametrs);

			IAMVideoProcAmp *pProcAmp = NULL;
			HRESULT hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

			if (SUCCEEDED(hr))
			{
				for(unsigned int i = 0; i < 10; i++)
				{
					if(pPrevParametr[i].CurrentValue != pParametr[i].CurrentValue || pPrevParametr[i].Flag != pParametr[i].Flag)
						hr = pProcAmp->Set(VideoProcAmp_Brightness + i, pParametr[i].CurrentValue, pParametr[i].Flag);
					
				}

				pProcAmp->Release();
			}

			IAMCameraControl *pProcControl = NULL;
			hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

			if (SUCCEEDED(hr))
			{
				for(unsigned int i = 0; i < 7; i++)
				{
					if(pPrevParametr[10 + i].CurrentValue != pParametr[10 + i].CurrentValue || pPrevParametr[10 + i].Flag != pParametr[10 + i].Flag)
					hr = pProcControl->Set(CameraControl_Pan+i, pParametr[10 + i].CurrentValue, pParametr[10 + i].Flag);					
				}

				pProcControl->Release();
			}

			vd_PrevParametrs = parametrs;
		}
	}
}

CamParametrs videoDevice::getParametrs()
{
	CamParametrs out;

	if(vd_IsSetuped)
	{
		if(vd_pSource)
		{
			unsigned int shift = sizeof(Parametr);

			Parametr *pParametr = (Parametr *)(&out);

			IAMVideoProcAmp *pProcAmp = NULL;
			HRESULT hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

			if (SUCCEEDED(hr))
			{
				for(unsigned int i = 0; i < 10; i++)
				{
					Parametr temp;
					
					hr = pProcAmp->GetRange(VideoProcAmp_Brightness+i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);
										
					if (SUCCEEDED(hr))
					{
						long currentValue = temp.Default;

						hr = pProcAmp->Get(VideoProcAmp_Brightness+i, &currentValue, &temp.Flag);

						temp.CurrentValue = currentValue;

						pParametr[i] = temp;
					}
				}

				pProcAmp->Release();
			}

			IAMCameraControl *pProcControl = NULL;
			hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

			if (SUCCEEDED(hr))
			{
				for(unsigned int i = 0; i < 7; i++)
				{
					Parametr temp;
					
					hr = pProcControl->GetRange(CameraControl_Pan+i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);
										
					if (SUCCEEDED(hr))
					{
						long currentValue = temp.Default;

						hr = pProcAmp->Get(CameraControl_Pan+i, &currentValue, &temp.Flag);

						temp.CurrentValue = currentValue;
						
						pParametr[10 + i] = temp;
					}
				}

				pProcControl->Release();
			}
		}
	}

	return out;
}

long videoDevice::resetDevice(IMFActivate *pActivate)
{
	HRESULT hr = -1;

	vd_CurrentFormats.clear();

	if(vd_pFriendlyName)
		CoTaskMemFree(vd_pFriendlyName);

	vd_pFriendlyName = NULL;
	
	if(pActivate)
	{		
		IMFMediaSource *pSource = NULL;

		hr = pActivate->GetAllocatedString(
				MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
				&vd_pFriendlyName,
				NULL
				);

		
		hr = pActivate->ActivateObject(
            __uuidof(IMFMediaSource),
            (void**)&pSource
            );

		enumerateCaptureFormats(pSource);

		buildLibraryofTypes();



		SafeRelease(&pSource);

		
	
		if(FAILED(hr))	
		{			
			vd_pFriendlyName = NULL;

			DebugPrintOut *DPO = &DebugPrintOut::getInstance();

			DPO->printOut(L"VIDEODEVICE %i: IMFMediaSource interface cannot be created \n", vd_CurrentNumber);
		}
	}

	return hr;
}

long videoDevice::readInfoOfDevice(IMFActivate *pActivate, unsigned int Num)
{
	HRESULT hr = -1;

	vd_CurrentNumber = Num;

	hr = resetDevice(pActivate);

	return hr;
}

long videoDevice::checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice)
{
	HRESULT hr = S_OK;
		
	IMFActivate **ppDevices = NULL;
		
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	UINT32 count;

	wchar_t *newFriendlyName = NULL;
		
	hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);

	if (SUCCEEDED(hr))
    {
        if(count > 0)
		{
			if(count > vd_CurrentNumber)
			{			
				hr = ppDevices[vd_CurrentNumber]->GetAllocatedString(
				MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
				&newFriendlyName,
				NULL
				);

				if (SUCCEEDED(hr))
				{
					if(wcscmp(newFriendlyName, vd_pFriendlyName) != 0)
					{
						DPO->printOut(L"VIDEODEVICE %i: Chosen device cannot be found \n", vd_CurrentNumber);

						hr = -1;

						pDevice = NULL;
					}
					else
					{
						*pDevice = ppDevices[vd_CurrentNumber];

						(*pDevice)->AddRef();
					}
				}
				else
				{
					DPO->printOut(L"VIDEODEVICE %i: Name of device cannot be gotten \n", vd_CurrentNumber);
				}

			}
			else
			{
				DPO->printOut(L"VIDEODEVICE %i: Number of devices more than corrent number of the device \n", vd_CurrentNumber);

				hr = -1;
			}

			for(UINT32 i = 0; i < count; i++)
			{
				SafeRelease(&ppDevices[i]);
			}

			SafeReleaseAllCount(ppDevices);
		}
		else
			hr = -1;
    }
	else
	{
		DPO->printOut(L"VIDEODEVICE %i: List of DeviceSources cannot be enumerated \n", vd_CurrentNumber);
	}

	return hr;
}

long videoDevice::initDevice()
{
	HRESULT hr = -1;

	IMFAttributes *pAttributes = NULL;

	IMFActivate * vd_pActivate= NULL;
		
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

    CoInitialize(NULL);
	
    hr = MFCreateAttributes(&pAttributes, 1);
   
	if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
            );
    }	
	
	if (SUCCEEDED(hr))
    {
		hr = checkDevice(pAttributes, &vd_pActivate);

		if (SUCCEEDED(hr) && vd_pActivate)
		{
			SafeReleaseAllCount(&vd_pSource);
			
			hr = vd_pActivate->ActivateObject(
				__uuidof(IMFMediaSource),
				(void**)&vd_pSource
				);

			if (SUCCEEDED(hr))
			{

			}

			SafeReleaseAllCount(&vd_pActivate);
		}
		else
		{
			DPO->printOut(L"VIDEODEVICE %i: Device there is not \n", vd_CurrentNumber);
		}
    }	
	else
	{

		DPO->printOut(L"VIDEODEVICE %i: The attribute of video cameras cannot be getting \n", vd_CurrentNumber);
	
	}

	SafeReleaseAllCount(&pAttributes);

	return hr;
}

MediaType videoDevice::getFormat(unsigned int id)
{
	if(id < vd_CurrentFormats.size())
	{
		return vd_CurrentFormats[id];
	}
	else return MediaType();

}

int videoDevice::getCountFormats()
{
	return vd_CurrentFormats.size();
}

void videoDevice::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
	vd_func = func;

	vd_userData = userData;
}

void videoDevice::closeDevice()
{		
	if(vd_IsSetuped)
	{
		vd_IsSetuped = false;
		
		vd_pSource->Stop();

		if(vd_LockOut == RawDataLock)
		{
			vd_pImGrTh->stop();

			WaitForSingleObject(vd_pImGrTh->getMutexHandle(), 5000);
						
			vd_pImGrTh.reset(0);		

			SafeReleaseAllCount(&vd_pSource);
		}

		SafeRelease(&vd_pSource);
				
		vd_LockOut = OpenLock;	
				
		DebugPrintOut *DPO = &DebugPrintOut::getInstance();
		DPO->printOut(L"VIDEODEVICE %i: Device is stopped \n", vd_CurrentNumber);
	}
}

unsigned int videoDevice::getWidth()
{
	if(vd_IsSetuped)
		return vd_Width;
	else
		return 0;
}
	
unsigned int videoDevice::getHeight()
{
	if(vd_IsSetuped)
		return vd_Height;
	else 
		return 0;
}

IMFMediaSource *videoDevice::getMediaSource()
{
	IMFMediaSource *out = NULL;

	if(vd_LockOut == OpenLock)
	{
		vd_LockOut = MediaSourceLock;			

		out = vd_pSource;
	}

	return out;
}

int videoDevice::findType(unsigned int size, unsigned int frameRate)
{	
	if(vd_CaptureFormats.size() == 0)
		return 0;

	FrameRateMap FRM = vd_CaptureFormats[size];

	if(FRM.size() == 0)
		return 0;

	unsigned int frameRateMax = 0;  SUBTYPEMap STMMax;

	if(frameRate == 0)
	{
		std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();

		for(; f != FRM.end(); f++)
		{
			 if((*f).first >= frameRateMax)
			 {
				 frameRateMax = (*f).first;

				 STMMax = (*f).second;
			 }
		}		

	}
	else
	{
		std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();
		
		for(; f != FRM.end(); f++)
		{
			 if((*f).first >= frameRateMax)
			 {
				 if(frameRate > (*f).first)
				 {
					 frameRateMax = (*f).first;

					 STMMax = (*f).second;
				 }
			 }
		}
	}

	if(STMMax.size() == 0)
		return 0;


	std::map<String, vectorNum>::iterator S = STMMax.begin();

	vectorNum VN = (*S).second;

	if(VN.size() == 0)
		return 0;

	return VN[0];

}

void videoDevice::buildLibraryofTypes()
{
	unsigned int size;

	unsigned int framerate;

	std::vector<MediaType>::iterator i = vd_CurrentFormats.begin();
		
	int count = 0;

	for(; i != vd_CurrentFormats.end(); i++)
	{
		size = (*i).MF_MT_FRAME_SIZE;

		framerate = (*i).MF_MT_FRAME_RATE;
		
		FrameRateMap FRM = vd_CaptureFormats[size];

		SUBTYPEMap STM = FRM[framerate];

		String subType((*i).pMF_MT_SUBTYPEName);

		vectorNum VN = STM[subType];

		VN.push_back(count);

		STM[subType] = VN;

		FRM[framerate] = STM;

		vd_CaptureFormats[size] = FRM;

		count++;
	}
}

long videoDevice::setDeviceFormat(IMFMediaSource *pSource, unsigned long  dwFormatIndex)
{
    IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFMediaType *pType = NULL;

    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto done;
    }

    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr))
    {
        goto done;
    }

	hr = pHandler->GetMediaTypeByIndex((DWORD)dwFormatIndex, &pType);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pHandler->SetCurrentMediaType(pType);

done:
    SafeReleaseAllCount(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeRelease(&pType);
    return hr;
}

bool videoDevice::isDeviceSetup()
{
	return vd_IsSetuped;
}

RawImage * videoDevice::getRawImageOut()
{
	if(!vd_IsSetuped) return NULL;

	if(vd_pImGrTh.get())
			return vd_pImGrTh->getImageGrabber()->getRawImage();	
	else
	{
		DebugPrintOut *DPO = &DebugPrintOut::getInstance();

		DPO->printOut(L"VIDEODEVICE %i: The instance of ImageGrabberThread class does not exist  \n", vd_CurrentNumber);
	}
	return NULL;
}

bool videoDevice::isFrameNew()
{
	if(!vd_IsSetuped) return false;

	if(vd_LockOut == RawDataLock || vd_LockOut == OpenLock) 
	{
		if(vd_LockOut == OpenLock)
		{
			vd_LockOut = RawDataLock;

			ImageGrabberThread *temp;
			
			HRESULT hr = ImageGrabberThread::CreateInstance(&temp, vd_pSource, vd_CurrentNumber);
			
			vd_pImGrTh.reset(temp);
						
			if(FAILED(hr))
			{
				DebugPrintOut *DPO = &DebugPrintOut::getInstance();

				DPO->printOut(L"VIDEODEVICE %i: The instance of ImageGrabberThread class cannot be created.\n", vd_CurrentNumber);

				return false;
			}

			vd_pImGrTh->setEmergencyStopEvent(vd_userData, vd_func);

			vd_pImGrTh->start();

			return true;
		}

		if(vd_pImGrTh.get())
			return vd_pImGrTh->getImageGrabber()->getRawImage()->isNew();		

	}

	return false;
}

bool videoDevice::isDeviceMediaSource()
{
	if(vd_LockOut == MediaSourceLock) return true;

	return false;
}

bool videoDevice::isDeviceRawDataSource()
{
	if(vd_LockOut == RawDataLock) return true;

	return false;
}

bool videoDevice::setupDevice(unsigned int id)
{	
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(!vd_IsSetuped)
	{
		HRESULT hr = -1;

		hr = initDevice();

		if(SUCCEEDED(hr))
		{			
			vd_Width = vd_CurrentFormats[id].width; 

			vd_Height = vd_CurrentFormats[id].height;

			hr = setDeviceFormat(vd_pSource, (DWORD) id);

			vd_IsSetuped = (SUCCEEDED(hr));

			if(vd_IsSetuped)
				DPO->printOut(L"\n\nVIDEODEVICE %i: Device is setuped \n", vd_CurrentNumber);

			vd_PrevParametrs = getParametrs();

			return vd_IsSetuped;
		}
		else
		{
			DPO->printOut(L"VIDEODEVICE %i: Interface IMFMediaSource cannot be got \n", vd_CurrentNumber);

			return false;
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE %i: Device is setuped already \n", vd_CurrentNumber);

		return false;
	}	
}

bool videoDevice::setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate)
{	
	unsigned int id = findType(w * h, idealFramerate);

	return setupDevice(id);
}

wchar_t *videoDevice::getName()
{
	return vd_pFriendlyName;
}

videoDevice::~videoDevice(void)
{		
	closeDevice();

	SafeReleaseAllCount(&vd_pSource);
	
	if(vd_pFriendlyName)
		CoTaskMemFree(vd_pFriendlyName);
}

long videoDevice::enumerateCaptureFormats(IMFMediaSource *pSource)
{
	IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFMediaType *pType = NULL;

    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto done;
    }

    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr))
    {
        goto done;
    }

    DWORD cTypes = 0;
    hr = pHandler->GetMediaTypeCount(&cTypes);
    if (FAILED(hr))
    {
        goto done;
    }

    for (DWORD i = 0; i < cTypes; i++)
    {
        hr = pHandler->GetMediaTypeByIndex(i, &pType);

        if (FAILED(hr))
        {
            goto done;
        }
		
		MediaType MT = FormatReader::Read(pType);

		vd_CurrentFormats.push_back(MT);
		
        SafeReleaseAllCount(&pType);
    }

done:
    SafeReleaseAllCount(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeReleaseAllCount(&pType);
    return hr;
}
