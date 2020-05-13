#include "videoInput.h"
#include "Media_Foundation.h"
#include "videoDevices.h"
#include "videoDevice.h"
#include "DebugPrintOut.h"
#include "RawImage.h"

template <class T> void SafeRelease(T *ppT)
{
    if (ppT)
    {
        delete (ppT);
        ppT = NULL;
    }
}

Parametr::Parametr()
{
	CurrentValue = 0;

	Min = 0;
	
	Max = 0;
	
	Step = 0;
	
	Default = 0; 
	
	Flag = 0;
}

MediaType::MediaType()
{
	pMF_MT_AM_FORMAT_TYPEName = NULL;

	pMF_MT_MAJOR_TYPEName = NULL;

	pMF_MT_SUBTYPEName = NULL;

	Clear();
}

MediaType::~MediaType()
{
	Clear();
}

void MediaType::Clear()
{

	MF_MT_FRAME_SIZE = 0;

	height = 0;

	width = 0;
			
	MF_MT_YUV_MATRIX = 0;
	
	MF_MT_VIDEO_LIGHTING = 0;
	
	MF_MT_DEFAULT_STRIDE = 0;
	
	MF_MT_VIDEO_CHROMA_SITING = 0;
		
	MF_MT_FIXED_SIZE_SAMPLES = 0;
	
	MF_MT_VIDEO_NOMINAL_RANGE = 0;
	
	MF_MT_FRAME_RATE = 0;

	MF_MT_FRAME_RATE_low = 0;
	
	MF_MT_PIXEL_ASPECT_RATIO = 0;
		
	MF_MT_PIXEL_ASPECT_RATIO_low = 0;
	
	MF_MT_ALL_SAMPLES_INDEPENDENT = 0;
	
	MF_MT_FRAME_RATE_RANGE_MIN = 0;

	MF_MT_FRAME_RATE_RANGE_MIN_low = 0;
	
	MF_MT_SAMPLE_SIZE = 0;
	
	MF_MT_VIDEO_PRIMARIES = 0;
	
	MF_MT_INTERLACE_MODE = 0;
	
	MF_MT_FRAME_RATE_RANGE_MAX = 0;

	MF_MT_FRAME_RATE_RANGE_MAX_low = 0;
				
	memset(&MF_MT_MAJOR_TYPE, 0, sizeof(GUID));
		
	memset(&MF_MT_AM_FORMAT_TYPE, 0, sizeof(GUID));
			
	memset(&MF_MT_SUBTYPE, 0, sizeof(GUID));
}

videoInput::videoInput(void): accessToDevices(false)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	DPO->printOut(L"\n***** VIDEOINPUT LIBRARY - 2013 (Author: Evgeny Pereguda) *****\n\n");

	updateListOfDevices();
	
	if(!accessToDevices)
		DPO->printOut(L"INITIALIZATION: Ther is not any suitable video device\n");
}

void videoInput::updateListOfDevices()
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();
	
	Media_Foundation *MF = &Media_Foundation::getInstance();

	accessToDevices = MF->buildListOfDevices();

	if(!accessToDevices)
		DPO->printOut(L"UPDATING: Ther is not any suitable video device\n");
}

videoInput::~videoInput(void)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	DPO->printOut(L"\n***** CLOSE VIDEOINPUT LIBRARY - 2013 *****\n\n");
}

IMFMediaSource *videoInput::getMediaSource(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
		{
			IMFMediaSource *out = VD->getMediaSource();

			if(!out)
				DPO->printOut(L"VideoDevice %i: There is not any suitable IMFMediaSource interface\n", deviceID);

			return out;
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return NULL;
}

bool videoInput::setupDevice(unsigned int deviceID, unsigned int id)
{
	
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
		{
			bool out = VD->setupDevice(id);

			if(!out)
				DPO->printOut(L"VIDEODEVICE %i: This device cannot be started\n", deviceID);

			return out;
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return false;
}

bool videoInput::setupDevice(unsigned int deviceID, unsigned int w, unsigned int h, unsigned int idealFramerate)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
		{
			bool out = VD->setupDevice(w, h, idealFramerate);

			if(!out)
				DPO->printOut(L"VIDEODEVICE %i: this device cannot be started\n", deviceID);

			return out;
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n", deviceID);
	}

	return false;
}

MediaType videoInput::getFormat(unsigned int deviceID, unsigned int id)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)		
			return VD->getFormat(id);
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return MediaType();
}

bool videoInput::isDeviceSetup(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->isDeviceSetup();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return false;
}

bool videoInput::isDeviceMediaSource(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->isDeviceMediaSource();
	}
	else
	{
		DPO->printOut(L"Device(s): There is not any suitable video device\n");
	}

	return false;
}

bool videoInput::isDeviceRawDataSource(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->isDeviceRawDataSource();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return false;
}

bool videoInput::isFrameNew(unsigned int deviceID)
{	
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		if(!isDeviceSetup(deviceID))
		{
			if(isDeviceMediaSource(deviceID))
				return false;
		}

		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->isFrameNew();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return false;
}

unsigned int videoInput::getCountFormats(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->getCountFormats();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return 0;
}

void videoInput::closeAllDevices()
{
	videoDevices *VDS = &videoDevices::getInstance();

	for(unsigned int i = 0; i < VDS->getCount(); i++)
		closeDevice(i);
}

void videoInput::setParametrs(unsigned int deviceID, CamParametrs parametrs)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();
	
	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice *VD = VDS->getDevice(deviceID);

		if(VD)
			VD->setParametrs(parametrs);
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
}

CamParametrs videoInput::getParametrs(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	CamParametrs out;

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice *VD = VDS->getDevice(deviceID);

		if(VD)
			out = VD->getParametrs();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
	
	return out;
}

void videoInput::closeDevice(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice *VD = VDS->getDevice(deviceID);

		if(VD)
			VD->closeDevice();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
}

unsigned int videoInput::getWidth(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)	
			return VD->getWidth();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return 0;
}

unsigned int videoInput::getHeight(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->getHeight();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return 0;
}

wchar_t *videoInput::getNameVideoDevice(unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();
	
		videoDevice * VD = VDS->getDevice(deviceID);
		
		if(VD)
			return VD->getName();
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return L"Empty";
}

unsigned int videoInput::listDevices(bool silent)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	int out = 0;

	if(accessToDevices)
	{
		videoDevices *VDS = &videoDevices::getInstance();

		out = VDS->getCount();

		DebugPrintOut *DPO = &DebugPrintOut::getInstance();

		if(!silent)DPO->printOut(L"\nVIDEOINPUT SPY MODE!\n\n");

		if(!silent)DPO->printOut(L"SETUP: Looking For Capture Devices\n");

		for(int i = 0; i < out; i++)
		{
			if(!silent)DPO->printOut(L"SETUP: %i) %s \n",i, getNameVideoDevice(i));
		}

		if(!silent)DPO->printOut(L"SETUP: %i Device(s) found\n\n", out);

	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}

	return out;
}

videoInput& videoInput::getInstance() 
{
	static videoInput instance;

	return instance;
}

bool videoInput::isDevicesAcceable()
{
	return accessToDevices;
}

void videoInput::setVerbose(bool state)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	return DPO->setVerbose(state);	
}

void videoInput::setEmergencyStopEvent(unsigned int deviceID, void *userData, void(*func)(int, void *))
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		if(func)
		{
			videoDevices *VDS = &videoDevices::getInstance();

			videoDevice * VD = VDS->getDevice(deviceID);
		
			if(VD)	
				VD->setEmergencyStopEvent(userData, func);
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
}

bool videoInput::getPixels(unsigned int deviceID, unsigned char * dstBuffer, bool flipRedAndBlue, bool flipImage)
{
	bool success = false;

	unsigned int bytes = 3;

	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(accessToDevices)
	{
		if(isDeviceRawDataSource(deviceID))
		{
			videoDevices *VDS = &videoDevices::getInstance();

			DebugPrintOut *DPO = &DebugPrintOut::getInstance();

			RawImage *RIOut = VDS->getDevice(deviceID)->getRawImageOut();
	
			if(RIOut)
			{
				unsigned int height	= VDS->getDevice(deviceID)->getHeight();
				unsigned int width	= VDS->getDevice(deviceID)->getWidth(); 

				unsigned int size = bytes * width * height;

				if(size == RIOut->getSize())
				{										
					processPixels(RIOut->getpPixels(), dstBuffer, width, height, bytes, flipRedAndBlue, flipImage);

					success = true;
				}
				else
				{
					DPO->printOut(L"ERROR: GetPixels() - bufferSizes do not match!\n");
				}
			}
			else
			{
				DPO->printOut(L"ERROR: GetPixels() - Unable to grab frame for device %i\n", deviceID);
			}				
		
			
		}
	}
	else
	{
		DPO->printOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
		
	return success;
}

void videoInput::processPixels(unsigned char * src, unsigned char * dst, unsigned int width, unsigned int height, unsigned int bpp, bool bRGB, bool bFlip)
{
	
	unsigned int widthInBytes = width * bpp;

	unsigned int numBytes = widthInBytes * height;

	unsigned int numInts = numBytes >> 2;

	unsigned int widthInInts = widthInBytes >> 2;

	int *dstInt, *srcInt;
	
	if(!bRGB)
	{
		
		int x = 0;
		int y = 0;
	
		if(bFlip)
		{
			for(int y = 0; y < height; y++)
			{
				dstInt = (int *)(dst + (y * widthInBytes));
				
				srcInt = (int *)(src + ( (height -y -1) * widthInBytes));
				
				_asm
				{
					mov ESI, srcInt

					mov EDI, dstInt

					mov ECX, widthInInts

					cld

					rep movsd
				}
			}
									
		}
		else
		{		
			_asm
			{
				mov ESI, src

				mov EDI, dst

				mov ECX, numInts

				cld

				rep movsd
			}
		}
	}
	else
	{
		if(bFlip)
		{
			
			int x = 0;
			int y = (height - 1) * widthInBytes;
			src += y;
			
			for(unsigned int i = 0; i < numBytes; i+=3)
			{
				if(x >= width)
				{
					x = 0;
					src -= widthInBytes*2;
				}
				
				*dst = *(src+2);
				dst++;
				
				*dst = *(src+1);
				dst++; 
				
				*dst = *src;
				dst++; 
				
				src+=3;	
				x++;		
			}
		}
		else
		{						
			for(unsigned int i = 0; i < numBytes; i+=3)
			{
				*dst = *(src+2);
				dst++;
				
				*dst = *(src+1);
				dst++; 
				
				*dst = *src;
				dst++; 
				
				src+=3;			
			}
		}
	}

}

