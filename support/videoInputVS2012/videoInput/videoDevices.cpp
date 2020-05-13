#include <Mfobjects.h>
#include <Mfidl.h>


#include "videoDevices.h"
#include "videoDevice.h"
#include "DebugPrintOut.h"
#include "Common.h"

videoDevices::videoDevices(void): count(0)
{
}

void videoDevices::clearDevices()
{
	std::vector<videoDevice *>::iterator i = vds_Devices.begin();

	for(; i != vds_Devices.end(); ++i)
		delete (*i);

	vds_Devices.clear();
}

videoDevices::~videoDevices(void)
{	
	clearDevices();
}

videoDevice * videoDevices::getDevice(unsigned int i)
{
	if(i >= vds_Devices.size())
	{
		return NULL;
	}

	if(i < 0)
	{
		return NULL;
	}

	return vds_Devices[i];
}

long videoDevices::initDevices(IMFAttributes *pAttributes)
{
	HRESULT hr = S_OK;
		
	IMFActivate **ppDevices = NULL;

	clearDevices();
	
	hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);

	if (SUCCEEDED(hr))
    {
        if(count > 0)
		{
			for(UINT32 i = 0; i < count; i++)
			{
				videoDevice *vd = new videoDevice;

				vd->readInfoOfDevice(ppDevices[i], i);

				vds_Devices.push_back(vd);	
				
				SafeRelease(&ppDevices[i]);
			}	

			SafeReleaseAllCount(ppDevices);
		}
		else
			hr = -1;
    }
	else
	{
		DebugPrintOut *DPO = &DebugPrintOut::getInstance();

		DPO->printOut(L"VIDEODEVICES: The instances of the videoDevice class cannot be created\n");
	}

	return hr;
}

int videoDevices::getCount()
{
	return vds_Devices.size();
}

videoDevices& videoDevices::getInstance() 
{
	static videoDevices instance;

	return instance;
}