#pragma once

#include <basetsd.h>
#include <vector>
#include <memory>

struct IMFAttributes;

class videoDevice;


/// Class for managing of list of video devices
class videoDevices
{
public:
	~videoDevices(void);

	long initDevices(IMFAttributes *pAttributes);

	static videoDevices& getInstance();

	videoDevice *getDevice(unsigned int i);

	int getCount();
			
	void clearDevices();

private:
			
    UINT32 count;

	std::vector<videoDevice *> vds_Devices;
		
	videoDevices(void);
};

