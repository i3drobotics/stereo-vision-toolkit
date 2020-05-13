#pragma once

#include <map>
#include <vector>
#include <string>
#include <memory>


#include "videoInput.h"

struct IMFActivate;

struct IMFMediaSource;

struct IMFMediaType;

class ImageGrabberThread;

class RawImage;

typedef std::wstring String;

typedef std::vector<int> vectorNum;

typedef std::map<String, vectorNum> SUBTYPEMap;

typedef std::map<UINT64, SUBTYPEMap> FrameRateMap;

typedef void(*emergensyStopEventCallback)(int, void *);

/// Class for controlling of video device
class videoDevice
{

public:
	videoDevice(void);
	~videoDevice(void);

	void closeDevice();
	
	CamParametrs getParametrs();

	void setParametrs(CamParametrs parametrs);

	void setEmergencyStopEvent(void *userData, void(*func)(int, void *));
	
	long readInfoOfDevice(IMFActivate *pActivate, unsigned int Num);
		
	wchar_t *getName();

	int getCountFormats();

	unsigned int getWidth();
	
	unsigned int getHeight();

	MediaType getFormat(unsigned int id);
	
	bool setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate = 0);

	bool setupDevice(unsigned int id);

	bool isDeviceSetup();

	bool isDeviceMediaSource();
	
	bool isDeviceRawDataSource();

	bool isFrameNew();

	IMFMediaSource *getMediaSource();

	RawImage *getRawImageOut();

private:

	enum typeLock
	{
		MediaSourceLock,

		RawDataLock,

		OpenLock

	} vd_LockOut;
	
	wchar_t *vd_pFriendlyName;

	std::auto_ptr<ImageGrabberThread> vd_pImGrTh;

	CamParametrs vd_PrevParametrs;

	unsigned int vd_Width;

	unsigned int vd_Height;

	unsigned int vd_CurrentNumber;

	bool vd_IsSetuped;
		
	std::map<UINT64, FrameRateMap> vd_CaptureFormats;
	
	std::vector<MediaType> vd_CurrentFormats;

	IMFMediaSource *vd_pSource;

	emergensyStopEventCallback vd_func;

	void *vd_userData;
	
	long enumerateCaptureFormats(IMFMediaSource *pSource);
	
	long setDeviceFormat(IMFMediaSource *pSource, unsigned long dwFormatIndex);

	void buildLibraryofTypes();

	int findType(unsigned int size, unsigned int frameRate = 0);	
	
	long resetDevice(IMFActivate *pActivate);
	
	long initDevice();

	long checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice);

};

