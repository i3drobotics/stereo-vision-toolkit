#pragma once

struct IUnknown;

class CaptureManagerVideoRendererFactory
{
	int mloaderIndex = 0;

public:

	static CaptureManagerVideoRendererFactory* getInstance();

	void selectLoader(int aloaderIndex);

	long getMaxVideoRenderStreamCount(unsigned long* aPtrCount);

	long getRenderer(void* aPtrHandler, IUnknown** aPtrPtrUnknown);

	long getRenderer(IUnknown* aPtrRenderTarget, IUnknown** aPtrPtrUnknown);

	long getEVRControl(IUnknown** aPtrPtrUnknown);

private:
	CaptureManagerVideoRendererFactory();
	~CaptureManagerVideoRendererFactory();
};

