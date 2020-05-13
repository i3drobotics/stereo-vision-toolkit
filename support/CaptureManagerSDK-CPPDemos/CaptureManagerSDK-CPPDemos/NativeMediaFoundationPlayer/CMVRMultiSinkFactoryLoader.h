#pragma once
#include <Unknwnbase.h>
#include <vector>

#include "../Common/ComPtrCustom.h"

class CCMVRMultiSinkFactoryLoader
{
	void* mHCMVRMultiSinkFactoryLibrary = nullptr;
	
public:

	static CCMVRMultiSinkFactoryLoader& getInstance();

	long createRendererOutputNodes(
		void * aHandle,
		void *aPtrUnkTarget,
		unsigned long aOutputNodeAmount,
		std::vector<CComPtrCustom<IUnknown>>& aOutputNodes);

	long getMaxVideoRenderStreamCount(
		unsigned long* aPtrOutputNodeAmount);

	long createEVRStreamControl(
		IUnknown** aPtrPtrUnkIEVRStreamControl);
	
private:
	CCMVRMultiSinkFactoryLoader();
	~CCMVRMultiSinkFactoryLoader();

	long init();
};

