#pragma once
#include <Unknwnbase.h>
#include <vector>

#include "../Common/ComPtrCustom.h"

class CaptureManagerLoader
{
public:

	static CaptureManagerLoader& getInstance();

	HRESULT createRendererOutputNodes(
		void * aHandle,
		IUnknown *aPtrUnkTarget,
		unsigned long aOutputNodeAmount,
		std::vector<CComPtrCustom<IUnknown>>& aOutputNodes);

	HRESULT getMaxVideoRenderStreamCount(
		unsigned long* aPtrOutputNodeAmount);

	HRESULT createEVRStreamControl(
		IUnknown** aPtrPtrUnkIEVRStreamControl);

private:

	CaptureManagerLoader();

	~CaptureManagerLoader();

	HRESULT init();

	unsigned int mMaxPorts;

	IUnknown* m_pUnkEVRMultiSinkFactory;

	IUnknown* m_pUnkStreamControl;

	CaptureManagerLoader(const CaptureManagerLoader&) = delete;
	CaptureManagerLoader(CaptureManagerLoader&) = delete;
};
