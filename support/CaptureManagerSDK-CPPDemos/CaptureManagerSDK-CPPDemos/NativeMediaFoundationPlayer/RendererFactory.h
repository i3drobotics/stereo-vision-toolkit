#pragma once

#include "IRenderer.h"

class RendererFactory
{
public:

	enum RendererType
	{
		Directx3D9
	};

	static HRESULT createRenderer(RendererType aRendererType, HWND aWindowHandler, IRenderer** aPtrPtrIRenderer);

private:
	RendererFactory();
	~RendererFactory();
};

