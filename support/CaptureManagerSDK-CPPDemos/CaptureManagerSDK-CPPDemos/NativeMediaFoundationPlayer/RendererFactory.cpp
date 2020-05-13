#include "RendererFactory.h"
#include "Direct3D9Renderer.h"
#include "../Common/ComPtrCustom.h"


RendererFactory::RendererFactory()
{
}

RendererFactory::~RendererFactory()
{
}

HRESULT RendererFactory::createRenderer(RendererType aRendererType, HWND aWindowHandler, IRenderer** aPtrPtrIRenderer)
{
	HRESULT lresult(E_FAIL);

	do
	{
		if (aRendererType == RendererType::Directx3D9)
		{
			CComPtrCustom<Direct3D9Renderer> lDirect3D9Renderer = new Direct3D9Renderer();

			lDirect3D9Renderer->init(aWindowHandler);

			lDirect3D9Renderer->QueryInterface(IID_PPV_ARGS(aPtrPtrIRenderer));
		}

	} while (false);

	return lresult;
}