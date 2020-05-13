#pragma once

#include <Unknwnbase.h>

MIDL_INTERFACE("FA23BE93-4A87-456A-9627-1494DD20E8B3")
IRenderer : public IUnknown
{
public:

	enum RenderTargetType
	{
		Texture,
		SwapChain
	};

	IRenderer()
	{
	}

	virtual ~IRenderer()
	{
	}

	virtual HRESULT getRenderTarget(RenderTargetType aRenderTargetType,  IUnknown** a_pRenderTarget) = 0;

	virtual HRESULT presents() = 0;

	virtual HRESULT shutdown() = 0;  
};

