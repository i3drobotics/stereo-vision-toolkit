#include "Direct3D9Renderer.h"
#include <d3d9types.h>
#include <d3d9.h>
#include <math.h>


//Transformed vertex with 1 set of texture coordinates
const DWORD tri_fvf = D3DFVF_XYZRHW | D3DFVF_TEX1;

struct textured_vertex{
	float x, y, z, rhw;  // The transformed(screen space) position for the vertex.
	float tu, tv;         // Texture coordinates
};

Direct3D9Renderer::Direct3D9Renderer() :
mRenderTargetType(RenderTargetType::Texture)
{
}


Direct3D9Renderer::~Direct3D9Renderer()
{
}

HRESULT Direct3D9Renderer::init(HWND aWindowHandler)
{
	HRESULT lhresult(E_FAIL);

	do
	{
		list_count = 0;

		CComPtrCustom<IDirect3D9Ex> lD3D9;

		CComPtrCustom<IDirect3DDevice9> lDevice9;

		lhresult = Direct3DCreate9Ex(D3D_SDK_VERSION, &lD3D9);

		if (FAILED(lhresult))
			break;
		
		UINT lAdapter = D3DADAPTER_DEFAULT;

		D3DDISPLAYMODE dm;

		D3DPRESENT_PARAMETERS pp;


		lD3D9->GetAdapterDisplayMode(
			lAdapter,
			&dm);

		ZeroMemory(&pp, sizeof(pp));

		pp.Windowed = TRUE;
		pp.hDeviceWindow = aWindowHandler;
		pp.SwapEffect = D3DSWAPEFFECT_COPY;
		pp.BackBufferFormat = dm.Format;
		pp.BackBufferWidth = 800;
		pp.BackBufferHeight = 600;
		pp.Flags =
			D3DPRESENTFLAG_VIDEO |
			D3DPRESENTFLAG_DEVICECLIP |
			D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
		pp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;
		pp.BackBufferCount = 1;


		lhresult = lD3D9->CreateDevice(
			lAdapter,
			D3DDEVTYPE_HAL,
			aWindowHandler,
			D3DCREATE_HARDWARE_VERTEXPROCESSING |
			D3DCREATE_NOWINDOWCHANGES |
			D3DCREATE_MULTITHREADED |
			D3DCREATE_FPU_PRESERVE,
			&pp,
			&lDevice9);

		if (FAILED(lhresult))
			break;

		CComPtrCustom<IDirect3DSurface9> lSurface;

		CComPtrCustom<IDirect3DTexture9> lTexture;

		lhresult = lDevice9->CreateTexture(	
			800,
			600,
			1,
			D3DUSAGE_RENDERTARGET,
			D3DFMT_A8R8G8B8,
			D3DPOOL_DEFAULT,
			&lTexture,
			nullptr);

		if (FAILED(lhresult))
			break;

		textured_vertex data[] = {

				{ 150, 400, 1, 1, 0, 1 }, { 150, 100, 1, 1, 0, 0 }, { 450, 100, 1, 1, 1, 0 },
				{ 150, 400, 1, 1, 0, 1 }, { 450, 100, 1, 1, 1, 0 }, { 450, 400, 1, 1, 1, 1 }

		};
		int vert_count = sizeof(data) / sizeof(textured_vertex);
		int byte_count = vert_count*sizeof(textured_vertex);
		void *vb_vertices;
		HRESULT hr;

		list_count = vert_count / 3;

		CComPtrCustom<IDirect3DVertexBuffer9> llist_vb;

		hr = lDevice9->CreateVertexBuffer(byte_count,        //Length
			D3DUSAGE_WRITEONLY,//Usage
			tri_fvf,           //FVF
			D3DPOOL_DEFAULT,   //Pool
			&llist_vb,        //ppVertexBuffer
			NULL);             //Handle

		if (FAILED(hr)){
			return hr;
		}

		hr = llist_vb->Lock(0, //Offset
			0, //SizeToLock
			&vb_vertices, //Vertices
			0);  //Flags
		if (FAILED(hr)){
			return hr;
		}

		memcpy(vb_vertices, data, byte_count);

		llist_vb->Unlock();

		llist_vb->QueryInterface(IID_PPV_ARGS(&m_list_vb));
		
		CComPtrCustom<IDirect3DSwapChain9> lSwapChain;

		lDevice9->GetSwapChain(
			0,
			&lSwapChain);

		m_SwapChain = lSwapChain;

		lTexture->GetSurfaceLevel(0, &lSurface);

		m_RenderTexture = lSurface;

		lDevice9->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
		lDevice9->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
		lDevice9->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);   //Ignored		
		lDevice9->SetTexture(0, lTexture);

	} while (false);

	return lhresult;
}

HRESULT Direct3D9Renderer::getRenderTarget(RenderTargetType aRenderTargetType, IUnknown** a_pRenderTarget)
{
	if (m_RenderTexture == nullptr)
		return E_POINTER;

	mRenderTargetType = aRenderTargetType;

	if (aRenderTargetType == RenderTargetType::Texture)
		return m_RenderTexture->QueryInterface(a_pRenderTarget);
	else
		return m_SwapChain->QueryInterface(a_pRenderTarget);
}

HRESULT Direct3D9Renderer::presents()
{
	HRESULT hr;

	if (m_SwapChain == nullptr)
		return E_POINTER;

	CComQIPtrCustom<IDirect3DSwapChain9> lSwapChain(m_SwapChain);

	if (lSwapChain == nullptr)
		return E_POINTER;


	if (mRenderTargetType == RenderTargetType::Texture)
	{

		CComQIPtrCustom<IDirect3DVertexBuffer9> llist_vb(m_list_vb);

		CComPtrCustom<IDirect3DDevice9> lDevice9;

		hr = lSwapChain->GetDevice(&lDevice9);

		if (FAILED(hr)){
			return hr;
		}

		//Clear the buffer to our new colour.
		hr = lDevice9->Clear(0,  //Number of rectangles to clear, we're clearing everything so set it to 0
			NULL, //Pointer to the rectangles to clear, NULL to clear whole display
			D3DCLEAR_TARGET,   //What to clear.  We don't have a Z Buffer or Stencil Buffer
			0x00ff0000, //Colour to clear to (AARRGGBB)
			1.0f,  //Value to clear ZBuffer to, doesn't matter since we don't have one
			0);   //Stencil clear value, again, we don't have one, this value doesn't matter
		if (FAILED(hr)){
			return hr;
		}

		//Notify the device that we're ready to render
		hr = lDevice9->BeginScene();
		if (FAILED(hr)){
			return hr;
		}
		
		lDevice9->SetFVF(tri_fvf);

		//Bind our Vertex Buffer
		lDevice9->SetStreamSource(0,                   //StreamNumber
			llist_vb,           //StreamData
			0,                   //OffsetInBytes
			sizeof(textured_vertex)); //Stride
		
		//Render from our Vertex Buffer
		lDevice9->DrawPrimitive(D3DPT_TRIANGLELIST, //PrimitiveType
			0,                  //StartVertex
			list_count);      //PrimitiveCount


		//Notify the device that we're finished rendering for this frame
		lDevice9->EndScene();

		

		//Show the results
		hr = lDevice9->Present(NULL,  //Source rectangle to display, NULL for all of it
			NULL,  //Destination rectangle, NULL to fill whole display
			NULL,  //Target window, if NULL uses device window set in CreateDevice
			NULL);//Unused parameter, set it to NULL

		return hr;

	}
	else
	{
		
		//Show the results
		hr = lSwapChain->Present(
			nullptr,
			nullptr,
			nullptr,
			nullptr,
			0);

		return hr;
	}

}

HRESULT Direct3D9Renderer::shutdown()
{
	m_SwapChain.Release();

	m_RenderTexture.Release();

	m_list_vb.Release();

	return S_OK;
}