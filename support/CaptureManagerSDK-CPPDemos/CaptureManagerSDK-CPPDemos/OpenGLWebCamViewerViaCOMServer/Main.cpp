#define WIN32_LEAN_AND_MEAN    

#include <windows.h>
#include <Commdlg.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <gl/gl.h>
#include <memory>
#include <vector>
#include <atomic>
#include <Unknwnbase.h>
#include <fstream>
#include "../Common/ComPtrCustom.h"
#include "../Common/pugixml.hpp"
#include "../Common/CaptureManagerTypeInfo.h"
#include "../Common/CaptureManagerLoader.h"




#pragma comment(lib, "opengl32.lib")

#define GL_BGR                            0x80E0
#define GL_BGRA							  0x80E1

#define IID_PPV_ARGSIUnknown(ppType) __uuidof(**(ppType)), (IUnknown**)(ppType)

float halfQuadWidth = 0.75;
float halfQuadHeight = 0.75;

HDC hDC; 

HGLRC hRCDraw;

/**************************
* Function Declarations
*
**************************/

typedef HRESULT(STDAPICALLTYPE *PDllGetClassObject) (REFCLSID, REFIID, void**);

LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam);

void EnableOpenGL(HWND hWnd, HDC *hDC, HGLRC *hRCDraw);

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRCDraw);

LRESULT CALLBACK wndProcCallback(HWND, UINT, WPARAM, LPARAM);

HINSTANCE gHInstance;

int gNCmdShow;

static const GUID MFMediaType_Video =
{ 0x73646976, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };

static const GUID MFVideoFormat_RGB24 =
{ 20, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 } };

static const GUID MFVideoFormat_RGB32 =
{ 22, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 } };

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPTSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	using namespace std;
	using namespace pugi;


	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	gHInstance = hInstance;
	gNCmdShow = nCmdShow;

	CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	
	// get COM log interface
	CComPtrCustom<IClassFactory> lCoLogPrintOut;

	HRESULT lhresult = CaptureManagerLoader::getInstance().createCalssFactory(__uuidof(CoLogPrintOut), &lCoLogPrintOut);

	if (FAILED(lhresult))
		return lhresult;

	CComPtrCustom<ILogPrintOutControl> lLogPrintOutControl;

	lCoLogPrintOut->LockServer(true);

	lhresult = lCoLogPrintOut->CreateInstance(
		nullptr,
		IID_PPV_ARGS(&lLogPrintOutControl));

	if (FAILED(lhresult))
		return lhresult;

	// set log file for info
	lhresult = lLogPrintOutControl->addPrintOutDestination(
		(DWORD)INFO_LEVEL,
		L"Log.txt");

	if (FAILED(lhresult))
		return lhresult;

	// set log file for info
	lhresult = lLogPrintOutControl->addPrintOutDestination(
		(DWORD)ERROR_LEVEL,
		L"Log.txt");

	if (FAILED(lhresult))
		return lhresult;

	/* initialisation COM CaptureManager */

	CComPtrCustom<IClassFactory> lCoCaptureManager;

	lhresult = CaptureManagerLoader::getInstance().createCalssFactory(__uuidof(CoCaptureManager), &lCoCaptureManager);



	if (FAILED(lhresult))
		return lhresult;

	lCoCaptureManager->LockServer(true);

	// get ICaptureManagerControl interfrace
	CComPtrCustom<ICaptureManagerControl> lCaptureManagerControl;

	lhresult = lCoCaptureManager->CreateInstance(
		nullptr,
		IID_PPV_ARGS(&lCaptureManagerControl));

	if (FAILED(lhresult))
		return lhresult;



	// get ISourceControl inetrface
	CComPtrCustom<ISourceControl> lSourceControl;

	lhresult = lCaptureManagerControl->createControl(
		IID_PPV_ARGSIUnknown(&lSourceControl));

	if (FAILED(lhresult))
		return lhresult;

	BSTR lXMLString = nullptr;

	lhresult = lSourceControl->getCollectionOfSources(&lXMLString);

	if (FAILED(lhresult))
		return lhresult;


	xml_document lSourcesXmlDoc;

	lSourcesXmlDoc.load_string(lXMLString);

	// check Video Source via ValuePart
	auto lCheckVideoMediaType = [](const xml_node &node)
	{
		bool lresult = false;

		if (lstrcmpW(node.name(), L"ValuePart") == 0)
		{
			xml_attribute lAttrNode = node.attribute(L"Value");

			if (!lAttrNode.empty())
			{
				if (lstrcmpW(lAttrNode.as_string(), L"MFMediaType_Video") == 0)
					lresult = true;
			}
		}

		return lresult;
	};
	// check Video Source via Attribute 
	auto lCheckMediaType = [lCheckVideoMediaType](const xml_node &node)
	{
		bool lresult = false;

		if (lstrcmpW(node.name(), L"Source.Attributes") == 0)
		{
			xml_node lAttrNode = node.find_child_by_attribute(L"Name", L"MF_DEVSOURCE_ATTRIBUTE_MEDIA_TYPE");

			if (!lAttrNode.empty())
			{
				xml_node lValuePartNode = lAttrNode.find_node(lCheckVideoMediaType);

				if (!lValuePartNode.empty())
					lresult = true;
			}
		}

		return lresult;
	};

	// find first Video Source
	auto lFindFirstVideoSource = [lCheckMediaType](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"Source") == 0))
		{
			xml_node lAttrNode = node.find_node(lCheckMediaType);

			if (!lAttrNode.empty())
				lresult = true;
		}

		return lresult;
	};

	xml_node lVideoSourceXMLNode = lSourcesXmlDoc.find_node(lFindFirstVideoSource);

	if (lVideoSourceXMLNode.empty())
		return lhresult;

	// find symbolic link for Video Source
	auto lFindSymbolicLink = [](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"Attribute") == 0))
		{
			xml_attribute lAttrNode = node.attribute(L"Name");

			if (!lAttrNode.empty())
			{
				if ((lstrcmpW(lAttrNode.as_string(), L"MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK") == 0))
					lresult = true;
			}

		}

		return lresult;
	};

	xml_node lSymbolicLinkNode = lVideoSourceXMLNode.find_node(lFindSymbolicLink);

	if (lSymbolicLinkNode.empty())
		return lhresult;

	auto lAttrNode = lSymbolicLinkNode.first_child().attribute(L"Value");

	if (lAttrNode.empty())
		return lhresult;

	//BSTR lSymbolicLink = SysAllocString(L"CaptureManager///Software///Sources///ScreenCapture///ScreenCapture");

	BSTR lSymbolicLink = SysAllocString(lAttrNode.as_string());

	


	DWORD lIndexStream = 0; // Number of media stream

	DWORD lIndexMediaType = 6; // Number of Media Type

	SysFreeString(lXMLString);
	
	if (FAILED(lhresult))
		return lhresult;

	// Create ISinkControl interface

	CComPtrCustom<ISinkControl> lSinkControl;


	lhresult = lCaptureManagerControl->createControl(
		IID_PPV_ARGSIUnknown(&lSinkControl));

	if (FAILED(lhresult))
		return lhresult;

	lXMLString = SysAllocString(L"");

	// get Sink collection in XML document
	lhresult = lSinkControl->getCollectionOfSinks(&lXMLString);

	if (FAILED(lhresult))
		return lhresult;


	xml_document lSinksXmlDoc;

	lSinksXmlDoc.load_string(lXMLString);


	// find sample call sink by GUID - IID_ISampleGrabberCallSinkFactory
	auto lFindSink = [](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"SinkFactory") == 0))
		{
			xml_attribute lAttrNode = node.attribute(L"GUID");

			if (!lAttrNode.empty())
			{
				if ((lstrcmpW(lAttrNode.as_string(), L"{759D24FF-C5D6-4B65-8DDF-8A2B2BECDE39}") == 0))
					lresult = true;
			}
		}

		return lresult;
	};


	xml_node lSinkXMLNode = lSinksXmlDoc.find_node(lFindSink);

	if (lSinkXMLNode.empty())
		return -1;

	// get first read mode xml node (SYNC read mode)
	//                                  Value.ValueParts/ValuePart[0]
	//xml_node lValuePartNode = lSinkXMLNode.first_child().first_child();

	// PULL mode
	xml_node lValuePartNode = lSinkXMLNode.first_child().first_child().next_sibling().next_sibling();

	if (lValuePartNode.empty())
		return -1;


	xml_attribute lReadModeXMLAttr = lValuePartNode.attribute(L"GUID");

	if (lReadModeXMLAttr.empty())
		return -1;

	// get read mode IID
	IID lReadMode;

	lhresult = IIDFromString(lReadModeXMLAttr.as_string(),
		&lReadMode);

	if (FAILED(lhresult))
		return lhresult;

	SysFreeString(lXMLString);

	// create object with ISampleGrabberCallSinkFactory interface
	CComPtrCustom<ISampleGrabberCallSinkFactory> lSampleGrabberCallSinkFactory;

	lhresult = lSinkControl->createSinkFactory(
		lReadMode,
		IID_PPV_ARGSIUnknown(&lSampleGrabberCallSinkFactory));

	if (FAILED(lhresult))
		return lhresult;

	// find Video Source by Symbolic link
	auto lFindVideoSource = [lCheckMediaType, lFindSymbolicLink, lSymbolicLink](const xml_node &node)
	{
		bool lresult = false;

		std::wstring lSL = (lSymbolicLink);

		if ((lstrcmpW(node.name(), L"Source") == 0))
		{
			xml_node lAttrNode = node.find_node(lCheckMediaType);

			if (!lAttrNode.empty())
			{
				xml_node lSymbolicLinkNode = node.find_node(lFindSymbolicLink);

				if (!lSymbolicLinkNode.empty())
				{
					auto lValueAttrNode = lSymbolicLinkNode.first_child().attribute(L"Value");

					if (!lValueAttrNode.empty())
					{
						if (std::wstring(lValueAttrNode.as_string()) == std::wstring(lSymbolicLink))
						{
							lresult = true;
						}
					}
				}
			}
		}

		return lresult;
	};

	lVideoSourceXMLNode = lSourcesXmlDoc.find_node(lFindVideoSource);

	// find MediaType XML node by index
	auto lfindMediaType = [lIndexMediaType](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"MediaType") == 0))
		{
			xml_attribute lAttrNode = node.attribute(L"Index");

			if (!lAttrNode.empty())
			{
				auto lindex = lAttrNode.as_int(-1);

				if (lindex >= 0 && lindex == lIndexMediaType)
					lresult = true;
			}
		}

		return lresult;
	};

	// find StreamDescriptor XML node by index
	auto lfindStreamDescriptor = [lIndexStream](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"StreamDescriptor") == 0))
		{
			xml_attribute lAttrNode = node.attribute(L"Index");

			if (!lAttrNode.empty())
			{
				auto lindex = lAttrNode.as_int(-1);

				if (lindex >= 0 && lindex == lIndexStream)
					lresult = true;
			}
		}

		return lresult;
	};

	// find frame size MediaTypeItem
	auto lfindFrameSizeMediaTypeItem = [](const xml_node &node)
	{
		bool lresult = false;

		if ((lstrcmpW(node.name(), L"MediaTypeItem") == 0))
		{
			xml_attribute lAttrNode = node.attribute(L"Name");

			if (!lAttrNode.empty())
			{
				if ((lstrcmpW(lAttrNode.as_string(), L"MF_MT_FRAME_SIZE") == 0))
					lresult = true;
			}
		}
		return lresult;
	};



	xml_node lStreamDescriptorXMLNode = lVideoSourceXMLNode.find_node(lfindStreamDescriptor);

	if (lStreamDescriptorXMLNode.empty())
		return -1;

	xml_node lMediaTypeXMLNode = lStreamDescriptorXMLNode.find_node(lfindMediaType);

	if (lMediaTypeXMLNode.empty())
		return -1;

	xml_node lFrameSizeMediaTypeItemXMLNode = lMediaTypeXMLNode.find_node(lfindFrameSizeMediaTypeItem);

	if (lFrameSizeMediaTypeItemXMLNode.empty())
		return -1;

	// get size of video image in pixels
	//                                              Value.ValueParts / ValuePart / @Value

	DWORD lVideoWidth = lFrameSizeMediaTypeItemXMLNode.first_child().first_child().attribute(L"Value").as_uint();

	DWORD lVideoHeight = lFrameSizeMediaTypeItemXMLNode.first_child().first_child().next_sibling().attribute(L"Value").as_uint();


	// Create object for computing of Stride (byte length of one horizon line of image) For BitmapInfoHeader

	CComPtrCustom<IStrideForBitmap> lStrideForBitmap;

	lhresult = lCaptureManagerControl->createMisc(
		IID_PPV_ARGSIUnknown(&lStrideForBitmap));

	if (FAILED(lhresult))
		return lhresult;

	LONG lStride = 0;

	lhresult = lStrideForBitmap->getStrideForBitmap(
		MFVideoFormat_RGB32,
		lVideoWidth,
		&lStride);

	// define data buffer and sample grabber output node
	DWORD lSampleByteSize = abs(lStride) * lVideoHeight;

	std::unique_ptr<unsigned char> ldata(new unsigned char[lSampleByteSize]);

	// get object for getting of samples via direct calling.
	CComPtrCustom<ISampleGrabberCall> lSampleGrabberCall;

	lhresult = lSampleGrabberCallSinkFactory->createOutputNode(
		MFMediaType_Video,
		MFVideoFormat_RGB32,
		lSampleByteSize,
		IID_PPV_ARGSIUnknown(&lSampleGrabberCall));

	if (FAILED(lhresult))
		return lhresult;




	// get video source node
	CComPtrCustom<IUnknown> lVideoSourceNode;

	lhresult = lSourceControl->createSourceNodeWithDownStreamConnection(
		lSymbolicLink,
		lIndexStream,
		lIndexMediaType,
		lSampleGrabberCall,
		&lVideoSourceNode);
	
	if (FAILED(lhresult))
		return lhresult;

	SysFreeString(lSymbolicLink);



	// create session descriptor


	CComPtrCustom<ISessionControl> lSessionControl;

	lhresult = lCaptureManagerControl->createControl(
		IID_PPV_ARGSIUnknown(&lSessionControl));

	if (FAILED(lhresult))
		return lhresult;


	SAFEARRAY* pSA = NULL;
	SAFEARRAYBOUND bound[1];
	bound[0].lLbound = 0;
	bound[0].cElements = 1;
	pSA = SafeArrayCreate(VT_VARIANT, 1, bound);


	VARIANT lVar;

	VariantInit(&lVar);

	lVar.vt = VT_UNKNOWN;
	
	lVar.punkVal = lVideoSourceNode;

	long i = 0;

	SafeArrayPutElement(pSA, &i, &lVar);

	
	VARIANT theArray;

	VariantInit(&theArray);

	theArray.vt = VT_SAFEARRAY | VT_UNKNOWN;

	theArray.parray = pSA;


	CComPtrCustom<ISession> lSession;

	lhresult = lSessionControl->createSession(
		theArray,
		IID_PPV_ARGSIUnknown(&lSession));

	SafeArrayDestroy(pSA);

	VariantClear(&theArray);

	if (FAILED(lhresult))
		return lhresult;




	class SessionCallback : public ISessionCallback
	{
	public:

		SessionCallback() :
			mRefCount(1)
		{}

		virtual /* [helpstring] */ HRESULT STDMETHODCALLTYPE invoke(
			/* [in] */ DWORD aCallbackEventCode,
			/* [in] */ DWORD aSessionDescriptor)
		{
			switch (aCallbackEventCode)
			{
			case SessionCallbackEventCode::Error:
			{}
				break;
			case SessionCallbackEventCode::Status_Error:
			{}
				break;
			case SessionCallbackEventCode::Execution_Error:
			{}
				break;
			case SessionCallbackEventCode::ItIsReadyToStart:
			{}
				break;
			case SessionCallbackEventCode::ItIsStarted:
			{}
				break;
			case SessionCallbackEventCode::ItIsPaused:
			{}
				break;
			case SessionCallbackEventCode::ItIsStopped:
			{}
				break;
			case SessionCallbackEventCode::ItIsEnded:
			{}
				break;
			case SessionCallbackEventCode::ItIsClosed:
			{}
				break;
			case SessionCallbackEventCode::VideoCaptureDeviceRemoved:
			{}
				break;

			case SessionCallbackEventCode::UnknownEvent:
			default:
			{}
				break;
			}

			return S_OK;
		}

		virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, _COM_Outptr_ void __RPC_FAR *__RPC_FAR *ppvObject)
		{
			HRESULT lhresult = E_NOINTERFACE;

			do
			{
				if (ppvObject == NULL)
				{
					lhresult = E_POINTER;

					break;
				}

				lhresult = S_OK;

				if (riid == IID_IUnknown)
				{
					*ppvObject = static_cast<IUnknown*>(this);

					break;
				}
				else if (riid == __uuidof(ISessionCallback))
				{
					*ppvObject = static_cast<ISessionCallback*>(this);

					break;
				}

				*ppvObject = NULL;

				lhresult = E_NOINTERFACE;

			} while (false);

			if (SUCCEEDED(lhresult))
				AddRef();

			return lhresult;
		}

		virtual ULONG STDMETHODCALLTYPE AddRef(void)
		{
			return ++mRefCount;
		}

		virtual ULONG STDMETHODCALLTYPE Release(void)
		{
			ULONG lCount = --mRefCount;

			if (lCount == 0)
			{
				delete this;
			}
			return lCount;
		}

	private:

		std::atomic<ULONG> mRefCount;

		virtual ~SessionCallback(){}
	};

	CComPtrCustom<IConnectionPointContainer> IConnectionPointContainer;

	lhresult = lSession->getIConnectionPointContainer(
		IID_PPV_ARGSIUnknown(&IConnectionPointContainer));

	if (FAILED(lhresult))
		return lhresult;

	CComPtrCustom<IConnectionPoint> lConnectionPoint;

	lhresult = IConnectionPointContainer->FindConnectionPoint(
		__uuidof(ISessionCallback),
		&lConnectionPoint);

	if (FAILED(lhresult))
		return lhresult;

	CComPtrCustom<ISessionCallback> lSessionCallback = new SessionCallback();

	DWORD lStreamID;

	lhresult = lConnectionPoint->Advise(
		lSessionCallback,
		&lStreamID);

	if (FAILED(lhresult))
		return lhresult;

	lhresult = lSession->startSession(0, GUID_NULL);

	if (FAILED(lhresult))
		return lhresult;

	int lWidth = 800;
	int lHeight = 600;


	WNDCLASS wc;
	HWND hWnd;
	MSG msg;
	BOOL bQuit = FALSE;
	float theta = 0.0f;

	/* register window class */
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = L"OpenGLWebCamViewer";
	RegisterClass(&wc);

	/* create main window */
	hWnd = CreateWindow(
		L"OpenGLWebCamViewer", L"OpenGLWebCamViewer Sample",
		WS_OVERLAPPEDWINDOW | WS_VISIBLE,
		//WS_CAPTION | WS_POPUPWINDOW | WS_VISIBLE,
		0, 0, lWidth, lHeight,
		NULL, NULL, hInstance, NULL);

	/* enable OpenGL for the window */
	EnableOpenGL(hWnd, &hDC, &hRCDraw);

	GLuint lDrawTextureID;

	glGenTextures(1, &lDrawTextureID);


	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, lDrawTextureID);



	// create the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, lVideoWidth, lVideoHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	// start session.

	/* program main loop */
	while (!bQuit)
	{
		/* check for messages */
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			/* handle or dispatch messages */
			if (msg.message == WM_QUIT)
			{
				bQuit = TRUE;
			}
			else
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else
		{
			/* OpenGL animation code goes here */

			DWORD lByteSize;

			lhresult = lSampleGrabberCall->readData(ldata.get(), &lByteSize);
			
			glClear(GL_COLOR_BUFFER_BIT);
			glLoadIdentity();

			if (SUCCEEDED(lhresult))
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, lVideoWidth, lVideoHeight, GL_BGRA, GL_UNSIGNED_BYTE, ldata.get());

			glBegin(GL_QUADS);      // Begining of drawing of Square.

			glTexCoord2f(0, 1);  glVertex2f(-halfQuadWidth, -halfQuadHeight);	// Buttom Left
			glTexCoord2f(1, 1);  glVertex2f(halfQuadWidth, -halfQuadHeight);	// Buttom right
			glTexCoord2f(1, 0);  glVertex2f(halfQuadWidth, halfQuadHeight);		// Top right
			glTexCoord2f(0, 0);  glVertex2f(-halfQuadWidth, halfQuadHeight);	// Top Left
			glEnd();


			SwapBuffers(hDC);

			Sleep(30);
		}
	}

	/* stop session */
	lSession->closeSession();

	/* shutdown OpenGL */
	DisableOpenGL(hWnd, hDC, hRCDraw);

	/* destroy the window explicitly */
	DestroyWindow(hWnd);

	lCoLogPrintOut->LockServer(false);

	lCoCaptureManager->LockServer(false);

	return 0;
}


/********************
* Window Procedure
*
********************/

LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_CREATE:

		return 0;
	case WM_CLOSE:
		PostQuitMessage(0);
		return 0;

	case WM_DESTROY:
		return 0;

	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_ESCAPE:
			PostQuitMessage(0);
			return 0;
		}
		return 0;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}


/*******************
* Enable OpenGL
*
*******************/

void EnableOpenGL(HWND hWnd, HDC *hDC, HGLRC *hRCDraw)
{
	PIXELFORMATDESCRIPTOR pfd;
	int iFormat;

	/* get the device context (DC) */
	*hDC = GetDC(hWnd);

	/* set the pixel format for the DC */
	ZeroMemory(&pfd, sizeof (pfd));
	pfd.nSize = sizeof (pfd);
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW |
		PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 24;
	pfd.cDepthBits = 16;
	pfd.iLayerType = PFD_MAIN_PLANE;
	iFormat = ChoosePixelFormat(*hDC, &pfd);
	SetPixelFormat(*hDC, iFormat, &pfd);

	/* create and enable the render context (RC) */

	*hRCDraw = wglCreateContext(*hDC);
	
	wglMakeCurrent(*hDC, *hRCDraw);

	glEnable(GL_TEXTURE_2D);			// Enable of texturing for OpenGL

}

/******************
* Disable OpenGL
*
******************/

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRCDraw)
{
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRCDraw);
	ReleaseDC(hWnd, hDC);
}

LRESULT CALLBACK wndProcCallback(HWND ahwnd, UINT aMessage, WPARAM aWParam, LPARAM aLParam)
{
	HRESULT hr = S_OK;

	switch (aMessage)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;


	default:
		return DefWindowProc(ahwnd, aMessage, aWParam, aLParam);
	}
	return 0;
}
