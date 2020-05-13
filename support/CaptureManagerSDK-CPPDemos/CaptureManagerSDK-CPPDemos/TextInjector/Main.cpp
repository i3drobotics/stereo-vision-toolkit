#define WIN32_LEAN_AND_MEAN    

#include <windows.h>
#include <Commdlg.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <memory>
#include <vector>
#include <atomic>
#include <Unknwnbase.h>
#include <fstream>
#include "../Common/ComPtrCustom.h"
#include "../Common/pugixml.hpp"
#include "../Common/CaptureManagerTypeInfo.h"
#include "../Common/CaptureManagerLoader.h"
#include "TextInjectorMFT.h"


#define IID_PPV_ARGSIUnknown(ppType) __uuidof(**(ppType)), (IUnknown**)(ppType)

/**************************
* Function Declarations
*
**************************/

typedef HRESULT(STDAPICALLTYPE *PDllGetClassObject) (REFCLSID, REFIID, void**);

LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam);

HRESULT createTextInjectorTopologyNode(
	IUnknown** aPtrPtrTextInjectorTopologyNode,
	IUnknown* aPtrDownstreamNode,
	ITextWriter** aPtrPtrITextWriter);

HINSTANCE gHInstance;

int gNCmdShow;

BOOL bQuit = FALSE;

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

	HRESULT lhresult = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

	if (FAILED(lhresult))
		return lhresult;

	CComPtrCustom<IClassFactory> lCoLogPrintOut;

	lhresult = CaptureManagerLoader::getInstance().createCalssFactory(__uuidof(CoLogPrintOut), &lCoLogPrintOut);

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

	// set log file for info
	lhresult = lLogPrintOutControl->addPrintOutDestination(
		(DWORD)ERROR_LEVEL,
		L"Log.txt");

	if (FAILED(lhresult))
		return lhresult;

	/* initialisation CaptureManager */

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

	BSTR lSymbolicLink = SysAllocString(lAttrNode.as_string());

	//BSTR lSymbolicLink = SysAllocString(L"CaptureManager///Software///Sources///ScreenCapture///ScreenCapture");

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
				if ((lstrcmpW(lAttrNode.as_string(), L"{2F34AF87-D349-45AA-A5F1-E4104D5C458E}") == 0))
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
	xml_node lValuePartNode = lSinkXMLNode.first_child().first_child();

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
	CComPtrCustom<IEVRSinkFactory> lEVRSinkFactory;

	lhresult = lSinkControl->createSinkFactory(
		lReadMode,
		IID_PPV_ARGSIUnknown(&lEVRSinkFactory));

	if (FAILED(lhresult))
		return lhresult;

	WNDCLASS wc;
	HWND hWnd;
	MSG msg;
	float theta = 0.0f;

	/* register window class */
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = NULL;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = L"TextInjector";
	RegisterClass(&wc);

	/* create main window */
	hWnd = CreateWindow(
		L"TextInjector", L"TextInjector Sample",
		WS_OVERLAPPEDWINDOW | WS_VISIBLE,
		//WS_CAPTION | WS_POPUPWINDOW | WS_VISIBLE,
		0, 0, 800, 400,
		NULL, NULL, NULL, NULL);

	CComPtrCustom<IUnknown> RendererActivateTopologyNode;

	lhresult = lEVRSinkFactory->createOutputNode(
		hWnd,
		&RendererActivateTopologyNode);

	if (FAILED(lhresult))
		return lhresult;

	CComPtrCustom<IUnknown> lTextInjectorTopologyNode;

	CComPtrCustom<ITextWriter> lITextWriter;

	lhresult = createTextInjectorTopologyNode(
		&lTextInjectorTopologyNode, 
		RendererActivateTopologyNode,
		&lITextWriter);

	// get video source node
	CComPtrCustom<IUnknown> lVideoSourceNode;

	lhresult = lSourceControl->createSourceNodeWithDownStreamConnection(
		lSymbolicLink,
		lIndexStream,
		lIndexMediaType,
		lTextInjectorTopologyNode,
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

	lhresult = SafeArrayPutElement(pSA, &i, &lVar);

	if (FAILED(lhresult))
		return lhresult;

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
			{
				bQuit = TRUE;
			}
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

	ShowWindow(hWnd, SW_SHOW);

	int lFrameCount = 0;

	wchar_t ltext[MAXBYTE];

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

		_itow_s(++lFrameCount, ltext, 10);

		lITextWriter->writeText(ltext);

		Sleep(200);
	}

	return 0;
}

LRESULT CALLBACK WndProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
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


HRESULT createTextInjectorTopologyNode(
	IUnknown** aPtrPtrTextInjectorTopologyNode,
	IUnknown* aPtrDownstreamNode,
	ITextWriter** aPtrPtrITextWriter)
{
	HRESULT lhresult = S_OK;

	CComPtrCustom<IMFMediaTypeHandler> lHandler;

	CComPtrCustom<IMFActivate> lRendererActivate;

	CComPtrCustom<IMFTopologyNode> aNodeMFT;

	CComPtrCustom<IMFTopologyNode> lDownstreamNode;

	do
	{
		if (aPtrDownstreamNode == nullptr)
			break;

		if (aPtrPtrITextWriter == nullptr)
			break;			

		lhresult = aPtrDownstreamNode->QueryInterface(IID_PPV_ARGS(&lDownstreamNode));

		if (FAILED(lhresult))
			break;

		TextInjectorMFT *lMFT = new (std::nothrow) TextInjectorMFT();

		if (lMFT == nullptr)
			break;

		lhresult = MFCreateTopologyNode(MF_TOPOLOGY_TRANSFORM_NODE, &aNodeMFT);

		if (FAILED(lhresult))
			break;

		lhresult = lMFT->QueryInterface(IID_PPV_ARGS(aPtrPtrITextWriter));

		if (FAILED(lhresult))
			break;			

		lhresult = aNodeMFT->SetObject(*aPtrPtrITextWriter);

		if (FAILED(lhresult))
			break;

		lhresult = aNodeMFT->ConnectOutput(0, lDownstreamNode, 0);

		if (FAILED(lhresult))
			break;

		lhresult = aNodeMFT->QueryInterface(IID_PPV_ARGS(aPtrPtrTextInjectorTopologyNode));
		
	} while (false);

	return lhresult;
}