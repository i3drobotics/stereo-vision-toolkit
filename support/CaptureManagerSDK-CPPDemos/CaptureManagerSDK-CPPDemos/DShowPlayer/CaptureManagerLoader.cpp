#include "CaptureManagerLoader.h"
#include "Common/CaptureManagerTypeInfo.h"
#include "Common/pugixml.hpp"



#define IID_PPV_ARGSIUnknown(ppType) __uuidof(**(ppType)), (IUnknown**)(ppType)

class DECLSPEC_UUID("A2224D8D-C3C1-4593-8AC9-C0FCF318FF05")
	CaptureManagerVideoRendererMultiSinkCLSID;


using namespace pugi;

CaptureManagerLoader::CaptureManagerLoader() :
	mMaxPorts(0),
	m_pUnkEVRMultiSinkFactory(nullptr),
	m_pUnkStreamControl(nullptr)
{

}

CaptureManagerLoader::~CaptureManagerLoader()
{

}

CaptureManagerLoader& CaptureManagerLoader::getInstance()
{
	static CaptureManagerLoader lInstance;

	return lInstance;
}

HRESULT CaptureManagerLoader::createRendererOutputNodes(
	void * aHandle,
	IUnknown *aPtrUnkTarget,
	unsigned long aOutputNodeAmount,
	std::vector<CComPtrCustom<IUnknown>>& aOutputNodes)
{
	HRESULT lhresult(E_FAIL);

	do
	{
		lhresult = init();

		if (FAILED(lhresult))
			break;

		aOutputNodes.clear();

		if (m_pUnkEVRMultiSinkFactory == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}


		CComQIPtrCustom<IEVRMultiSinkFactory> lEVRMultiSinkFactory;

		lEVRMultiSinkFactory = m_pUnkEVRMultiSinkFactory;

		if (!lEVRMultiSinkFactory)
			break;

		VARIANT theOutputNodeArray;

		VariantInit(&theOutputNodeArray);

		lhresult = lEVRMultiSinkFactory->createOutputNodes(
			aHandle,
			aPtrUnkTarget,
			aOutputNodeAmount,
			&theOutputNodeArray);

		if (FAILED(lhresult))
			break;

		if (theOutputNodeArray.vt == VT_SAFEARRAY | VT_UNKNOWN)
		{
			long i = 0;

			VARIANT lVar;

			VariantInit(&lVar);

			lhresult = SafeArrayGetElement(theOutputNodeArray.parray, &i, &lVar);

			if (FAILED(lhresult))
				continue;

			CComPtrCustom<IUnknown> lUnkNode;

			if (lVar.punkVal != nullptr)
				lUnkNode = lVar.punkVal;

			aOutputNodes.push_back(lUnkNode);

			VariantClear(&lVar);

		}

		SafeArrayDestroy(theOutputNodeArray.parray);

		theOutputNodeArray.parray = nullptr;

		VariantClear(&theOutputNodeArray);
				
		lhresult = S_OK;

	} while (false);

	return lhresult;
}

HRESULT CaptureManagerLoader::getMaxVideoRenderStreamCount(
	unsigned long* aPtrOutputNodeAmount)
{
	HRESULT lhresult(E_FAIL);

	do
	{
		if (aPtrOutputNodeAmount == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		lhresult = init();

		if (FAILED(lhresult))
			break;

		*aPtrOutputNodeAmount = mMaxPorts;

		lhresult = S_OK;

	} while (false);

	return lhresult;
}


HRESULT CaptureManagerLoader::createEVRStreamControl(
	IUnknown** aPtrPtrUnkIEVRStreamControl)
{
	HRESULT lhresult(E_FAIL);

	do
	{
		if (aPtrPtrUnkIEVRStreamControl == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (m_pUnkStreamControl == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		lhresult = init();

		if (FAILED(lhresult))
			break;

		m_pUnkStreamControl->QueryInterface(IID_PPV_ARGS(aPtrPtrUnkIEVRStreamControl));

		lhresult = S_OK;

	} while (false);

	return lhresult;
}

HRESULT CaptureManagerLoader::init()
{
	HRESULT lhresult(E_FAIL);

	do
	{
		
		if (m_pUnkEVRMultiSinkFactory != nullptr &&
			m_pUnkStreamControl != nullptr)
		{
			lhresult = S_OK;

			break;
		}

		HMODULE lHCaptureManagerLibrary = LoadLibraryEx(L"CaptureManager.dll", NULL, 0);

		CComPtrCustom<IClassFactory> lCoCaptureManager;

		if (lHCaptureManagerLibrary == nullptr)
		{
			if (lHCaptureManagerLibrary == nullptr)
			{
				lhresult = CoGetClassObject(__uuidof(CoCaptureManager), CLSCTX_INPROC_SERVER, nullptr, IID_PPV_ARGS(&lCoCaptureManager));

				if (SUCCEEDED(lhresult))
					break;
			}

			return -1;
		}

		{

			typedef HRESULT(STDAPICALLTYPE *PDllGetClassObject) (REFCLSID, REFIID, void**);
			
			PDllGetClassObject mPtrFuncDllGetClassObject = nullptr;
			
			mPtrFuncDllGetClassObject = (PDllGetClassObject)GetProcAddress(lHCaptureManagerLibrary, "DllGetClassObject");

			if (mPtrFuncDllGetClassObject == nullptr)
			{
				return -1;
			}

			lhresult = mPtrFuncDllGetClassObject(__uuidof(CoCaptureManager), IID_PPV_ARGS(&lCoCaptureManager));
		}

		if (!lCoCaptureManager)
		{
			lhresult = E_POINTER;

			break;
		}


		lCoCaptureManager->LockServer(true);

		CComPtrCustom<ICaptureManagerControl> lCaptureManagerControl;

		// get ICaptureManagerControl interfrace
		lhresult = lCoCaptureManager->CreateInstance(
			nullptr,
			IID_PPV_ARGS(&lCaptureManagerControl));

		if (FAILED(lhresult))
			return lhresult;

		CComPtrCustom<ISinkControl> lSinkControl;

		lhresult = lCaptureManagerControl->createControl(
			IID_PPV_ARGSIUnknown(&lSinkControl));

		if (FAILED(lhresult))
			return lhresult;

		BSTR lXMLstring = nullptr;

		lhresult = lSinkControl->getCollectionOfSinks(&lXMLstring);

		if (FAILED(lhresult))
			return lhresult;


		xml_document lSinksXmlDoc;

		lSinksXmlDoc.load_string(lXMLstring);

		if (lXMLstring != nullptr)
			SysFreeString(lXMLstring);


		auto lSinkXPathNode = lSinksXmlDoc.select_node(L"SinkFactories/SinkFactory[@GUID='{A2224D8D-C3C1-4593-8AC9-C0FCF318FF05}']");

		if (lSinkXPathNode.node().empty())
			break;

		auto lMaxPortCountXPathNode = lSinkXPathNode.node().select_node(L"Value.ValueParts/ValuePart/@MaxPortCount");

		if (lMaxPortCountXPathNode.attribute().empty())
			break;

		unsigned int lmaxPorts = lMaxPortCountXPathNode.attribute().as_uint(0);

		if (lmaxPorts == 0)
			break;

		mMaxPorts = lmaxPorts;

		lhresult = lSinkControl->createSinkFactory(
			GUID_NULL,
			__uuidof(CaptureManagerVideoRendererMultiSinkCLSID),
			&m_pUnkEVRMultiSinkFactory);

		if (FAILED(lhresult))
			return lhresult;


		CComPtrCustom<IEVRStreamControl> lEVRStreamControl;

		lhresult = lCaptureManagerControl->createMisc(
			IID_PPV_ARGSIUnknown(&lEVRStreamControl));

		if (FAILED(lhresult))
			return lhresult;

		lhresult = lEVRStreamControl->QueryInterface(IID_PPV_ARGS(&m_pUnkStreamControl));

	} while (false);

	return lhresult;
}