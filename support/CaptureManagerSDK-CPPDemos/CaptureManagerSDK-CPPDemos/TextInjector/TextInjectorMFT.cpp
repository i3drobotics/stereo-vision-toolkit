#include "TextInjectorMFT.h"


TextInjectorMFT::TextInjectorMFT():
mRefCount(1)
{}


TextInjectorMFT::~TextInjectorMFT()
{
}

void TextInjectorMFT::writeText(LPCWSTR aText)
{
	m_frameParser.writeText(aText);
}

STDMETHODIMP TextInjectorMFT::GetStreamLimits(DWORD* aPtrInputMinimum, DWORD* aPtrInputMaximum,
	DWORD* aPtrOutputMinimum, DWORD* aPtrOutputMaximum)
{
	if (aPtrInputMinimum == NULL ||
		aPtrInputMaximum == NULL ||
		aPtrOutputMinimum == NULL ||
		aPtrOutputMaximum == NULL)
	{
		return E_POINTER;
	}

	*aPtrInputMinimum = 1;
	*aPtrInputMaximum = 1;
	*aPtrOutputMinimum = 1;
	*aPtrOutputMaximum = 1;

	return S_OK;
}

STDMETHODIMP TextInjectorMFT::GetStreamIDs(DWORD aInputIDArraySize, DWORD* aPtrInputIDs,
	DWORD aOutputIDArraySize, DWORD* aPtrOutputIDs)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::GetStreamCount(DWORD* aPtrInputStreams, DWORD* aPtrOutputStreams)
{
	if (aPtrInputStreams == NULL || aPtrOutputStreams == NULL)
	{
		return E_POINTER;
	}

	*aPtrInputStreams = 1;

	*aPtrOutputStreams = 1;

	return S_OK;
}

STDMETHODIMP TextInjectorMFT::GetInputStreamInfo(DWORD aInputStreamID,
	MFT_INPUT_STREAM_INFO* aPtrStreamInfo)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrStreamInfo == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}
		

		if (aInputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
			break;
		}


		aPtrStreamInfo->dwFlags = MFT_INPUT_STREAM_WHOLE_SAMPLES |
			MFT_INPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER;


		aPtrStreamInfo->cbMaxLookahead = 0;

		aPtrStreamInfo->cbAlignment = 0;

		aPtrStreamInfo->hnsMaxLatency = 0;

		aPtrStreamInfo->cbSize = 0;

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetOutputStreamInfo(DWORD aOutputStreamID,
	MFT_OUTPUT_STREAM_INFO* aPtrStreamInfo)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrStreamInfo == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aOutputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
			break;
		}


		aPtrStreamInfo->dwFlags =
			MFT_OUTPUT_STREAM_WHOLE_SAMPLES |
			MFT_OUTPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER |
			MFT_OUTPUT_STREAM_FIXED_SAMPLE_SIZE |
			MFT_OUTPUT_STREAM_PROVIDES_SAMPLES;


		aPtrStreamInfo->cbAlignment = 0;

		aPtrStreamInfo->cbSize = 0;

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetInputStreamAttributes(DWORD aInputStreamID,
	IMFAttributes** aPtrPtrAttributes)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::GetOutputStreamAttributes(DWORD aOutputStreamID,
	IMFAttributes** aPtrPtrAttributes)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::DeleteInputStream(DWORD aStreamID)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::AddInputStreams(DWORD aStreams, DWORD* aPtrStreamIDs)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::GetInputAvailableType(DWORD aInputStreamID, DWORD aTypeIndex,
	IMFMediaType** aPtrPtrType)
{
	HRESULT lhresult = S_OK;
	CComPtrCustom<IMFMediaType> lMediaType;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrPtrType == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aInputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;

			if (FAILED(lhresult))
				break;
		}

		if (!m_pOutputType)
		{
			lhresult = GetSupportedMediaType(aTypeIndex, &lMediaType);

			if (FAILED(lhresult))
				break;

			*aPtrPtrType = lMediaType.Detach();
		}
		else if (aTypeIndex == 0)
		{
			*aPtrPtrType = m_pOutputType.Detach();
		}
		else
		{
			lhresult = MF_E_NO_MORE_TYPES;
		}

	} while (false);

	if (FAILED(lhresult) && aPtrPtrType != NULL)
	{
		*aPtrPtrType = NULL;
	}

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetOutputAvailableType(DWORD aOutputStreamID, DWORD aTypeIndex,
	IMFMediaType** aPtrPtrType)
{
	HRESULT lhresult = S_OK;
	CComPtrCustom<IMFMediaType> lMediaType;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrPtrType == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aOutputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;

			if (FAILED(lhresult))
				break;
		}


		if (!m_pInputType)
		{
			lhresult = GetSupportedMediaType(aTypeIndex, &lMediaType);

			if (FAILED(lhresult))
				break;

			*aPtrPtrType = lMediaType.get();
			(*aPtrPtrType)->AddRef();
		}
		else
		{
			*aPtrPtrType = m_pInputType.get();

			(*aPtrPtrType)->AddRef();
		}
	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::SetInputType(DWORD aInputStreamID, IMFMediaType* aPtrType,
	DWORD aFlags)
{
	HRESULT lhresult = S_OK;
	CComPtrCustom<IMFAttributes> lTypeAttributes;

	do
	{
		lTypeAttributes = aPtrType;

		std::unique_lock<std::mutex> lock(mMutex);


		if (aInputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
	
			break;
		}

		lhresult = CheckMediaType(aPtrType);

		if (FAILED(lhresult))
			break;


		if (!(!m_pSample))
		{
			lhresult = MF_E_TRANSFORM_CANNOT_CHANGE_MEDIATYPE_WHILE_PROCESSING;
			
			break;
		}


		if (aPtrType != NULL && !(!m_pOutputType))
		{
			BOOL lresult = FALSE;

			lhresult = aPtrType->Compare(lTypeAttributes, MF_ATTRIBUTES_MATCH_INTERSECTION, &lresult);

			if (FAILED(lhresult))
				break;

			if (!lresult)
			{
				lhresult = MF_E_INVALIDMEDIATYPE;

				break;
			}
		}


		if (aFlags != MFT_SET_TYPE_TEST_ONLY)
		{
			m_pInputType = aPtrType;

			lhresult = m_frameParser.SetFrameType(m_pInputType);
		}

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::SetOutputType(DWORD aOutputStreamID, IMFMediaType* aPtrType,
	DWORD aFlags)
{
	HRESULT lhresult = S_OK;

	CComPtrCustom<IMFMediaType> lType;

	do
	{
		lType = aPtrType;

		std::unique_lock<std::mutex> lock(mMutex);
		
		if (aOutputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
			
			break;
		}

		lhresult = CheckMediaType(lType);

		if (FAILED(lhresult))
			break;


		if (!(!m_pSample))
		{
			lhresult = MF_E_TRANSFORM_CANNOT_CHANGE_MEDIATYPE_WHILE_PROCESSING;
			
			break;
		}


		if (!(!lType) && !(!m_pInputType))
		{
			DWORD flags = 0;

			lhresult = lType->IsEqual(m_pInputType, &flags);

			if (FAILED(lhresult))
				break;
		}
		
		if (aFlags != MFT_SET_TYPE_TEST_ONLY)
		{
			m_pOutputType = lType.Detach();
		}

	} while (false);


	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetInputCurrentType(DWORD aInputStreamID, IMFMediaType** aPtrPtrType)
{
	HRESULT lhresult = S_OK;

	do
	{

		std::unique_lock<std::mutex> lock(mMutex);


		if (aPtrPtrType == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aInputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
		}
		else if (!m_pInputType)
		{
			lhresult = MF_E_TRANSFORM_TYPE_NOT_SET;
		}
		else
		{
			*aPtrPtrType = m_pInputType;

			(*aPtrPtrType)->AddRef();
		}

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetOutputCurrentType(DWORD aOutputStreamID, IMFMediaType** aPtrPtrType)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrPtrType == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}


		if (aOutputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;
		}
		else if (m_pOutputType.get() == NULL)
		{
			lhresult = MF_E_TRANSFORM_TYPE_NOT_SET;
		}
		else
		{
			*aPtrPtrType = m_pOutputType;
			(*aPtrPtrType)->AddRef();
		}

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetInputStatus(DWORD aInputStreamID, DWORD* aPtrFlags)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrFlags == nullptr)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aInputStreamID != 0)
		{
			lhresult = MF_E_INVALIDSTREAMNUMBER;

			break;
		}
		
		if (!m_pSample)
		{
			*aPtrFlags = MFT_INPUT_STATUS_ACCEPT_DATA;
		}
		else
		{
			*aPtrFlags = 0;
		}

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::GetOutputStatus(DWORD* aPtrFlags)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::SetOutputBounds(LONGLONG aLowerBound, LONGLONG aUpperBound)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::ProcessEvent(DWORD aInputStreamID, IMFMediaEvent* aPtrEvent)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::GetAttributes(IMFAttributes** aPtrPtrAttributes)
{
	return E_NOTIMPL;
}

STDMETHODIMP TextInjectorMFT::ProcessMessage(MFT_MESSAGE_TYPE aMessage, ULONG_PTR aParam)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);
		
		if (aMessage == MFT_MESSAGE_COMMAND_FLUSH)
		{
			m_pSample = NULL;
		}
		else if (aMessage == MFT_MESSAGE_COMMAND_DRAIN)
		{
		}
		else if (aMessage == MFT_MESSAGE_NOTIFY_BEGIN_STREAMING)
		{
		}
		else if (aMessage == MFT_MESSAGE_NOTIFY_END_STREAMING)
		{
		}
		else if (aMessage == MFT_MESSAGE_NOTIFY_END_OF_STREAM)
		{
		}
		else if (aMessage == MFT_MESSAGE_NOTIFY_START_OF_STREAM)
		{
		}

	} while (false);

	return lhresult;
}

STDMETHODIMP TextInjectorMFT::ProcessInput(DWORD aInputStreamID, IMFSample* aPtrSample,
	DWORD aFlags)
{
	HRESULT lhresult = S_OK;
	DWORD dwBufferCount = 0;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrSample == NULL)
		{
			lhresult = E_POINTER;

			break;
		}
		
		if (aInputStreamID != 0 || aFlags != 0)
		{
			lhresult = E_INVALIDARG;

			break;
		}


		if (!m_pInputType)
		{
			lhresult = MF_E_NOTACCEPTING;

			break;
		}


		if (!m_pOutputType)
		{
			lhresult = MF_E_NOTACCEPTING;

			break;
		}

		if (!(!m_pSample))
		{
			lhresult = MF_E_NOTACCEPTING;

			break;
		}

		m_pSample = aPtrSample;

	} while (false);
	
	return lhresult;
}

STDMETHODIMP TextInjectorMFT::ProcessOutput(DWORD aFlags, DWORD aOutputBufferCount,
	MFT_OUTPUT_DATA_BUFFER* aPtrOutputSamples, DWORD* aPtrStatus)
{
	HRESULT lhresult = S_OK;

	do
	{
		std::unique_lock<std::mutex> lock(mMutex);

		if (aPtrOutputSamples == NULL)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aPtrStatus == NULL)
		{
			lhresult = E_POINTER;

			break;
		}
				
		if (aOutputBufferCount != 1 || aFlags != 0)
		{
			lhresult = E_INVALIDARG;
			break;
		}
		
		if (!m_pSample)
		{
			lhresult = MF_E_TRANSFORM_NEED_MORE_INPUT;

			break;
		}


		
		lhresult = m_frameParser.LockFrame(m_pSample);

		if (FAILED(lhresult))
			break;

		lhresult = m_frameParser.DrawBitmap();

		if (FAILED(lhresult))
			break;

		lhresult = m_frameParser.UnlockFrame();

		if (FAILED(lhresult))
			break;

		aPtrOutputSamples[0].pSample = m_pSample.Detach();

		aPtrOutputSamples[0].dwStatus = 0;

		*aPtrStatus = 0;

	} while (false);

	return lhresult;
}

HRESULT STDMETHODCALLTYPE TextInjectorMFT::QueryInterface(REFIID aRefIID, void** aPtrPtrVoidObject)
{
	HRESULT lhresult = S_OK;

	do
	{
	
		if (aPtrPtrVoidObject == NULL)
		{
			lhresult = E_POINTER;

			break;
		}

		if (aRefIID == IID_IUnknown)
		{
			*aPtrPtrVoidObject = static_cast<IUnknown*>(static_cast<IMFTransform*>(this));
		}
		else if (aRefIID == IID_IMFTransform)
		{
			*aPtrPtrVoidObject = static_cast<IMFTransform*>(this);
		}
		else if (aRefIID == __uuidof(ITextWriter))
		{
			*aPtrPtrVoidObject = static_cast<ITextWriter*>(this);
		}
		else
		{
			*aPtrPtrVoidObject = NULL;

			lhresult = E_NOINTERFACE;
		}

		if (SUCCEEDED(lhresult))
			AddRef();

	} while (false);

	return lhresult;
}

ULONG STDMETHODCALLTYPE TextInjectorMFT::AddRef()
{
	return ++mRefCount;
}
ULONG STDMETHODCALLTYPE TextInjectorMFT::Release()
{
	ULONG refCount = --mRefCount;

	if (refCount == 0)
	{
		delete this;
	}

	return refCount;
}

HRESULT TextInjectorMFT::GetSupportedMediaType(DWORD aTypeIndex, IMFMediaType** aPtrPtrMediaType)
{
	HRESULT lhresult = S_OK;

	CComPtrCustom<IMFMediaType> lMediaType;

	do
	{
		lhresult = MFCreateMediaType(&lMediaType);

		if (FAILED(lhresult))
			break;

		lhresult = lMediaType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);

		if (FAILED(lhresult))
			break;

		if (aTypeIndex == 0)
		{
			lhresult = lMediaType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
		}
		else
		{
			lhresult = MF_E_NO_MORE_TYPES;
		}

		if (FAILED(lhresult))
			break;

		*aPtrPtrMediaType = lMediaType.Detach();

	} while (false);

	return lhresult;
}

HRESULT TextInjectorMFT::CheckMediaType(IMFMediaType* aPtrMediaType)
{
	GUID majorType = GUID_NULL;
	GUID subtype = GUID_NULL;
	MFVideoInterlaceMode interlacingMode = MFVideoInterlace_Unknown;
	HRESULT lhresult = S_OK;

	CComPtrCustom<IMFMediaType> lType;

	do
	{
		lType = aPtrMediaType;

		if (!lType)
		{
			lhresult = E_POINTER;

			break;
		}

		lhresult = lType->GetGUID(MF_MT_MAJOR_TYPE, &majorType);

		if (FAILED(lhresult))
			break;

		if (majorType != MFMediaType_Video)
		{
			lhresult = MF_E_INVALIDMEDIATYPE;
			break;
		}

		lhresult = lType->GetGUID(MF_MT_SUBTYPE, &subtype);

		if (FAILED(lhresult))
			break;

		if (subtype != MFVideoFormat_RGB32)
		{
			lhresult = MF_E_INVALIDMEDIATYPE;
			break;
		}

		lhresult = lType->GetUINT32(MF_MT_INTERLACE_MODE, (UINT32*)&interlacingMode);

		if (FAILED(lhresult))
			break;

		if (interlacingMode != MFVideoInterlace_Progressive)
		{
			lhresult = MF_E_INVALIDMEDIATYPE;

			break;
		}

	} while (false);

	return lhresult;
}