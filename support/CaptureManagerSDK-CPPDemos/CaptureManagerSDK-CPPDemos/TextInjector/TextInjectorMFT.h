#pragma once

#include <atomic>
#include <condition_variable>

#include "FrameParser.h"

#include "MF.h"
#include "../Common/ComPtrCustom.h"

MIDL_INTERFACE("DEAA6BF0-0715-413A-89F8-3C5D335839E5")
ITextWriter : public IUnknown
{
	virtual void writeText(LPCWSTR aText)=0;
};

class TextInjectorMFT : public IMFTransform, public ITextWriter
{
public:
	TextInjectorMFT();
	virtual ~TextInjectorMFT();

	void writeText(LPCWSTR aText)override;


	STDMETHODIMP GetStreamLimits(DWORD* aPtrInputMinimum, DWORD* aPtrInputMaximum,
		DWORD* aPtrOutputMinimum, DWORD* aPtrOutputMaximum);

	STDMETHODIMP GetStreamIDs(DWORD aInputIDArraySize, DWORD* aPtrInputIDs,
		DWORD aOutputIDArraySize, DWORD* aPtrOutputIDs);

	STDMETHODIMP GetStreamCount(DWORD* aPtrInputStreams, DWORD* aPtrOutputStreams);

	STDMETHODIMP GetInputStreamInfo(DWORD aInputStreamID,
		MFT_INPUT_STREAM_INFO* aPtrStreamInfo);

	STDMETHODIMP GetOutputStreamInfo(DWORD aOutputStreamID,
		MFT_OUTPUT_STREAM_INFO* aPtrStreamInfo);

	STDMETHODIMP GetInputStreamAttributes(DWORD aInputStreamID,
		IMFAttributes** aPtrPtrAttributes);

	STDMETHODIMP GetOutputStreamAttributes(DWORD aOutputStreamID,
		IMFAttributes** aPtrPtrAttributes);

	STDMETHODIMP DeleteInputStream(DWORD aStreamID);

	STDMETHODIMP AddInputStreams(DWORD aStreams, DWORD* aPtrStreamIDs);


	STDMETHODIMP GetInputAvailableType(DWORD aInputStreamID, DWORD aTypeIndex,
		IMFMediaType** aPtrPtrType);

	STDMETHODIMP GetOutputAvailableType(DWORD aOutputStreamID, DWORD aTypeIndex,
		IMFMediaType** aPtrPtrType);

	STDMETHODIMP SetInputType(DWORD aInputStreamID, IMFMediaType* aPtrType,
		DWORD aFlags);

	STDMETHODIMP SetOutputType(DWORD aOutputStreamID, IMFMediaType* aPtrType,
		DWORD aFlags);

	STDMETHODIMP GetInputCurrentType(DWORD aInputStreamID, IMFMediaType** aPtrPtrType);

	STDMETHODIMP GetOutputCurrentType(DWORD aOutputStreamID, IMFMediaType** aPtrPtrType);


	STDMETHODIMP GetInputStatus(DWORD aInputStreamID, DWORD* aPtrFlags);

	STDMETHODIMP GetOutputStatus(DWORD* aPtrFlags);

	STDMETHODIMP SetOutputBounds(LONGLONG aLowerBound, LONGLONG aUpperBound);

	STDMETHODIMP ProcessEvent(DWORD aInputStreamID, IMFMediaEvent* aPtrEvent);

	STDMETHODIMP GetAttributes(IMFAttributes** aPtrPtrAttributes);


	STDMETHODIMP ProcessMessage(MFT_MESSAGE_TYPE aMessage, ULONG_PTR aParam);

	STDMETHODIMP ProcessInput(DWORD aInputStreamID, IMFSample* aPtrSample,
		DWORD aFlags);

	STDMETHODIMP ProcessOutput(DWORD aFlags, DWORD aOutputBufferCount,
		MFT_OUTPUT_DATA_BUFFER* aPtrOutputSamples, DWORD* aPtrStatus);


	virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID aRefIID, void** aPtrPtrVoidObject);
	virtual ULONG STDMETHODCALLTYPE AddRef(void);
	virtual ULONG STDMETHODCALLTYPE Release(void);


private:

	std::atomic<unsigned long> mRefCount;

	std::condition_variable mConditionVariable;

	std::mutex mMutex;

	CComPtrCustom<IMFSample>  m_pSample;           
	CComPtrCustom<IMFMediaType> m_pInputType;     
	CComPtrCustom<IMFMediaType> m_pOutputType;     

	CFrameParser m_frameParser;              

	HRESULT GetSupportedMediaType(DWORD aTypeIndex, IMFMediaType** aPtrPtrMediaType);

	HRESULT CheckMediaType(IMFMediaType* aPtrMediaType);
};

