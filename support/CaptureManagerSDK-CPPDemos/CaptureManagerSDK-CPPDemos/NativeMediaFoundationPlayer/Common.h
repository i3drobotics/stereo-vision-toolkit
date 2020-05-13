#pragma once

#include <new>
#include <atomic>
#include <windows.h>
#include <shobjidl.h> 
#include <shlwapi.h>
#include <assert.h>
#include <strsafe.h>

// Media Foundation headers
#include <mfapi.h>
#include <mfidl.h>
#include <mferror.h>
#include <evr.h>
#include "../Common/ComPtrCustom.h"

template <class T> void SafeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

template <typename... Interfaces>
struct BaseUnknown :
	public Interfaces...
{

public:

	BaseUnknown() :
		mRefCount(1){}

	// IUnknown implementation

	STDMETHODIMP QueryInterface(
		REFIID aRefIID,
		void** aPtrPtrVoidObject)
	{
		HRESULT lresult = S_OK;

		do
		{
			if (aPtrPtrVoidObject == NULL)
			{
				lresult = E_POINTER;

				break;
			}

			bool lboolResult = findInterface(aRefIID, aPtrPtrVoidObject);

			if (!lboolResult)
			{
				*aPtrPtrVoidObject = NULL;

				lresult = E_NOINTERFACE;
			}

		} while (false);

		if (SUCCEEDED(lresult))
		{
			AddRef();
		}

		return lresult;
	}

	STDMETHODIMP_(ULONG) AddRef()
	{
		return ++mRefCount;
	}

	STDMETHODIMP_(ULONG) Release()
	{
		auto lCount = --mRefCount;

		if (lCount == 0)
		{
			delete this;
		}

		return lCount;
	}

protected:

	virtual ~BaseUnknown(){}


	virtual bool findInterface(
		REFIID aRefIID,
		void** aPtrPtrVoidObject)
	{
		if (aRefIID == IID_IUnknown)
		{
			return castToIUnknow(
				aPtrPtrVoidObject,
				static_cast<Interfaces*>(this)...);
		}
		else
		{
			return castInterfaces(
				aRefIID,
				aPtrPtrVoidObject,
				static_cast<Interfaces*>(this)...);
		}
	}



	template<typename Interface>
	bool castInterfaces(
		REFIID aRefIID,
		void** aPtrPtrVoidObject,
		Interface* aThis)
	{
		bool lresult = aRefIID == __uuidof(Interface);

		if (lresult)
		{
			*aPtrPtrVoidObject = aThis;

			if (*aPtrPtrVoidObject == nullptr)
				lresult = false;
		}

		return lresult;
	}

private:
	std::atomic<ULONG> mRefCount;



	template<typename Interface, typename... Args>
	bool castToIUnknow(
		void** aPtrPtrVoidObject,
		Interface* aThis,
		Args... aRest)
	{
		return castToIUnknow(aPtrPtrVoidObject, aThis);
	}

	template<typename Interface>
	bool castToIUnknow(
		void** aPtrPtrVoidObject,
		Interface* aThis)
	{
		bool lresult = false;

		*aPtrPtrVoidObject = static_cast<IUnknown*>(aThis);

		if (*aPtrPtrVoidObject != nullptr)
			lresult = true;

		return lresult;
	}

	template<typename Interface, typename... Args>
	bool castInterfaces(
		REFIID aRefIID,
		void** aPtrPtrVoidObject,
		Interface* aThis,
		Args... aRest)
	{
		bool lresult = castInterfaces(aRefIID, aPtrPtrVoidObject, aThis);

		if (!lresult)
		{
			lresult = castInterfaces(
				aRefIID,
				aPtrPtrVoidObject,
				aRest...);
		}

		return lresult;
	}

};

template <typename... Interfaces>
struct BaseMFAttributes :
	public BaseUnknown<Interfaces...>
{

public:

	BaseMFAttributes(IMFAttributes* aPtrIMFAttributes)
	{
		mAttributes = aPtrIMFAttributes;
	}


	// IMFAttributes methods

	STDMETHODIMP GetItem(__RPC__in REFGUID guidKey, __RPC__inout_opt PROPVARIANT* pValue)
	{
		return mAttributes->GetItem(guidKey, pValue);
	}

	STDMETHODIMP GetItemType(__RPC__in REFGUID guidKey, __RPC__out MF_ATTRIBUTE_TYPE* pType)
	{
		return mAttributes->GetItemType(guidKey, pType);
	}

	STDMETHODIMP CompareItem(__RPC__in REFGUID guidKey, __RPC__in REFPROPVARIANT Value, __RPC__out BOOL* pbResult)
	{
		return mAttributes->CompareItem(guidKey, Value, pbResult);
	}

	STDMETHODIMP Compare(__RPC__in_opt IMFAttributes* pTheirs, MF_ATTRIBUTES_MATCH_TYPE MatchType, __RPC__out BOOL* pbResult)
	{
		return mAttributes->Compare(pTheirs, MatchType, pbResult);
	}

	STDMETHODIMP GetUINT32(__RPC__in REFGUID guidKey, __RPC__out UINT32* punValue)
	{
		return mAttributes->GetUINT32(guidKey, punValue);
	}

	STDMETHODIMP GetUINT64(__RPC__in REFGUID guidKey, __RPC__out UINT64* punValue)
	{
		return mAttributes->GetUINT64(guidKey, punValue);
	}

	STDMETHODIMP GetDouble(__RPC__in REFGUID guidKey, __RPC__out double* pfValue)
	{
		return mAttributes->GetDouble(guidKey, pfValue);
	}

	STDMETHODIMP GetGUID(__RPC__in REFGUID guidKey, __RPC__out GUID* pguidValue)
	{
		return mAttributes->GetGUID(guidKey, pguidValue);
	}

	STDMETHODIMP GetStringLength(__RPC__in REFGUID guidKey, __RPC__out UINT32* pcchLength)
	{
		return mAttributes->GetStringLength(guidKey, pcchLength);
	}

	STDMETHODIMP GetString(__RPC__in REFGUID guidKey, __RPC__out_ecount_full(cchBufSize) LPWSTR pwszValue, UINT32 cchBufSize, __RPC__inout_opt UINT32* pcchLength)
	{
		return mAttributes->GetString(guidKey, pwszValue, cchBufSize, pcchLength);
	}

	STDMETHODIMP GetAllocatedString(__RPC__in REFGUID guidKey, __RPC__deref_out_ecount_full_opt((*pcchLength + 1)) LPWSTR* ppwszValue, __RPC__out UINT32* pcchLength)
	{
		return mAttributes->GetAllocatedString(guidKey, ppwszValue, pcchLength);
	}

	STDMETHODIMP GetBlobSize(__RPC__in REFGUID guidKey, __RPC__out UINT32* pcbBlobSize)
	{
		return mAttributes->GetBlobSize(guidKey, pcbBlobSize);
	}

	STDMETHODIMP GetBlob(__RPC__in REFGUID guidKey, __RPC__out_ecount_full(cbBufSize) UINT8* pBuf, UINT32 cbBufSize, __RPC__inout_opt UINT32* pcbBlobSize)
	{
		return mAttributes->GetBlob(guidKey, pBuf, cbBufSize, pcbBlobSize);
	}

	STDMETHODIMP GetAllocatedBlob(__RPC__in REFGUID guidKey, __RPC__deref_out_ecount_full_opt(*pcbSize) UINT8** ppBuf, __RPC__out UINT32* pcbSize)
	{
		return mAttributes->GetAllocatedBlob(guidKey, ppBuf, pcbSize);
	}

	STDMETHODIMP GetUnknown(__RPC__in REFGUID guidKey, __RPC__in REFIID riid, __RPC__deref_out_opt LPVOID* ppv)
	{
		return mAttributes->GetUnknown(guidKey, riid, ppv);
	}

	STDMETHODIMP SetItem(__RPC__in REFGUID guidKey, __RPC__in REFPROPVARIANT Value)
	{
		return mAttributes->SetItem(guidKey, Value);
	}

	STDMETHODIMP DeleteItem(__RPC__in REFGUID guidKey)
	{
		return mAttributes->DeleteItem(guidKey);
	}

	STDMETHODIMP DeleteAllItems(void)
	{
		return mAttributes->DeleteAllItems();
	}

	STDMETHODIMP SetUINT32(__RPC__in REFGUID guidKey, UINT32 unValue)
	{
		return mAttributes->SetUINT32(guidKey, unValue);
	}

	STDMETHODIMP SetUINT64(__RPC__in REFGUID guidKey, UINT64 unValue)
	{
		return mAttributes->SetUINT64(guidKey, unValue);
	}

	STDMETHODIMP SetDouble(__RPC__in REFGUID guidKey, double fValue)
	{
		return mAttributes->SetDouble(guidKey, fValue);
	}

	STDMETHODIMP SetGUID(__RPC__in REFGUID guidKey, __RPC__in REFGUID guidValue)
	{
		return mAttributes->SetGUID(guidKey, guidValue);
	}

	STDMETHODIMP SetString(__RPC__in REFGUID guidKey, __RPC__in_string LPCWSTR wszValue)
	{
		return mAttributes->SetString(guidKey, wszValue);
	}

	STDMETHODIMP SetBlob(__RPC__in REFGUID guidKey, __RPC__in_ecount_full(cbBufSize) const UINT8* pBuf, UINT32 cbBufSize)
	{
		return mAttributes->SetBlob(guidKey, pBuf, cbBufSize);
	}

	STDMETHODIMP SetUnknown(__RPC__in REFGUID guidKey, __RPC__in_opt IUnknown* pUnknown)
	{
		return mAttributes->SetUnknown(guidKey, pUnknown);
	}

	STDMETHODIMP LockStore(void)
	{
		return mAttributes->LockStore();
	}

	STDMETHODIMP UnlockStore(void)
	{
		return mAttributes->UnlockStore();
	}

	STDMETHODIMP GetCount(__RPC__out UINT32* pcItems)
	{
		return mAttributes->GetCount(pcItems);
	}

	STDMETHODIMP GetItemByIndex(UINT32 unIndex, __RPC__out GUID* pguidKey, __RPC__inout_opt PROPVARIANT* pValue)
	{
		return mAttributes->GetItemByIndex(unIndex, pguidKey, pValue);
	}

	STDMETHODIMP CopyAllItems(__RPC__in_opt IMFAttributes* pDest)
	{
		return mAttributes->CopyAllItems(pDest);
	}



protected:

	virtual ~BaseMFAttributes(){}

	virtual bool findInterface(
		REFIID aRefIID,
		void** aPtrPtrVoidObject)
	{
		if (aRefIID == __uuidof(IMFAttributes))
		{
			return castInterfaces(
				aRefIID,
				aPtrPtrVoidObject,
				static_cast<IMFAttributes*>(this));
		}
		else
		{
			return BaseUnknown::findInterface(
				aRefIID,
				aPtrPtrVoidObject);
		}
	}

private:

	CComPtrCustom<IMFAttributes> mAttributes;
};