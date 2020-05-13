#pragma once

#include "BmpFile.h"
#include "MF.h"
#include "../Common/ComPtrCustom.h"
#include <atlbase.h>
#include <atlstr.h>

//
// Helper class that processes passed-in uncompressed frames and draws bitmaps on them.
//
class CFrameParser
{
    public:
        CFrameParser(void);
		CFrameParser(const WCHAR* filename);
        ~CFrameParser(void);

        // Set the media type which contains the frame format.
        HRESULT SetFrameType(IMFMediaType* pMT);  

        // Pass in the sample with the video frame to modify.
        HRESULT LockFrame(IMFSample* pSmp);
        HRESULT UnlockFrame(void);

        // Draw the bitmap on the passed-in frame.
		HRESULT DrawBitmap(void);

		HRESULT SetBitmap();

        // Load the bitmap from the file.
		HRESULT SetBitmap(const WCHAR* filename);

    private:
		CComPtrCustom<IMFMediaBuffer> m_pMediaBuffer;
		CComQIPtr<IMF2DBuffer> m_p2dBuffer;

        BYTE* m_pScanline0;
        GUID m_subtype;
        LONG m_stride;
        UINT32 m_imageWidthInPixels;
        UINT32 m_imageHeightInPixels;

        CBmpFile* m_pBmp;       // The bitmap to inject.

        HRESULT DrawBitmap_NV12(CBmpFile* pBmp);    // Draw bitmap on an NV12 frame.
        HRESULT DrawBitmap_UYVY(CBmpFile* pBmp);    // Draw bitmap on a UYVY frame.
};

