#pragma once

#include "WriteSurface.h"
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
        ~CFrameParser(void);

        // Set the media type which contains the frame format.
        HRESULT SetFrameType(IMFMediaType* pMT);  

        // Pass in the sample with the video frame to modify.
        HRESULT LockFrame(IMFSample* pSmp);
        HRESULT UnlockFrame(void);

        // Draw the bitmap on the passed-in frame.
        HRESULT DrawBitmap(void);

		void writeText(LPCWSTR aText);

    private:
		CComPtrCustom<IMFMediaBuffer> m_pMediaBuffer;
		CComQIPtr<IMF2DBuffer> m_p2dBuffer;

        BYTE* m_pScanline0;
        GUID m_subtype;
        LONG m_stride;
        UINT32 m_imageWidthInPixels;
        UINT32 m_imageHeightInPixels;

        CWriteSurface* m_pWriteSurface;       // The bitmap to inject.

		HRESULT DrawBitmap_RGB32(CWriteSurface* pBmp);    // Draw bitmap on an RGB32 frame
};

