
#include "FrameParser.h"

#include <Mmsystem.h>
#include <uuids.h>


CFrameParser::CFrameParser(void)
{
	m_pWriteSurface = new (std::nothrow) CWriteSurface();
}

CFrameParser::~CFrameParser(void)
{
    if(m_pWriteSurface != NULL)
        delete m_pWriteSurface;
}

//
// Set the frame media type and stride, and precalculate the chroma values for the frame
//
HRESULT CFrameParser::SetFrameType(IMFMediaType* pType)
{
    HRESULT hr = S_OK;
    LONG lStride = 0;

    do
    {
        // reset the frame size information
        m_imageWidthInPixels = 0;
        m_imageHeightInPixels = 0;

        // if the media type is NULL, it means that the type is being cleared - reset 
        // internal variables.
        if(pType == NULL)
        {
            m_stride = 0;
            m_subtype = GUID_NULL;
        }

        // get the frame width and height in pixels from the media type
        hr = MFGetAttributeSize(pType, 
                                MF_MT_FRAME_SIZE, 
                                &m_imageWidthInPixels, 
								&m_imageHeightInPixels);

		if (FAILED(hr))
			break;

        // image dimensions must be divisible by 2
        if( m_imageWidthInPixels % 2 != 0 || m_imageHeightInPixels % 2 != 0)
        {
            hr = E_UNEXPECTED;
            break;
        }

        // Try to get the default stride from the media type.  A stride is the length of a 
        // single scan line in a frame in bytes - IE the number of bytes per pixel times the
        // width of a frame.  
        hr = pType->GetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32*)&m_stride);
        
        // if we failed to get the stride from the media type, we will try to get it by 
        // looking at the subtype
        if(FAILED(hr))
        {
            m_stride = 0;
        }

        // Get the subtype from the media type.  The first 4 bytes of the subtype GUID will
        // be the FOURCC code for this video format.
		hr = pType->GetGUID(MF_MT_SUBTYPE, &m_subtype);

		if (FAILED(hr))
			break;

        // precalculate the chroma values for the media type
        if(m_pWriteSurface != NULL && m_pWriteSurface->IsCreated())
        {
        }

        // if m_stride is zero, then we failed to get the stride from the media type.  In 
        // that case use the frame FOURCC type and width to calculate the expected stride 
        // (length of each pixel line).
        if(m_stride == 0)
        {
            hr = MFGetStrideForBitmapInfoHeader(m_subtype.Data1, m_imageWidthInPixels, 
				&lStride);

			if (FAILED(hr))
				break;
        }
    }
    while(false);

    return hr;
}


// 
// Lock and extract the sample buffer, ensuring that it will not be accessed by other components
//
HRESULT CFrameParser::LockFrame(IMFSample* pSmp)
{
    HRESULT hr = S_OK;
	CComPtrCustom<IMFSample> pSample;

    do
    {
		pSample = pSmp;

		if (!pSample)
		{
			hr = E_UNEXPECTED;

			break;
		}
		
        // convert the frame buffer to a single continuous block
		hr = pSample->ConvertToContiguousBuffer(&m_pMediaBuffer);

		if (FAILED(hr))
			break;

		if (!m_pMediaBuffer)
		{
			break;
		}

		//CComQIPtr<IMF2DBuffer> 1m_p2dBuffer;


		m_p2dBuffer = m_pMediaBuffer.get();

        if(!m_p2dBuffer)
        {
			hr = m_pMediaBuffer->Lock(&m_pScanline0, NULL, NULL);

			if (FAILED(hr))
				break;
        }  
		else
		{
			hr = m_p2dBuffer->Lock2D(&m_pScanline0, &m_stride);

			if (FAILED(hr))
				break;
		}
    }
    while(false);

    return hr;
}


HRESULT CFrameParser::UnlockFrame(void)
{
    HRESULT hr = S_OK;

    do
    {
        if(m_p2dBuffer != NULL)
        {
            hr = m_p2dBuffer->Unlock2D();
        }
        else if(m_pMediaBuffer.get() != NULL)
        {
            hr = m_pMediaBuffer->Unlock();
        }

        m_p2dBuffer.Release();
        m_pMediaBuffer.Release();

        m_pScanline0 = NULL;
    }
    while(false);

    return hr;
}

void CFrameParser::writeText(LPCWSTR aText)
{
	do
	{
		if (m_pWriteSurface == NULL || !m_pWriteSurface->IsCreated())
		{
			break;
		}

		m_pWriteSurface->writeText(aText);

	} while (false);

}

HRESULT CFrameParser::DrawBitmap(void)
{
    HRESULT hr = S_OK;

    do
    {
        if(m_pWriteSurface == NULL || !m_pWriteSurface->IsCreated())
        {
            break;
        }

        // if this is an NV12 frame, use the NV12 function.
        // if this is a UYVY frame, use the UYVY function.
		if (m_subtype == MFVideoFormat_RGB32)
        {
			hr = DrawBitmap_RGB32(m_pWriteSurface);
        }
        else
        {
            // didn't match any frame format - fail out
            hr = MF_E_INVALIDMEDIATYPE;
        }
    }
    while(false);

    return hr;
}

//
// Inject the specified image into the current UYVY-encoded frame.
//
HRESULT CFrameParser::DrawBitmap_RGB32(CWriteSurface* pBmp)
{
    HRESULT hr = S_OK;
	RGBQUAD* lineStart = NULL;
    DWORD imageWidthInMacroPixels = 0;

    do
    {
        lineStart = (RGBQUAD*)m_pScanline0;

  

        for(DWORD y = 0; y < pBmp->Height() && y < m_imageHeightInPixels; y++)
        {
			for (DWORD x = 0; x < (pBmp->Width() - 1) && x < m_imageWidthInPixels; x++)
            {
				RGBQUAD* lPixel = pBmp->GetRgbPixel(x, pBmp->Height() - 1 - y);

				if (lPixel->rgbBlue > 0 && lPixel->rgbGreen > 0 && lPixel->rgbRed > 0)
					lineStart[x] = *lPixel;
            }

            // the stride is specified in bytes - but we made the lineStart an array of 
            // macropixels.  Therefore we need to figure out by how many macropixels we 
            // need to move the lineStart pointer in order to point to the next line of 
            // pixels in the frame.
			lineStart += (m_stride / sizeof(RGBQUAD));
        }
    }
    while(false);

    return hr;
}
