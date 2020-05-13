
#include "FrameParser.h"

#include <Mmsystem.h>
#include <uuids.h>

// A struct representing the chroma information about four pixels in the NV12 format.
// In NV12 chroma is stored as a set of interleaved U and V values, and each pair is
// used for four final pixels in the frame.  Luma is stored in a separate array.
struct NV12_CHROMA
{
    BYTE U;
    BYTE V;
};


// A struct representing a single YUY2 format macropixel.  The macropixel describes
// two pixels in the final frame.  Each pixel has its own luma value (Y1 and Y2), but
// both share the same chroma (U and V).
struct UYVY_MACRO_PIXEL
{
    BYTE U;
    BYTE Y1;
    BYTE V;
    BYTE Y2;
};


CFrameParser::CFrameParser(void)
{
    m_pBmp = NULL;
}


CFrameParser::CFrameParser(const WCHAR* filename)
{
    m_pBmp = new (std::nothrow) CBmpFile(filename);
}


CFrameParser::~CFrameParser(void)
{
    if(m_pBmp != NULL)
        delete m_pBmp;
}



HRESULT CFrameParser::SetBitmap()
{
	if (m_pBmp != NULL)
		delete m_pBmp;

	m_pBmp = new (std::nothrow) CBmpFile();

	if (m_pBmp->ImageLoaded())
	{
		return S_OK;
	}
	else
	{
		return E_UNEXPECTED;
	}
}

HRESULT CFrameParser::SetBitmap(const WCHAR* filename)
{
    if(m_pBmp != NULL)
        delete m_pBmp;

    m_pBmp = new (std::nothrow) CBmpFile(filename);

    if(m_pBmp->ImageLoaded())
    {
        return S_OK;
    }
    else
    {
        return E_UNEXPECTED;
    }
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
        if(m_pBmp != NULL && m_pBmp->ImageLoaded())
        {
            m_pBmp->ConvertToYuv();
            
            if(m_subtype == MEDIASUBTYPE_UYVY)
            {
                m_pBmp->PrecalcChroma_422();
            }
            else if(m_subtype == MEDIASUBTYPE_NV12)
            {
                m_pBmp->PrecalcChroma_420();
            }
            else
            {
                hr = MF_E_INVALIDMEDIATYPE;
                break;
            }
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

HRESULT CFrameParser::DrawBitmap(void)
{
    HRESULT hr = S_OK;

    do
    {
        if(m_pBmp == NULL || !m_pBmp->ImageLoaded())
        {
            break;
        }

        // if this is an NV12 frame, use the NV12 function.
        // if this is a UYVY frame, use the UYVY function.
        if(m_subtype == MEDIASUBTYPE_NV12)
        {
            hr = DrawBitmap_NV12(m_pBmp);
        }
        else if(m_subtype == MEDIASUBTYPE_UYVY)
        {
            hr = DrawBitmap_UYVY(m_pBmp);
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
// Inject the specified image into the current NV12 encoded frame.
//
HRESULT CFrameParser::DrawBitmap_NV12(CBmpFile* pBmp)
{
    HRESULT hr = S_OK;
    BYTE* lumaLineStart = m_pScanline0;
    NV12_CHROMA* chromaLineStart = NULL;
    DWORD chromaStride = 0;

    do
    {        
        // in NV12 the chroma is stored as interleaved U and V values in an array 
        // immediately following the array of Y values.  Therefore, to get to the UV array 
        // we need to skip past the Y array - IE width of each pixel line (m_stride) times 
        // the number of pixel lines in the image (m_imageHeightInPixels)
        chromaLineStart = (NV12_CHROMA*)(lumaLineStart + (m_stride * m_imageHeightInPixels));

        // Because NV12 is a 4:2:0 format, the chroma is at half of the vertical resolution 
        // of the luma, and at half of the vertical resolution of the luma.  This means that 
        // the chroma stride is half of the luma stride.  Note that this is still true 
        // despite the values being stored in two-byte U,V pairs, since the chroma is 
        // treated as an array of two-byte variables
        chromaStride = m_stride / 2;

        // iterate through every pixel of the image/frame as long as we are not outside
        // the bounds of the frame or of the image
        for(DWORD y = 0; y < pBmp->Height() && y < m_imageHeightInPixels; y++)
        {
            for(DWORD x = 0; x < pBmp->Width() && x < m_imageWidthInPixels; x++)
            {
                // extract the YUV pixel of the image
                YUVTRIPLE* yuvPixel = pBmp->GetYUVPixel(x, y);
                
                // set the luma of the target pixel
                lumaLineStart[x] = yuvPixel->Y;

                // Because the chroma is at half vertical and horizontal resolution of the 
                // luma, we need to set the chroma pixels only for every 2 vertical and 2 
                // horizontal pixels.  Therefore set the chroma only if the x and y 
                // coordinates of the pixel are not divisible by 2.
                if(x%2 == 0  &&  y%2 == 0)
                {
                    chromaLineStart[x / 2].U = yuvPixel->U;
                    chromaLineStart[x / 2].V = yuvPixel->V;
                }
            }

            // go to the next line in the luma array
            lumaLineStart += m_stride;

            // go to the next line of the chroma array if we need to
            if(y%2 == 0)
            {
                chromaLineStart += chromaStride;
            }
        }
    }
    while(false);

    return hr;
}



//
// Inject the specified image into the current UYVY-encoded frame.
//
HRESULT CFrameParser::DrawBitmap_UYVY(CBmpFile* pBmp)
{
    HRESULT hr = S_OK;
    UYVY_MACRO_PIXEL* lineStart = NULL;
    DWORD imageWidthInMacroPixels = 0;

    do
    {
        lineStart = (UYVY_MACRO_PIXEL*)m_pScanline0;

        // each macro pixel represents two actual pixels on the screen, with two
        // luma samples (Y1 and Y2), and one chroma sample (U + V).  Therefore,
        // each line of the image array will contain widthInPixels/2 macropixels
        imageWidthInMacroPixels = m_imageWidthInPixels / 2;

        for(DWORD y = 0; y < pBmp->Height() && y < m_imageHeightInPixels; y++)
        {
            for(DWORD x = 0; x < (pBmp->Width() - 1) && x < imageWidthInMacroPixels; x+=2)
            {
                // extract two YUV pixels of the image
                YUVTRIPLE* yuvImagePixel1 = pBmp->GetYUVPixel(x, y);
                YUVTRIPLE* yuvImagePixel2 = pBmp->GetYUVPixel(x+1, y);

                // extract a single macro pixel from the frame
                UYVY_MACRO_PIXEL* framePixel = &(lineStart[x/2]);

                // set the luma pixel values in the frame pixel
                framePixel->Y1 = yuvImagePixel1->Y;
                framePixel->Y2 = yuvImagePixel2->Y;
                
                // set the chroma values in the frame pixel
                framePixel->U = yuvImagePixel1->U;
                framePixel->V = yuvImagePixel1->V;
            }

            // the stride is specified in bytes - but we made the lineStart an array of 
            // macropixels.  Therefore we need to figure out by how many macropixels we 
            // need to move the lineStart pointer in order to point to the next line of 
            // pixels in the frame.
            lineStart += (m_stride / sizeof(UYVY_MACRO_PIXEL));
        }
    }
    while(false);

    return hr;
}
