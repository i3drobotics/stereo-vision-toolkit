#include "BmpFile.h"
#include "resource.h"


CBmpFile::CBmpFile() :
m_pBmp(NULL),
m_width(0),
m_height(0)
{
	HRESULT hr = ReadResource();

	if (FAILED(hr))
	{
		ClearData();
	}
}

CBmpFile::CBmpFile(const WCHAR* filename) :
    m_pBmp(NULL),
    m_width(0),
    m_height(0)
{
    HRESULT hr = ReadFile(filename);

    if(FAILED(hr))
    {
        ClearData();
    }
}


CBmpFile::~CBmpFile(void)
{
    ClearData();
}


void CBmpFile::ClearData(void)
{
    DWORD height = m_height;

    m_width = 0;
    m_height = 0;

    if(m_pBmp != NULL && height > 0)
    {
        for(DWORD x = 0; x < m_height; x ++)
        {
            delete m_pBmp[x];
        }

        delete m_pBmp;
        m_pBmp = NULL;
    }
}


HRESULT CBmpFile::ReadResource()
{
	HRESULT hr = S_OK;
	bool isTopDownDib = false;
	int nCurrentPixelLine = 0;

	do
	{
		HBITMAP m_hBitmap = (HBITMAP)LoadImage(GetModuleHandle(NULL), MAKEINTRESOURCE(IDB_BITMAP1),
			IMAGE_BITMAP, 0, 0, LR_CREATEDIBSECTION);

		if (m_hBitmap == NULL)
		{
			return E_POINTER;
		}

		BITMAP Bitmap;
		
		int error = GetObject(m_hBitmap, sizeof(BITMAP), &Bitmap); 

		m_width = Bitmap.bmWidth;
		m_height = Bitmap.bmHeight;

		if (m_height < 0)
		{
			isTopDownDib = true;
			m_height *= -1;
		}

		
		m_pBmp = new (std::nothrow) RGBTRIPLE*[m_height];

		if (m_pBmp == NULL)
		{
			hr = E_OUTOFMEMORY;

			break;
		}

		if (isTopDownDib)
		{
			nCurrentPixelLine = 0;
		}
		else
		{
			nCurrentPixelLine = m_height - 1;
		}

		BYTE* lPtrBits = (BYTE*)Bitmap.bmBits;

		// read the pixel lines one by one, and store them in the m_pBmp array
		for (DWORD x = 0; x < m_height; x++)
		{
			RGBTRIPLE* pixelLine = new (std::nothrow) RGBTRIPLE[m_width];

			if (pixelLine == NULL)
			{
				return E_OUTOFMEMORY;
			}

			RGBTRIPLE* lPtrPixel = (RGBTRIPLE*)lPtrBits;

			for (size_t i = 0; i < m_width; i++)
			{
				pixelLine[i] = lPtrPixel[i];
			}

			lPtrBits += Bitmap.bmWidthBytes;


			m_pBmp[nCurrentPixelLine] = pixelLine;


			// if this is a top-down DIB, then the next line will go down in the array - increment
			// the current pixel line counter.  If this is a bottom-up DIB, then we are reading data
			// from the bottom to the top - therefore decriment the counter.
			if (isTopDownDib)
			{
				nCurrentPixelLine++;
			}
			else
			{
				nCurrentPixelLine--;
			}
		}
	} while (false);
	
	return hr;
}

HRESULT CBmpFile::ReadFile(const WCHAR* filename)
{
    HRESULT hr = S_OK;
    FILE* bmpFile = NULL;
    BITMAPFILEHEADER bmpFileHeader = {0};
    BITMAPINFOHEADER bmpInfo = {0};
    UINT nBytesRead = 0;
    DWORD offsetToData = 0;
    bool isTopDownDib = false;
    int nCurrentPixelLine = 0;
    DWORD padding = 0;

    do
    {
        _wfopen_s(&bmpFile, filename, L"rb");

        if(bmpFile == NULL)
        {
            return E_POINTER;
        }

        nBytesRead = (DWORD)fread(&bmpFileHeader, sizeof(BITMAPFILEHEADER), 1, bmpFile);
        if(nBytesRead != 1)
            return E_FAIL;

        nBytesRead = (DWORD)fread(&bmpInfo, sizeof(BITMAPINFOHEADER), 1, bmpFile);
        if(nBytesRead != 1)
            return E_FAIL;

        // this class handles only basic 24-bit BMP files
        if(bmpInfo.biBitCount != 24)
            return E_FAIL;

        // accept only uncompressed bitmaps - no support for JPG, PNG, etc.
        if(bmpInfo.biCompression != BI_RGB)
            return E_FAIL;

        offsetToData = bmpFileHeader.bfOffBits;
        m_width = bmpInfo.biWidth;
        m_height = bmpInfo.biHeight;

        // DIBs come in two major flavors - bottom-up, or top-down.  In bottom-up DIBs the first
        // pixel encountered is the bottom left one - IE they are vertically inverted.  In top-down 
        // DIBs the first pixel is the top-left one in the image.  To indicate this, top-down DIBs
        // have a negative height value, and bottom-up have a positive height.
        if(m_height < 0)
        {
            isTopDownDib = true;
            m_height *= -1;
        }


        // seek to the beginning of the actual pixel data in the BMP file
        fseek(bmpFile, offsetToData, SEEK_SET);

        // calculate the padding on every line - a BMP line of pixels is padded at the
        // end in such a way that all of the pixels + padding come out to a multiple of 4
        padding = 4 - ((m_width * sizeof(RGBTRIPLE)) % 4);

        m_pBmp = new (std::nothrow) RGBTRIPLE*[m_height];

		if (m_pBmp == NULL)
		{
			hr = E_OUTOFMEMORY;

			break;
		}

        if(isTopDownDib)
        {
            nCurrentPixelLine = 0;
        }
        else
        {
            nCurrentPixelLine = m_height - 1;
        }
		
        // read the pixel lines one by one, and store them in the m_pBmp array
        for(DWORD x = 0; x < m_height; x++)
        {
			RGBTRIPLE* pixelLine = new (std::nothrow) RGBTRIPLE[m_width];

			if (pixelLine == NULL)
			{
				return E_OUTOFMEMORY;
			}

            nBytesRead = (DWORD)fread(pixelLine, sizeof(RGBTRIPLE), m_width, bmpFile);

            // if we didn't read all of the data, something must have gone wrong - fail out
            if(nBytesRead < m_width)
            {
                return E_UNEXPECTED;
            }

            m_pBmp[nCurrentPixelLine] = pixelLine;

            // skip the padding bytes that are used to make the pixel line take a multiple of 
            // four bytes
            fseek(bmpFile, padding, SEEK_CUR);

            // if this is a top-down DIB, then the next line will go down in the array - increment
            // the current pixel line counter.  If this is a bottom-up DIB, then we are reading data
            // from the bottom to the top - therefore decriment the counter.
            if(isTopDownDib)
            {
                nCurrentPixelLine++;
            }
            else
            {
                nCurrentPixelLine--;
            }
        }
    }
    while(false);

    if(bmpFile != NULL)
        fclose(bmpFile);

    return hr;
}


//
// Convert all of the RGB pixels in the image into the YUV format
//
void CBmpFile::ConvertToYuv(void)
{
    if( m_width == 0 || m_height == 0 || m_pBmp == NULL )
        return;

    // explicitly cast the array into YUVTRIPLE items in order to make sure we never
    // mix up Y, U, and V order in the array.  The order is arbitrary, as long as everyone
    // uses it in the same way - while setting, and while reading the values.
    YUVTRIPLE** pYuv = (YUVTRIPLE**)m_pBmp;

    for(DWORD y = 0; y < m_height; y++)
    {
        for(DWORD x = 0; x < m_width; x++)
        {
            // store the RGB data in temporary variables since it will be modified
            short R = m_pBmp[y][x].rgbtRed;
            short G = m_pBmp[y][x].rgbtGreen;
            short B = m_pBmp[y][x].rgbtBlue;

            // use integer calculations to derive the Y, U, and V values for the 
            // YUV format of every single pixel.  This essentially converts the
            // data from 4:4:4 RGB to 4:4:4 YUV
            pYuv[y][x].Y = ( (  66 * R + 129 * G +  25 * B + 128) >> 8) +  16;    // Y
            pYuv[y][x].U = ( ( -38 * R -  74 * G + 112 * B + 128) >> 8) + 128;    // U
            pYuv[y][x].V =  ( ( 112 * R -  94 * G -  18 * B + 128) >> 8) + 128;   // V
        }
    }
}

//
// Smooth out chroma for 4:2:0 format
//
void CBmpFile::PrecalcChroma_420(void)
{
    if( m_width == 0 || m_height == 0 || m_pBmp == NULL )
        return;

    // explicitly cast the array into YUVTRIPLE items in order to make sure we never
    // mix up Y, U, and V order in the array.  The order is arbitrary, as long as everyone
    // uses it in the same way - while setting, and while reading the values.
    YUVTRIPLE** pYuv = (YUVTRIPLE**)m_pBmp;

    for(DWORD y = 0; y < (m_height - 1); y+=2)
    {
        for(DWORD x = 0; x < (m_width - 1); x+=2)
        {
            float uSum = 0;
            float vSum = 0;

            // add up the U and V portions of every pixel
            uSum = (float)(pYuv[y][x].U + pYuv[y][x+1].U + pYuv[y+1][x].U + pYuv[y+1][x+1].U);
            vSum = (float)(pYuv[y][x].V + pYuv[y][x+1].V + pYuv[y+1][x].V + pYuv[y+1][x+1].V);

            // Since a single chroma value for 4:2:0 format represents four pixels
            // at once (the same color is used for every four pixels) set the chroma
            // values of all of the pixels to the calculated average.
            pYuv[y][x+1].U = pYuv[y+1][x].U = pYuv[y+1][x+1].U = pYuv[y][x].U = (char)(uSum / 4);
            pYuv[y][x+1].V = pYuv[y+1][x].V = pYuv[y+1][x+1].V = pYuv[y][x].V = (char)(vSum / 4);
        }
    }
}


//
// Smooth out chroma for 4:2:2 format
//
void CBmpFile::PrecalcChroma_422(void)
{
    if( m_width == 0 || m_height == 0 || m_pBmp == NULL )
        return;

    // explicitly cast the array into YUVTRIPLE items in order to make sure we never
    // mix up Y, U, and V order in the array.  The order is arbitrary, as long as everyone
    // uses it in the same way - while setting, and while reading the values.
    YUVTRIPLE** pYuv = (YUVTRIPLE**)m_pBmp;

    for(DWORD y = 0; y < m_height; y++)
    {
        for(DWORD x = 0; x < (m_width - 1); x+=2)
        {
            float uSum = 0;
            float vSum = 0;

            // add up the U and V portions of every pixel
            uSum = (float)(pYuv[y][x].U + pYuv[y][x+1].U);
            vSum = (float)(pYuv[y][x].V + pYuv[y][x+1].V);

            // Since a single chroma value for 4:2:2 format represents four pixels
            // at once (the same color is used for every four pixels) set the chroma
            // values of all of the pixels to the calculated average.
            pYuv[y][x].U = pYuv[y][x+1].U = (char)(uSum / 2);
            pYuv[y][x].V = pYuv[y][x+1].V = (char)(vSum / 2);
        }
    }
}