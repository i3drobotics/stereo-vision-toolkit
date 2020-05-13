#pragma once
#include <Windows.h>
#include <Wingdi.h>
#include <iostream>

// Helper structure defining the YUV format and byte positioning.
struct YUVTRIPLE
{
    BYTE Y;
    BYTE U;
    BYTE V;
};

//
// Helper class that holds the bitmap and converts the bitmap into a common format.
//
class CBmpFile
{
public:
		CBmpFile();
		CBmpFile(const WCHAR* filename);
        ~CBmpFile(void);

        bool ImageLoaded(void) { return m_pBmp != NULL; };

        // Get an RGB pixel from the specified coordinates.
        inline RGBTRIPLE* GetRgbPixel(DWORD x, DWORD y)
        {
            if(x < 0 || x >= m_width)
                return NULL;
            if(y < 0 || y >= m_width)
                return NULL;

            return &(m_pBmp[y][x]);
        }

        // Get a YUV pixel from the specified coordinates.
        inline YUVTRIPLE* GetYUVPixel(DWORD x, DWORD y)
        {
            if(x < 0 || x >= m_width)
                return NULL;
            if(y < 0 || y >= m_width)
                return NULL;

            return (YUVTRIPLE*)(&(m_pBmp[y][x]));
        }

        // Convert file into one format and precalculate the chroma.
        void ConvertToYuv(void);
        void PrecalcChroma_420(void);
        void PrecalcChroma_422(void);

        // Get image dimensions.
        inline DWORD Width(void) { return m_width; }
        inline DWORD Height(void) { return m_height; }

    private:
        RGBTRIPLE** m_pBmp;
        DWORD m_width;
		DWORD m_height;

		HRESULT ReadResource();
		HRESULT ReadFile(const WCHAR* filename);
        void ClearData(void);
};

