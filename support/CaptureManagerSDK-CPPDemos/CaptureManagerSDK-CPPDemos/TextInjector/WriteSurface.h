#pragma once
#include <Windows.h>
#include <Wingdi.h>
#include <iostream>


#include "FontMgr.h"


class CWriteSurface
{
    public:
		CWriteSurface();
        ~CWriteSurface(void);

        bool IsCreated(void) { return m_pWriteSurface != NULL; };

        // Get an RGB pixel from the specified coordinates.
		inline RGBQUAD* GetRgbPixel(DWORD x, DWORD y)
        {
            if(x < 0 || x >= m_width)
                return NULL;
            if(y < 0 || y >= m_width)
                return NULL;

            return &(m_pWriteSurface[y][x]);
        }

		bool writeText(LPCWSTR aText);

        // Get image dimensions.
        inline DWORD Width(void) { return m_width; }
        inline DWORD Height(void) { return m_height; }

    private:
		RGBQUAD** m_pWriteSurface;
        DWORD m_width;
        DWORD m_height;

		HDC mBitmapHDC;

		HBITMAP mBitmap;

		CFontMgr mFontMgr;

		RECT mClientArea;

		HRESULT Create();
        void ClearData(void);
};

