#include <new>
#include <stdio.h>
#include <windows.h>
#include <Mfapi.h>


#include "RawImage.h"


RawImage::RawImage(unsigned int size): ri_new(false), ri_pixels(NULL)
{
	ri_size = size;

	ri_pixels.reset(new unsigned char[size]);

	memset((void *)ri_pixels.get(),0,ri_size);
}

bool RawImage::isNew()
{
	return ri_new;
}

unsigned int RawImage::getSize()
{
	return ri_size;
}

RawImage::~RawImage(void)
{
}

long RawImage::CreateInstance(RawImage **ppRImage,unsigned int size)
{
	*ppRImage = new (std::nothrow) RawImage(size);

    if (ppRImage == NULL)
    {
        return E_OUTOFMEMORY;
    }
    return S_OK;
}

void RawImage::setCopy(const BYTE * pSampleBuffer)
{
	memcpy(ri_pixels.get(), pSampleBuffer, ri_size);

	ri_new = true;
}

void RawImage::fastCopy(const BYTE * pSampleBuffer)
{
	int *bsrc = (int *)pSampleBuffer;

	int *dst = (int *)ri_pixels.get();

	unsigned int buffersize = ri_size/4;
	
	_asm
	{
		mov ESI, bsrc

		mov EDI, dst

		mov ECX, buffersize

		cld

		rep movsd
	}

	ri_new = true;

}

unsigned char * RawImage::getpPixels()
{
	return ri_pixels.get();

	ri_new = false;
}