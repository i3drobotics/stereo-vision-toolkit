//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTERMONOPACKED_H_9365961
#define INCLUDED_PIXELFORMATCONVERTERMONOPACKED_H_9365961

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PixelFormatConverter.h>

namespace Pylon
{
    class  //PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterMonoPacked : public CPixelFormatConverter
    {
    protected:
        // This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.
        CPixelFormatConverterMonoPacked()
        {
            m_converter.OutputPixelFormat.SetValue( PixelType_Mono8);
        }
    public:
        //  Unpacks data from Mono12packed to Mono16 (short).
        static void PYLON_UTILITY_3_0_DEPRECATED("This method has been deprecated. Warning: This method uses more processing time than the one in pylon 2.x.")
            Unpack12(   unsigned short *pDest16,        // Pointer to destination buffer.
                                const unsigned char *pSrc,      // Pointer to source buffer.
                                int nPixels                     // Number of Pixel to be unpacked.
                                )
        {
            CImageFormatConverter converter; //CImageFormatConverter creation causes additional processing time.
            converter.MonoConversionMethod = MonoConversionMethod_Truncate;
            converter.OutputBitAlignment = OutputBitAlignment_LsbAligned;
            converter.AdditionalLeftShift = 0;
            converter.OutputPaddingX = 0;
            converter.OutputPixelFormat.SetValue( PixelType_Mono16);
            converter.Convert( pDest16, nPixels * 2, pSrc, nPixels * 2, PixelType_Mono12packed, nPixels, 1, 0, ImageOrientation_TopDown);
        }


        //  Unpacks data from Mono10packed to Mono16 (short).
        static void PYLON_UTILITY_3_0_DEPRECATED("This method has been deprecated. Warning: This method uses more processing time than the one in pylon 2.x.")
            Unpack10(  unsigned short *pDest16,        // Pointer to destination buffer.
                                const unsigned char *pSrc,     // Pointer to source buffer.
                                int nPixels                    // Number of Pixel to be unpacked.
                                )
        {
            CImageFormatConverter converter; //CImageFormatConverter creation causes additional processing time.
            converter.MonoConversionMethod = MonoConversionMethod_Truncate;
            converter.OutputBitAlignment = OutputBitAlignment_LsbAligned;
            converter.AdditionalLeftShift = 0;
            converter.OutputPaddingX = 0;
            converter.OutputPixelFormat.SetValue( PixelType_Mono16);
            converter.Convert( pDest16, nPixels * 2, pSrc, nPixels * 2, PixelType_Mono10packed, nPixels, 1, 0, ImageOrientation_TopDown);
        }

    };
}


#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTERMONOPACKED_H_9365961 */

