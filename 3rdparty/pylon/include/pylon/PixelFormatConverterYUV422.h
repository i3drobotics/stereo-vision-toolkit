//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTERYUV422_H_8946192
#define INCLUDED_PIXELFORMATCONVERTERYUV422_H_8946192

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PixelFormatConverter.h>

namespace Pylon
{
    class  PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterYUV422 : public CPixelFormatConverter
    {
    public:
        CPixelFormatConverterYUV422()
        {
            m_converter.OutputPixelFormat.SetValue( PixelType_BGRA8packed);
        }
    };

    class  PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterYUV422YUYV : public CPixelFormatConverter
    {
    public:
        CPixelFormatConverterYUV422YUYV()
        {
            m_converter.OutputPixelFormat.SetValue( PixelType_BGRA8packed);
        }
    };
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTERYUV422_H_8946192 */
