//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTERBAYER_H_4787496
#define INCLUDED_PIXELFORMATCONVERTERBAYER_H_4787496

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PixelFormatConverter.h>

namespace Pylon
{
    class  PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterBayer : public CPixelFormatConverter
    {
    public:
        CPixelFormatConverterBayer()
        {
            m_converter.OutputPixelFormat.SetValue( PixelType_BGRA8packed);
        }
    };
}


#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTERBAYER_H_4787496 */
