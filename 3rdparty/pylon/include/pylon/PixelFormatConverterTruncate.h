//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTERTRUNCATE_H_3141552
#define INCLUDED_PIXELFORMATCONVERTERTRUNCATE_H_3141552

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PixelFormatConverterMonoXX.h>

namespace Pylon
{
    class  PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterTruncate : public CPixelFormatConverterMonoXX
    {
    public:
        CPixelFormatConverterTruncate()
        {
            m_converter.MonoConversionMethod.SetValue(MonoConversionMethod_Truncate);
        }

        int GetHighOffsetBit() const
        {
            return (int)m_converter.AdditionalLeftShift.GetValue();
        }

        void SetHighOffsetBit( unsigned int highOffset )
        {
            m_converter.AdditionalLeftShift.SetValue(highOffset);
        }

        int GetRangeHighOffsetBit() const
        {
            return 16;
        }
    };
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTERTRUNCATE_H_3141552 */
