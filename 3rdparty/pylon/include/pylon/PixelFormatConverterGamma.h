//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTERGAMMA_H_02221316
#define INCLUDED_PIXELFORMATCONVERTERGAMMA_H_02221316

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PixelFormatConverterMonoXX.h>

namespace Pylon
{
    class  PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverterGamma : public CPixelFormatConverterMonoXX
    {
    public:
        CPixelFormatConverterGamma()
        {
            m_converter.MonoConversionMethod.SetValue(MonoConversionMethod_Gamma);
        }

        double GetGamma() const
        {
            return m_converter.Gamma.GetValue();
        }

        void SetGamma( double gamma)
        {
            m_converter.Gamma.SetValue( gamma);
        }
    };
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTERGAMMA_H_02221316 */
