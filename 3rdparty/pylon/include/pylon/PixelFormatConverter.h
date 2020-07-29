//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  The class contained in this file has been deprecated. Use the Pylon::CImageFormatConverter class instead.
*/

#ifndef INCLUDED_PIXELFORMATCONVERTER_H_9769251
#define INCLUDED_PIXELFORMATCONVERTER_H_9769251

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/ImageFormatConverter.h>
#include <pylon/ImageFormat.h>

namespace Pylon
{
    class //PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.")
        CPixelFormatConverter
    {
    protected:
        // This class has been deprecated. Use the CImageFormatConverter class instead. See the migration guide section in the pylon programmer's guide for c++ for more information.
        CPixelFormatConverter()
            : m_isInitialized( false)
        {
        }

    public:
        virtual void Init( const struct SImageFormat &imageFormat)
        {
            m_converter.Initialize( imageFormat.PixelFormat);
            m_isInitialized = true;
        }

#ifdef __GNUC__
        virtual ~CPixelFormatConverter()
        {
        }
#endif

        bool IsInitialized( )  const
        {
            return m_isInitialized;
        }

        virtual void Convert( void *pDest,
                      unsigned int sizeDest,
                      const void *pSrcRaw,
                      unsigned int sizeRaw,
                      const struct SImageFormat &imageFormat,
                      const struct SOutputImageFormat &outputImageFormat )
        {
            m_converter.OutputPaddingX = ComputePaddingValue( SImageFormat( m_converter.OutputPixelFormat.GetValue(), imageFormat.Width, imageFormat.Height, outputImageFormat.LinePitch));
            m_converter.Convert(
                pDest,
                sizeDest,
                pSrcRaw,
                sizeRaw,
                imageFormat.PixelFormat,
                static_cast<uint32_t>(imageFormat.Width),
                static_cast<uint32_t>(imageFormat.Height),
                ComputePaddingValue( imageFormat),
                ImageOrientation_TopDown
                );
        }

        void SetOutputBitAlignment( OutputBitAlignmentEnums alignment)
        {
            m_converter.OutputBitAlignment = alignment;
        }

    private:

        size_t ComputePaddingValue( const SImageFormat& imageFormat)
        {
            if ( imageFormat.LinePitch)
            {
                size_t paddingX = Pylon::ComputePaddingX( imageFormat.LinePitch, imageFormat.PixelFormat, imageFormat.Width);
                return paddingX;
            }

            return 0;
        }

        CPixelFormatConverter( const CPixelFormatConverter& src );
        CPixelFormatConverter& operator=( const CPixelFormatConverter& src );
    protected:
        CImageFormatConverter m_converter;
        bool m_isInitialized;
    };
}


#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_PIXELFORMATCONVERTER_H_9769251 */
