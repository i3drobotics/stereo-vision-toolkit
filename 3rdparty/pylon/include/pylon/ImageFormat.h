//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2007-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------
/*!
    \file
    \brief  Contains deprecated items required by deprecated CPixelFormatConverter based classes.
*/

#ifndef __PYLON_ImageFormat_H_061212_061215_IS_INCLUDED__
#define __PYLON_ImageFormat_H_061212_061215_IS_INCLUDED__

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PylonUtility.h>
#include <pylon/PixelType.h>
#include <pylon/Pixel.h>

namespace Pylon
{
    //This class has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    struct SImageFormat
    {
        SImageFormat()
            : Width(0)
            , Height(0)
            , LinePitch(0)
            , PixelFormat(PixelType_Undefined)
        {
        }

        SImageFormat(
            EPixelType  aPixelFormat,
            int             aWidth,
            int             aHeight,
            int             aLinePitch = 0
        )
            : Width(aWidth)
            , Height(aHeight)
            , LinePitch(aLinePitch)
            , PixelFormat(aPixelFormat)
        {
        }

        ~SImageFormat()
        {
        }

        bool IsValid() const
        {
            if (
                   PixelType_Undefined != PixelFormat
                && Width > 0
                && Height > 0
                && LinePitch >= 0
                )
            {
                return true;
            }
            return false;
        }

        int             Width;
        int             Height;
        int             LinePitch;
        EPixelType  PixelFormat;
    };

    //This function has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    inline bool operator==(const SImageFormat& s1, const SImageFormat& s2)
    {
        bool equal= s1.Height == s2.Height
                 && s1.Width == s2.Width
                 && s1.LinePitch == s2.LinePitch
                 && s1.PixelFormat == s2.PixelFormat;

        return equal;
    }

    //This function has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    inline bool operator!=(const SImageFormat& s1, const SImageFormat& s2)
    {
        return ! ( s1 == s2 );
    }


    //This class has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    struct SOutputImageFormat
    {

        SOutputImageFormat()
            : LinePitch(0)
            , PixelFormat(PixelType_Undefined)
        {
        }

        SOutputImageFormat(
            EPixelType  aPixelFormat,
            int             aLinePitch = 0 //optional
        )
            : LinePitch(aLinePitch)
            , PixelFormat(aPixelFormat)
        {
        }

        SOutputImageFormat( const SImageFormat& imageFormat)
            : LinePitch(imageFormat.LinePitch)
            , PixelFormat(imageFormat.PixelFormat)
        {
        }


        ~SOutputImageFormat()
        {
        }


        bool IsValid() const
        {
            if (   PixelType_Undefined!=PixelFormat
                && LinePitch >= 0
            )
            {
                return true;
            }
            return false;
        }


        int             LinePitch;
        EPixelType  PixelFormat;

    };

    //This function has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    inline bool operator==(const SOutputImageFormat& s1, const SOutputImageFormat& s2)
    {
        return ( s1.LinePitch == s2.LinePitch && s1.PixelFormat == s2.PixelFormat );
    }

    //This function has been deprecated. It is only required by deprecated CPixelFormatConverter based classes.
    inline bool operator!=(const SOutputImageFormat& s1, const SOutputImageFormat& s2)
    {
        return ! ( s1 == s2 );
    }

#pragma pack( push)
#pragma pack( 1)

    // This class has been deprecated. Use the class Pylon::SBGRA8Pixel instead.
    struct PYLON_UTILITY_3_0_DEPRECATED("This class has been deprecated. Use the class Pylon::SBGRA8Pixel instead.")  SRGBAPixel
    {
        unsigned char B;
        unsigned char G;
        unsigned char R;
        unsigned char A;
    };

#pragma pack( pop)
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif // __PYLON_ImageFormat_H_061212_061215_IS_INCLUDED__
