/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this header file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        VmbTransformTypes.h

  Description: Definition of types used in the Vimba Image Transform library.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#ifndef VMB_TRANSFORM_TYPES_H_
#define VMB_TRANSFORM_TYPES_H_
#include "VmbCommonTypes.h"


    typedef VmbInt8_t*              VmbInt8Ptr_t;
    typedef VmbInt16_t*             VmbInt16Ptr_t;
    typedef VmbInt32_t*             VmbInt32Ptr_t;
    typedef VmbInt64_t*             VmbInt64Ptr_t;

    typedef VmbUint8_t*             VmbUint8Ptr_t;
    typedef VmbUint16_t*            VmbUint16Ptr_t;
    typedef VmbUint32_t*            VmbUint32Ptr_t;
    typedef VmbUint64_t*            VmbUint64Ptr_t;

    typedef const VmbInt8_t*        VmbConstInt8Ptr_t;
    typedef const VmbInt16_t*       VmbConstInt16Ptr_t;
    typedef const VmbInt32_t*       VmbConstInt32Ptr_t;
    typedef const VmbInt64_t*       VmbConstInt64Ptr_t;

    typedef const VmbUint8_t*       VmbConstUint8Ptr_t;
    typedef const VmbUint16_t*      VmbConstUint16Ptr_t;
    typedef const VmbUint32_t*      VmbConstUint32Ptr_t;
    typedef const VmbUint64_t*      VmbConstUint64Ptr_t;

    typedef char                    VmbANSIChar_t;
    
    typedef float                   VmbFloat_t;
    typedef VmbFloat_t*             VmbFloatPtr_t;
    typedef const VmbFloat_t*       VmbConstFloatPtr_t;

    typedef void*                   VmbVoidPtr_t;
    typedef const void *            VmbConstVoidPtr_t;
    typedef VmbVoidPtr_t            VmbColorCorrectionHandle_t;

//! Enumeration for the Bayer pattern
typedef enum VmbBayerPattern
{
    VmbBayerPatternRGGB=0,     //!< RGGB pattern, red pixel comes first
    VmbBayerPatternGBRG,       //!< RGGB pattern, green pixel of blue row comes first
    VmbBayerPatternGRBG,       //!< RGGB pattern, green pixel of red row comes first
    VmbBayerPatternBGGR,       //!< RGGB pattern, blue pixel comes first
    VmbBayerPatternCYGM=128,   //!< CYGM pattern, cyan pixel comes first in the first row, green in the second row (of the sensor)
    VmbBayerPatternGMCY,       //!< CYGM pattern, green pixel comes first in the first row, cyan in the second row (of the sensor)
    VmbBayerPatternCYMG,       //!< CYGM pattern, cyan pixel comes first in the first row, magenta in the second row (of the sensor)
    VmbBayerPatternMGCY,       //!< CYGM pattern, magenta pixel comes first in the first row, cyan in the second row (of the sensor)
    VmbBayerPatternLAST=255
} VmbBayerPattern;
typedef VmbUint32_t  VmbBayerPattern_t;

//! Enumeration for the endianness
typedef enum VmbEndianness
{
    VmbEndiannessLittle=0, //!< Little endian data format
    VmbEndiannessBig,      //!< Big endian data format
    VmbEndiannessLast=255
} VmbEndianness;
typedef VmbUint32_t VmbEndianness_t;
//! Enumeration for the image alignment
typedef enum VmbAlignment
{
    VmbAlignmentMSB=0,    //!< Data is MSB aligned (pppp pppp pppp ....)
    VmbAlignmentLSB,      //!< Data is LSB aligned (.... pppp pppp pppp)
    VmbAlignmentLAST=255
} VmbAlignment;
typedef VmbUint32_t VmbAlignment_t;


//! Structure for accessing data in 12-bit transfer mode, two pixel are coded into 3 bytes
typedef struct Vmb12BitPackedPair_t
{
    VmbUint8_t   m_nVal8_1       ;   //!< High byte of the first Pixel
    VmbUint8_t   m_nVal8_1Low : 4;   //!< Low nibble of the first pixel
    VmbUint8_t   m_nVal8_2Low : 4;   //!< Low nibble of the second pixel
    VmbUint8_t   m_nVal8_2       ;   //!< High byte of the second pixel
}Vmb12BitPackedPair_t;

/** states of the multi media technology support for operating system and processor.
*/
typedef struct VmbSupportState_t
{
    VmbBool_t Processor;         //!< technology supported by the processor
    VmbBool_t OperatingSystem;   //!< technology supported by the OS
}VmbSupportState_t;

/** states of the support for different multimedia technologies*/
typedef struct VmbTechInfo_t
{
    VmbSupportState_t IntelMMX;       //!< INTEL first gen MultiMedia eXtension
    VmbSupportState_t IntelSSE;       //!< INTEL Streaming SIMD Extension
    VmbSupportState_t IntelSSE2;      //!< INTEL Streaming SIMD Extension 2
    VmbSupportState_t IntelSSE3;      //!< INTEL Streaming SIMD Extension 3
    VmbSupportState_t IntelSSSE3;     //!< INTEL Supplemental Streaming SIMD Extension 3
    VmbSupportState_t AMD3DNow;       //!< AMD 3DNow
} VmbTechInfo_t;

/**api info types*/
typedef enum VmbAPIInfo
{
    VmbAPIInfoAll,
    VmbAPIInfoPlatform,     //!< Platform the api was build for
    VmbAPIInfoBuild,        //!< build type (debug or release)
    VmbAPIInfoTechnology,   //!< info about special technoklogies uses in building the api
    VmbAPIInfoLast
} VmbAPIInfo;
typedef VmbUint32_t VmbAPIInfo_t;

typedef struct VmbMono8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t Y;   //!< gray part
}  VmbMono8_t;

typedef struct VmbMono8s_t
{
#ifdef __cplusplus
    typedef VmbInt8_t value_type;
#endif
    VmbInt8_t Y;   //!< gray part
}  VmbMono8s_t;

typedef struct VmbMono10_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t Y;   //!< gray part
}  VmbMono10_t;

typedef struct VmbMono12_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t Y;   //!< gray part
}  VmbMono12_t;


typedef struct VmbMono14_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t Y;   //!< gray part
}  VmbMono14_t;

typedef struct VmbMono16_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t Y;   //!< gray part
}  VmbMono16_t;

typedef struct VmbMono16s_t
{
#ifdef __cplusplus
    typedef VmbInt16_t value_type;
#endif
    VmbInt16_t Y;   //!< gray part
}  VmbMono16s_t;


/** Structure for accessing Windows RGB data */
typedef struct VmbRGB8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t R;   //!< red part
    VmbUint8_t G;   //!< green part
    VmbUint8_t B;   //!< blue part
}  VmbRGB8_t;

typedef struct VmbRGB10_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
}  VmbRGB10_t;

typedef struct VmbRGB12_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
}  VmbRGB12_t;

typedef struct VmbRGB14_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
}  VmbRGB14_t;

typedef struct VmbRGB16_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
}  VmbRGB16_t;


/** Structure for accessing Windows RGB data */
typedef struct VmbBGR8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t B;   //!< blue part
    VmbUint8_t G;   //!< green part
    VmbUint8_t R;   //!< red part
} VmbBGR8_t;

/** Structure for accessing Windows RGB data */
typedef struct VmbBGR10_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
} VmbBGR10_t;

/** Structure for accessing Windows RGB data */
typedef struct VmbBGR12_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
} VmbBGR12_t;
/** struct for 14 bit bgr*/
typedef struct VmbBGR14_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
} VmbBGR14_t;

//! Structure for accessing 16-bit Windows-type RGB data
typedef struct VmbBGR16_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;  //!< blue part
    VmbUint16_t G;  //!< green part
    VmbUint16_t R;  //!< red part
} VmbBGR16_t;


/** Structure for accessing non-Windows RGB data */
typedef struct VmbRGBA8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t R;   //!< red part
    VmbUint8_t G;   //!< green part
    VmbUint8_t B;   //!< blue part
    VmbUint8_t A;   //!< unused
} VmbRGBA8_t, VmbRGBA32_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbBGRA8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t B;   //!< blue part
    VmbUint8_t G;   //!< green part
    VmbUint8_t R;   //!< red part
    VmbUint8_t A;   //!< unused
} VmbBGRA8_t, VmbBGRA32_t;

typedef struct VmbARGB8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t A;   //!< unused
    VmbUint8_t R;   //!< red part
    VmbUint8_t G;   //!< green part
    VmbUint8_t B;   //!< blue part
} VmbARGB8_t, VmbARGB32_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbABGR8_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t A;   //!< unused
    VmbUint8_t B;   //!< blue part
    VmbUint8_t G;   //!< green part
    VmbUint8_t R;   //!< red part
} VmbABGR8_t, VmbABGR32_t;


/** Structure for accessing non-Windows RGB data */
typedef struct VmbRGBA10_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
    VmbUint16_t A;   //!< unused
} VmbRGBA10_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbBGRA10_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
    VmbUint16_t A;   //!< unused
} VmbBGRA10_t;

/** Structure for accessing non-Windows RGB data */
typedef struct VmbRGBA12_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
    VmbUint16_t A;   //!< unused
} VmbRGBA12_t;

/** Structure for accessing non-Windows RGB data */
typedef struct VmbRGBA14_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
    VmbUint16_t A;   //!< unused
} VmbRGBA14_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbBGRA12_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
    VmbUint16_t A;   //!< unused
} VmbBGRA12_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbBGRA14_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
    VmbUint16_t A;   //!< unused
} VmbBGRA14_t;
/** Structure for accessing non-Windows RGB data */
typedef struct VmbRGBA16_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t R;   //!< red part
    VmbUint16_t G;   //!< green part
    VmbUint16_t B;   //!< blue part
    VmbUint16_t A;   //!< unused
} VmbRGBA16_t, VmbRGBA64_t;

/** Structure for accessing Windows RGBA data.*/
typedef struct VmbBGRA16_t
{
#ifdef __cplusplus
    typedef VmbUint16_t value_type;
#endif
    VmbUint16_t B;   //!< blue part
    VmbUint16_t G;   //!< green part
    VmbUint16_t R;   //!< red part
    VmbUint16_t A;   //!< unused
} VmbBGRA16_t, VmbBGRA64_t;


/**Structure for accessing data in the YUV 4:4:4 format (YUV)
    prosilica component order
*/
typedef struct VmbYUV444_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t U;   //!< U
    VmbUint8_t Y;   //!< Luma
    VmbUint8_t V;   //!< V
} VmbYUV444_t;

//! Structure for accessing data in the YUV 4:2:2 format (UYVY)
typedef struct VmbYUV422_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t U;   //!< the U part for both pixels
    VmbUint8_t Y0;  //!< the intensity of the first pixel
    VmbUint8_t V;   //!< the V part for both pixels
    VmbUint8_t Y1;  //!< the intensity of the second pixel
}  VmbYUV422_t;

//! Structure for accessing data in the YUV 4:1:1 format (UYYVYY)
typedef struct VmbYUV411_t
{
#ifdef __cplusplus
    typedef VmbUint8_t value_type;
#endif
    VmbUint8_t U;   //!< the U part for all four pixels
    VmbUint8_t Y0;  //!< the intensity of the first pixel
    VmbUint8_t Y1;  //!< the intensity of the second pixel
    VmbUint8_t V;   //!< the V part for all four pixels
    VmbUint8_t Y2;  //!< the intensity of the third pixel
    VmbUint8_t Y3;  //!< the intensity of the fourth pixel
}  VmbYUV411_t;

//! \endcond



/** image pixel layout information*/
typedef enum VmbPixelLayout
{
    VmbPixelLayoutMono,
    VmbPixelLayoutMonoPacked,
    VmbPixelLayoutRaw,
    VmbPixelLayoutRawPacked,
    VmbPixelLayoutRGB,
    VmbPixelLayoutBGR,
    VmbPixelLayoutRGBA,
    VmbPixelLayoutBGRA,
    VmbPixelLayoutYUV411,
    VmbPixelLayoutYUV422,
    VmbPixelLayoutYUV444,
    VmbPixelLayoutMonoP,
    VmbPixelLayoutMonoPl,
    VmbPixelLayoutRawP,
    VmbPixelLayoutRawPl,
    VmbPixelLayoutYYCbYYCr411,
    VmbPixelLayoutCbYYCrYY411 = VmbPixelLayoutYUV411,
    VmbPixelLayoutYCbYCr422,
    VmbPixelLayoutCbYCrY422 = VmbPixelLayoutYUV422,
    VmbPixelLayoutYCbCr444,
    VmbPixelLayoutCbYCr444 = VmbPixelLayoutYUV444,

    VmbPixelLayoutLAST,
}VmbPixelLayout;
typedef VmbUint32_t VmbPixelLayout_t;
/**image color space information*/
typedef enum VmbColorSpace
{
    VmbColorSpaceUndefined,
    VmbColorSpaceITU_BT709,
    VmbColorSpaceITU_BT601,

}VmbColorSpace;
typedef VmbUint32_t VmbColorSpace_t;
/**image pixel information*/
typedef struct VmbPixelInfo
{
    VmbUint32_t         BitsPerPixel;
    VmbUint32_t         BitsUsed;
    VmbAlignment_t      Alignment;
    VmbEndianness_t     Endianness;
    VmbPixelLayout_t    PixelLayout;
    VmbBayerPattern_t   BayerPattern;
    VmbColorSpace_t     Reserved;
} VmbPixelInfo;
/**image information*/
typedef struct VmbImageInfo
{
    VmbUint32_t     Width;
    VmbUint32_t     Height;
    VmbInt32_t      Stride;
    VmbPixelInfo    PixelInfo;
}VmbImageInfo;
/** wimba image type*/
typedef struct VmbImage
{
    VmbUint32_t     Size; // struct size
    void *          Data;
    VmbImageInfo    ImageInfo;

}VmbImage;

/**transform info for special debayering modes*/
typedef enum VmbDebayerMode
{
    VmbDebayerMode2x2,
    VmbDebayerMode3x3,
    VmbDebayerModeLCAA,
    VmbDebayerModeLCAAV,
    VmbDebayerModeYUV422,
}VmbDebayerMode;
typedef VmbUint32_t  VmbDebayerMode_t;

/**transform info types*/
typedef enum VmbTransformType
{
    VmbTransformTypeNone,
    VmbTransformTypeDebayerMode,
    VmbTransformTypeColorCorrectionMatrix,
    VmbTransformTypeGammaCorrection,
    VmbTransformTypeOffset,
    VmbTransformTypeGain,
}VmbTransformType;
typedef VmbUint32_t VmbTransformType_t;

/**transform info for color correction using a 3x3 matrix multiplication*/
typedef struct VmbTransformParameterMatrix3x3
{
    VmbFloat_t          Matrix[9];
}VmbTransformParameterMatrix3x3;
/**transform info for gama correcting the immage, currently unsuported*/
typedef struct VmbTransformParameterGamma
{
    VmbFloat_t          Gamma;
}VmbTransformParameterGamma;
/**transform info for special debayering modes*/
typedef struct VmbTransformParameteDebayer
{
    VmbDebayerMode_t  Method;
}VmbTransformParameterDebayer;

typedef struct VmbTransformParameterOffset
{
    VmbInt32_t Offset;
} VmbTransformParameterOffset;

typedef struct VmbTransformParameterGain
{
    VmbUint32_t Gain;
} VmbTransformParameterGain;

/**transform info variant*/
typedef union VmbTransformParameter
{
    VmbTransformParameterMatrix3x3  Matrix3x3;
    VmbTransformParameterDebayer    Debayer;
    VmbTransformParameterGamma      Gamma;
    VmbTransformParameterOffset     Offset;
    VmbTransformParameterGain       Gain;
}VmbTransformParameter;
/**transform info interface structure*/
typedef struct VmbTransformInfo
{
    VmbTransformType_t      TransformType;
    VmbTransformParameter   Parameter;
}VmbTransformInfo;

#endif