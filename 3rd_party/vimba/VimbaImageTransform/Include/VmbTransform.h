/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this header file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        VmbTransform.h

  Description: Definition of image transform functions for the Vimba APIs.

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

#ifndef VMB_TRANSFORM_H_
#define VMB_TRANSFORM_H_
#ifndef VMB_TRANSFORM
#define VMB_TRANSFORM
#endif
#pragma once

#include "VmbTransformTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VMB_WIN32DLL_API
#ifndef VMB_NO_EXPORT
#   ifdef VMB_EXPORTS
#       if defined(__ELF__) && (defined(__clang__) || defined(__GNUC__))
#           define VMB_WIN32DLL_API __attribute__((visibility("default")))
#       elif defined( __APPLE__ ) || defined(__MACH__)
#           define VMB_WIN32DLL_API __attribute__((visibility("default")))
#       else
#           ifndef _WIN64
#               define VMB_WIN32DLL_API __declspec(dllexport) __stdcall
#           else
#               define VMB_WIN32DLL_API __stdcall
#           endif
#       endif
#   else
#       if defined (__ELF__) && (defined(__clang__) || defined(__GNUC__))
#           define VMB_WIN32DLL_API
#       elif defined( __APPLE__ ) || defined(__MACH__)
#           define VMB_WIN32DLL_API
#       else
#           define VMB_WIN32DLL_API __declspec(dllimport) __stdcall
#       endif
#   endif
#else
#       define VMB_WIN32DLL_API 
#endif
#endif

//
// Method:      VmbGetVersion()
//
// Purpose:     Inquire the library version.
//
// Parameters:
//
//  [out]    VmbUint32_t *   pValue      Contains the library version (Major,Minor,Sub,Build)
//
// Returns:
//
//  - VmbErrorSuccess:       If no error
//  - VmbErrorBadParameter:  if the given pointer is NULL
//
// Details:     This function can be called at anytime, even before the library is
//              initialized.
//
VmbError_t VMB_WIN32DLL_API VmbGetVersion ( VmbUint32_t*  pValue );

//
// Method:      VmbGetTechnoInfo()
//
// Purpose:     Get information about processor supported features.
//
// Parameters:
//
//  [out]       VmbTechInfo_t*    pTechnoInfo    Returns the supported SIMD technologies
//
// Returns:
//
//  - VmbErrorSuccess:        if no error
//  - VmbErrorBadParameter:   if the given pointer is NULL
//
// Details:     This should be called before using any SIMD (MMX,SSE) optimized functions
//
VmbError_t VMB_WIN32DLL_API VmbGetTechnoInfo( VmbTechInfo_t*  pTechnoInfo );

//
// Method:      VmbGetErrorInfo()
//
// Purpose:     Translate Vimba error codes into a human-readable string.
//
// Parameters:
//
//  [in ]       VmbError_t      errorCode       The error code to get a readable string for
//  [out]       VmbANSIChar_t*  pInfo           Pointer to a zero terminated string that will 
//                                              contain the error information on return
//  [in ]       VmbUint32_t     maxInfoLength   The length of the pInfo buffer
//
// Returns:
//
//  - VmbErrorSuccess:       if no error
//  - VmbErrorBadParameter:  if the given pointer is NULL, or if maxInfoLength is 0
//  - VmbErrorMoreData:      if maxInfoLength is too small to hold the complete information
//
VmbError_t VMB_WIN32DLL_API VmbGetErrorInfo( VmbError_t      errorCode,
                                             VmbANSIChar_t*  pInfo,
                                             VmbUint32_t     maxInfoLength );
                                                
//
// Method:      VmbGetApiInfoString()
//
// Purpose:     Get information about the currently loaded Vimba ImageTransform API.
//
// Parameters:
//
//  [in ]       VmbAPIInfo_t    infoType        Type of information to return
//  [out]       VmbANSIChar_t*  pInfo           Pointer to a zero terminated string that 
//                                              will contain the information on return
//  [in ]       VmbUint32_t     maxInfoLength   The length of the pInfo buffer
//
// Returns:
//
//  - VmbErrorSuccess:      if no error
//  - VmbErrorBadParameter: if the given pointer is NULL
//  - VmbErrorMoreData:     if maxInfoLength is too small to hold the complete information
//
// Details: infoType may be one of the following values:
//          - VmbAPIInfoAll:         Returns all information about the API
//          - VmbAPIInfoPlatform:    Returns information about the platform the API was built for (x86 or x64)
//          - VmbAPIInfoBuild:       Returns info about the API built (debug or release)
//          - VmbApiInfoTechnology:  Returns info about the supported technologies the API was built for (OpenMP or OpenCL)
//
VmbError_t VMB_WIN32DLL_API VmbGetApiInfoString( VmbAPIInfo_t    infoType,
                                                 VmbANSIChar_t*  pInfo,
                                                 VmbUint32_t     maxInfoLength );

//
// Method:      VmbSetDebayerMode()
//
// Purpose:     Set transformation options to a predefined debayering mode.
//
// Parameters:
//
//  [in ]       VmbDebayerMode_t    debayerMode     The mode used for debayering the source raw image, default mode is
//                                                  2x2 debayering. Debayering modes only work for image widths and
//                                                  heights divisible by two
//  [in,out]    VmbTransformInfo*   pTransformInfo  Parameter that contains information about special
//                                                  transform functionality
//
// Returns:
//
//  - VmbErrorSuccess:       if no error
//  - VmbErrorBadParameter:  if the given pointer is NULL
//
// Details:     Debayering is only applicable to image formats with both an even width and an even height.
//
VmbError_t VMB_WIN32DLL_API VmbSetDebayerMode( VmbDebayerMode_t   debayerMode,
                                               VmbTransformInfo*  pTransformInfo );

//
// Method:      VmbSetColorCorrectionMatrix3x3()
//
// Purpose:     Set transformation options to a 3x3 color matrix transformation.
//
// Parameters:
//
//  [in ]       const VmbFloat_t*   pMatrix          Color correction matrix
//  [in,out]    VmbTransformInfo*   pTransformInfo   Parameter that is filled with information 
//                                                   about special transform functionality
//
// Returns:
//
//  - VmbErrorSuccess:       If no error
//  - VmbErrorBadParameter:  if one of the given pointers is NULL
//
VmbError_t VMB_WIN32DLL_API VmbSetColorCorrectionMatrix3x3( const VmbFloat_t*  pMatrix,
                                                            VmbTransformInfo*  pTransformInfo );

//
// Method:      VmbSetGammaCorrection()
//
// Purpose:     Retrieve the version number of VimbaC.
//
// Parameters:
//
//  [in ]       VmbFloat_t          gamma           Float gamma correction to set
//  [in,out]    VmbTransformInfo*   pTransformInfo  Transform info to set gamma correction to
//
// Returns:
//
//  - VmbErrorSuccess:       If no error
//  - VmbErrorBadParameter:  if the given pointer is NULL
//
VmbError_t VMB_WIN32DLL_API VmbSetGammaCorrection( VmbFloat_t         gamma,
                                                   VmbTransformInfo*  pTransformInfo );

//
// Method:      VmbSetImageInfoFromPixelFormat()
//
// Purpose:     Set image info member values in VmbImage from pixel format.
//
// Parameters:
//
//  [in ]       VmbPixelFormat_t    pixelFormat     PixelFormat describes the pixel format used by
//                                                  the image data member
//  [in ]       VmbUint32_t         width           Width of the image in pixels
//  [in ]       VmbUint32_t         height          Height of the image in pixels
//  [in,out]    VmbImage*           pImage          Pointer to Vimba image to set the info to
//
// Returns:
//
//  - VmbErrorSuccess:       if no error
//  - VmbErrorBadParameter:  if the given pointer is NULL or one of the "image" members is invalid
//  - VmbErrorStructSize:    if the image struct size doesn't match its "Size" member
//
// Details:     A VmbPixelFormat_t can be obtained from Vimba C/C++ APIs frame
//              or from the PixelFormat feature. For displaying images, it is suggested to use
//              VmbSetImageInfoFromString() or to look up a matching VmbPixelFormat_t
//
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromPixelFormat( VmbPixelFormat_t  pixelFormat, 
                                                            VmbUint32_t       width,
                                                            VmbUint32_t       height,
                                                            VmbImage*         pImage );

//
// Method:      VmbSetImageInfoFromString()
//
// Purpose:     Set image info member values in VmbImage from string.
//
// Parameters:
//
//  [in ]       const VmbANSIChar_t*    pImageFormat    Image format as a (const) case insensitive string
//  [in ]       VmbUint32_t             stringLength    The length of the pixel format string 
//  [in ]       VmbUint32_t             width           Width of the image in pixels
//  [in ]       VmbUint32_t             height          Height of the image in pixels
//  [in,out]    VmbImage *              pImage          Pointer to Vimba image to set the info to
//
// Returns:
//
//  - VmbErrorSuccess:        if no error
//  - VmbErrorBadParameter:   if one of the given pointers or the "Data" member in "image" is NULL, or
//                            if width or hight don't match between source and destination, or
//                            if one of the parameters for the conversion does not fit
//  - VmbErrorStructSize:     if the "image" struct size does not match its "Size" member
//  - VmbErrorResources:      if the there was a memory fault during processing
//
// Details:     function does not read or write to VmbImage::Data member.
//
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromString( const VmbANSIChar_t*  pImageFormat,
                                                       VmbUint32_t           stringLength,
                                                       VmbUint32_t           width,
                                                       VmbUint32_t           height,
                                                       VmbImage*             pImage );

/* Method: VmbSetImageInfoFromInputParameters.
*
* Purpose::  set output image dependent on the input image, user specifies pixel layout and bit depth of out format.
*
* Parameters:
* [in]    VmbPixelFormat_t    InputPixelFormat    input vimba pixel format
* [in]    VmbUint32_t         width               width of the output image
* [in]    VmbUint32_t         height              height of the output image
* [in]    VmbPixelLayout_t    OutputPixelLayout   pixel component layout for output image
* [in]    VmbUint32_t         BitsPerPixel        bit depth of output 8 and 16 supported
* [out]    VmbImage *          pOutputImage        pointer to output simple vimba image
*/
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromInputParameters( VmbPixelFormat_t    InputPixelFormat,
                                                                VmbUint32_t         Width,
                                                                VmbUint32_t         Height,
                                                                VmbPixelLayout_t    OutputPixelLayout,
                                                                VmbUint32_t         BitsPerPixel,
                                                                VmbImage*           pOutputImage);

/* Method: VmbSetImageInfoFromInputImage.
*
* Purpose:  set output image compatible to input image with given layout and bit depth
*           the output image will have same dimensions as the input image
*
* [in]      const VmbImage *        pInputImage         input image with fully initialized inmage info elemets
* [in]      VmbPixelLayout_t        OutputPixelLayout   layout for the output image
* [in]      VmbUint32_t             BitsPerPixel        bit depth for output image 8bit and 16bit supported
* [out]     VmbImage *              pOutputImage        output image to set the compatible format to
*/
VmbError_t VMB_WIN32DLL_API VmbSetImageInfoFromInputImage(  const VmbImage *    pInputImage,
                                                            VmbPixelLayout_t    OutputPixelLayout,
                                                            VmbUint32_t         BitsPerPixel,
                                                            VmbImage*           pOutputImage);

//
// Method:      VmbImageTransform()
//
// Purpose:     Transform images from one pixel format to another with possible transformation options.
//              The transformation is defined by the provided images and the desired transformation.
//
// Parameters:
//
//  [in ]       const VmbImage*             pSource         Pointer to source image
//  [in,out]    VmbImage*                   pDestination    Pointer to destination image
//  [in ]       const VmbTransformInfo*     pParameter      Optional transform parameters
//  [in ]       VmbUint32_t                 parameterCount  Number of transform parameters
//
// Returns:
//
//  - VmbErrorSuccess:          if no error
//  - VmbErrorBadParameter:     if any image pointer or their "Data" members is NULL, or
//                              if "Width" or "Height" don't match between source and destination, or
//                              if one of the parameters for the conversion does not fit
//  - VmbErrorStructSize:       if the image structs size don't match their "Size" member
//  - VmbErrorNotImplemented:   if there is no transformation between source and destination format
//
// Details:     Create the source and destination image info structure with VmbSetImageInfoFromPixelFormat
//               or VmbSetimageInfoFromString and keep those structures as template.
//              For calls to transform, simply attach the image to the Data member.
//              The optional parameters, when set, are constraints on the transform.
//
VmbError_t VMB_WIN32DLL_API VmbImageTransform( const VmbImage*          pSource,
                                               VmbImage*                pDestination,
                                               const VmbTransformInfo*  pParameter,
                                               VmbUint32_t              parameterCount );

#ifdef __cplusplus
}
#endif // #ifdef __cplusplus
#endif //#ifndef UNI_TRANSFORM_H_
