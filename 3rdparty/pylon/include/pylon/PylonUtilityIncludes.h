//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2008-2019 Basler AG
//  http://www.baslerweb.com
//  Author:  JS
//-----------------------------------------------------------------------------
/*!
\file
\brief  Master include for the PylonUtility library
*/

#ifndef PYLONUTILITIESINCLUDES_H_INCLUDED__
#define PYLONUTILITIESINCLUDES_H_INCLUDED__

#include <pylon/Platform.h>
#include <pylon/PylonUtility.h>
#include <pylon/ImageFormat.h>
#include <pylon/PylonImage.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/FeaturePersistence.h>
#include <pylon/ImagePersistence.h>
#if defined(PYLON_WIN_BUILD)
// windows only headers
#    include <pylon/PylonBitmapImage.h>
#    include <pylon/AviWriter.h>
#endif
#include <pylon/VideoWriter.h>

// The image format converter classes based on Pylon::CPixelFormatConverter are now deprecated
// and replaced by wrapper classes that use the new, much more convenient, Pylon::CImageFormatConverter class internally.
#include <pylon/PixelFormatConverter.h>
#include <pylon/PixelFormatConverterBayer.h>
#include <pylon/PixelFormatConverterBayer16.h>
#include <pylon/PixelFormatConverterMonoXX.h>
#include <pylon/PixelFormatConverterGamma.h>
#include <pylon/PixelFormatConverterMonoPacked.h>
#include <pylon/PixelFormatConverterGammaPacked.h>
#include <pylon/PixelFormatConverterRGB.h>
#include <pylon/PixelFormatConverterTruncate.h>
#include <pylon/PixelFormatConverterTruncatePacked.h>
#include <pylon/PixelFormatConverterYUV422.h>

#if defined (PYLON_WIN_BUILD) && !defined (PYLON_NO_AUTO_IMPLIB)
#    include <pylon/PylonLinkage.h>
#    pragma comment(lib, PYLON_LIB_NAME( "PylonUtility" ))
#endif

#endif
