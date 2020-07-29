//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2008-2019 Basler AG
//  http://www.baslerweb.com
//  Author:  Thomas Koeller
//-----------------------------------------------------------------------------
/*!
\file
\brief Device class definitions
*/

#ifndef __DEVICE_CLASS_H__
#define __DEVICE_CLASS_H__

#if _MSC_VER > 1000
#pragma once
#endif //_MSC_VER > 1000

#include <pylon/Platform.h>


namespace Pylon
{
    /** \addtogroup Pylon_TransportLayer
     * @{
     */
    // PYLON_WIN_BUILD only controls whether the DeviceClass is listed in the API reference guide
    const char* const Basler1394DeviceClass = "Basler1394";
    const char* const BaslerGigEDeviceClass = "BaslerGigE"; ///< This device class can be used to create the corresponding Transport Layer object or when creating Devices with the Transport Layer Factory.
    const char* const BaslerCamEmuDeviceClass = "BaslerCamEmu";
    const char* const BaslerIpCamDeviceClass = "BaslerIPCam";
#if defined(PYLON_WIN_BUILD)
    const char* const BaslerCameraLinkDeviceClass = "BaslerCameraLink"; ///< This device class can be used to create the corresponding Transport Layer object or when creating Devices with the Transport Layer Factory.
#else
    const char* const BaslerCameraLinkDeviceClass = "BaslerCameraLink";
#endif
    const char* const BaslerGenTlDeviceClassPrefix = "BaslerGTC";    ///< The actual device class string is made up of this prefix + '/' + [TL Vendor] + '/' + [TL Model].
    const char* const BaslerGenTlGevDeviceClass = "BaslerGTC/Basler/GEV";
    const char* const BaslerGenTlU3vDeviceClass = "BaslerGTC/Basler/U3V";
    const char* const BaslerGenTlCxpDeviceClass = "BaslerGTC/Basler/CXP";
    const char* const BaslerUsbDeviceClass = "BaslerUsb";            ///< This device class can be used to create the corresponding Transport Layer object or when creating Devices with the Transport Layer Factory.
    const char* const BaslerBconDeviceClass = "BaslerBcon";          ///< This device class can be used to create the corresponding Transport Layer object or when creating Devices with the Transport Layer Factory.
    PYLON_DEPRECATED("Use BaslerCameraLinkDeviceClass") static const char* const BaslerCLSerDeviceClass = "BaslerCameraLink";
    PYLON_DEPRECATED("Use BaslerIpCamDeviceClass") static const char* const IpCamDeviceClass = "BaslerIPCam";
    /**
     * @}
     */
}
#endif
